#!/usr/bin/env python3
"""can_id_find.py - c6_can_link(Rev.2 以降)の USB ログから CAN ID を探す

ログの取り方(Mac): can_id_confirm_procedure.md の 1.2
  cat /dev/cu.usbmodemXXXX | tee ~/Documents/workspace/obd_meter/logs/can_reverse.txt

使い方(このフォルダで):
  python3 can_id_find.py switch ログ            リバースなど「切り替えた時だけ変わる」信号を探す
  python3 can_id_find.py boost  ログ            アクセル(0618A001 b7)に遅れて追従するバイトを探す(ブースト)
  python3 can_id_find.py slow   ログ            ゆっくり一方向に動くバイトを探す(燃料。長い走行)
  python3 can_id_find.py fuel   ログ            給油の前後で値が変わったバイトを探す(IG OFF の切れ目を自動で見つける)
  python3 can_id_find.py fuel   前ログ 後ログ   同上。給油前と給油後が別ファイルのとき
  python3 can_id_find.py show   ログ ID [b]     指定 ID(とバイト)の値の移り変わりを時刻つきで出す

ログの L 行の形式: L <C|B> <millis> <ID 16進8桁> <len> <データ 16進>
L 行以外(HB、NEWID、ID 一覧)は読み飛ばす。
"""
import sys
import re
from collections import defaultdict

LINE = re.compile(r"^L ([CB]) (\d+) ([0-9A-Fa-f]{8}) (\d+)((?: [0-9A-Fa-f]{2})*)\s*$")
ID_ACCEL = "0618A001"
ACCEL_BYTE = 7


def read_log(path):
    """{(bus, id): [(t_ms, bytes), ...]}  t_ms は昇順(XIAO の millis)"""
    series = defaultdict(list)
    n = 0
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for ln in f:
            m = LINE.match(ln)
            if not m:
                continue
            bus, t, cid, dlc, data = m.groups()
            b = bytes.fromhex(data.replace(" ", ""))
            if len(b) != int(dlc):
                continue
            series[(bus, cid.upper())].append((int(t), b))
            n += 1
    if n == 0:
        sys.exit("L 行が 1 行もありません。c6_can_link が Rev.2 以降か、ログの中身を確かめてください: " + path)
    for k in series:
        series[k].sort(key=lambda x: x[0])
    return series


def fmt_t(ms):
    return "%d:%02d.%d" % (ms // 60000, (ms // 1000) % 60, (ms % 1000) // 100)


def byte_transitions(samples, j):
    """バイト j の値が変わった所: [(t_ms, 前の値, 後の値)]"""
    out = []
    prev = None
    for t, b in samples:
        if j >= len(b):
            continue
        v = b[j]
        if prev is not None and v != prev:
            out.append((t, prev, v))
        prev = v
    return out


def cmd_switch(series, max_trans=8):
    t0 = min(s[0][0] for s in series.values())
    t1 = max(s[-1][0] for s in series.values())
    print("ログの範囲: %s 〜 %s(%d 秒)" % (fmt_t(t0 - t0), fmt_t(t1 - t0), (t1 - t0) // 1000))
    print("変わった回数が 1〜%d 回のバイト(回数の少ない順)。時刻は電源 ON からの 分:秒。" % max_trans)
    print("探している信号は、操作した時刻と一致し、N→R→N→R なら 3 回変わっているもの。\n")
    rows = []
    for (bus, cid), samples in series.items():
        dlc = max(len(b) for _, b in samples)
        for j in range(dlc):
            tr = byte_transitions(samples, j)
            if 1 <= len(tr) <= max_trans:
                rows.append((len(tr), bus, cid, j, tr))
    rows.sort(key=lambda r: (r[0], r[1], r[2], r[3]))
    if not rows:
        print("該当なし。操作の前後でログが取れているか確かめてください。")
        return
    for n, bus, cid, j, tr in rows:
        steps = "  ".join("%s %02X→%02X" % (fmt_t(t - t0), a, b) for t, a, b in tr)
        print("%s %s b%d  %d 回: %s" % (bus, cid, j, n, steps))
    print("\n行の読み方: 「0:45.2 00→40」= 電源 ON の 45.2 秒後に 00 から 40 に変わった。")


def resample(samples, j, grid):
    """バイト j をグリッド時刻で直前値に補間"""
    out = []
    k = 0
    cur = None
    for t in grid:
        while k < len(samples) and samples[k][0] <= t:
            b = samples[k][1]
            cur = b[j] if j < len(b) else cur
            k += 1
        out.append(cur)
    return out


def corr(x, y):
    pairs = [(a, b) for a, b in zip(x, y) if a is not None and b is not None]
    if len(pairs) < 20:
        return 0.0
    n = len(pairs)
    mx = sum(a for a, _ in pairs) / n
    my = sum(b for _, b in pairs) / n
    sxx = sum((a - mx) ** 2 for a, _ in pairs)
    syy = sum((b - my) ** 2 for _, b in pairs)
    if sxx == 0 or syy == 0:
        return 0.0
    sxy = sum((a - mx) * (b - my) for a, b in pairs)
    return sxy / (sxx * syy) ** 0.5


def cmd_boost(series):
    key = ("C", ID_ACCEL)
    if key not in series:
        sys.exit("アクセルの ID %s がログにありません。" % ID_ACCEL)
    acc = series[key]
    t0 = acc[0][0]
    t1 = acc[-1][0]
    step = 100
    grid = list(range(t0, t1, step))
    a = resample(acc, ACCEL_BYTE, grid)
    amax = max(v for v in a if v is not None)
    print("アクセル(0618A001 b7)の最大 %d/255 = %.0f %%。全開が無いとブーストは見つけにくい(95 %% 以上が数秒あること)。" % (amax, amax / 2.55))
    lags = [0, 300, 500, 800, 1200]
    print("各バイトとアクセルの相関(遅れ 0〜1.2 秒で最大のもの)。上位 25。相関が高く、遅れが 0.3〜1 秒にあるものがブーストの候補。")
    print("遅れ 0 で最大のものはスロットル開度などアクセルそのもの。\n")
    rows = []
    for (bus, cid), samples in series.items():
        if (bus, cid) == key:
            continue
        dlc = max(len(b) for _, b in samples)
        for j in range(dlc):
            tr = byte_transitions(samples, j)
            if len(tr) < 4:
                continue
            y = resample(samples, j, grid)
            vals = [v for v in y if v is not None]
            lo, hi = min(vals), max(vals)
            if hi - lo < 4:
                continue
            best = (0.0, 0)
            for lag in lags:
                sh = lag // step
                c = corr(a[: len(a) - sh] if sh else a, y[sh:] if sh else y)
                if abs(c) > abs(best[0]):
                    best = (c, lag)
            rows.append((abs(best[0]), best[0], best[1], bus, cid, j, lo, hi))
    rows.sort(reverse=True)
    for _, c, lag, bus, cid, j, lo, hi in rows[:25]:
        print("%s %s b%d  相関 %+.2f  遅れ %.1f 秒  値 %02X〜%02X(%d〜%d)" % (bus, cid, j, c, lag / 1000, lo, hi, lo, hi))
    print("\n候補が決まったら  python3 can_id_find.py show ログ ID b  で時系列を見て、全開の数秒後に最大・戻すと最小、を確かめる。")


def cmd_slow(series):
    print("変わった回数が少なく(2〜40 回)、ほぼ一方向に動いたバイト。燃料は給油で増え、走行で減る。\n")
    rows = []
    for (bus, cid), samples in series.items():
        dlc = max(len(b) for _, b in samples)
        for j in range(dlc):
            tr = byte_transitions(samples, j)
            if not (2 <= len(tr) <= 40):
                continue
            up = sum(1 for _, a, b in tr if b > a)
            down = len(tr) - up
            mono = max(up, down) / len(tr)
            if mono < 0.8:
                continue
            first = tr[0][1]
            last = tr[-1][2]
            rows.append((len(tr), bus, cid, j, first, last, "増" if up >= down else "減"))
    rows.sort(key=lambda r: (r[0], r[1], r[2], r[3]))
    for n, bus, cid, j, first, last, d in rows:
        print("%s %s b%d  %d 回 %s  %02X → %02X(%d → %d)" % (bus, cid, j, n, d, first, last, first, last))
    if not rows:
        print("該当なし。")


def stable_values(samples, j, t_from, t_to):
    """時間 t_from〜t_to の間のバイト j の値の集合(直前値も含める)"""
    vals = set()
    cur = None
    for t, b in samples:
        if j >= len(b):
            continue
        if t < t_from:
            cur = b[j]
            continue
        if t > t_to:
            break
        if cur is not None and not vals:
            vals.add(cur)
        vals.add(b[j])
        cur = b[j]
    if not vals and cur is not None:
        vals.add(cur)
    return vals


def cmd_fuel(before, after, win_ms=60000, skip_ms=15000):
    """before/after: {(bus,id): samples}。給油前の最後 win_ms と、給油後の最初(skip_ms 飛ばして)win_ms を比べる"""
    tb = max(s[-1][0] for s in before.values())
    ta = min(s[0][0] for s in after.values())
    b_from, b_to = tb - win_ms, tb
    a_from, a_to = ta + skip_ms, ta + skip_ms + win_ms
    print("比べる区間: 給油前 = 切れ目の前 %d 秒、給油後 = 再開の %d 秒後から %d 秒" % (win_ms // 1000, skip_ms // 1000, win_ms // 1000))
    print("給油前に一定(値 1 つ)、給油後も一定で、値が変わったバイト。燃料なら給油後のほうが大きい(増)。\n")
    rows = []
    for key in sorted(set(before) & set(after)):
        sb, sa = before[key], after[key]
        dlc = max(len(b) for _, b in sb)
        for j in range(dlc):
            vb = stable_values(sb, j, b_from, b_to)
            va = stable_values(sa, j, a_from, a_to)
            if len(vb) == 1 and len(va) == 1 and vb != va:
                x, y = next(iter(vb)), next(iter(va))
                rows.append((key[0], key[1], j, x, y))
    for bus, cid, j, x, y in rows:
        print("%s %s b%d  %02X → %02X(%d → %d)%s" % (bus, cid, j, x, y, x, y, "  増" if y > x else "  減"))
    if not rows:
        print("該当なし。給油前後の区間に L 行があるか(show で見る)、給油量が足りているかを確かめる。")
    print("\n値が少し揺れるバイト(給油前後とも 2〜3 通りの値だが、範囲がずれたもの):")
    rows2 = []
    for key in sorted(set(before) & set(after)):
        sb, sa = before[key], after[key]
        dlc = max(len(b) for _, b in sb)
        for j in range(dlc):
            vb = stable_values(sb, j, b_from, b_to)
            va = stable_values(sa, j, a_from, a_to)
            if 1 <= len(vb) <= 3 and 1 <= len(va) <= 3 and not (vb & va) and (len(vb) > 1 or len(va) > 1):
                rows2.append((key[0], key[1], j, sorted(vb), sorted(va)))
    for bus, cid, j, vb, va in rows2:
        print("%s %s b%d  %s → %s" % (bus, cid, j, "/".join("%02X" % v for v in vb), "/".join("%02X" % v for v in va)))
    if not rows2:
        print("なし")


def split_at_gap(series, gap_ms=20000):
    """C-CAN の L 行が gap_ms 以上途切れた所(IG OFF)で前後に分ける。戻り値 (before, after)。切れ目が無ければ None"""
    ts = sorted(t for (bus, _), s in series.items() if bus == "C" for t, _ in s)
    if len(ts) < 2:
        return None
    best = None
    for a, b in zip(ts, ts[1:]):
        if b - a >= gap_ms and (best is None or b - a > best[1] - best[0]):
            best = (a, b)
    kind = "C-CAN の無信号"
    if best is None:
        # C-CAN が止まらない車(10/22: IG OFF でも流れ続けた)では、回転数 0 が続く区間をエンジン停止とみなす
        key = ("C", ID_ACCEL)
        if key in series:
            off_from = None
            prev_t = None
            for t, d in series[key]:
                rpm = (d[2] << 8 | d[3]) if len(d) >= 4 else 0
                if rpm == 0:
                    if off_from is None:
                        off_from = prev_t if prev_t is not None else t
                else:
                    if off_from is not None and t - off_from >= gap_ms and (best is None or t - off_from > best[1] - best[0]):
                        best = (off_from, t)
                    off_from = None
                prev_t = t
        kind = "エンジン停止(回転数 0)"
    if best is None:
        return None
    before, after = {}, {}
    for k, s in series.items():
        sb = [x for x in s if x[0] <= best[0]]
        sa = [x for x in s if x[0] >= best[1]]
        if sb:
            before[k] = sb
        if sa:
            after[k] = sa
    print("切れ目: %s 〜 %s(%d 秒。%s)" % (fmt_t(best[0] - ts[0]), fmt_t(best[1] - ts[0]), (best[1] - best[0]) // 1000, kind))
    return before, after


def cmd_show(series, cid, j=None):
    cid = cid.upper().replace("0X", "")
    keys = [k for k in series if k[1] == cid]
    if not keys:
        sys.exit("ID %s がログにありません。" % cid)
    samples = series[keys[0]]
    t0 = min(s[0][0] for s in series.values())
    print("%s %s  %d 行" % (keys[0][0], cid, len(samples)))
    prev = None
    for t, b in samples:
        v = b[j] if j is not None and j < len(b) else None
        if j is not None:
            if v == prev:
                continue
            prev = v
            print("%s  b%d = %02X(%d)" % (fmt_t(t - t0), j, v, v))
        else:
            print("%s  %s" % (fmt_t(t - t0), " ".join("%02X" % x for x in b)))


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        sys.exit(1)
    cmd, path = argv[1], argv[2]
    series = read_log(path)
    if cmd == "switch":
        cmd_switch(series)
    elif cmd == "boost":
        cmd_boost(series)
    elif cmd == "slow":
        cmd_slow(series)
    elif cmd == "fuel":
        if len(argv) >= 4:
            cmd_fuel(series, read_log(argv[3]))
        else:
            parts = split_at_gap(series)
            if parts is None:
                sys.exit("切れ目(C-CAN が 20 秒以上途切れた所、または回転数 0 が 20 秒以上続く所)が見つかりません。給油前と給油後が別ファイルなら  fuel 前ログ 後ログ")
            cmd_fuel(*parts)
    elif cmd == "show":
        if len(argv) < 4:
            sys.exit("show には ID が要ります")
        cmd_show(series, argv[3], int(argv[4]) if len(argv) > 4 else None)
    else:
        print(__doc__)
        sys.exit(1)


if __name__ == "__main__":
    main(sys.argv)

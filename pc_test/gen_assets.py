#!/usr/bin/env python3
# gen_assets.py - 595 メーター表示器の書体データとアイコンを作る
#
# 出力: fonts_data.h / fonts_data.cpp(表示器スケッチの src/ に置く)
# 書体: Saira, Saira Semi Condensed, IBM Plex Sans JP, IBM Plex Mono(すべて SIL OFL 1.1)。fetch_fonts.sh で fonts/ に取ってくる。
# 形式: 1 文字ごとに 4bit の濃さを RLE で持つ。1 バイト = (連続数-1)<<4 | 濃さ。
#       連続数-1 が 15 のときは次の 1 バイトを足す(連続数 = 16 + 次のバイト)。
#
# いつ使うか: 画面に出す文字や文字の大きさを増やしたとき(下の SPECS と文字の一覧を直して作り直す)。
#   画面の文言を変えたときは、新しい日本語の文字を UI_JP に足して作り直す(24px の日本語は広めに入れてある)。
#   足りない文字があると、表示器のシリアルの「無い文字」と PC 上の試験の「無かった文字」が 0 でなくなる。
#
# 使い方: python3 gen_assets.py fonts ../meter_display/src
# 必要なもの: pip install freetype-py uharfbuzz fonttools numpy pillow playwright
#             playwright install chromium(アイコンを Chromium で描くのに使う)
import sys, pathlib, json
import numpy as np
import freetype
import uharfbuzz as hb
from fontTools.ttLib import TTFont

FONTS = pathlib.Path(sys.argv[1])
OUT = pathlib.Path(sys.argv[2])

ASCII = "".join(chr(c) for c in range(0x20, 0x7F))
DIGITS = "0123456789"
HIRA = "".join(chr(c) for c in range(0x3041, 0x3097))
KATA = "".join(chr(c) for c in range(0x30A1, 0x30FB)) + "ー"
JP_PUNCT = "、。・「」（）〜：／％℃°–—…"
KANJI = (
    "水温燃料全開走行確認設定基準変所件取戻点自動明輪部回転数警告温度受信状態通速誤欠落送元描画起電源投入最大小"
    "模擬実車値時間分秒表示器線圧油気吸排度距離平均現在今前後次左右上下中外内高低強弱閉入切有無効正常異不可能"
    "保存失敗成功初期化版番号色赤青緑黄白黒灰橙紺背景枠棒太細長短広狭角丸形式種類別名称単位量率比割合計測記録"
    "再生停止始終了完待機準備読書込更新接続断続信波周字文面輝暗夜昼照光消灯滅検整調節換算義範囲限界超過足残満"
    "空給補助械制御運操作押離触指方向進退加減増倍半同違差和積商余絶対相標規格品質安危険注意禁許証試験結果原因"
    "策処理復帰使用未特出本体側端子配品番年月日曜週毎回目的地図位置北南東西道路駐場店家族人名自他者手足頭体心"
    "以探池済々夏冬春秋晴雨雪風熱冷暖房窓扉鍵席荷重軽静騒音声楽映像写真撮影録画再開閉鎖解除登録削除追加編集選択"
    "衛星横応答捨針圧"
)
JP_BROAD = "".join(dict.fromkeys(HIRA + KATA + JP_PUNCT + KANJI))

# 画面に出る日本語。10/23: ui.cpp の文字列(注釈を除く)に出てくる ASCII 以外の文字を、作るたびに ui.cpp から読む(書き足し忘れが無いように)。
# 30px・34px・24px にはこれを全部入れる(24px は広めの JP_BROAD も)。大きい字は使う所の文字だけ(下の UI_NAME_JP など)
import re as _re
_UI_SRC = pathlib.Path(__file__).resolve().parent.parent / "meter_display" / "ui.cpp"
def _ui_chars():
    src = _re.sub(r"//[^\n]*", "", _UI_SRC.read_text(encoding="utf-8"))
    out = []
    for lit in _re.findall(r'"((?:[^"\\]|\\.)*)"', src):
        out += [ch for ch in lit if ord(ch) >= 0x80]
    return "".join(dict.fromkeys(out))
UI_JP = "".join(dict.fromkeys("°–−" + _ui_chars()))
UI_JP_BROAD = "".join(dict.fromkeys(JP_BROAD + UI_JP))
def _jp(*texts):  # 文字列の ASCII 以外の文字(重なりを除く)
    return "".join(dict.fromkeys(ch for t in texts for ch in t if ord(ch) >= 0x80))
# 1 画面 1 項目の画面(10/23)の大きい字。項目の名前(48 / 44 / 34 px)、値(100 / 64 / 44 / 34 px)、下の帯(44 px)に出る日本語
UI_NAME_JP = _jp("シフトポイント", "自動で戻る", "明るさ", "水温の警告", "デモ", "RaceChrono 送信", "時間の記録", "受信の状態", "CAN確認", "やり直す",
                 "通信速度", "受信", "誤り・欠落", "起動", "送信", "位置", "向きの補正", "設定", "戻る", "サーキット",
                 "燃料",  # 2.2.0: 受信の状態の「燃料」。2.3.0: 「描画」は「FPS」にした
                 "本体の状態", "CANリーダー", "C-CAN・B-CAN", "測位", "速度", "加速度", "接続", "送った数",  # 2.4.0: 受信の状態を分け直した
                 "クロック", "温度", "フラッシュ",  # 2.4.3: 本体の状態の CPU など
                 "前後で比べる",  # 2.6.0: CAN確認の前後で比べる
                 "ライト",  # 2.9.0: 受信の状態の「ライト」(設定の「イルミネーションの明るさ」は 1 行に入らないので 34 px の 2 行。34 px は UI_JP で入る)
                 "過負荷時間 黄", "過負荷時間 赤", "過負荷が リセットされる時間", "デフォルトに戻す",  # 2.10.0: 設定の項目(2.12.0: 高負荷 → 過負荷、クールダウン時間はなくした)
                 "回転数の輪",  # 2.12.0: 設定の項目
                 "画面", "シフトポイント", "水温・過負荷", "その他", "過負荷 回転数", "メンテナンスまでの 累積時間", "累積時間 リセット", "累積時間",
                 "メンテナンス")  # 2.12.0: 設定のカテゴリ、過負荷・累積時間の項目、データ画面の「メンテナンス」
UI_VALUE_JP = _jp("–正常異常待ち入切", "受信なし", "バイト/秒", "版が違います", "模擬", "不明", "5V 投入", "PWR ボタン", "電池", "切(設定)", "開始できない",
                  "接続中", "未接続(待機中)", "3D 衛星 2D 測位なし", "応答なし", "前後 横", "上 済 前 未", "済み", "件分",
                  "平均 最低", "(次の起動から)",  # 2.3.0: 「FPS」の値、RaceChrono の「送信」の値
                  "切(RaceChrono 送信)",  # 2.4.0: GPS・IMU・RaceChrono の、送信が切のとき
                  "候補 件 記録 回", "空き コア なし",  # 2.4.3: CAN確認のページ、本体の状態の CPU・RAM・PSRAM(温度の「°」は英数字の書体)
                  "点 消",  # 2.9.0: 受信の状態の「ライト」の値(「不明」「受信なし」は上にある)
                  "両側 片側")  # 2.12.0: 設定の「回転数の輪」の値
UI_BOTTOM_JP = _jp("決定", "リセット", "変化を記録", "やり直す", "記録 0 回", "切", "入", "取り直す", "戻す")  # 2.10.0: 「戻す」(デフォルトに戻す)  # 2.4.4: 「変更」「開く」「詳細」「向きを取り直す」は使わない
UI_PP_JP = _jp("記録なし", "前を記録した", "前を記録", "後を記録", "待ち 秒")  # 2.6.0: 前後で比べるのページの値・画面・下の帯(44 px)
UI_OPEN_JP = _jp("長押しで開く")  # 2.4.4: メニューに入る下の帯(40 px)
UI_SELFCHECK_JP = _jp("タッチ", "待ち", "衛星待ち", "測位", "切")  # 2.5.0: データのオープニングの自己診断(44 px)
# 2.3.0: 触れたときの案内(2.3.1: 長押しの表示と同じ 48 px の札)
UI_GUIDE_JP = _jp("スワイプ", "長押し")
# 2.10.0: サーキットの真ん中(高負荷時間のタイトル・「時刻 待ち」、接続の状態の値。18 px)
UI_CENTER_JP = _jp("過負荷時間", "クールダウン", "時刻 待ち", "接続中", "未接続", "受信中", "なし", "切", "不可", "–", "メンテナンス")  # 2.12.0: メンテナンスの印

SSC = "SairaSemiCondensed-%s.ttf"
VF = "Saira-VF.ttf"
PRIV = 0xE030  # 等幅でない数字(キャンバスと同じ幅)の置き場所

def fspec(name, px, parts):
    return dict(name=name, px=px, parts=parts)

def ssc(weight_name, wght, px, chars, digits=True):
    """Saira Semi Condensed: 数字は可変書体(wdth 87.5)の等幅数字、ほかは静的な書体。"""
    parts = [dict(file=SSC % weight_name, chars="".join(c for c in chars if c not in DIGITS))]
    if digits:
        parts.append(dict(file=VF, var={"wght": wght, "wdth": 87.5}, feats={"tnum": True}, chars=DIGITS))
        parts.append(dict(file=SSC % weight_name, chars=DIGITS, remap=PRIV))
    return parts

# 2026-10-22: 実走行で「文字が小さく見えない」ので大きくした。使わなくなった大きさは外した
#   (GEAR_230/190、DIAL700_26、NUM600_80/76/54/29/26、NUM500_28、JP600_26/24/60/21/20、JP500_19、MONO 28〜32、MONO500_22)
SPECS = [
    fspec("GEAR_100", 100, [dict(file=VF, var={"wght": 600, "wdth": 100}, feats={"tnum": True}, chars="0123456789NR–")]),  # タコのギア(2.10.0。サーキットの真ん中の 160 px から移した)
    fspec("DIAL600_60", 60, [dict(file=VF, var={"wght": 600, "wdth": 100}, feats={"tnum": True}, chars="0123456789")]),  # アナログ計の回転計の数字(10/22 夜: 太さ 700 → 600)
    fspec("DIAL500_56", 56, [dict(file=VF, var={"wght": 500, "wdth": 100}, feats={"tnum": True}, chars="0123456789.−– ")]),  # アナログ計のブーストの今の値(太さ 500。2.10.0: 76 → 56 px)
    fspec("DIAL500_40", 40, [dict(file=VF, var={"wght": 500, "wdth": 100}, feats={"tnum": True}, chars="0123456789−")]),  # アナログ計の水温・燃料の札
    fspec("DIAL500_28", 28, [dict(file=VF, var={"wght": 500, "wdth": 100}, feats={"tnum": True}, chars="0123456789")]),  # アナログ計のブーストの輪の数字 0・1・2
    fspec("NUM600_100", 100, ssc("SemiBold", 600, 100, "– ")),  # 設定の値(1 画面 1 項目)
    fspec("NUM600_96", 96, ssc("SemiBold", 600, 96, "– :")),  # 設定の値、サーキットの 3000rpm 以下の経過時間
    fspec("NUM600_88", 88, ssc("SemiBold", 600, 88, "")),  # サーキットの真ん中のピークの回転数(2.10.0)
    fspec("NUM600_64", 64, ssc("SemiBold", 600, 64, ASCII + "–−°")),  # データ画面の水温・燃料・全開の割合、受信の状態の値
    fspec("NUM600_56", 56, ssc("SemiBold", 600, 56, ASCII + "–°")),  # データ画面の ▲▼ の累計時間
    fspec("NUM600_48", 48, ssc("SemiBold", 600, 48, ASCII + "–°")),  # 設定の値の単位
    fspec("NUM600_44", 44, ssc("SemiBold", 600, 44, ASCII + "–°")),  # 設定の刻み、大きい字が入らないときの代わり
    fspec("NUM600_34", 34, ssc("SemiBold", 600, 34, ASCII + "–−°")),  # 見出し、単位、時間、オープニング、「– –」
    fspec("NUM600_30", 30, ssc("SemiBold", 600, 30, ASCII + "–°")),
    fspec("NUM600_28", 28, ssc("SemiBold", 600, 28, ASCII + "–−°")),  # データ画面の GPS・IMU・CAN の値、件数
    fspec("NUM600_26", 26, ssc("SemiBold", 600, 26, ":")),  # サーキットの ▼ 3000rpm の経過時間(高負荷時間の下。2.10.0)
    fspec("NUM600_24", 24, ssc("SemiBold", 600, 24, ASCII + "–°")),  # データ画面の診断の行
    fspec("NUM600_22", 22, ssc("SemiBold", 600, 22, ASCII + "–")),  # サーキットの「rpm」、接続の状態の名前と値(2.10.0)
    fspec("NUM600_20", 20, ssc("SemiBold", 600, 20, ASCII + "–°")),  # 「SIM」と場面番号
    fspec("NUM600_16", 16, ssc("SemiBold", 600, 16, "rpm")),  # サーキットの ▼ 3000rpm(高負荷時間の下。2.10.0)
    fspec("JP600_100", 100, [dict(file="IBMPlexSansJP-SemiBold.ttf", chars="入切正常異待ち両側片")]),  # 設定の値(入・切、受信の状態の判定。2.12.0: 両側・片側)
    fspec("JP600_64", 64, [dict(file="IBMPlexSansJP-SemiBold.ttf", chars=UI_VALUE_JP)]),  # 受信の状態の値
    fspec("JP600_56", 56, [dict(file="IBMPlexSansJP-SemiBold.ttf", chars=_jp("タコ", "データ"))]),  # 長押しの次の画面名
    fspec("JP600_48", 48, [dict(file="IBMPlexSansJP-SemiBold.ttf", chars="".join(dict.fromkeys(UI_NAME_JP + UI_GUIDE_JP)))]),  # 項目の名前、「設定」「‹ 戻る」、長押しの「サーキット」、案内
    fspec("JP600_44", 44, [dict(file="IBMPlexSansJP-SemiBold.ttf", chars="".join(dict.fromkeys(UI_NAME_JP + UI_VALUE_JP + UI_BOTTOM_JP + UI_SELFCHECK_JP + UI_PP_JP)))]),  # 下の帯、単位、大きい字が入らないときの代わり、データのオープニング
    fspec("JP600_40", 40, [dict(file="IBMPlexSansJP-SemiBold.ttf", chars=UI_OPEN_JP)]),  # 下の帯「長押しで開く」(2.4.4)
    fspec("JP600_34", 34, [dict(file="IBMPlexSansJP-SemiBold.ttf", chars="".join(dict.fromkeys("タッチ待ち衛星測位接続切受信" + UI_JP)))]),  # 小さい名前
    fspec("JP600_30", 30, [dict(file="IBMPlexSansJP-SemiBold.ttf", chars=UI_JP)]),  # ボタン、データ画面の名前、「タップで変更」
    fspec("JP500_24", 24, [dict(file="IBMPlexSansJP-Medium.ttf", chars=UI_JP_BROAD)]),  # 説明、件数、受信の状態の行
    fspec("JP600_18", 18, [dict(file="IBMPlexSansJP-SemiBold.ttf", chars=" " + UI_CENTER_JP)]),  # サーキットの真ん中(2.10.0)
    fspec("JP500_14", 14, [dict(file="IBMPlexSansJP-Medium.ttf", chars="衛星")]),  # サーキットの接続の状態の「衛星」(2.10.0)
    fspec("MONO600_34", 34, [dict(file="IBMPlexMono-SemiBold.ttf", chars="0123456789ABCDEF -")]),
    fspec("MONO600_38", 38, [dict(file="IBMPlexMono-SemiBold.ttf", chars="0123456789ABCDEF -")]),
    fspec("MONO700_38", 38, [dict(file="IBMPlexMono-Bold.ttf", chars="0123456789ABCDEF -")]),
    fspec("MONO500_28", 28, [dict(file="IBMPlexMono-Medium.ttf", chars="b0123456789 -")]),
]

_cache = {}
def load(file):
    if file not in _cache:
        path = str(FONTS / file)
        blob = hb.Blob.from_file_path(path)
        face = hb.Face(blob)
        tt = TTFont(path, lazy=True)
        axes = [a.axisTag for a in tt["fvar"].axes] if "fvar" in tt else []
        _cache[file] = (path, face, axes)
    return _cache[file]

def rle4(vals):
    out = bytearray(); i = 0; n = len(vals)
    while i < n:
        v = vals[i]; j = i + 1
        while j < n and vals[j] == v and j - i < 271: j += 1
        run = j - i
        if run <= 15: out.append(((run - 1) << 4) | v)
        else:
            out.append(0xF0 | v); out.append(run - 16)
        i = j
    return bytes(out)

def render_part(part, px):
    path, face, axes = load(part["file"])
    font = hb.Font(face)
    ft = freetype.Face(path)
    var = part.get("var")
    if var:
        font.set_variations(var)
        ft.set_var_design_coords([var[a] for a in axes])
    ft.set_pixel_sizes(0, px)
    glyphs = []
    for ch in part["chars"]:
        buf = hb.Buffer(); buf.add_str(ch); buf.guess_segment_properties()
        hb.shape(font, buf, part.get("feats", {}))
        info, pos = buf.glyph_infos[0], buf.glyph_positions[0]
        if info.codepoint == 0 and ch != " ":
            raise SystemExit(f"glyph missing: U+{ord(ch):04X} {ch} in {part['file']}")
        adv = pos.x_advance * px / face.upem
        ft.load_glyph(info.codepoint, freetype.FT_LOAD_NO_HINTING | freetype.FT_LOAD_RENDER)
        bm = ft.glyph.bitmap
        w, h = bm.width, bm.rows
        a = np.array(bm.buffer, np.uint8).reshape(h, w) if w and h else np.zeros((0, 0), np.uint8)
        if w > 255 or h > 255: raise SystemExit("glyph too large")
        q = ((a.astype(np.uint16) * 15 + 127) // 255).astype(np.uint8)
        code = ord(ch) if "remap" not in part else part["remap"] + (ord(ch) - 0x30)
        glyphs.append(dict(code=code, w=w, h=h, xoff=ft.glyph.bitmap_left, yoff=ft.glyph.bitmap_top,
                           adv=int(round(adv * 16)), data=rle4(q.flatten().tolist()), ch=ch))
    return glyphs

def render_icons():
    """SVG のアイコンを Chromium で描き、白の濃さを取り出す。"""
    from playwright.sync_api import sync_playwright
    from PIL import Image
    import io
    W = "#FFFFFF"
    water = f'''<g transform="scale(0.833333)"><path d="M24 5 V28" fill="none" stroke="{W}" stroke-width="6.5" stroke-linecap="round"/>
<circle cx="24" cy="30.5" r="7.5" fill="{W}"/><path d="M27 8.5 H37 M27 17 H37" fill="none" stroke="{W}" stroke-width="4.2" stroke-linecap="round"/>
<path d="M2.5 32 Q5.25 27 8 32 T13.5 32 M45.5 32 Q42.75 27 40 32 T34.5 32 M4 43 Q8 37.5 12 43 T20 43 T28 43 T36 43 T44 43" fill="none" stroke="{W}" stroke-width="4" stroke-linecap="round" stroke-linejoin="round"/></g>'''
    fuel = f'''<g transform="scale(0.833333)"><path d="M10 5 H26 A3 3 0 0 1 29 8 V41 H7 V8 A3 3 0 0 1 10 5 Z M12 10 V20 H24 V10 Z" fill="{W}" fill-rule="evenodd"/>
<rect x="4" y="40" width="28" height="5" rx="1.5" fill="{W}"/>
<path d="M29 23 H32 A3.5 3.5 0 0 1 35.5 26.5 V35.5 A3.25 3.25 0 0 0 42 35.5 V17.5 L36.5 12" fill="none" stroke="{W}" stroke-width="4.2" stroke-linecap="round" stroke-linejoin="round"/></g>'''
    chev_r = f'<path d="M2 2 L10 10 L2 18" fill="none" stroke="{W}" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"/>'
    chev_l = f'<path d="M10 2 L2 10 L10 18" fill="none" stroke="{W}" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"/>'
    chev_d = f'<path d="M2 2 L16 14 L30 2" fill="none" stroke="{W}" stroke-width="4" stroke-linecap="round" stroke-linejoin="round"/>'
    chev_u = f'<path d="M2 14 L16 2 L30 14" fill="none" stroke="{W}" stroke-width="4" stroke-linecap="round" stroke-linejoin="round"/>'
    chev_ds = f'<path d="M2 2 L10 10 L18 2" fill="none" stroke="{W}" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"/>'
    water48 = water.replace('scale(0.833333)', 'scale(1)')  # データ画面用の 48 px(10/21。2.10.0 まではサーキットも)
    fuel48 = fuel.replace('scale(0.833333)', 'scale(1)')
    water32 = water.replace('scale(0.833333)', 'scale(0.666667)')  # サーキット用の 32 px(2.10.0。弧の端の先)
    fuel32 = fuel.replace('scale(0.833333)', 'scale(0.666667)')
    icons = [("WATER", 40, 40, water), ("FUEL", 40, 40, fuel), ("WATER_L", 48, 48, water48), ("FUEL_L", 48, 48, fuel48), ("WATER_M", 32, 32, water32), ("FUEL_M", 32, 32, fuel32), ("CHEV_RIGHT", 12, 20, chev_r), ("CHEV_LEFT", 12, 20, chev_l), ("CHEV_DOWN", 32, 16, chev_d), ("CHEV_UP", 32, 16, chev_u), ("CHEV_DOWN_S", 20, 12, chev_ds)]
    res = []
    with sync_playwright() as p:
        b = p.chromium.launch()
        for name, w, h, body in icons:
            pg = b.new_page(viewport={"width": w, "height": h}, device_scale_factor=1)
            pg.set_content(f'<html><body style="margin:0;background:#000"><svg width="{w}" height="{h}" style="display:block">{body}</svg></body></html>')
            im = Image.open(io.BytesIO(pg.screenshot())).convert("L")
            a = np.array(im, np.uint8)
            q = ((a.astype(np.uint16) * 15 + 127) // 255).astype(np.uint8)
            res.append(dict(name=name, w=w, h=h, data=rle4(q.flatten().tolist())))
            pg.close()
        b.close()
    return res

def carray(data, per=24):
    lines = []
    for i in range(0, len(data), per):
        lines.append("  " + ",".join(str(b) for b in data[i:i + per]) + ",")
    return "\n".join(lines)

def main():
    h = ["// fonts_data.h - 書体とアイコンのデータ(gen_assets.py が作る。手で書き換えない)",
         "// 書体: Saira / Saira Semi Condensed (The Saira Project Authors), IBM Plex Sans JP / IBM Plex Mono (IBM Corp.)。",
         "// どれも SIL Open Font License 1.1。ここにあるのは、決まった大きさで描いた文字の濃さのデータ。",
         "#pragma once", "#include <stdint.h>", "",
         "struct GlyphInfo {", "  uint16_t code;    // 文字コード(Unicode、U+FFFF まで)", "  uint8_t w, h;     // 絵の幅と高さ [px]",
         "  int16_t xoff;     // 書き出し位置から絵の左端まで [px]", "  int16_t yoff;     // ベースラインから絵の上端まで(上が正)[px]",
         "  uint16_t adv16;   // 次の文字までの送り [1/16 px]", "  uint32_t offset;  // rle の中の位置", "};",
         "struct FontData {", "  const GlyphInfo *glyphs;  // code の昇順", "  uint16_t count;", "  uint16_t px;", "  const uint8_t *rle;", "  const char *name;", "};",
         "struct IconData {", "  uint8_t w, h;", "  const uint8_t *rle;", "};",
         f"#define FONT_PROP_DIGIT_BASE 0x{PRIV:04X}  // 幅をそろえていない数字(キャンバスの絵と同じ)の文字コード", ""]
    c = ['// fonts_data.cpp - 書体とアイコンのデータ(gen_assets.py が作る。手で書き換えない)', '#include "fonts_data.h"', ""]
    total = 0; report = []
    for sp in SPECS:
        glyphs = {}
        for part in sp["parts"]:
            for g in render_part(part, sp["px"]):
                glyphs[g["code"]] = g
        gl = [glyphs[k] for k in sorted(glyphs)]
        blob = bytearray(); rows = []
        for g in gl:
            rows.append(f"  {{0x{g['code']:04X},{g['w']},{g['h']},{g['xoff']},{g['yoff']},{g['adv']},{len(blob)}}},")
            blob += g["data"]
        n = sp["name"]
        c.append(f"static const uint8_t RLE_{n}[] = {{\n{carray(blob)}\n}};")
        c.append(f"static const GlyphInfo GLY_{n}[] = {{\n" + "\n".join(rows) + "\n};")
        c.append(f'const FontData FONT_{n} = {{GLY_{n}, {len(gl)}, {sp["px"]}, RLE_{n}, "{n}"}};\n')
        h.append(f"extern const FontData FONT_{n};")
        total += len(blob) + 16 * len(gl)
        report.append((n, len(gl), len(blob)))
    h.append("")
    for ic in render_icons():
        c.append(f"static const uint8_t RLE_ICON_{ic['name']}[] = {{\n{carray(ic['data'])}\n}};")
        c.append(f"const IconData ICON_{ic['name']} = {{{ic['w']}, {ic['h']}, RLE_ICON_{ic['name']}}};\n")
        h.append(f"extern const IconData ICON_{ic['name']};")
        total += len(ic["data"])
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "fonts_data.h").write_text("\n".join(h) + "\n", encoding="utf-8")
    (OUT / "fonts_data.cpp").write_text("\n".join(c) + "\n", encoding="utf-8")
    for r in report: print("%-12s glyphs=%4d rle=%7d B" % r)
    print("total bytes", total, " broad JP chars", len(JP_BROAD))

main()

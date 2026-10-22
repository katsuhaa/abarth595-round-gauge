// e2e_main.cpp - PC 上の通し試験: 模擬データの送信側(c6_display_debug の sim)→ UART → 表示器の受信・画面
//   ・フレームが 1 つも欠けず、CRC 誤りも無いこと
//   ・少しずつ描き直した画面が、同じ状態を最初から描いた画面と 1 画素も違わないこと
//   ・1 回の描き直しで画面へ送る量(40 MHz の QSPI での時間の見積もり)
// 模擬の場面は 1 周 約 121 秒(sim.cpp)。ここでは 128 秒ぶん流す
#include "../meter_display/app.h"
#include "../meter_display/gfx.h"
#include "../meter_display/ui.h"
#include "../c6_display_debug/sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include "pc_util.h"

static uint16_t fb[gfx::W * gfx::H], fbRef[gfx::W * gfx::H];
static float scratch[gfx::W * (gfx::H + 2)];
static Settings set = { SET_SHIFT_DEFAULT, SET_AUTORET_DEFAULT, SET_BRIGHT_DEFAULT, SET_WARN_DEFAULT, SET_DEMO_DEFAULT, SET_RC_DEFAULT };
static LinkStats ls;
static PerfStats ps;
static SysInfo si;
static std::string outdir;

static void saveShot(const std::string &name) {
  saveBmp(outdir + "/" + name + ".bmp", fb, gfx::W, gfx::H);
}

// 画面へ送る時間の見積もり [ms]: 範囲を偶数に広げた画素数 × 2 バイト ÷ 20 MB/s(40 MHz × 4 本)+ 範囲ごとに 0.2 ms
static double flushMs(uint32_t *pxOut) {
  uint32_t px = 0;
  for (int i = 0; i < gfx::dirtyCount(); i++) {
    gfx::Rect r = gfx::dirtyAt(i);
    int x0 = r.x0 & ~1, y0 = r.y0 & ~1, x1 = (r.x1 + 1) & ~1, y1 = (r.y1 + 1) & ~1;
    px += (uint32_t)(x1 - x0) * (uint32_t)(y1 - y0);
  }
  *pxOut = px;
  return px * 2.0 / 20e6 * 1000.0 + gfx::dirtyCount() * 0.2;
}

int main(int argc, char **argv) {
  outdir = argc > 1 ? argv[1] : ".";
  bool doCheck = argc > 2 && !strcmp(argv[2], "check");  // 照合つき(時刻を飛ばすので、時間の統計は照合なしで取る)
  gfx::begin(fb, scratch);
  mlink::begin();
  cantab::begin();
  ui::begin(&set);
  sim::begin();
  memset(&ps, 0, sizeof(ps));
  memset(&si, 0, sizeof(si));

  std::vector<uint8_t> wire;
  size_t wirePos = 0;
  uint8_t seq = 0;
  uint32_t lastSig = 0, lastInfo = 0, txFrames = 0, txBytes = 0;
  uint32_t nextFrame = 0, frames = 0, checks = 0, mismatches = 0, loops = 0;
  double worstFlush = 0, sumFlush = 0;
  uint32_t worstPx = 0;
  uint32_t over50 = 0;
  double sceneWorst[16] = { 0 };
  // 画面を切り替える予定: {時刻 ms, x, y}
  struct TapAt { uint32_t t; int x, y; const char *what; };
  const TapAt taps[] = {
    { 62000, 233, 233, "走行→ブースト計" },  // 場面 7 の途中
    { 68000, 233, 233, "ブースト計→データ" },
    { 75000, 233, 233, "データ→CAN確認" },
    { 79500, 233, 70, "CAN確認: 見出しの「›」で選べる状態に" },
    { 80000, 233, 150, "CAN確認: 1 行目→詳細" },
    { 86000, 233, 378, "詳細→戻る" },
    { 88000, 233, 378, "基準を取る" },
    { 90000, 233, 70, "CAN確認: 見出しで選べない状態に" },
    { 92000, 233, 233, "CAN確認→設定" },
    { 93000, 233, 70, "設定: 見出しの「›」で選べる状態に" },
    { 94000, 233, 380, "設定: 続きへ" },
    { 96000, 233, 220, "設定: 2 行目(受信の状態)" },
    { 100000, 233, 378, "受信の状態→戻る" },
    { 100500, 60, 233, "設定: 項目のない所で選べない状態に" },
    { 101000, 233, 233, "設定→走行" },
    { 108000, 233, 233, "走行→ブースト計(場面 9 のブーストの動きと、場面 10 の送信停止をこのページで見る)" },
  };
  size_t tapIdx = 0;
  uint32_t baudSwitches = 0, linkStateErrors = 0, downChecks = 0, upChecks = 0, sceneStartMs = 0, falseDown = 0, linkOnSince = 0;
  int prevScene = 0;
  uint32_t chk[16] = { 0 };
  const uint32_t shots[] = { 3000, 9000, 15000, 21000, 27000, 33000, 36500, 40000, 44000, 50000, 55000, 58000, 64000, 70000, 79000, 85000, 87500, 91000, 93000, 95000, 99000, 106000, 110000, 112000, 115000, 119000, 124000 };
  size_t shotIdx = 0;
  // 照合で時刻を進める幅: 異常の点滅で (t / 半周期) の偶奇が同じになる時間。輪だけを点滅させる設定(BLINK_WHOLE_SCREEN 0)
  // のときは、回転数の点滅の周期(照合の間は回転数が止まっているので一定)との最小公倍数にする
  auto samePhaseNow = []() {
    uint32_t p = 2 * ALARM_BLINK_HALF_MS;
    if (!BLINK_WHOLE_SCREEN && ui::blinkOnMs()) {
      uint32_t q = ui::blinkOnMs() + ui::blinkOffMs();
      while (p % q != 0) p += 2 * ALARM_BLINK_HALF_MS;
    }
    return p;
  };

  for (uint32_t now = 1; now <= 128000; now++) {
    // ---- 送る側(1 ms ごとの loop)
    sim::step(now, false);
    auto send = [&](uint8_t type, const void *p, uint8_t len) {
      uint8_t buf[ML_MAX_PAYLOAD + ML_OVERHEAD];
      size_t n = mlBuild(buf, type, seq++, p, len);
      wire.insert(wire.end(), buf, buf + n);
      txFrames++;
      txBytes += n;
    };
    if (sim::linkOn()) {
      if (now - lastSig >= 20) {
        lastSig = now;
        MlSignals s;
        sim::signals(&s, now);
        send(ML_TYPE_SIGNALS, &s, sizeof(s));
      }
      MlCanHead head;
      uint8_t data[8];
      for (int i = 0; i < 8 && sim::canPoll(now, &head, data); i++) {
        uint8_t p[sizeof(MlCanHead) + 8];
        memcpy(p, &head, sizeof(head));
        memcpy(p + sizeof(head), data, head.info & 0x0F);
        send(ML_TYPE_CAN, p, (uint8_t)(sizeof(head) + (head.info & 0x0F)));
      }
      if (now - lastInfo >= 1000) {
        lastInfo = now;
        MlInfo info;
        memset(&info, 0, sizeof(info));
        info.proto_ver = METER_LINK_PROTO_VER;
        info.flags = ML_F_SIM;
        uint16_t fps[2], ids[2];
        sim::canRates(fps, ids);
        memcpy(info.can_fps, fps, sizeof(fps));
        memcpy(info.can_ids, ids, sizeof(ids));
        strcpy(info.name, "c6dbg r1");
        send(ML_TYPE_INFO, &info, sizeof(info));
      }
    }
    // ---- 線: 921600 bps なら 1 ms に 92 バイトまで
    size_t n = wire.size() - wirePos;
    if (n > 92) n = 92;
    // 受信タスクは loop() とは別に時刻を読む。loop() が now を読んだあとに届いたフレームを模すため、
    // 受信側の時刻を 2 ms 進めて渡す(届いた時刻が、描画側の now より新しくなる)
    uint32_t rxNow = now + 2;
    if (n) {
      mlink::feed(wire.data() + wirePos, n, rxNow);
      wirePos += n;
    }
    mlink::tick(rxNow);
    cantab::tick(rxNow);
    if (mlink::autoBaud(rxNow, n)) baudSwitches++;
    if (!sim::linkOn()) linkOnSince = now;
    // ---- 送信を止める場面: 1.5 秒たった時点で「信号がない」になっていること。次の場面で戻ること
    if (sim::scene() != prevScene) {
      prevScene = sim::scene();
      sceneStartMs = now;
    }
    if (now - sceneStartMs == 1500) {
      Signals s;
      LinkStats l2;
      mlink::getSignals(&s, now);
      mlink::getStats(&l2);
      if (!sim::linkOn()) {
        downChecks++;
        if (s.linkUp || s.rpmOk || l2.locked) linkStateErrors++;
      } else {
        upChecks++;
        if (!s.linkUp || !s.rpmOk || !l2.locked) linkStateErrors++;
      }
    }
    // ---- 表示器の 1 周
    if (tapIdx < sizeof(taps) / sizeof(taps[0]) && now >= taps[tapIdx].t) {
      ui::tap(taps[tapIdx].x, taps[tapIdx].y, now);
      tapIdx++;
    }
    if (now >= nextFrame) {
      Signals s;
      mlink::getSignals(&s, now);
      mlink::getStats(&ls);
      if (now - linkOnSince > 1500 && !s.linkUp) {  // 送信が続いているのに「信号なし」になった回数
        if (falseDown < 5) printf("FALSE DOWN t=%u scene=%d linkOnSince=%u\n", now, sim::scene(), linkOnSince);
        falseDown++;
      }
      trip::update(s, now);
      ui::update(s, ls, ps, si, now);
      uint32_t px;
      double ms = flushMs(&px);
      if (gfx::dirtyCount()) {
        frames++;
        sumFlush += ms;
        if (ms > worstFlush) { worstFlush = ms; worstPx = px; }
        if (ms > sceneWorst[s.scene & 15]) sceneWorst[s.scene & 15] = ms;
        if (ms > 50) over50++;
      }
      gfx::dirtyClear();
      nextFrame = now + 1 + (uint32_t)(ms + 0.999) + 3;  // 送る時間 + 描画ぶんの見込み 3 ms
      // ---- 少しずつ描いた画面と、最初から描いた画面を比べる(背景の塗り直しが終わるよう、値を止めて 200 ms 進める)
      // (点滅の切り替えの遅れは ui::blinkStats が数えている。PC 上では描画の合間の platBlinkTick が無いので、遅れ = 描画の間隔)
      if (doCheck && ++loops % 61 == 0) {
        ui::update(s, ls, ps, si, now + 100);
        ui::update(s, ls, ps, si, now + 200);
        // 走行以外の画面は 2 回目以降に値の行を描くので、もう一度
        ui::update(s, ls, ps, si, now + 300);
        ui::Screen sc = ui::screen();
        // ピークホールドの印(回転計・サーキットの三角形)は時間で下がるので、少しずつ描いた側も最初から描く側も同じ時刻で終える。
        // サーキットは点滅の向きも揃える(+300 と +300+samePhase は、どちらの点滅も (t/半周期) の偶奇が同じ)
        uint32_t samePhase = (sc == ui::SC_BOOST) ? 0 : samePhaseNow();
        const uint32_t tEnd = now + 300 + samePhase;
        if (samePhase) ui::update(s, ls, ps, si, tEnd);
        memcpy(fbRef, fb, sizeof(fb));
        if (sc == ui::SC_DRIVE || sc == ui::SC_BOOST || sc == ui::SC_DATA || sc == ui::SC_SET_INFO) {
          ui::testShow(sc);
          ui::update(s, ls, ps, si, tEnd);
          ui::update(s, ls, ps, si, tEnd);
          checks++;
          chk[(int)sc]++;
          int diff = 0;
          // 点滅の向きが同じになる時刻どうしで比べる(+300 と +300+samePhase は、どちらの点滅も (t/半周期) の偶奇が同じ)
          for (int i = 0; i < gfx::W * gfx::H; i++) diff += fb[i] != fbRef[i];
          if (diff) {
            mismatches++;
            printf("MISMATCH t=%u screen=%d scene=%d: %d px\n", now, (int)sc, s.scene, diff);
            if (mismatches <= 3) {
              saveShot("mismatch_full_" + std::to_string(now));
              memcpy(fb, fbRef, sizeof(fb));
              saveShot("mismatch_incr_" + std::to_string(now));
            }
          }
        }
        gfx::dirtyClear();
        now += 300 + samePhase;  // 照合で進めた時刻のまま続ける(時刻を巻き戻さない)
        nextFrame = now;
      }
    }
    if (shotIdx < sizeof(shots) / sizeof(shots[0]) && now >= shots[shotIdx]) {
      char name[32];
      snprintf(name, sizeof(name), "e2e_%06u", shots[shotIdx]);
      saveShot(name);
      shotIdx++;
    }
  }
  mlink::getStats(&ls);
  printf("送信 %u フレーム %u バイト / 受信 OK %u  CRC誤り %u  欠落 %u  捨てたバイト %u  ID数 %d\n", txFrames, txBytes, ls.framesOk, ls.crcErrors, ls.lostFrames, ls.junkBytes, cantab::count());
  printf("平均の回線使用率 %.1f%%(921600 bps)\n", txBytes * 10.0 / 921600.0 / 128.0 * 100.0);
  if (doCheck) {
    // 照合つきのときは時刻を飛ばしているので、時間の統計は出さない
    printf("全体を描き直したものとの照合 %u 回(走行 %u ブースト %u データ %u 受信の状態 %u)、不一致 %u 回  無かった文字 %u\n", checks, chk[0], chk[1], chk[2], chk[7], mismatches,
           (unsigned)gfx::missingGlyphs());
  } else {
    printf("描き直し %u 回  画面へ送る時間の見積もり(40 MHz × 4 本で送る時間だけ。画素を並べ替える時間は含まない): 平均 %.1f ms  最長 %.1f ms(%u 画素)  50 ms 超え %u 回\n", frames,
           sumFlush / frames, worstFlush, worstPx, over50);
    for (int i = 1; i <= sim::sceneCount() && i < 16; i++) printf("  場面 %d の最長 %.1f ms\n", i, sceneWorst[i]);
    ui::BlinkStats bs;
    ui::blinkStats(&bs, false);
    if (bs.toggles) printf("点滅の切り替え %u 回: 予定の時刻からの遅れ 最長 %u ms、消灯の時間の 2 割を超えた遅れ %u 回\n", bs.toggles, bs.lateMaxMs, bs.lateOver);
    printf("無かった文字 %u\n", (unsigned)gfx::missingGlyphs());
  }
  printf("受信の状態の確認: 送信中 %u 回、停止中 %u 回、合わなかった回数 %u  速度の切り替え %u 回(0 が正常)  送信中なのに「信号なし」になった描画 %u 回(0 が正常)\n", upChecks,
         downChecks, linkStateErrors, baudSwitches, falseDown);
  bool ok = ls.framesOk + 3 >= txFrames && ls.crcErrors == 0 && ls.lostFrames == 0 && mismatches == 0 && gfx::missingGlyphs() == 0;
  ok = ok && baudSwitches == 0 && linkStateErrors == 0 && falseDown == 0 && (doCheck || (downChecks >= 1 && upChecks >= 8));
  printf("%s\n", ok ? "E2E PASS" : "E2E FAIL");
  return ok ? 0 : 1;
}

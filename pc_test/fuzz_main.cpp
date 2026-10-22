// fuzz_main.cpp - PC 上の乱数試験: でたらめな受信データとでたらめなタップを長時間流す
//   ・メモリーの範囲外アクセスや未定義の動作が無いこと(-fsanitize=address,undefined で動かす)
//   ・「書き換えた範囲」が画面の中に収まっていること
//   ・少しずつ描き直した画面が、同じ状態を最初から描いた画面と 1 画素も違わないこと(全部の画面で)
//   ・書体データに無い文字を描こうとしないこと
#include "../meter_display/app.h"
#include "../meter_display/gfx.h"
#include "../meter_display/ui.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include "pc_util.h"

extern bool g_hostSaveOk;
static uint16_t fb[gfx::W * gfx::H], fbRef[gfx::W * gfx::H];
static float scratch[gfx::W * (gfx::H + 2)];
static Settings set = { SET_SHIFT_DEFAULT, SET_AUTORET_DEFAULT, SET_BRIGHT_DEFAULT, SET_WARN_DEFAULT, SET_DEMO_DEFAULT, SET_RC_DEFAULT };
static LinkStats ls;
static PerfStats ps;
static SysInfo si;

static uint64_t rngState = 1;
static uint32_t rnd() {
  rngState ^= rngState << 13;
  rngState ^= rngState >> 7;
  rngState ^= rngState << 17;
  return (uint32_t)(rngState >> 16);
}
static uint32_t rndN(uint32_t n) {
  return rnd() % n;
}
static bool chance(uint32_t perMille) {
  return rndN(1000) < perMille;
}

static uint8_t seq = 0;
static void sendFrame(uint8_t type, const void *p, uint8_t len, uint32_t now, int damage) {
  uint8_t buf[ML_MAX_PAYLOAD + ML_OVERHEAD];
  size_t n = mlBuild(buf, type, seq++, p, len);
  if (damage == 1) buf[2 + rndN((uint32_t)n - 2)] ^= (uint8_t)(1u << rndN(8));  // どこか 1 ビット壊す
  if (damage == 2) n = rndN((uint32_t)n);                                         // 途中で切る
  // でたらめな大きさに分けて渡す
  size_t pos = 0;
  while (pos < n) {
    size_t k = 1 + rndN(40);
    if (k > n - pos) k = n - pos;
    mlink::feed(buf + pos, k, now);
    pos += k;
  }
}

// 車として起こりうる範囲の値(ときどき範囲の端や、ありえない値も混ぜる)
static void randomSignals(MlSignals *m, int mode) {
  memset(m, 0, sizeof(*m));
  if (mode == 2) {  // 全部でたらめ
    for (size_t i = 0; i < sizeof(*m); i++) ((uint8_t *)m)[i] = (uint8_t)rnd();
    return;
  }
  m->valid = chance(850) ? 0x7F : (uint8_t)rnd();
  m->flags = chance(500) ? ML_F_SIM : 0;
  m->rpm = (uint16_t)(chance(900) ? rndN(7600) : rnd());
  m->speed_c = (uint16_t)rndN(22000);
  m->throttle_h = (uint8_t)(chance(900) ? rndN(201) : rnd());
  m->fuel_h = (uint8_t)(chance(900) ? rndN(201) : rnd());
  m->water_d = (int16_t)(chance(900) ? (int)rndN(1700) - 300 : (int)(int16_t)rnd());
  static const char gears[] = { '1', '2', '3', '4', '5', 'N', 'R', 0, '6', 'P', (char)0x80, (char)0xE3 };
  m->gear = chance(900) ? gears[rndN(7)] : gears[rndN(sizeof(gears))];
  m->scene = (uint8_t)(chance(800) ? rndN(10) : rnd());
  m->boost_c = (int16_t)(chance(900) ? (int)rndN(300) - 80 : (int)(int16_t)rnd());
}

int main(int argc, char **argv) {
  uint32_t runs = argc > 1 ? (uint32_t)atoi(argv[1]) : 4;
  uint32_t steps = argc > 2 ? (uint32_t)atoi(argv[2]) : 200000;
  std::string outdir = argc > 3 ? argv[3] : ".";
  gfx::begin(fb, scratch);
  mlink::begin();
  cantab::begin();
  ui::begin(&set);
  memset(&ps, 0, sizeof(ps));
  memset(&si, 0, sizeof(si));

  uint32_t now = 1000;
  uint64_t updates = 0, taps = 0, compares = 0, mismatches = 0, badDirty = 0, cmpByScreen[16] = { 0 }, seenScreen[16] = { 0 };
  for (uint32_t run = 0; run < runs; run++) {
    rngState = 0x9E3779B97F4A7C15ull * (run + 1) + 12345;
    int sigMode = 0;
    uint32_t quietUntil = 0;
    bool held = false;        // 押したまま(長押しの試験)
    uint32_t releaseAt = 0;
    for (uint32_t i = 0; i < steps; i++) {
      now += 1 + rndN(chance(20) ? 400 : 30);
      if (chance(2)) now += 30000 + rndN(400000);  // たまに長い間を空ける(自動で戻る、受信の途切れ)
      if (chance(1)) now += 0x40000000u;            // ごくまれに時刻を大きく進める(millis の一巡のあたりも通す)
      if (chance(3)) sigMode = (int)rndN(3);
      if (chance(2)) quietUntil = now + rndN(5000);
      // ---- 受信
      if ((int32_t)(now - quietUntil) >= 0) {
        int nf = (int)rndN(6);
        for (int k = 0; k < nf; k++) {
          int damage = chance(30) ? 1 + (int)rndN(2) : 0;
          uint32_t kind = rndN(100);
          if (kind < 45) {
            MlSignals m;
            randomSignals(&m, sigMode);
            sendFrame(ML_TYPE_SIGNALS, &m, sizeof(m), now, damage);
          } else if (kind < 90) {
            uint8_t p[sizeof(MlCanHead) + 8];
            MlCanHead h;
            uint8_t dlc = (uint8_t)rndN(9);
            h.info = (uint8_t)((chance(300) ? 0x80 : 0) | (chance(900) ? 0x40 : 0) | dlc);
            h.id = chance(700) ? (0x0618A000u + rndN(24)) : (chance(500) ? rndN(250) * 0x01000001u : rnd());
            memcpy(p, &h, sizeof(h));
            for (int b = 0; b < 8; b++) p[sizeof(h) + b] = (uint8_t)(chance(700) ? 0x10 * b : rnd());
            sendFrame(ML_TYPE_CAN, p, (uint8_t)(sizeof(h) + dlc), now, damage);
          } else if (kind < 96) {
            MlInfo info;
            for (size_t b = 0; b < sizeof(info); b++) ((uint8_t *)&info)[b] = (uint8_t)rnd();
            if (chance(800)) info.proto_ver = METER_LINK_PROTO_VER;
            if (chance(700)) strncpy(info.name, "c6dbg r1", sizeof(info.name));
            sendFrame(ML_TYPE_INFO, &info, sizeof(info), now, damage);
          } else {
            uint8_t junk[96];
            size_t n = 1 + rndN(sizeof(junk));
            for (size_t b = 0; b < n; b++) junk[b] = (uint8_t)(chance(100) ? (chance(500) ? 0xA5 : 0x5A) : rnd());
            mlink::feed(junk, n, now);
          }
        }
      }
      mlink::tick(now);
      cantab::tick(now);
      mlink::autoBaud(now, rndN(200));
      // ---- タッチ(10/23: 触れる・離す。サーキット・タコ・データの長押しで画面が変わる。ほかは離したときにタップ)
      ui::Screen cur = ui::screen();
      if (cur == ui::SC_SET_LIST && chance(10)) {  // 中の画面はタップの並びが長くて滅多に届かないので、たまに直接出す
        static const ui::Screen deep[] = { ui::SC_SET_RC, ui::SC_SET_INFO, ui::SC_CAN, ui::SC_CAN_ID, ui::SC_SET_RESET, ui::SC_CAN_REDO };
        cur = deep[rndN(sizeof(deep) / sizeof(deep[0]))];
        ui::testShow(cur);
      }
      if (held && (int32_t)(now - releaseAt) >= 0) {
        ui::touchUp(now);
        held = false;
      } else if (!held && chance((cur == ui::SC_DRIVE || cur == ui::SC_BOOST || cur == ui::SC_DATA) ? 6 : (cur == ui::SC_CAN_DETAIL ? 150 : 60))) {
        g_hostSaveOk = !chance(100);
        // 押せる所に当たりやすいよう、半分は 4 つの範囲(上・下・左・右)の真ん中から選ぶ
        static const int spots[][2] = { { 233, 60 }, { 233, 400 }, { 100, 233 }, { 366, 233 }, { 233, 119 }, { 233, 346 }, { 232, 233 }, { 233, 233 } };
        int x, y;
        if (chance(600)) {
          int k = (int)rndN(sizeof(spots) / sizeof(spots[0]));
          x = spots[k][0];
          y = spots[k][1];
        } else {
          x = (int)rndN(gfx::W);
          y = (int)rndN(gfx::H);
        }
        if (chance(300)) {
          ui::tap(x, y, now);  // すぐのタップ(試験の入口)
        } else {
          ui::touchDown(x, y, now);
          held = true;
          releaseAt = now + (chance(500) ? rndN(HOLD_MS) : rndN(2000));
        }
        taps++;
      }
      // ---- 画面
      Signals s;
      mlink::getSignals(&s, now);
      mlink::getStats(&ls);
      trip::update(s, now);
      ps.fps = rndN(100);
      ps.loopMaxMs = rndN(60);
      si.axpOk = chance(900);
      si.powerOnSrc = (uint8_t)rnd();
      si.rc.enabled = chance(900);
      si.rc.bleOk = chance(900);
      si.rc.connected = chance(500);
      si.rc.canPerSec = rndN(200);
      si.rc.gpsPerSec = rndN(30);
      si.rc.dropped = chance(900) ? 0 : rnd();
      si.rc.gpsOk = chance(800);
      si.rc.fixType = (uint8_t)rndN(6);
      si.rc.numSV = (uint8_t)rndN(40);
      si.rc.pvtPerSec = rndN(30);
      si.rc.lat = (int32_t)rnd();
      si.rc.lon = (int32_t)rnd();
      si.rc.gpsKmh = (float)rndN(3000) / 10.0f;
      si.rc.imuOk = chance(900);
      si.rc.imuLevel = chance(500);
      si.rc.imuForward = chance(500);
      si.rc.gLon = (float)((int)rndN(400) - 200) / 100.0f;
      si.rc.gLat = (float)((int)rndN(400) - 200) / 100.0f;
      if (chance(300)) {  // GPS の解析にもでたらめなバイト列を流す(範囲外アクセスが無いこと)
        uint8_t gb[64];
        size_t gn = 1 + rndN(sizeof(gb));
        for (size_t b = 0; b < gn; b++) gb[b] = (uint8_t)(chance(100) ? (chance(500) ? 0xB5 : 0x62) : rnd());
        gnss::feed(gb, gn, now);
      }
      ui::update(s, ls, ps, si, now);
      updates++;
      seenScreen[(int)ui::screen() & 15]++;
      for (int d = 0; d < gfx::dirtyCount(); d++) {
        gfx::Rect r = gfx::dirtyAt(d);
        if (r.x0 < 0 || r.y0 < 0 || r.x1 > gfx::W || r.y1 > gfx::H || r.x1 <= r.x0 || r.y1 <= r.y0) badDirty++;
      }
      gfx::dirtyClear();
      // ---- 照合: 入力を止めて描き直しが落ち着くまで進め、同じ時刻で最初から描いたものと比べる
      if (chance(15)) {
        if (held) {  // 照合の前に離す(長押しの表示の間は描き直さないので)
          ui::touchUp(now);
          held = false;
        }
        for (int k = 0; k < 8; k++) {
          now += 100;
          mlink::getSignals(&s, now);
          ui::update(s, ls, ps, si, now);
        }
        // 受信が途切れる境目(1 秒)と自動で戻る境目をまたがないよう、値はここで止める
        ui::Screen sc = ui::screen();
        ui::update(s, ls, ps, si, now);
        if (ui::screen() == sc) {
          memcpy(fbRef, fb, sizeof(fb));
          ui::testRedraw();
          ui::update(s, ls, ps, si, now);
          compares++;
          cmpByScreen[(int)sc & 15]++;
          int diff = 0;
          for (int p = 0; p < gfx::W * gfx::H; p++) diff += fb[p] != fbRef[p];
          if (diff) {
            mismatches++;
            if (mismatches <= 5) {
              printf("MISMATCH run=%u step=%u now=%u screen=%d: %d px\n", run, i, now, (int)sc, diff);
              saveBmp(outdir + "/fuzz_incr_" + std::to_string(mismatches) + ".bmp", fbRef, gfx::W, gfx::H);
              saveBmp(outdir + "/fuzz_full_" + std::to_string(mismatches) + ".bmp", fb, gfx::W, gfx::H);
            }
          }
        }
        gfx::dirtyClear();
      }
    }
  }
  mlink::getStats(&ls);
  printf("乱数試験: %u 回 × %u 歩  画面の更新 %llu 回  タップ %llu 回\n", runs, steps, (unsigned long long)updates, (unsigned long long)taps);
  static const char *const NM[] = { "サーキット", "タコ", "データ", "CAN", "CAN詳細", "設定", "変更", "受信の状態", "RaceChrono", "オープニング", "CAN一覧", "CANやり直す", "時間リセット" };
  printf("  画面ごとの更新回数 / 照合回数:");
  for (int k = 0; k < 13; k++) printf(" %s %llu/%llu", NM[k], (unsigned long long)seenScreen[k], (unsigned long long)cmpByScreen[k]);
  printf("\n  最初から描いたものとの照合 %llu 回、不一致 %llu 回\n", (unsigned long long)compares, (unsigned long long)mismatches);
  printf("  画面の外にはみ出した「書き換えた範囲」 %llu 個  無かった文字 %u  CAN の ID 数 %d(上限 %d)\n", (unsigned long long)badDirty, (unsigned)gfx::missingGlyphs(), cantab::count(), CAN_MAX_IDS);
  printf("  受信: OK %u  CRC誤り %u  欠落 %u  捨てたバイト %u\n", ls.framesOk, ls.crcErrors, ls.lostFrames, ls.junkBytes);
  bool ok = mismatches == 0 && badDirty == 0 && gfx::missingGlyphs() == 0;
  printf("%s\n", ok ? "FUZZ PASS" : "FUZZ FAIL");
  return ok ? 0 : 1;
}

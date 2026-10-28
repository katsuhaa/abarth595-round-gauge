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
static Settings set;  // main で settingsDefaults(2.10.0)
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
  settingsDefaults(&set);
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
    ui::testShow(ui::SC_OPENING, (int)(run % 3));  // 2.5.0: オープニング 3 種類(回ごとに替える。途中でもときどき出す)
    int sigMode = 0;
    uint32_t quietUntil = 0;
    bool held = false;        // 押したまま(長押しの試験)
    uint32_t releaseAt = 0;
    int heldX = 0, heldY = 0, heldDx = 0;  // 押している所と、1 回に動かす横の量(0 = 動かさない。2.1.0: スワイプと長押しの取り消し)
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
            if (chance(60)) h.id = chance(500) ? VEH_ID_LIGHT : VEH_ID_FUEL;  // 2.11.0: ライトと、燃料・スポーツモード(b4 の 0x10)の ID も混ぜる
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
      // ---- タッチ(触れる・動かす・離す。2.1.0: サーキット・タコ・データは横のスワイプで画面が変わり、長押しで設定。
      //      2.3.0: TOUCH_CONFIRM_MS 押し続けたらタッチが決まる。サーキット・タコ・データは案内を出す。
      //      2.4.0: どの画面も離したときに決める(横のスワイプ = 次・前、動きが小さければタップ))
      ui::Screen cur = ui::screen();
      if ((cur == ui::SC_DRIVE || cur == ui::SC_BOOST || cur == ui::SC_DATA) && rndN(20000) == 0) {
        ui::testShow(ui::SC_OPENING, (int)rndN(3));  // 2.5.0: オープニング(タコは描き直さずにタコの画面になるので、そのあとの照合で確かめる)
        cur = ui::SC_OPENING;
      }
      if (cur == ui::SC_SET_LIST && chance(10)) {  // 中の画面はタップの並びが長くて滅多に届かないので、たまに直接出す
        static const ui::Screen deep[] = { ui::SC_SET_GRP, ui::SC_SET_INFO, ui::SC_CAN, ui::SC_CAN_ID, ui::SC_SET_RESET, ui::SC_CAN_REDO, ui::SC_SET_EDIT, ui::SC_CAN_DETAIL,
                                           ui::SC_CAN_PP, ui::SC_SET_CAT };  // 2.6.0: 前後で比べる。2.12.0: カテゴリの中
        cur = deep[rndN(sizeof(deep) / sizeof(deep[0]))];
        // 2.4.0: 設定 8 項目、受信の状態 4 グループ、グループの中は 2〜6 ページ(多すぎるページの番号も試す)。
        // 2.9.0: 設定 9 項目(イルミネーションの明るさ)、CAN は 9 ページ(ライト)
        // 2.5.0: CAN確認の詳細も直接出す(2.4.4 から一覧の長押しで開くので、乱数のタッチでは滅多に届かない。表の番号は範囲外も試す)
        if (cur == ui::SC_CAN_DETAIL) {
          ui::testShow(cur, (int)rndN(CAN_MAX_IDS + 2) - 1, (int)rndN(9));
        } else if (cur == ui::SC_CAN_ID || cur == ui::SC_CAN_REDO) {  // 2.6.0: 変化を記録・前後で比べるの両方から
          const int pp = (int)rndN(2);
          ui::testShow(cur, cur == ui::SC_CAN_REDO ? pp : 0, cur == ui::SC_CAN_ID ? pp : 0);
        } else {
          // 2.12.0: 設定の項目 17(ITEMS。カテゴリの中・変更は範囲外の番号も試す。累積時間 リセットは変更の画面)。リセットの確認は向きの補正・
          // デフォルトに戻す
          ui::testShow(cur, (cur == ui::SC_SET_EDIT || cur == ui::SC_SET_CAT) ? (int)rndN(19) : (cur == ui::SC_SET_INFO ? (int)rndN(5) : (cur == ui::SC_SET_GRP ? (int)rndN(10) : (cur == ui::SC_SET_RESET ? (int)rndN(3) : 0))),
                       cur == ui::SC_SET_EDIT ? (int)rndN(2) : (cur == ui::SC_SET_GRP ? (int)rndN(6) : 0));
        }
      }
      if (held && heldDx && chance(400)) {  // 押している間の動き(横に進めて、縦は少し揺らす)
        heldX += heldDx + (int)rndN(11) - 5;
        heldY += (int)rndN(11) - 5;
        heldX = heldX < 0 ? 0 : (heldX >= gfx::W ? gfx::W - 1 : heldX);
        heldY = heldY < 0 ? 0 : (heldY >= gfx::H ? gfx::H - 1 : heldY);
        ui::touchMove(heldX, heldY, now);
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
        if (chance(150)) {
          ui::tap(x, y, now);  // すぐのタップ(試験の入口)
        } else if (chance(150)) {
          ui::swipe(chance(500) ? 1 : -1, now);  // すぐのスワイプ(試験の入口。2.4.0)
        } else {
          ui::touchDown(x, y, now);
          held = true;
          heldX = x;
          heldY = y;
          heldDx = chance(500) ? 0 : (int)rndN(41) - 20;  // 半分は横に動かす(スワイプになるもの、ならないもの)
          releaseAt = now + (chance(300) ? rndN(TOUCH_CONFIRM_MS + 50) : (chance(500) ? rndN(HOLD_MS) : rndN(2000)));
        }
        taps++;
      }
      // ---- 画面
      Signals s;
      mlink::getSignals(&s, now);
      mlink::getStats(&ls);
      // 2.12.0: 累積時間(ときどき値を決める。メンテナンスの時間の前後・達したあとも)
      if (chance(3)) {
        const uint32_t near = (uint32_t)set.maintHours * 3600000u + rndN(4000) * 1000u;  // メンテナンスの時間の −2000 秒〜+2000 秒
        trip::setTimes(chance(500) ? rndN(400) * 1000u : (near >= 2000000u ? near - 2000000u : near), rndN(400000) * 1000u);
      }
      trip::update(s, now, set.loadRpm);
      // 2.10.0: 高負荷時間(ときどき値・設定を決める)と時計(GPS の時刻。分からない間・飛ぶことも)
      if (chance(2)) load::testSet(rndN(40000) * 1000, chance(500));
      if (chance(2)) load::testReset();
      if (chance(3)) {
        set.loadYellowMin = chance(200) ? 0 : (int)rndN(19) * SET_LOAD_STEP;
        set.loadRedMin = chance(200) ? 0 : (int)rndN(19) * SET_LOAD_STEP;
        set.loadResetMin = SET_LOADRS_MIN + (int)rndN(SET_LOADRS_MAX);
      }
      if (chance(3)) set.ringOne = chance(500);  // 2.12.0: 回転数の輪(両側・片側)。走行画面のまま変わっても全部を描き直す
      // 2.12.0: 過負荷 回転数(サーキットの ▼ の数字。走行画面のまま変わっても全部を描き直す)とメンテナンスまでの累積時間(0 = 切)
      if (chance(3)) set.loadRpm = SET_LOADRPM_MIN + SET_LOADRPM_STEP * (int)rndN((SET_LOADRPM_MAX - SET_LOADRPM_MIN) / SET_LOADRPM_STEP + 1);
      if (chance(3)) set.maintHours = chance(300) ? 0 : (chance(500) ? 1 + (int)rndN(3) : (int)rndN(SET_MAINT_MAX + 1));
      load::update(s, set, now);
      if (chance(5)) si.timeOk = chance(800);
      if (chance(5)) si.timeDemo = chance(300);  // 2.11.0: デモの時刻(高負荷時間は「時刻 待ち」のまま)
      si.utc = 1793065338u + now / 1000 + (chance(1) ? rnd() : 0);
      ps.fps = rndN(100);
      ps.loopMaxMs = rndN(60);
      ps.pageMaxUs = chance(100) ? 0 : rndN(120000);
      ps.pageFrames = chance(100) ? rndN(FPS_MIN_FRAMES) : rndN(3000);  // 本体の状態の「FPS」
      ps.histN = (uint16_t)rndN(PERF_FPS_SECS + 1);
      for (int k = 0; k < PERF_FPS_SECS; k++) ps.fpsHist[k] = (uint8_t)rndN(256);
      ps.pageAvgUs = chance(50) ? 0 : 1 + rndN(60000);
      ps.pageSlowUs = chance(50) ? 0 : ps.pageAvgUs + rndN(80000);
      si.axpOk = chance(900);
      si.powerOnSrc = (uint8_t)rnd();
      si.rc.enabled = chance(900);
      si.rc.bleOk = chance(900);
      si.rc.connected = chance(500);
      si.rc.canPerSec = rndN(200);
      si.rc.gpsPerSec = rndN(30);
      si.rc.dropped = chance(900) ? 0 : rnd();
      si.rc.gpsOk = chance(800);
      si.rc.gpsOn = chance(900);  // 2.10.0: GPS は送信の入・切に関係なく動かす
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
        if (held) {  // 照合の前に離す(案内・長押しの表示の間は描き直さないので)
          ui::touchUp(now);
          held = false;
        }
        for (int k = 0; k < 8; k++) {
          now += 100;
          mlink::getSignals(&s, now);
          ui::update(s, ls, ps, si, now);
        }
        // 2.11.0: 値を 0.5 秒ごとに描き直す画面(受信の状態・グループの中)は、描き直した時刻で比べる。間の時刻で比べると、
        // 時間で変わる判定(ライトの「受信なし」= VEH_LIGHT_TIMEOUT_MS など)が、最初から描いたものと違ってよいため
        now += 500;
        mlink::getSignals(&s, now);
        ui::update(s, ls, ps, si, now);
        // 受信が途切れる境目(1 秒)と自動で戻る境目をまたがないよう、値はここで止める
        // オープニングは照合しない(最初から描くと始めからやり直す作りなので、同じ時刻の絵にならない。2.4.3)
        ui::Screen sc = ui::screen();
        ui::update(s, ls, ps, si, now);
        if (ui::screen() == sc && sc != ui::SC_OPENING) {
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
  static const char *const NM[] = { "サーキット", "タコ", "データ", "CAN", "CAN詳細", "設定", "変更", "受信の状態", "グループの中", "オープニング", "CAN一覧", "CANやり直す", "時間リセット",
                                    "前後で比べる", "カテゴリの中" };
  printf("  画面ごとの更新回数 / 照合回数:");
  for (int k = 0; k < (int)(sizeof(NM) / sizeof(NM[0])); k++) printf(" %s %llu/%llu", NM[k], (unsigned long long)seenScreen[k], (unsigned long long)cmpByScreen[k]);
  printf("\n  最初から描いたものとの照合 %llu 回、不一致 %llu 回\n", (unsigned long long)compares, (unsigned long long)mismatches);
  printf("  画面の外にはみ出した「書き換えた範囲」 %llu 個  無かった文字 %u  CAN の ID 数 %d(上限 %d)\n", (unsigned long long)badDirty, (unsigned)gfx::missingGlyphs(), cantab::count(), CAN_MAX_IDS);
  printf("  受信: OK %u  CRC誤り %u  欠落 %u  捨てたバイト %u\n", ls.framesOk, ls.crcErrors, ls.lostFrames, ls.junkBytes);
  bool ok = mismatches == 0 && badDirty == 0 && gfx::missingGlyphs() == 0;
  printf("%s\n", ok ? "FUZZ PASS" : "FUZZ FAIL");
  return ok ? 0 : 1;
}

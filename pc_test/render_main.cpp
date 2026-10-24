// render_main.cpp - PC 上で表示器の各画面を描いて BMP に書き出す(キャンバスの絵と見比べる用)
// 使い方: render <出力フォルダ> [prop]   prop を付けると、数字を幅のそろっていない形(キャンバスと同じ)で描く
#include "../meter_display/app.h"
#include "../meter_display/gfx.h"
#include "../meter_display/ui.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include "pc_util.h"

static uint16_t fb[gfx::W * gfx::H];
static float scratch[gfx::W * (gfx::H + 2)];
static Settings set = { SET_SHIFT_DEFAULT, SET_AUTORET_DEFAULT, SET_BRIGHT_DEFAULT, SET_WARN_DEFAULT, SET_DEMO_DEFAULT, SET_RC_DEFAULT };
static LinkStats ls;
static PerfStats ps;
static SysInfo si;
static std::string outdir;

static void saveShot(const char *name) {
  saveBmp(outdir + "/" + name + ".bmp", fb, gfx::W, gfx::H);
}

static Signals sig(int rpm, char gear, float water, float fuel, bool wOk = true, bool fOk = true, float boost = 0.0f, bool bOk = true) {
  Signals s;
  memset(&s, 0, sizeof(s));
  s.linkUp = true;
  s.rpmOk = true; s.rpm = rpm;
  s.gearOk = gear != 0; s.gear = gear;
  s.waterOk = wOk; s.waterC = water;
  s.fuelOk = fOk; s.fuelPct = fuel;
  s.boostOk = bOk; s.boostBar = boost;
  s.speedOk = true; s.speedKmh = 0;
  s.throttleOk = true; s.throttlePct = 0;
  return s;
}

static uint32_t now = 100000;  // (now / 50) が偶数 = 点滅の点灯側
static bool g_keepPeak = false;  // 次の絵だけ、前の絵のピークホールドを残す(ふだんは絵ごとに消す)
static void shot(const char *name, const Signals &s, ui::Screen sc, int a = 0, int b = 0) {
  if (!g_keepPeak) ui::testPeakReset();
  g_keepPeak = false;
  ui::testShow(sc, a, b);
  ui::testTouch(now);  // 時刻が進んでいても自動で走行画面に戻らないように(10/22: Data の絵が走行画面になっていた)
  ui::update(s, ls, ps, si, now);
  ui::update(s, ls, ps, si, now + 100);  // 走行以外の画面の 2 回目(値の行など)。点滅は同じ側
  saveShot(name);
  printf("%-22s dirty=%d px=%u missing=%u\n", name, gfx::dirtyCount(), (unsigned)gfx::dirtyPixels(), (unsigned)gfx::missingGlyphs());
  gfx::dirtyClear();
}

static void canFrame(uint32_t id, const uint8_t d[8], uint8_t dlc, uint32_t t, uint8_t bus = 0) {
  cantab::onFrame(bus, 1, id, dlc, d, t);
}

int main(int argc, char **argv) {
  outdir = argc > 1 ? argv[1] : ".";
  bool tabular = !(argc > 2 && !strcmp(argv[2], "prop"));
  gfx::begin(fb, scratch);
  mlink::begin();
  cantab::begin();
  ui::begin(&set);
  gfx::setTabularDigits(tabular);
  memset(&ls, 0, sizeof(ls));
  memset(&ps, 0, sizeof(ps));
  memset(&si, 0, sizeof(si));

  // ---- 走行画面の 5 状態(仕様書 5.9)
  shot("Drive", sig(4200, '3', 92, 62), ui::SC_DRIVE);
  shot("DriveIdle", sig(850, 'N', 88, 12), ui::SC_DRIVE);
  shot("DriveHot", sig(5600, '4', 112, 45), ui::SC_DRIVE);
  shot("DriveRed", sig(6300, '3', 97, 41), ui::SC_DRIVE);
  shot("DriveNoData", sig(3400, '2', 0, 0, false, false), ui::SC_DRIVE);

  // ---- アナログ計(仕様書に無いページ)。絵ごとに 10 秒進める(ピークホールドは shot が絵ごとに消す)
  shot("Boost", sig(4200, '3', 92, 62, true, true, 1.25f), ui::SC_BOOST);
  now += 10000;
  shot("BoostIdle", sig(850, 'N', 88, 62, true, true, -0.52f), ui::SC_BOOST);        // 負圧: 帯は 0(目印だけ)
  now += 10000;
  shot("BoostMax", sig(5000, '4', 92, 62, true, true, 2.35f), ui::SC_BOOST);         // 目盛の上限で止まる
  now += 10000;
  shot("BoostShift", sig(6300, '3', 97, 41, true, true, 1.6f), ui::SC_BOOST);        // シフト点以上: 回転計の目盛・数字・針が全部赤(2.4.0)
  now += 10000;
  shot("BoostBelowShift", sig(5600, '3', 97, 41, true, true, 1.5f), ui::SC_BOOST);   // シフト点の手前: ふだんの色
  now += 10000;
  shot("BoostNoData", sig(3400, '2', 90, 50, true, true, 0, false), ui::SC_BOOST);   // ブーストの信号なし: ブースト計の帯と目印なし
  now += 10000;
  shot("BoostHalf", sig(3000, '2', 90, 50, true, true, 0.74f), ui::SC_BOOST);
  now += 10000;
  shot("BoostHot", sig(5600, '4', 112, 12, true, true, 0.9f), ui::SC_BOOST);         // 水温の異常と燃料の異常(右下の弧とアイコンが赤)
  now += 10000;
  shot("BoostPeakUp", sig(6400, '3', 92, 62, true, true, 1.4f), ui::SC_BOOST);       // 6400 まで上げてから
  now += 200;
  g_keepPeak = true;
  shot("BoostPeak", sig(4200, '3', 92, 62, true, true, 0.6f), ui::SC_BOOST);         // 4200 に下げる: ピークホールドの印が 6400 に残る
  now += 10000;

  // ---- サーキット(10/23): ピークホールドの三角形、3000rpm 以下の経過時間
  {
    shot("DrivePeakUp", sig(6000, '3', 92, 62), ui::SC_DRIVE);
    now += 200;
    g_keepPeak = true;
    shot("DrivePeak", sig(4200, '3', 92, 62), ui::SC_DRIVE);   // ピーク 6000 が残る
    now += 10000;
    g_keepPeak = true;
    shot("DriveLowPeak", sig(2600, '2', 92, 62), ui::SC_DRIVE);  // 10 秒たつとピークは今の回転数まで下がり切って、三角形は消える
    // N が 1 分続いた: ギアの所に「▼ 3000rpm」と経過時間(2:35)
    Signals n = sig(850, 'N', 96, 55);
    uint32_t t0 = now;
    ui::testShow(ui::SC_DRIVE);
    for (uint32_t t = t0; t <= t0 + 155000; t += 1000) {
      ui::testTouch(t);
      ui::update(n, ls, ps, si, t);
      gfx::dirtyClear();
    }
    now = t0 + 155000;
    shot("DriveIdleTime", n, ui::SC_DRIVE);
    now += 20000;
  }

  // ---- データ画面(10/23): GPS・IMU・CAN の値と、3000rpm 超・以下の累計時間
  {
    trip::setTimes(5025000, 45296000);  // 1:23:45 と 12:34:56
    ls.infoValid = true; ls.info.proto_ver = METER_LINK_PROTO_VER; ls.info.can_fps[0] = 1015; ls.info.can_fps[1] = 143;
    si.rc.enabled = true; si.rc.gpsOk = true; si.rc.fixType = 3; si.rc.numSV = 11; si.rc.pvtPerSec = 25; si.rc.imuOk = true;
    si.rc.gLon = 0.32f; si.rc.gLat = -0.85f;
    Signals s = sig(4200, '3', 92, 62);
    shot("Data", s, ui::SC_DATA);
    si.rc.fixType = 0; si.rc.numSV = 3; si.rc.pvtPerSec = 0; si.rc.imuOk = false; ls.info.can_fps[1] = 0;
    shot("DataNg", s, ui::SC_DATA);
    Signals a = sig(5600, '4', 112, 12);
    shot("DataAlarm", a, ui::SC_DATA);
    // 2.5.0: 外周の回転数の輪。振り切り(全周が赤)と、行の字がいちばん長くなる値(輪に掛からないこと)
    si.rc.fixType = 3; si.rc.numSV = 99; si.rc.pvtPerSec = 25; si.rc.imuOk = true; si.rc.gLon = -1.99f; si.rc.gLat = -1.99f;
    ls.info.can_fps[0] = 9999; ls.info.can_fps[1] = 9999;
    trip::setTimes(359999000u, 359999000u);  // 99:59:59
    shot("DataRingMax", sig(8000, '3', 118, 100), ui::SC_DATA);
    shot("DataRingIdle", sig(850, 'N', -18, 100), ui::SC_DATA);
    trip::setTimes(5025000, 45296000);
    memset(&si.rc, 0, sizeof(si.rc));
    memset(&ls, 0, sizeof(ls));
  }

  // ---- CAN確認: 基準を取ったあとに 3 か所が変わった状態
  {
    uint8_t d0618[8] = { 0x00, 0x00, 0x0D, 0x6C, 0x00, 0x00, 0x00, 0x00 };
    uint8_t d0810[4] = { 0x01, 0xFE, 0x70, 0x00 };
    uint8_t d0628[8] = { 0xFF, 0x24, 0x00, 0x80, 0x0B, 0x00, 0x00, 0x20 };
    uint8_t dstat[8] = { 0x20, 0x0D, 0x40, 0x7F, 0x00, 0x49, 0x20, 0x02 };
    canFrame(0x0618A001, d0618, 8, 100);
    canFrame(0x0810A000, d0810, 4, 100);
    d0628[5] = 0x00;
    canFrame(0x0628A001, d0628, 8, 100);
    canFrame(0x0A18A000, dstat, 8, 100);
    canFrame(0x04214001, dstat, 8, 100, 1);
    Signals s0 = sig(4200, '3', 92, 62);
    shot("CanTop0", s0, ui::SC_CAN);  // 記録 0 回
    cantab::record();  // 1 回目: 全部が候補
    // 履歴(折れ線)用に b7 を動かす
    static const uint8_t wave[] = { 0, 0, 0, 0, 40, 76, 76, 76, 76, 12, 0, 0, 0, 0, 50, 66, 66, 36, 25, 25, 25, 25, 25 };
    uint32_t t = 200;
    for (unsigned i = 0; i < sizeof(wave); i++) {
      for (int k = 0; k < 8; k++) {
        d0618[7] = wave[i];
        canFrame(0x0618A001, d0618, 8, t);
        cantab::tick(t);
        t += 100;
      }
    }
    d0628[5] = 0x20; canFrame(0x0628A001, d0628, 8, 1000);
    d0810[2] = 0x10; canFrame(0x0810A000, d0810, 4, 2000);
    d0618[7] = 0x4C; canFrame(0x0618A001, d0618, 8, t + 3000);
    cantab::tick(t + 3000);
    cantab::record();  // 2 回目: 1〜8 回だけ変わった 0628(b5)・0810(b2)が残る(0618 の b7 は 184 回変わったので外れる)
    Signals s = sig(4200, '3', 92, 62);
    shot("CanTop", s, ui::SC_CAN);
    shot("CanId", s, ui::SC_CAN_ID, 0);
    shot("CanId2", s, ui::SC_CAN_ID, 1);
    shot("CanDetail", s, ui::SC_CAN_DETAIL, 0, 7);
    shot("CanDetailB", s, ui::SC_CAN_DETAIL, 2, 5);
    shot("CanRedo", s, ui::SC_CAN_REDO);
  }

  // ---- 設定(2.4.0: 8 項目。受信の状態は CAN・GPS・IMU・RaceChrono のグループ、本体の状態は FPS・起動)
  {
    Signals s = sig(4200, '3', 92, 62);
    trip::setTimes(5025000, 45296000);
    const char *items[] = { "Set1Bright", "Set2Info", "Set3Sys", "Set4Shift", "Set5Warn", "Set6Time", "Set7Auto", "Set8Demo" };
    for (int i = 0; i < 8; i++) shot(items[i], s, ui::SC_SET_LIST, i);
    shot("SettingsBright", s, ui::SC_SET_EDIT, 0);
    shot("SettingsShift", s, ui::SC_SET_EDIT, 3);
    shot("SettingsWarn", s, ui::SC_SET_EDIT, 4);
    shot("SettingsAuto", s, ui::SC_SET_EDIT, 6);
    set.demoOn = 1;
    shot("SettingsDemo", s, ui::SC_SET_EDIT, 7);
    set.demoOn = 0;
    shot("SettingsRcSend", s, ui::SC_SET_EDIT, 0, 1);
    shot("SetReset", s, ui::SC_SET_RESET);
    // 保存したあとの点滅(2.4.3): 変更の画面で、押した「決定」の帯が消える(1 回目)・点く(1 回目)。その間は値が太い
    {
      ui::testShow(ui::SC_SET_EDIT, 3);
      ui::testTouch(now);
      ui::update(s, ls, ps, si, now);
      ui::tap(366, 233, now + 10);  // + 100
      ui::tap(233, 400, now + 20);  // 決定
      ui::update(s, ls, ps, si, now + 20);
      saveShot("SavedFlash1");
      ui::update(s, ls, ps, si, now + 20 + SAVED_FLASH_MS + 10);
      saveShot("SavedFlash2");
      printf("%-22s missing=%u\n", "SavedFlash", (unsigned)gfx::missingGlyphs());
      gfx::dirtyClear();
      set.shiftRpm = SET_SHIFT_DEFAULT;
    }
    ls.baud = 921600; ls.locked = true; ls.framesPerSec = 248; ls.crcErrors = 0; ls.lostFrames = 0; ls.infoValid = true;
    ls.info.proto_ver = METER_LINK_PROTO_VER; ls.info.flags = ML_F_SIM; ls.info.can_fps[0] = 1015; ls.info.can_fps[1] = 143;
    strcpy(ls.info.name, "c6dbg r1");
    ps.fps = 58; ps.loopMaxMs = 21; ps.pageMaxUs = 61000; ps.pageFrames = 5400; ps.pageAvgUs = 21400; ps.pageSlowUs = 111000;
    ps.histN = PERF_FPS_SECS;  // 直近 3 分: ふだん 40〜50、背景の色を塗り直す秒は 9〜20
    for (int i = 0; i < PERF_FPS_SECS; i++) ps.fpsHist[i] = (uint8_t)((i % 37) < 30 ? 44 + (i * 7) % 7 : 9 + (i * 5) % 12);
    si.axpOk = true; si.powerOnSrc = 0x04;
    si.cpu.ok = true;
    snprintf(si.cpu.model, sizeof(si.cpu.model), "ESP32-S3");
    si.cpu.rev = 2; si.cpu.cores = 2; si.cpu.cpuMHz = 240; si.cpu.flashBytes = 16u << 20; si.cpu.flashHz = 80000000;
    si.cpu.heapBytes = 327680; si.cpu.heapFree = 184320; si.cpu.psramBytes = 8u << 20; si.cpu.psramFree = 7340032; si.cpu.tempOk = true; si.cpu.tempC = 41.25f;
    // 受信の状態(グループのまとめ)と CAN の中
    shot("InfoCan", s, ui::SC_SET_INFO, 0);
    shot("CanBaud", s, ui::SC_SET_GRP, 0, 0);
    shot("CanReader", s, ui::SC_SET_GRP, 3, 0);
    shot("CanBus", s, ui::SC_SET_GRP, 4, 0);
    shot("CanFuel", s, ui::SC_SET_GRP, 5, 0);
    shot("CanCheck", s, ui::SC_SET_GRP, 6, 0);
    // 本体の状態
    shot("SysFps", s, ui::SC_SET_GRP, 0, 4);
    ps.pageFrames = 3;  // 描いた回が少ない(受信が無いとき)
    shot("SysFpsFew", s, ui::SC_SET_GRP, 0, 4);
    ps.pageFrames = 100;
    shot("SysCpu", s, ui::SC_SET_GRP, 1, 4);
    shot("SysClock", s, ui::SC_SET_GRP, 2, 4);
    shot("SysTemp", s, ui::SC_SET_GRP, 3, 4);
    shot("SysRam", s, ui::SC_SET_GRP, 4, 4);
    shot("SysPsram", s, ui::SC_SET_GRP, 5, 4);
    shot("SysFlash", s, ui::SC_SET_GRP, 6, 4);
    shot("SysBoot", s, ui::SC_SET_GRP, 7, 4);
    ls.locked = false; ls.infoValid = false; si.axpOk = false; ls.crcErrors = 3; ls.bytesPerSec = 1480; ls.framesPerSec = 0;
    shot("CanNoLink", s, ui::SC_SET_GRP, 2, 0);
    shot("InfoCanNg", s, ui::SC_SET_INFO, 0);
    shot("Set2InfoNg", s, ui::SC_SET_LIST, 1);
    si.rc.enabled = true; si.rc.bleOk = true; si.rc.connected = true; si.rc.canPerSec = 62; si.rc.gpsPerSec = 25; si.rc.gpsOk = true; si.rc.fixType = 3; si.rc.numSV = 11;
    si.rc.pvtPerSec = 25; si.rc.lat = 355812345; si.rc.lon = 1396821234; si.rc.gpsKmh = 48.3f; si.rc.imuOk = true; si.rc.imuLevel = true; si.rc.imuForward = true;
    si.rc.gLon = 0.32f; si.rc.gLat = -0.85f;
    shot("InfoGps", s, ui::SC_SET_INFO, 1);
    shot("InfoImu", s, ui::SC_SET_INFO, 2);
    shot("InfoRc", s, ui::SC_SET_INFO, 3);
    shot("GpsFix", s, ui::SC_SET_GRP, 0, 1);
    shot("GpsPos", s, ui::SC_SET_GRP, 1, 1);
    shot("GpsSpeed", s, ui::SC_SET_GRP, 2, 1);
    shot("ImuAcc", s, ui::SC_SET_GRP, 0, 2);
    shot("ImuCal", s, ui::SC_SET_GRP, 1, 2);
    shot("ImuReset", s, ui::SC_SET_RESET, 1);
    shot("RcSend", s, ui::SC_SET_GRP, 0, 3);
    si.rc.enabled = false;  // 入にしたが、動いているのは切(次の起動から)
    shot("RcSendPending", s, ui::SC_SET_GRP, 0, 3);
    si.rc.enabled = true;
    shot("RcLink", s, ui::SC_SET_GRP, 1, 3);
    shot("RcRate", s, ui::SC_SET_GRP, 2, 3);
    memset(&si.rc, 0, sizeof(si.rc));
    si.rc.enabled = true; si.rc.bleOk = true; si.rc.imuOk = true;
    shot("RcIdle", s, ui::SC_SET_GRP, 1, 3);
    memset(&si.rc, 0, sizeof(si.rc));
    shot("RcOff", s, ui::SC_SET_GRP, 1, 3);
    shot("GpsOff", s, ui::SC_SET_GRP, 0, 1);
    shot("InfoRcOff", s, ui::SC_SET_INFO, 3);
    memset(&si.rc, 0, sizeof(si.rc));
  }

  // ---- 触れたときの案内(2.3.0。タッチが決まった所。2.4.0: 上に「‹ スワイプ ›」、下に「長押し ▶ 設定」。上下対称)
  {
    Signals s = sig(4200, '3', 92, 62);
    const ui::Screen scs[3] = { ui::SC_DRIVE, ui::SC_BOOST, ui::SC_DATA };
    const char *nm[3] = { "GuideCircuit", "GuideTach", "GuideData" };
    for (int k = 0; k < 3; k++) {
      ui::testShow(scs[k]);
      ui::testTouch(now);
      ui::update(s, ls, ps, si, now);
      ui::touchDown(233, 233, now + 10);
      ui::update(s, ls, ps, si, now + 10 + TOUCH_CONFIRM_MS + 10);
      saveShot(nm[k]);
      printf("%-22s guide=%d missing=%u\n", nm[k], (int)ui::testGuide(), (unsigned)gfx::missingGlyphs());
      if (k == 0) {  // 指がずれた: 案内のまま、長押しの札が灰
        ui::touchMove(233 + HOLD_CANCEL_PX + 5, 233, now + 10 + TOUCH_CONFIRM_MS + 30);
        ui::update(s, ls, ps, si, now + 10 + TOUCH_CONFIRM_MS + 40);
        saveShot("GuideCancel");
        printf("%-22s guide=%d mode=%d\n", "GuideCancel", (int)ui::testGuide(), ui::testGuideMode());
      }
      ui::touchUp(now + 10 + TOUCH_CONFIRM_MS + 60);
      ui::testShow(scs[k]);
      gfx::dirtyClear();
      now += 5000;
    }
  }

  // ---- 長押しの途中(押したまま、表示を出してから外周が 6 割ほど進んだ所。2.4.0: 案内のまま、スワイプの札が灰で外周が回る)
  {
    Signals s = sig(4200, '3', 92, 62);
    const uint32_t at = HOLD_SHOW_MS + (HOLD_MS - HOLD_SHOW_MS) * 6 / 10;
    const ui::Screen scs[3] = { ui::SC_DRIVE, ui::SC_BOOST, ui::SC_DATA };
    const char *nm[3] = { "HoldCircuit", "HoldTach", "HoldData" };
    for (int k = 0; k < 3; k++) {
      ui::testShow(scs[k]);
      ui::testTouch(now);
      ui::update(s, ls, ps, si, now);
      ui::touchDown(233, 233, now);
      ui::update(s, ls, ps, si, now + TOUCH_CONFIRM_MS + 10);
      ui::update(s, ls, ps, si, now + HOLD_SHOW_MS + 10);
      ui::update(s, ls, ps, si, now + at);
      saveShot(nm[k]);
      if (k == 0) {  // 長押しの途中で指がずれた: 外周を消し、案内のまま長押しの札が灰
        ui::touchMove(233, 233 + HOLD_CANCEL_PX + 5, now + at + 5);
        ui::update(s, ls, ps, si, now + at + 10);
        saveShot("HoldCancel");
        printf("%-22s guide=%d mode=%d holding=%d\n", "HoldCancel", (int)ui::testGuide(), ui::testGuideMode(), (int)ui::testHolding());
      }
      ui::touchUp(now + at + 20);
      ui::testShow(scs[k]);
      gfx::dirtyClear();
      now += 5000;
    }
    now += 20000;
  }

  // ---- 設定の中の「長押しで開く」(2.4.4): 右の矢印を押して決まった所(2.5.0: 帯は変えない)、長押しの途中(帯が水色、外周が 6 割)、
  // 動いて取り消した所(帯の字が灰)
  {
    Signals s = sig(4200, '3', 92, 62);
    const uint32_t at = HOLD_SHOW_MS + (HOLD_MS - HOLD_SHOW_MS) * 6 / 10;
    ui::testShow(ui::SC_SET_LIST, 3);
    ui::testTouch(now);
    ui::update(s, ls, ps, si, now);
    ui::touchDown(366, 233, now + 10);
    ui::update(s, ls, ps, si, now + 10 + TOUCH_CONFIRM_MS + 10);
    saveShot("MenuTap");
    ui::touchMove(366, 233 + HOLD_CANCEL_PX + 5, now + 10 + TOUCH_CONFIRM_MS + 15);  // 縦に動かして離す(タップにもスワイプにもしない)
    ui::touchUp(now + 10 + TOUCH_CONFIRM_MS + 20);
    now += 1000;
    ui::update(s, ls, ps, si, now);
    ui::touchDown(233, 400, now + 10);
    ui::update(s, ls, ps, si, now + 10 + TOUCH_CONFIRM_MS + 10);
    ui::update(s, ls, ps, si, now + 10 + at);
    saveShot("MenuHold");
    ui::touchMove(233 + HOLD_CANCEL_PX + 5, 400, now + 10 + at + 5);
    ui::update(s, ls, ps, si, now + 10 + at + 10);
    saveShot("MenuCancel");
    printf("%-22s guide=%d mode=%d holding=%d missing=%u\n", "MenuCancel", (int)ui::testGuide(), ui::testGuideMode(), (int)ui::testHolding(), (unsigned)gfx::missingGlyphs());
    ui::touchUp(now + 10 + at + 20);
    ui::testShow(ui::SC_SET_LIST, 3);
    gfx::dirtyClear();
    now += 5000;
  }

  // ---- そのほかの状態
  {
    Signals s = sig(4200, '3', 92, 62, true, true, 0.9f);
    s.sim = true; s.scene = 7;
    shot("DriveSim", s, ui::SC_DRIVE);
    shot("BoostSim", s, ui::SC_BOOST);
    shot("DataSim", s, ui::SC_DATA);
    Signals down;
    memset(&down, 0, sizeof(down));
    shot("DriveNoLink", down, ui::SC_DRIVE);
    shot("BoostNoLink", down, ui::SC_BOOST);
    shot("DataNoLink", down, ui::SC_DATA);
    Signals hot = sig(2000, '5', 104, 100);
    shot("DataHot3", hot, ui::SC_DATA);
    Signals hot2 = sig(6100, '2', 118, 3);
    shot("DataAlarm3", hot2, ui::SC_DATA);
    shot("DriveCold", sig(1500, '1', 35, 100), ui::SC_DRIVE);
    shot("Drive5900", sig(5900, '2', 69, 15.5f), ui::SC_DRIVE);
    now += BLINK_ON_FAST_MS;  // 点滅の消灯の側(画面全体を消す設定では絵は同じで、明るさだけが変わる)
    shot("DriveRedOff", sig(6300, '3', 97, 41), ui::SC_DRIVE);
    now -= BLINK_ON_FAST_MS;
    now += ALARM_BLINK_HALF_MS;  // 異常の表示の消灯の側
    shot("DriveIdleOff", sig(850, 'N', 88, 12), ui::SC_DRIVE);
    shot("DriveHotOff", sig(5600, '4', 112, 45), ui::SC_DRIVE);
    shot("DataAlarmOff", sig(5600, '4', 112, 12), ui::SC_DATA);
    now -= ALARM_BLINK_HALF_MS;
    shot("DataCold", sig(1200, 'N', -5.4f, 100), ui::SC_DATA);    // 氷点下: 「−5」
    shot("DataCold2", sig(900, 'N', -18, 9), ui::SC_DATA);        // 氷点下 2 桁 + 燃料が少ない
    shot("DriveCold2", sig(900, 'R', -18, 9), ui::SC_DRIVE);      // バーは左端で止まる。ギアは R
    Signals odd = sig(3000, 'P', 80, 50);                         // 表示できないギアの文字 → 「–」
    odd.gearOk = false;
    shot("DriveGearUnknown", odd, ui::SC_DRIVE);
  }
  // ---- オープニング(2.5.0: 覚えた画面ごとに 3 種類。時刻で進む)
  {
    LinkStats lo;
    memset(&lo, 0, sizeof(lo));
    SysInfo so;
    memset(&so, 0, sizeof(so));
    so.touchOk = true; so.nvsOk = true; so.rc.enabled = true; so.rc.bleOk = true; so.rc.imuOk = true; so.rc.gpsOk = true;
    Signals o = sig(850, 'N', 62, 74, true, true, -0.55f);
    struct Mark { int kind; uint32_t at; const char *name; };
    const Mark marks[] = {
      { 0, OPC_SCAN_MS / 2, "OpenCircuit1" },                                     // 白い光が下から上へ走る
      { 0, OPC_SCAN_MS + OPC_FLASH_MS + OPC_RISE_MS * 8 / 10, "OpenCircuit2" },  // 点いていく
      { 0, OPC_SCAN_MS + OPC_FLASH_MS + OPC_RISE_MS + 100, "OpenCircuit3" },     // 全部点いて背景も赤(リミッター)
      { 0, OPC_SCAN_MS + OPC_FLASH_MS + OPC_RISE_MS + OPC_LIMIT_MS + OPC_FALL_MS / 4, "OpenCircuit4" },  // 消えていく
      { 0, OPC_TOTAL_MS - 40, "OpenCircuit5" },                                   // ギアが浮かぶ
      { 1, OPT_UP_MS * 4 / 10, "OpenTach1" },                                     // 針が通った所から文字盤が出る
      { 1, OPT_UP_MS + OPT_TOP_MS / 2, "OpenTach2" },                             // 振り切り(赤)
      { 1, OPT_UP_MS + OPT_TOP_MS + OPT_DOWN_MS * 6 / 10, "OpenTach3" },          // 戻っていく
      { 2, OPD_RING_MS + OPD_ROW_MS + OPD_CHECK_MS / 2, "OpenData1" },            // 2 行目を確かめている
      { 2, OPD_TOTAL_MS - 100, "OpenData2" },                                     // 全部出た(輪が緑、版)
    };
    const uint32_t t0 = 300000;
    int kind = -1;
    uint32_t tt = 0, base = 0;
    for (const Mark &m : marks) {
      if (m.kind != kind) {
        kind = m.kind;
        base = t0 + 10000u * (uint32_t)(kind + 1);
        tt = base;
        ui::testShow(ui::SC_OPENING, kind);
      }
      for (; tt <= base + m.at; tt += 5) {
        lo.locked = tt - base >= 700;
        ui::update(o, lo, ps, so, tt);
        gfx::dirtyClear();
      }
      saveShot(m.name);
      printf("%-22s level=%d screen=%d\n", m.name, (int)ui::blinkLevel(), (int)ui::screen());
    }
    so.touchOk = false;  // NG: 輪が赤
    ui::testShow(ui::SC_OPENING, 2);
    for (tt = t0 + 50000; tt <= t0 + 50000 + OPD_TOTAL_MS - 100; tt += 5) {
      ui::update(o, lo, ps, so, tt);
      gfx::dirtyClear();
    }
    saveShot("OpenDataNg");
    ui::testShow(ui::SC_OPENING, 0);  // 覚えた画面をサーキットに戻す
    ui::tap(233, 233, t0 + 60000);
    ui::testShow(ui::SC_DRIVE);
  }
  printf("missing glyphs total: %u\n", (unsigned)gfx::missingGlyphs());
  return gfx::missingGlyphs() ? 1 : 0;
}

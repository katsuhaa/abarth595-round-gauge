// blink_demo_main.cpp - 回転数の点滅を PC で確かめる(実機に書くのと同じ ui.cpp をそのまま動かす)
//
// 場面 6(c6_display_debug の sim.cpp と同じ式: 12 秒で 800→6600 rpm、6 秒で 6600→800)の走行画面を 1 ms ごとに進め、
//   1. 明るさの段階(設定の値 / 消灯 / 最大)が変わった時刻と回転数を出し、
//   2. 期待どおりか自分で判定して「BLINK PASS / FAIL」を出し、
//   3. 時系列を <出力フォルダ>/blink_timeline.csv に書く(t_ms, rpm, level, on_ms, off_ms)。
// 使い方: blink <出力フォルダ> [-]
//   「-」を付けると、60 fps の絵(466×466、RGB24、明るさを掛けたもの。左上に時刻・回転数・段階)を標準出力に流す。
//   run_tests.sh はこれを ffmpeg に渡して out/blink_scene6_60fps.mp4 を作る(ffmpeg が無ければ絵は作らない)。
//   絵を流すときは、文字の出力は標準エラーに出す。
// 期待(config.h の値から計算。シフト点は SET_SHIFT_DEFAULT):
//   - 点滅はシフト点 + RING_BLINK_OFFSET から始まり、点灯側から始まる
//   - 消灯の時間はいつも BLINK_OFF_MS。上りでは点灯の時間が BLINK_ON_SLOW_MS から BLINK_ON_FAST_MS まで途切れなく縮む
//     (長くなることはない)
//   - シフト点 + RING_SOLID_OFFSET 以上では点滅せず、段階が最大(BL_MAX)
//   - 下りで点滅に戻り、シフト点 + RING_BLINK_OFFSET を下回ると止まって設定の明るさに戻る
#include "../meter_display/app.h"
#include "../meter_display/gfx.h"
#include "../meter_display/ui.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>

static uint16_t fb[gfx::W * gfx::H];
static float scratch[gfx::W * (gfx::H + 2)];
static Settings set;  // main で settingsDefaults(2.10.0)
static LinkStats ls;
static PerfStats ps;
static SysInfo si;
static FILE *msg = stdout;  // 文字の出力先(絵を標準出力に流すときは stderr)

// 場面 6 と同じ式
static float rpmAt(uint32_t t) {
  return (t < 12000) ? 800 + 5800.0f * t / 12000.0f : 6600 - 5800.0f * (t - 12000) / 6000.0f;
}

// 絵を 1 枚出す: 左上に説明を描き、明るさを掛けて RGB24 で書き、描いた所を元に戻す
static void emitFrame(FILE *out, uint32_t tMs, int rpm, int level, uint32_t onMs, int pct) {
  const int CAP_H = 30;
  static uint16_t saved[gfx::W * CAP_H];
  static uint8_t rgb[gfx::W * gfx::H * 3];
  memcpy(saved, fb, sizeof(saved));
  char cap[96];
  const char *name = level == BL_OFF ? "OFF" : level == BL_MAX ? "MAX" : "ON";
  if (onMs) snprintf(cap, sizeof(cap), "%u.%02u s  %d rpm  %s  on %u / off %u ms", (unsigned)(tMs / 1000), (unsigned)((tMs % 1000) / 10), rpm, name, (unsigned)onMs, (unsigned)BLINK_OFF_MS);
  else snprintf(cap, sizeof(cap), "%u.%02u s  %d rpm  %s", (unsigned)(tMs / 1000), (unsigned)((tMs % 1000) / 10), rpm, name);
  gfx::fillRect(0, 0, gfx::W, CAP_H, gfx::rgb(0x000000));
  gfx::drawText(FONT_NUM600_24, 84, 23, cap, gfx::rgb(0xFFFFFF));  // ASCII が全部ある書体
  for (int i = 0; i < gfx::W * gfx::H; i++) {
    uint16_t p = fb[i];
    int r = (p >> 11) & 31, g = (p >> 5) & 63, b = p & 31;
    int k = (i < gfx::W * CAP_H) ? 100 : pct;  // 説明の行は明るさを掛けない
    rgb[i * 3 + 0] = (uint8_t)((r * 255 / 31) * k / 100);
    rgb[i * 3 + 1] = (uint8_t)((g * 255 / 63) * k / 100);
    rgb[i * 3 + 2] = (uint8_t)((b * 255 / 31) * k / 100);
  }
  fwrite(rgb, 1, sizeof(rgb), out);
  memcpy(fb, saved, sizeof(saved));
  gfx::dirtyClear();
}

int main(int argc, char **argv) {
  std::string outdir = argc > 1 ? argv[1] : ".";
  bool frames = argc > 2 && !strcmp(argv[2], "-");
  if (frames) msg = stderr;
  settingsDefaults(&set);
  gfx::begin(fb, scratch);
  mlink::begin();
  cantab::begin();
  ui::begin(&set);
  gfx::setTabularDigits(true);
  memset(&ls, 0, sizeof(ls));
  memset(&ps, 0, sizeof(ps));
  memset(&si, 0, sizeof(si));
  ui::testShow(ui::SC_DRIVE);

  const int sp = set.shiftRpm;
  const int blinkFrom = sp + RING_BLINK_OFFSET, solidFrom = sp + RING_SOLID_OFFSET;
  fprintf(msg, "シフト点 %d: 点滅 %d〜%d rpm(消灯 %d ms、点灯 %d → %d ms)、%d rpm 以上は点滅せず最大輝度\n", sp, blinkFrom, solidFrom - 1, BLINK_OFF_MS,
          BLINK_ON_SLOW_MS, BLINK_ON_FAST_MS, solidFrom);

  FILE *csv = fopen((outdir + "/blink_timeline.csv").c_str(), "w");
  if (csv) fprintf(csv, "t_ms,rpm,level,on_ms,off_ms\n");
  const uint32_t T0 = 1000000, DUR = 18000;
  int nFrames = 0;
  int prevLevel = -1;
  uint32_t prevEdge = 0, prevOnUp = 0;
  // 判定用
  bool firstChangeOk = false, seenFirst = false;
  int badGapUp = 0, gapGrewUp = 0, gapsUp = 0, togglesInSolid = 0, togglesDown = 0, badGapDown = 0, badOff = 0;
  bool maxSeen = false, maxOnlyAbove = true, lowLevelOk = true;
  int firstChangeRpm = -1;
  for (uint32_t t = 0; t <= DUR; t++) {
    uint32_t now = T0 + t;
    Signals s;
    memset(&s, 0, sizeof(s));
    s.linkUp = true;
    s.rpmOk = true;
    s.rpm = (uint16_t)(rpmAt(t) + 0.5f);
    s.gearOk = true;
    s.gear = '3';
    s.waterOk = true;
    s.waterC = 90;
    s.fuelOk = true;
    s.fuelPct = 50;
    s.speedOk = true;
    s.speedKmh = s.rpm / 48.0f;
    s.throttleOk = true;
    s.throttlePct = t < 12000 ? 100 : 0;
    ui::update(s, ls, ps, si, now);
    gfx::dirtyClear();
    int level = (int)ui::blinkLevel();
    uint32_t onMs = ui::blinkOnMs();
    if (csv) fprintf(csv, "%u,%u,%d,%u,%u\n", t, s.rpm, level, (unsigned)onMs, (unsigned)ui::blinkOffMs());
    bool up = t < 12000;
    if (s.rpm >= solidFrom) {
      if (level != BL_MAX) maxOnlyAbove = false;
      maxSeen = maxSeen || level == BL_MAX;
    } else if (level == BL_MAX) {
      maxOnlyAbove = false;
    }
    if (s.rpm < blinkFrom && level != BL_NORMAL) lowLevelOk = false;
    if (level != prevLevel) {
      if (prevLevel >= 0) {
        uint32_t gap = t - prevEdge;
        fprintf(msg, "t=%5u ms rpm=%4u 段階 %d→%d(前の切り替えから %u ms)\n", t, s.rpm, prevLevel, level, gap);
        if (!seenFirst) {  // 最初の切り替え: 点滅の始まりの直後で、点灯 → 消灯
          seenFirst = true;
          firstChangeRpm = s.rpm;
          firstChangeOk = prevLevel == BL_NORMAL && level == BL_OFF && s.rpm >= blinkFrom && s.rpm < blinkFrom + 200;
        } else if (s.rpm >= solidFrom && level != BL_MAX && prevLevel != BL_MAX) {
          togglesInSolid++;
        } else if (level == BL_MAX || prevLevel == BL_MAX) {
          // 最大輝度との行き来: 間隔は見ない
        } else if (prevLevel == BL_OFF) {  // 消灯 → 点灯: 終わった区間は消灯で、いつも BLINK_OFF_MS
          if (gap != BLINK_OFF_MS) badOff++;
          if (!up) togglesDown++;
        } else if (up) {  // 点灯 → 消灯(上り): 終わった区間は点灯で、だんだん短くなる
          gapsUp++;
          if (gap < BLINK_ON_FAST_MS || gap > BLINK_ON_SLOW_MS) badGapUp++;
          if (prevOnUp && gap > prevOnUp) gapGrewUp++;
          prevOnUp = gap;
        } else {  // 点灯 → 消灯(下り)
          togglesDown++;
          if (gap < BLINK_ON_FAST_MS || gap > BLINK_ON_SLOW_MS) badGapDown++;
        }
      }
      prevLevel = level;
      prevEdge = t;
    }
    if (frames && (uint64_t)t * 60 >= (uint64_t)nFrames * 1000) {
      int pct = level == BL_OFF ? 0 : level == BL_MAX ? 100 : set.brightnessPct;
      emitFrame(stdout, t, s.rpm, level, onMs, pct);
      nFrames++;
    }
  }
  if (csv) fclose(csv);
  ui::BlinkStats bs;
  ui::blinkStats(&bs, false);
  bool ok = firstChangeOk && gapsUp >= 5 && badGapUp == 0 && gapGrewUp == 0 && badOff == 0 && maxSeen && maxOnlyAbove && togglesInSolid == 0 &&
            togglesDown >= 5 && badGapDown == 0 && lowLevelOk && prevLevel == BL_NORMAL && bs.lateOver == 0;
  fprintf(msg, "最初の切り替え %d rpm(点灯→消灯) %s / 上りの点灯 %d 回(範囲外 %d、長くなった %d) / 消灯が %d ms でない回数 %d / %d 以上で最大輝度 %s(その間の点滅 %d 回) / 下りの切り替え %d 回(点灯が範囲外 %d) / 終わりは設定の明るさ %s / 切り替えの遅れ 2 割超え %u 回\n",
          firstChangeRpm, firstChangeOk ? "OK" : "NG", gapsUp, badGapUp, gapGrewUp, BLINK_OFF_MS, badOff, solidFrom, (maxSeen && maxOnlyAbove) ? "OK" : "NG", togglesInSolid,
          togglesDown, badGapDown, (lowLevelOk && prevLevel == BL_NORMAL) ? "OK" : "NG", bs.lateOver);
  if (frames) fprintf(msg, "絵 %d 枚(60 fps)\n", nFrames);
  fprintf(msg, "%s\n", ok ? "BLINK PASS" : "BLINK FAIL");
  return ok ? 0 : 1;
}

// plat_pc.cpp - PC 上の試験用: 基板に依存する部分(meter_display.ino にある plat)の代わり
#include "../meter_display/app.h"
#include <stdlib.h>
#include <string.h>
int g_hostBrightness = -1;
int g_hostSaves = 0;
bool g_hostSaveOk = true;
void platLock() {}
void platUnlock() {}
void *platAlloc(size_t bytes) { return calloc(1, bytes); }
void platSetBrightness(int pct) { g_hostBrightness = pct; }
int g_hostBlank = -1;   // 最後に platBlink で知らされた段階が消灯(1)かそれ以外(0)。-1 = まだ呼ばれていない
int g_hostLevel = -1;   // 最後に platBlink で知らされた段階(BlinkLevel)
int g_hostBlinks = 0;   // platBlink が呼ばれた回数
void platBlink(uint8_t level) { g_hostLevel = level; g_hostBlank = level == BL_OFF ? 1 : 0; g_hostBlinks++; }
void platBlinkTick() {}  // PC 上では時刻は update に渡した値だけで進む
bool platSaveSettings(const Settings &) { g_hostSaves++; return g_hostSaveOk; }
void platCanFrame(uint8_t, uint8_t, uint32_t, uint8_t, const uint8_t *, uint32_t) {}
int g_hostImuResets = 0;
void platResetImuCal() {
  g_hostImuResets++;
  imu::reset();
}
int g_hostTimeSaves = 0;
void platSaveTimes() { g_hostTimeSaves++; }
// CAN確認の「前後で比べる」の前の記録(2.6.0)。電源を切っても残る NVS の代わり(cantab::begin のあとも残る)
unsigned char g_hostCanPre[8192];
size_t g_hostCanPreN = 0;
int g_hostCanPreSaves = 0;
bool platSaveCanPre(const void *data, size_t n) {
  g_hostCanPreSaves++;
  if (n > sizeof(g_hostCanPre)) return false;
  if (n) memcpy(g_hostCanPre, data, n);
  g_hostCanPreN = n;
  return true;
}
size_t platLoadCanPre(void *data, size_t maxN) {
  if (g_hostCanPreN == 0 || g_hostCanPreN > maxN) return 0;
  memcpy(data, g_hostCanPre, g_hostCanPreN);
  return g_hostCanPreN;
}
// 走行記録の保存領域(2.15.0)。基板のパーティション spiffs(917,504 バイト)の代わり。フラッシュと同じく、書くとビットは 1 → 0 にしか
// ならない(消すと 0xFF)。電源を切っても残る(odo::begin のあとも残る)
uint32_t g_hostOdoSize = 0xE0000;
unsigned char g_hostOdo[0xE0000];
bool g_hostOdoInit = false;
bool g_hostOdoWriteOk = true;   // false で書けない(試験用)
int g_hostOdoWrites = 0, g_hostOdoErases = 0;
static void hostOdoInit() {
  if (!g_hostOdoInit) memset(g_hostOdo, 0xFF, sizeof(g_hostOdo)), g_hostOdoInit = true;
}
uint32_t platOdoSize() { return g_hostOdoSize; }
bool platOdoRead(uint32_t off, void *data, size_t n) {
  hostOdoInit();
  if (off + n > g_hostOdoSize) return false;
  memcpy(data, g_hostOdo + off, n);
  return true;
}
bool platOdoWrite(uint32_t off, const void *data, size_t n) {
  hostOdoInit();
  if (!g_hostOdoWriteOk || off + n > g_hostOdoSize) return false;
  for (size_t i = 0; i < n; i++) g_hostOdo[off + i] &= ((const unsigned char *)data)[i];
  g_hostOdoWrites++;
  return true;
}
bool platOdoErase(uint32_t off, size_t n) {
  hostOdoInit();
  if (off % 4096 || n % 4096 || off + n > g_hostOdoSize) return false;
  memset(g_hostOdo + off, 0xFF, n);
  g_hostOdoErases++;
  return true;
}

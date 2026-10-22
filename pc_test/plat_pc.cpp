// plat_pc.cpp - PC 上の試験用: 基板に依存する部分(meter_display.ino にある plat)の代わり
#include "../meter_display/app.h"
#include <stdlib.h>
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

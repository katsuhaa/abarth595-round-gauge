// app.cpp - 受信データの解釈(mlink)、車の CAN から値を作る(veh)、CAN の表(cantab)、全開と走行の時間(trip)。説明は app.h
#include "app.h"
#include <string.h>
#include <stdlib.h>
#include <algorithm>
#include <math.h>

// ================================================================ mlink
namespace mlink {

static uint8_t g_buf[192];
static int g_len = 0;

static MlSignals g_sig;
static bool g_sigSeen = false;
static uint32_t g_sigMs = 0;

static LinkStats g_st;
static uint32_t g_infoMs = 0;
static bool g_haveSeq = false;
static uint8_t g_lastSeq = 0;
static uint32_t g_lastGoodMs = 0;
static bool g_anySeen = false;  // 何かの正しいフレームを受けた(値のフレームが来ないとき、getSignals の linkUp に使う)
static uint32_t g_tickMs = 0, g_tickFrames = 0;
static uint32_t g_rxBytes = 0, g_tickBytes = 0;
static volatile bool g_graceOn = false;
static volatile uint32_t g_graceUntil = 0;

void graceUntil(uint32_t untilMs) {
  g_graceUntil = untilMs;
  g_graceOn = true;
}
static bool inGrace(uint32_t nowMs) {
  if (!g_graceOn) return false;
  if ((int32_t)(nowMs - g_graceUntil) >= 0) {
    g_graceOn = false;
    return false;
  }
  return true;
}

// 速度の自動合わせ
static const uint32_t BAUDS[] = METER_LINK_BAUD_CANDIDATES;
static const int N_BAUDS = sizeof(BAUDS) / sizeof(BAUDS[0]);
static int g_abIdx = 0;
static bool g_abStarted = false;
static uint32_t g_abWinMs = 0, g_abWinRx = 0, g_abWinGood = 0;
static int g_abBadWins = 0;

void begin() {
  g_len = 0;
  g_sigSeen = false;
  g_haveSeq = false;
  g_anySeen = false;
  g_tickMs = 0;
  g_tickFrames = 0;
  g_rxBytes = 0;
  g_tickBytes = 0;
  g_graceOn = false;
  g_abIdx = 0;
  g_abStarted = false;
  g_abBadWins = 0;
  memset(&g_st, 0, sizeof(g_st));
  g_st.baud = BAUDS[0];
}

// 種類と長さの組み合わせが、決めた形に合っているか
static bool plausible(uint8_t type, uint8_t len, const uint8_t *payload) {
  switch (type) {
    case ML_TYPE_SIGNALS: return len == sizeof(MlSignals);
    case ML_TYPE_INFO: return len == sizeof(MlInfo);
    case ML_TYPE_CAN: return len >= sizeof(MlCanHead) && (payload[0] & 0x0F) <= 8 && len == sizeof(MlCanHead) + (payload[0] & 0x0F);
    default: return false;
  }
}

static void handle(uint8_t type, uint8_t seq, const uint8_t *p, uint32_t nowMs) {
  platLock();
  if (g_haveSeq && (uint32_t)(nowMs - g_lastGoodMs) <= LINK_TIMEOUT_MS && !inGrace(nowMs)) {
    g_st.lostFrames += (uint8_t)(seq - g_lastSeq - 1);
  }
  g_haveSeq = true;
  g_lastSeq = seq;
  g_lastGoodMs = nowMs;
  g_anySeen = true;
  g_st.framesOk++;
  if (type == ML_TYPE_SIGNALS) {
    memcpy(&g_sig, p, sizeof(g_sig));
    g_sigSeen = true;
    g_sigMs = nowMs;
  } else if (type == ML_TYPE_INFO) {
    memcpy(&g_st.info, p, sizeof(MlInfo));
    g_st.info.name[sizeof(g_st.info.name) - 1] = 0;
    for (char *c = g_st.info.name; *c; c++) {  // 画面に出せる文字(ASCII)だけにする
      if (*c < 0x20 || *c > 0x7E) *c = '?';
    }
    g_st.infoValid = true;
    g_infoMs = nowMs;
  }
  platUnlock();
  if (type == ML_TYPE_CAN) {
    MlCanHead h;
    memcpy(&h, p, sizeof(h));
    cantab::onFrame((h.info >> 7) & 1, (h.info >> 6) & 1, h.id, h.info & 0x0F, p + sizeof(h), nowMs);
    platCanFrame((h.info >> 7) & 1, (h.info >> 6) & 1, h.id, h.info & 0x0F, p + sizeof(h), nowMs);
    veh::onFrame((h.info >> 7) & 1, (h.info >> 6) & 1, h.id, h.info & 0x0F, p + sizeof(h), nowMs);
  }
}

void feed(const uint8_t *data, size_t n, uint32_t nowMs) {
  g_rxBytes += (uint32_t)n;
  while (n > 0) {
    size_t room = sizeof(g_buf) - (size_t)g_len;
    size_t take = n < room ? n : room;
    memcpy(g_buf + g_len, data, take);
    g_len += (int)take;
    data += take;
    n -= take;

    int i = 0;
    for (;;) {
      while (i + 1 < g_len && !(g_buf[i] == ML_SYNC0 && g_buf[i + 1] == ML_SYNC1)) {
        i++;
        g_st.junkBytes++;
      }
      if (g_len - i < 5) break;  // 種類・連番・長さまでそろうのを待つ
      uint8_t type = g_buf[i + 2], seq = g_buf[i + 3], len = g_buf[i + 4];
      if (len > ML_MAX_PAYLOAD) {
        i++;
        g_st.junkBytes++;
        continue;
      }
      int total = (int)len + ML_OVERHEAD;
      if (g_len - i < total) break;  // 残りが届くのを待つ
      if (!plausible(type, len, g_buf + i + 5)) {
        i++;
        g_st.junkBytes++;
        continue;
      }
      uint16_t crc = mlCrc16(g_buf + i + 2, (size_t)len + 3);
      uint16_t got = (uint16_t)(g_buf[i + 5 + len] | (g_buf[i + 6 + len] << 8));
      if (crc == got) {
        handle(type, seq, g_buf + i + 5, nowMs);
        i += total;
      } else {
        if (!inGrace(nowMs)) g_st.crcErrors++;
        i++;  // 同期の 1 バイト目だけを捨てて探し直す(続きに正しいフレームがあれば拾える)
      }
    }
    if (i > 0) {
      memmove(g_buf, g_buf + i, (size_t)(g_len - i));
      g_len -= i;
    }
  }
}

void tick(uint32_t nowMs) {
  if ((uint32_t)(nowMs - g_tickMs) >= 1000) {
    g_tickMs = nowMs;
    g_st.framesPerSec = g_st.framesOk - g_tickFrames;
    g_tickFrames = g_st.framesOk;
    g_st.bytesPerSec = g_rxBytes - g_tickBytes;
    g_tickBytes = g_rxBytes;
    if (g_st.infoValid && (uint32_t)(nowMs - g_infoMs) > 3000) g_st.infoValid = false;
    // 長く途切れた値は「見ていない」に戻す(getSignals が時刻の差を符号つきで見るので、古いまま残さない)
    if (g_sigSeen && (uint32_t)(nowMs - g_sigMs) > 2 * LINK_TIMEOUT_MS) {
      platLock();
      g_sigSeen = false;
      platUnlock();
    }
    if (g_anySeen && (uint32_t)(nowMs - g_lastGoodMs) > 2 * LINK_TIMEOUT_MS) {
      platLock();
      g_anySeen = false;
      platUnlock();
    }
  }
}

void resyncSeq() {
  platLock();
  g_haveSeq = false;
  g_len = 0;
  platUnlock();
}

uint32_t baud() {
  return BAUDS[g_abIdx];
}

uint32_t autoBaud(uint32_t nowMs, size_t rxBytes) {
  if (!g_abStarted) {
    g_abStarted = true;
    g_abWinMs = nowMs;
    g_abWinRx = 0;
    g_abWinGood = g_st.framesOk;
    g_abBadWins = 0;
  }
  g_abWinRx += (uint32_t)rxBytes;
  bool locked = g_haveSeq && (uint32_t)(nowMs - g_lastGoodMs) <= LINK_TIMEOUT_MS;
  uint32_t next = 0;
  if ((uint32_t)(nowMs - g_abWinMs) >= LINK_BAUD_WINDOW_MS) {
    // この区間に、バイトは届いたのに正しいフレームが 1 つも無かったか
    bool bad = g_abWinRx >= LINK_BAUD_MIN_BYTES && g_st.framesOk == g_abWinGood;
    g_abBadWins = bad ? g_abBadWins + 1 : 0;
    g_abWinMs = nowMs;
    g_abWinRx = 0;
    g_abWinGood = g_st.framesOk;
    if (g_abBadWins >= LINK_BAUD_BAD_WINDOWS) {
      g_abBadWins = 0;
      g_abIdx = (g_abIdx + 1) % N_BAUDS;
      g_len = 0;  // 前の速度で読んだ途中のバイトは捨てる
      next = BAUDS[g_abIdx];
      locked = false;
    }
  }
  platLock();
  g_st.baud = BAUDS[g_abIdx];
  g_st.locked = locked;
  platUnlock();
  return next;
}

// 値のフレームが LINK_TIMEOUT_MS 以内に届いているか(platLock の中で呼ぶ)。
// 時刻は受信タスクが付けたもの。呼び出し側が nowMs を読んだあとに届いたフレームだと nowMs より新しい(差が負)
static bool sigUpLocked(uint32_t nowMs) {
  return g_sigSeen && (int32_t)(nowMs - g_sigMs) <= (int32_t)LINK_TIMEOUT_MS;
}
static bool anyUpLocked(uint32_t nowMs) {
  return g_anySeen && (int32_t)(nowMs - g_lastGoodMs) <= (int32_t)LINK_TIMEOUT_MS;
}

bool usingRaw(uint32_t nowMs) {
  platLock();
  bool raw = !sigUpLocked(nowMs) && anyUpLocked(nowMs);
  platUnlock();
  return raw;
}

void getSignals(Signals *out, uint32_t nowMs) {
  MlSignals s;
  bool sigUp, anyUp;
  platLock();
  s = g_sig;
  sigUp = sigUpLocked(nowMs);
  anyUp = anyUpLocked(nowMs);
  platUnlock();
  memset(out, 0, sizeof(*out));
  if (!sigUp) {
    // 値のフレームが来ていない(2.0.0 の c6_can_link は送らない): CAN の生データから作る(veh)
    out->linkUp = anyUp;
    if (anyUp) veh::getSignals(out, nowMs);
    return;
  }
  out->linkUp = true;
  out->rpmOk = s.valid & ML_V_RPM;
  out->speedOk = s.valid & ML_V_SPEED;
  out->throttleOk = s.valid & ML_V_THROTTLE;
  out->waterOk = s.valid & ML_V_WATER;
  out->fuelOk = s.valid & ML_V_FUEL;
  out->boostOk = s.valid & ML_V_BOOST;
  // ギアは、表示できる文字('1'〜'9'、'N'、'R')のときだけ使う。それ以外は「分からない」
  out->gearOk = (s.valid & ML_V_GEAR) && ((s.gear >= '1' && s.gear <= '9') || s.gear == 'N' || s.gear == 'R');
  out->rpm = s.rpm;
  out->speedKmh = s.speed_c / 100.0f;
  out->throttlePct = s.throttle_h / 2.0f;
  out->waterC = s.water_d / 10.0f;
  out->fuelPct = s.fuel_h / 2.0f;
  out->boostBar = s.boost_c / 100.0f;
  out->gear = s.gear;
  out->sim = s.flags & ML_F_SIM;
  out->scene = s.scene;
}

void getStats(LinkStats *out) {
  platLock();
  *out = g_st;
  platUnlock();
}

}  // namespace mlink

// ================================================================ veh(config.h の VEH_*。c6_can_link 1.1.0 の decode・decodeB・updateGear・sendSignals を移した)
namespace veh {

struct State {
  uint16_t rpm;
  uint8_t accel;          // 0〜255
  uint16_t speedRaw;      // km/h × 128
  bool clutch;
  int water;              // ℃
  uint8_t boostRaw;       // 0A1CA001 b2(そのまま。2.8.0。2.7.0 までは 0A18A001 b4 = 燃料の量だった)
  bool reverse;           // R に入っている
  uint32_t rpmMs, speedMs, clutchMs, engMs, boostMs;  // 最後に受けた時刻(0 = まだ)
  // 燃料(2.7.0: 0A18A000 b5 = %)
  uint32_t fuelSeenMs;    // 0A18A000 を最後に受けた時刻(b5 = 0 も含む)
  uint8_t fuelPct;        // 最後に使った値 [%]
  uint8_t fuelLastB5;     // 最後に受けた b5(0 も含む)
  bool fuelHave;          // fuelPct に値がある
  uint32_t fuelIgnored;   // 捨てた b5(0 と 100 超)のフレームの数
  // ライト(2.9.0: B-CAN 06214000 b0 の 0x08 = ライトのスイッチ)
  bool sport;             // スポーツモード(0A18A000 b4 の VEH_SPORT_MASK。2.11.0。受けた時刻は fuelSeenMs)
  uint32_t lightSeenMs;   // 最後に受けた時刻(0 = まだ)
  uint32_t lightFirstMs;  // 最初に受けた時刻(電源を入れてからの ms)
  uint8_t lightLastB0;    // 最後に受けた b0
  bool lightOn;           // ライトのスイッチ(使っている値)
  bool lightPend;         // lightOn と違う値を受けて、VEH_LIGHT_SETTLE_MS 待っている
  uint32_t lightPendMs;   // その値を最初に受けた時刻
  uint32_t lightChanges;  // 切り替えた回数(最初の値は数えない)
};
static State g;              // 受信タスクが書き、loop() が読む(platLock で守る)
static char g_gear = 0;      // 推定したギア('1'〜'5'、'N'、0 = 分からない)。getSignals だけが使う
static float g_lastRatio = 0;

static inline uint32_t stamp(uint32_t t) {  // 時刻 0 は「まだ」に使うので、0 は 1 にする
  return t ? t : 1;
}
// t が timeout 以内か。受信タスクの時刻が now より新しいことがある(差が負)
static bool within(uint32_t t, uint32_t now, uint32_t timeout) {
  return t && (int32_t)(now - t) <= (int32_t)timeout;
}

void begin() {
  platLock();
  memset(&g, 0, sizeof(g));
  platUnlock();
  g_gear = 0;
  g_lastRatio = 0;
}

// ライト: lightOn と違う値を受けてから VEH_LIGHT_SETTLE_MS のあいだに元の値を受けなければ、切り替える(platLock の中で呼ぶ。
// 次のフレームを待たずに切り替わるよう、lightInfo からも呼ぶ)。受信タスクの時刻が now より新しいとき(差が負)は待つ
static void lightSettle(uint32_t now) {
  if (g.lightPend && (int32_t)(now - g.lightPendMs) >= (int32_t)VEH_LIGHT_SETTLE_MS) {
    g.lightOn = !g.lightOn;
    g.lightPend = false;
    g.lightChanges++;
  }
}

void onFrame(uint8_t bus, uint8_t ext, uint32_t id, uint8_t dlc, const uint8_t *d, uint32_t now) {
  if (!ext) return;  // 使う値はどれも 29 ビット ID
  if (bus == 1) {
    if (id == VEH_ID_LIGHT && dlc > VEH_LIGHT_BYTE) {  // ライト(2.9.0)。b0 のほかのビットは見ない
      const bool on = (d[VEH_LIGHT_BYTE] & VEH_LIGHT_MASK) != 0;
      platLock();
      if (!g.lightSeenMs) {  // 電源を入れてから最初: すぐ使う
        g.lightOn = on;
        g.lightFirstMs = stamp(now);
      } else {
        lightSettle(now);
        if (on == g.lightOn) {
          g.lightPend = false;  // 元の値: 待っていた切り替えはやめる
        } else if (!g.lightPend) {
          g.lightPend = true;
          g.lightPendMs = stamp(now);
        }
      }
      g.lightLastB0 = d[VEH_LIGHT_BYTE];
      g.lightSeenMs = stamp(now);
      platUnlock();
    }
    return;
  }
  if (bus == 0) {
    if (id == VEH_ID_RPM_ACCEL && dlc >= 8) {
      platLock();
      g.rpm = (uint16_t)(((uint16_t)d[2] << 8) | d[3]);
      g.accel = d[7];
      g.rpmMs = stamp(now);
      platUnlock();
    } else if (id == VEH_ID_SPEED && dlc >= 6) {
      platLock();
      g.speedRaw = (uint16_t)(((uint16_t)d[4] << 8) | d[5]);
      g.speedMs = stamp(now);
      platUnlock();
    } else if (id == VEH_ID_CLUTCH && dlc >= 6) {
      platLock();
      g.clutch = (d[5] & 0x20) != 0;
      g.clutchMs = stamp(now);
      platUnlock();
    } else if (id == VEH_ID_ENGINE && dlc >= 8) {
      platLock();
      g.water = (int)d[3] - 40;
      g.reverse = (d[7] & VEH_REVERSE_MASK) != 0;
      g.engMs = stamp(now);
      platUnlock();
    } else if (id == VEH_ID_BOOST && dlc >= 3) {
      platLock();
      g.boostRaw = d[2];
      g.boostMs = stamp(now);
      platUnlock();
    } else if (id == VEH_ID_FUEL && dlc >= 6) {
      platLock();
      g.fuelSeenMs = stamp(now);
      g.fuelLastB5 = d[5];
      g.sport = (d[4] & VEH_SPORT_MASK) != 0;  // 2.11.0
      if (d[5] == 0 || d[5] > 100) {
        g.fuelIgnored++;  // 0 = 測れていない(エンジンが止まっている間など)。前の値のまま
      } else {
        g.fuelPct = d[5];
        g.fuelHave = true;
      }
      platUnlock();
    }
  }
}

// ---- ギアの推定(c6_can_link 1.1.0 の updateGear と同じ)
// rpm ÷ km/h の計算値。1 km/h = 1000 m/h → タイヤ 1000/外周 回転/h → ÷60 回転/分 × 減速比
static float rpmPerKmh(int gear) {
  static const float RATIO[5] = VEH_GEAR_RATIOS;
  return RATIO[gear] * VEH_FINAL_DRIVE * (1000.0f / VEH_TIRE_CIRC_M) / 60.0f;
}
static void updateGear(bool rpmOk, bool speedOk, bool clutchOk, const State &r) {
  if (!rpmOk || !speedOk) {
    g_gear = 0;
    return;
  }
  float kmh = r.speedRaw / 128.0f;
  if (kmh < VEH_STOP_KMH) {
    g_gear = 'N';
    return;
  }
  if (kmh < VEH_GEAR_MIN_KMH || r.rpm < VEH_GEAR_MIN_RPM) return;  // 遅いときは前の値のまま
  if (VEH_GEAR_HOLD_ON_CLUTCH && clutchOk && r.clutch) return;    // クラッチを踏んでいる間は前の値のまま(VEH_GEAR_HOLD_ON_CLUTCH 1 のとき)
  (void)clutchOk;
  float ratio = r.rpm / kmh;
  g_lastRatio = ratio;
  for (int k = 0; k < 5; k++) {
    float ref = rpmPerKmh(k);
    if (ratio >= ref * (1.0f - VEH_GEAR_TOL_PCT / 100.0f) && ratio <= ref * (1.0f + VEH_GEAR_TOL_PCT / 100.0f)) {
      g_gear = (char)('1' + k);
      return;
    }
  }
  g_gear = 0;  // どのギアにも合わない(変速の途中など)
}

void getSignals(Signals *out, uint32_t now) {
  State r;
  platLock();
  r = g;
  platUnlock();
  bool rpmOk = within(r.rpmMs, now, VEH_TIMEOUT_MS);
  bool speedOk = within(r.speedMs, now, VEH_TIMEOUT_MS);
  bool clutchOk = within(r.clutchMs, now, VEH_TIMEOUT_MS);
  bool waterOk = within(r.engMs, now, VEH_WATER_TIMEOUT_MS);
  bool engOk = within(r.engMs, now, VEH_TIMEOUT_MS);  // 0A18A001(リバース)が届いている
  bool boostOk = within(r.boostMs, now, VEH_TIMEOUT_MS);  // 0A1CA001(ブースト)が届いている
  bool fuelOk = r.fuelHave && within(r.fuelSeenMs, now, VEH_FUEL_TIMEOUT_MS);
  updateGear(rpmOk, speedOk, clutchOk, r);
  char gear = (engOk && r.reverse) ? 'R' : g_gear;  // R のビットは推定より優先

  // 値の刻みは 1.x の c6_can_link が値のフレームで送っていたときと同じにする(表示と判定が変わらないように)
  out->rpmOk = rpmOk;
  out->throttleOk = rpmOk;
  out->speedOk = speedOk;
  out->waterOk = waterOk;
  out->boostOk = boostOk;
  out->fuelOk = fuelOk;
  out->gearOk = gear != 0;
  out->rpm = r.rpm;
  out->speedKmh = (uint16_t)((uint32_t)r.speedRaw * 100 / 128) / 100.0f;
  out->throttlePct = (uint8_t)(((uint32_t)r.accel * 200 + 127) / 255) / 2.0f;
  int w = r.water < -3276 ? -3276 : (r.water > 3276 ? 3276 : r.water);
  out->waterC = (int16_t)(w * 10) / 10.0f;
  out->fuelPct = r.fuelPct;
  int bc = (int)(r.boostRaw * VEH_BOOST_BAR_PER_LSB * 100.0f + 0.5f);
  out->boostBar = (int16_t)(bc > 32767 ? 32767 : bc) / 100.0f;
  out->gear = gear;
  out->sportOk = within(r.fuelSeenMs, now, VEH_FUEL_TIMEOUT_MS);  // 2.11.0: 0A18A000 を受けている
  out->sport = out->sportOk && r.sport;
}

void fuelInfo(FuelInfo *out, uint32_t now) {
  platLock();
  out->seen = within(g.fuelSeenMs, now, VEH_FUEL_TIMEOUT_MS);
  out->have = g.fuelHave;
  out->pct = g.fuelPct;
  out->lastB5 = g.fuelLastB5;
  out->ignored = g.fuelIgnored;
  platUnlock();
}

float lastRatio() {
  return g_lastRatio;
}

void lightInfo(LightInfo *out, uint32_t now) {
  platLock();
  lightSettle(now);
  out->have = g.lightSeenMs != 0;
  out->seen = within(g.lightSeenMs, now, VEH_LIGHT_TIMEOUT_MS);
  out->on = g.lightOn;
  out->lastB0 = g.lightLastB0;
  out->firstMs = g.lightFirstMs;
  out->changes = g.lightChanges;
  platUnlock();
}

}  // namespace veh

// ================================================================ bright(画面の明るさ。2.9.0)
namespace bright {

int normalPct(const Settings &s, bool lightOn) {
  return lightOn ? s.illumPct : s.brightnessPct;
}

int panelPct(uint8_t level, int normal) {
  return level == BL_OFF ? 0 : level == BL_MAX ? 100 : normal;
}

}  // namespace bright

// ================================================================ cantab
namespace cantab {

struct Entry {
  uint32_t id;
  uint8_t bus, ext, dlc;
  uint8_t cand;           // 候補のバイトの印(bit b = バイト b)。記録 0 回のときは使わない
  uint8_t data[8];
  uint8_t base[8];        // 前の記録の値
  uint16_t changes[8];    // 前の記録から変わった回数
  uint8_t hist[8][CAN_HIST_SAMPLES];
};

static Entry *g_e = nullptr;
static int g_n = 0;
static int g_histPos = 0, g_histCount = 0;
static uint32_t g_sampleMs = 0;
static int g_rec = 0;                    // 記録した回数
static int g_recHist[CAN_REC_HIST];      // 件数の移り変わり(新しいものが最後)
static int g_recHistN = 0;
static bool g_firstSeen = false;         // begin のあとにフレームを受けた(2.6.0)
static uint32_t g_firstMs = 0;           // 最初のフレームを受けた時刻(2.6.0。前後で比べるのは CAN_PP_SETTLE_MS たってから)

// ---- 前後で比べる(2.6.0)。前の記録は、この形のまま NVS に書く(PpHead のあとに PpRec が n 個)
struct PpHead {
  uint16_t magic;  // PP_MAGIC
  uint8_t ver;     // PP_VER
  uint8_t pad;
  uint16_t n;      // PpRec の数
  uint16_t pad2;
};
struct PpRec {
  uint32_t id;
  uint8_t bus, ext, dlc;
  uint8_t quiet;  // bit b = バイト b の幅が CAN_PP_QUIET 以下
  uint8_t md[8];  // 真ん中の値(幅が CAN_PP_QUIET 以下のバイトだけ)
};
static const uint16_t PP_MAGIC = 0x5050;
static const uint8_t PP_VER = 1;
static const size_t PP_BYTES = sizeof(PpHead) + sizeof(PpRec) * CAN_MAX_IDS;
static uint8_t *g_ppBuf = nullptr;  // 前の記録(PP_BYTES)
static bool g_ppHave = false;       // 前の記録がある
static PpResult g_ppRes[CAN_PP_MAX];
static int g_ppResN = 0, g_ppUp = 0, g_ppDown = 0;
static bool g_ppDone = false;       // 後を記録して、候補を作った

static bool ppLoad() {  // 保存した前の記録を読み戻す(begin から)
  if (!g_ppBuf) g_ppBuf = (uint8_t *)platAlloc(PP_BYTES);
  if (!g_ppBuf) return false;
  const size_t n = platLoadCanPre(g_ppBuf, PP_BYTES);
  const PpHead *h = (const PpHead *)g_ppBuf;
  return n >= sizeof(PpHead) && h->magic == PP_MAGIC && h->ver == PP_VER && h->n <= CAN_MAX_IDS && n == sizeof(PpHead) + sizeof(PpRec) * h->n;
}

void begin() {
  if (!g_e) g_e = (Entry *)platAlloc(sizeof(Entry) * CAN_MAX_IDS);
  g_n = 0;
  g_histPos = 0;
  g_histCount = 0;
  g_rec = 0;
  g_recHistN = 0;
  g_firstSeen = false;
  g_ppHave = ppLoad();
  g_ppDone = false;
  g_ppResN = g_ppUp = g_ppDown = 0;
}

void onFrame(uint8_t bus, uint8_t ext, uint32_t id, uint8_t dlc, const uint8_t *data, uint32_t nowMs) {
  if (!g_e) return;
  if (dlc > 8) dlc = 8;
  platLock();
  if (!g_firstSeen) {
    g_firstSeen = true;
    g_firstMs = nowMs;
  }
  int i = 0;
  while (i < g_n && !(g_e[i].id == id && g_e[i].bus == bus && g_e[i].ext == ext)) i++;
  if (i == g_n) {
    if (g_n >= CAN_MAX_IDS) {
      platUnlock();
      return;
    }
    Entry &e = g_e[g_n++];
    memset(&e, 0, sizeof(e));
    e.id = id;
    e.bus = bus;
    e.ext = ext;
    e.dlc = dlc;
    memcpy(e.data, data, dlc);
    memcpy(e.base, e.data, 8);  // 初めて見たときの値を基準にする
    e.cand = 0;                 // 記録を始めたあとに初めて見た ID は候補にしない
    for (int b = 0; b < 8; b++) memset(e.hist[b], e.data[b], CAN_HIST_SAMPLES);
  } else {
    Entry &e = g_e[i];
    for (int b = 0; b < dlc; b++) {
      if (e.data[b] != data[b]) {
        e.data[b] = data[b];
        if (e.changes[b] < 0xFFFF) e.changes[b]++;
      }
    }
    e.dlc = dlc;
  }
  platUnlock();
}

void tick(uint32_t nowMs) {
  if (!g_e || (uint32_t)(nowMs - g_sampleMs) < CAN_HIST_INTERVAL_MS) return;
  g_sampleMs = nowMs;
  platLock();
  for (int i = 0; i < g_n; i++) {
    for (int b = 0; b < 8; b++) g_e[i].hist[b][g_histPos] = g_e[i].data[b];
  }
  g_histPos = (g_histPos + 1) % CAN_HIST_SAMPLES;
  if (g_histCount < CAN_HIST_SAMPLES) g_histCount++;
  platUnlock();
}

int count() {
  return g_n;
}

static int candCountLocked() {
  int n = 0;
  for (int i = 0; i < g_n; i++) n += g_e[i].cand ? 1 : 0;
  return n;
}

int record() {
  if (!g_e) return 0;
  platLock();
  for (int i = 0; i < g_n; i++) {
    Entry &e = g_e[i];
    const uint8_t all = (uint8_t)((1u << e.dlc) - 1u);
    if (g_rec == 0) {
      e.cand = all;  // 1 回目: 全部が候補
    } else {
      uint8_t keep = 0;
      for (int b = 0; b < e.dlc; b++) {
        if ((e.cand >> b & 1) && e.data[b] != e.base[b] && e.changes[b] >= 1 && e.changes[b] <= CAN_REC_MAX_CHANGES) keep |= (uint8_t)(1u << b);
      }
      e.cand = keep;
    }
    memcpy(e.base, e.data, 8);
    memset(e.changes, 0, sizeof(e.changes));
  }
  g_rec++;
  int n = candCountLocked();
  if (g_recHistN == CAN_REC_HIST) {
    memmove(g_recHist, g_recHist + 1, sizeof(int) * (CAN_REC_HIST - 1));
    g_recHistN--;
  }
  g_recHist[g_recHistN++] = n;
  platUnlock();
  return n;
}

void clearRecord() {
  if (!g_e) return;
  platLock();
  for (int i = 0; i < g_n; i++) g_e[i].cand = 0;
  g_rec = 0;
  g_recHistN = 0;
  platUnlock();
}

int recordCount() {
  return g_rec;
}

int recordHistory(int *out) {
  platLock();
  int n = g_recHistN;
  memcpy(out, g_recHist, sizeof(int) * (size_t)n);
  platUnlock();
  return n;
}

int candidates(uint16_t *entries, uint8_t *masks, int maxOut) {
  if (!g_e || g_rec == 0) return 0;
  platLock();
  int n = 0;
  for (int i = 0; i < g_n; i++) {
    if (!g_e[i].cand) continue;
    if (n < maxOut) {
      entries[n] = (uint16_t)i;
      masks[n] = g_e[i].cand;
    }
    n++;
  }
  platUnlock();
  return n;
}

bool getView(int entry, View *out) {
  if (!g_e || entry < 0) return false;
  platLock();
  bool ok = entry < g_n;
  if (ok) {
    const Entry &e = g_e[entry];
    out->id = e.id;
    out->bus = e.bus;
    out->ext = e.ext;
    out->dlc = e.dlc;
    memcpy(out->data, e.data, 8);
    memcpy(out->base, e.base, 8);
  }
  platUnlock();
  return ok;
}

int getHistory(int entry, int byteIndex, uint8_t *out) {
  if (!g_e || entry < 0 || byteIndex < 0 || byteIndex > 7) return 0;
  platLock();
  int n = 0;
  if (entry < g_n) {
    n = g_histCount;
    int start = (g_histPos - n + CAN_HIST_SAMPLES) % CAN_HIST_SAMPLES;
    for (int i = 0; i < n; i++) out[i] = g_e[entry].hist[byteIndex][(start + i) % CAN_HIST_SAMPLES];
  }
  platUnlock();
  return n;
}

// ---- 前後で比べる(2.6.0)
static const int PP_WIN = CAN_PP_WIN_MS / CAN_HIST_INTERVAL_MS;  // 見る点の数
// 履歴の新しい方から PP_WIN 点で、バイトごとの幅が CAN_PP_QUIET 以下か(quiet の bit)と、そのバイトの真ん中の値(md)。platLock の中で呼ぶ
static void ppStats(const Entry &e, uint8_t *quiet, uint8_t *md) {
  *quiet = 0;
  for (int b = 0; b < 8; b++) {
    md[b] = 0;
    if (b >= e.dlc) continue;
    uint8_t lo = 255, hi = 0;
    for (int k = 0; k < PP_WIN; k++) {
      const uint8_t v = e.hist[b][(g_histPos - 1 - k + 2 * CAN_HIST_SAMPLES) % CAN_HIST_SAMPLES];
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
    if (hi - lo > CAN_PP_QUIET) continue;
    int cnt[CAN_PP_QUIET + 1] = {};
    for (int k = 0; k < PP_WIN; k++) cnt[e.hist[b][(g_histPos - 1 - k + 2 * CAN_HIST_SAMPLES) % CAN_HIST_SAMPLES] - lo]++;
    int acc = 0, v = 0;
    while (v < CAN_PP_QUIET && acc + cnt[v] <= PP_WIN / 2) acc += cnt[v++];  // 小さい方から PP_WIN / 2 番目(0 から数える)
    md[b] = (uint8_t)(lo + v);
    *quiet |= (uint8_t)(1u << b);
  }
}

int ppWaitMs(uint32_t nowMs) {
  if (!g_e || !g_firstSeen) return -1;
  const uint32_t el = nowMs - g_firstMs;
  int w = el >= (uint32_t)CAN_PP_SETTLE_MS ? 0 : (int)(CAN_PP_SETTLE_MS - el);
  const int hw = g_histCount >= PP_WIN ? 0 : (PP_WIN - g_histCount) * CAN_HIST_INTERVAL_MS;
  return w > hw ? w : hw;
}

bool ppRecordPre(uint32_t nowMs) {
  if (ppWaitMs(nowMs) != 0) return false;
  if (!g_ppBuf) g_ppBuf = (uint8_t *)platAlloc(PP_BYTES);
  if (!g_ppBuf) return false;
  PpHead *h = (PpHead *)g_ppBuf;
  PpRec *r = (PpRec *)(g_ppBuf + sizeof(PpHead));
  int n = 0;
  platLock();
  for (int i = 0; i < g_n && n < CAN_MAX_IDS; i++) {
    const Entry &e = g_e[i];
    PpRec &p = r[n++];
    memset(&p, 0, sizeof(p));
    p.id = e.id;
    p.bus = e.bus;
    p.ext = e.ext;
    p.dlc = e.dlc;
    ppStats(e, &p.quiet, p.md);
  }
  platUnlock();
  memset(h, 0, sizeof(*h));
  h->magic = PP_MAGIC;
  h->ver = PP_VER;
  h->n = (uint16_t)n;
  g_ppHave = true;
  g_ppDone = false;
  g_ppResN = g_ppUp = g_ppDown = 0;
  platSaveCanPre(g_ppBuf, sizeof(PpHead) + sizeof(PpRec) * (size_t)n);  // 書けなくても、電源を切るまでは使える
  return true;
}

bool ppHasPre() {
  return g_ppHave;
}

int ppRecordPost(uint32_t nowMs) {
  if (!g_ppHave || ppWaitMs(nowMs) != 0) return -1;
  const PpHead *h = (const PpHead *)g_ppBuf;
  const PpRec *r = (const PpRec *)(g_ppBuf + sizeof(PpHead));
  // 差の大きい方から CAN_PP_MAX 個を残す(best は |差| の大きい順)
  PpResult best[CAN_PP_MAX];
  int bestD[CAN_PP_MAX];
  int nb = 0, up = 0, down = 0;
  platLock();
  for (int i = 0; i < g_n; i++) {
    const Entry &e = g_e[i];
    const PpRec *p = nullptr;
    for (int k = 0; k < h->n && !p; k++)
      if (r[k].id == e.id && r[k].bus == e.bus && r[k].ext == e.ext) p = &r[k];
    if (!p) continue;
    uint8_t q, md[8];
    ppStats(e, &q, md);
    const int nbyte = e.dlc < p->dlc ? e.dlc : p->dlc;
    for (int b = 0; b < nbyte; b++) {
      if (!((p->quiet & q) >> b & 1)) continue;
      const int d = (int)md[b] - (int)p->md[b];
      const int ad = d < 0 ? -d : d;
      if (ad < CAN_PP_MIN_DIFF) continue;
      if (d > 0) up++;
      else down++;
      int at = nb;
      while (at > 0 && (bestD[at - 1] < 0 ? -bestD[at - 1] : bestD[at - 1]) < ad) at--;
      if (at >= CAN_PP_MAX) continue;
      const int last = nb < CAN_PP_MAX ? nb : CAN_PP_MAX - 1;
      for (int k = last; k > at; k--) {
        best[k] = best[k - 1];
        bestD[k] = bestD[k - 1];
      }
      best[at] = PpResult{ (uint16_t)i, (uint8_t)b, p->md[b], md[b] };
      bestD[at] = d;
      if (nb < CAN_PP_MAX) nb++;
    }
  }
  platUnlock();
  // 並べ方: 増えたもの(差の大きい順)のあとに減ったもの(差の大きい順)
  int k = 0;
  for (int pass = 0; pass < 2; pass++)
    for (int i = 0; i < nb; i++)
      if ((bestD[i] > 0) == (pass == 0)) g_ppRes[k++] = best[i];
  g_ppResN = nb;
  g_ppUp = up;
  g_ppDown = down;
  g_ppDone = true;
  return up + down;
}

bool ppHasResult() {
  return g_ppDone;
}

int ppResults(PpResult *out, int maxOut, int *up, int *down) {
  if (up) *up = g_ppUp;
  if (down) *down = g_ppDown;
  const int n = g_ppResN < maxOut ? g_ppResN : maxOut;
  for (int i = 0; i < n; i++) out[i] = g_ppRes[i];
  return g_ppResN;
}

void ppClear() {
  g_ppHave = false;
  g_ppDone = false;
  g_ppResN = g_ppUp = g_ppDown = 0;
  platSaveCanPre(nullptr, 0);
}

}  // namespace cantab

// ================================================================ trip
namespace trip {

static uint32_t g_lastMs = 0;
static bool g_started = false;
static uint32_t g_moving = 0, g_wot = 0;
static uint32_t g_above = 0, g_below = 0;

void update(const Signals &s, uint32_t nowMs, int rpm) {
  uint32_t dt = g_started ? (uint32_t)(nowMs - g_lastMs) : 0;
  g_started = true;
  g_lastMs = nowMs;
  if (dt > 500) dt = 500;  // 画面の切り替えなどで間が空いても、数えすぎない
  if (s.speedOk && s.speedKmh >= MOVING_SPEED_KMH) {
    g_moving += dt;
    if (s.throttleOk && s.throttlePct >= WOT_THROTTLE_PCT) g_wot += dt;
  }
  if (s.rpmOk) {  // 停止・走行に関係なく数える
    if (s.rpm >= rpm) g_above += dt;  // 2.12.0: 過負荷時間が増える所とそろえた(2.11.0 までは「超える」)
    else g_below += dt;
  }
}
uint32_t aboveMs() {
  return g_above;
}
uint32_t belowMs() {
  return g_below;
}
void setTimes(uint32_t above, uint32_t below) {
  g_above = above;
  g_below = below;
}
void resetTimes() {
  g_above = g_below = 0;
}
uint32_t movingMs() {
  return g_moving;
}
uint32_t wotMs() {
  return g_wot;
}

}  // namespace trip

// ================================================================ 設定の初期値(2.10.0。設定の「デフォルトに戻す」と起動時)
void settingsDefaults(Settings *s) {
  s->shiftRpm = SET_SHIFT_DEFAULT;
  s->autoReturnMin = SET_AUTORET_DEFAULT;
  s->brightnessPct = SET_BRIGHT_DEFAULT;
  s->warnTempC = SET_WARN_DEFAULT;
  s->demoOn = SET_DEMO_DEFAULT;
  s->rcOn = SET_RC_DEFAULT;
  s->illumPct = SET_ILLUM_DEFAULT;
  s->loadYellowMin = SET_LOADY_DEFAULT;
  s->loadRedMin = SET_LOADR_DEFAULT;
  s->loadResetMin = SET_LOADRS_DEFAULT;
  s->ringOne = SET_RING_DEFAULT;
  s->loadRpm = SET_LOADRPM_DEFAULT;
  s->maintHours = SET_MAINT_DEFAULT;
  s->fuelWarnPct = SET_FUELWARN_DEFAULT;  // 2.14.0
}

// ================================================================ load(高負荷時間。2.10.0。説明は app.h)
namespace load {

static uint32_t g_value = 0;       // 今の値 [ms]
static uint32_t g_lastMs = 0;      // 前に update を呼んだ時刻
static bool g_started = false;
static uint32_t g_lowSince = 0;    // 回転数が過負荷 回転数未満になった時刻(0 = 今は以上・信号なし)
static bool g_hot = false;         // リセットのあとに一度でも黄(黄が切なら赤)になった(2.12.0。クールダウンを出す条件)
static bool g_haveSaved = false;   // restore で保存した値を受けた
static uint32_t g_savedUtc = 0;    // 前の起動で最後に書いた時刻(0 = 分からなかった)
static uint32_t g_savedLow = 0;    // そのときの過負荷 回転数未満の経過 [ms]
static bool g_timeDone = false;    // 起動してから時刻を受けた(電源が切れていた時間を引くのは 1 回だけ)

static uint32_t minMs(int m) {
  return m > 0 ? (uint32_t)m * 60000UL : 0;
}
bool enabled(const Settings &set) {
  return set.loadYellowMin > 0 || set.loadRedMin > 0;
}
static void hotTrack(const Settings &set) {  // 2.12.0: 黄(黄が切なら赤)になったら覚え、0 になったら忘れる
  if (g_value == 0) g_hot = false;
  else if (level(set) >= 1) g_hot = true;
}

void update(const Signals &s, const Settings &set, uint32_t nowMs) {
  uint32_t dt = g_started ? (uint32_t)(nowMs - g_lastMs) : 0;
  g_started = true;
  g_lastMs = nowMs;
  if (dt > 500) dt = 500;  // 画面の切り替えなどで間が空いても、数えすぎない(trip と同じ)
  // 過負荷 回転数未満が続いている時間(リセットの数え。▼ 3000rpm の経過時間と同じ数え方。信号なしは 0 から)。2.12.0: 設定の過負荷 回転数。
  // 境目は過負荷時間が増える所(以上)とそろえた(2.11.0 までは「以下」で、ちょうどのときは増えながらリセットも数えていた)
  if (s.rpmOk && s.rpm < set.loadRpm) {
    if (!g_lowSince) g_lowSince = nowMs ? nowMs : 1;
  } else {
    g_lowSince = 0;
  }
  if (!enabled(set)) {  // 機能が切: 数えず 0
    g_value = 0;
    g_hot = false;
    return;
  }
  if (!s.rpmOk) return;  // 信号がない間は変わらない
  if (s.rpm >= set.loadRpm) g_value = g_value + dt > LOAD_MAX_MS ? LOAD_MAX_MS : g_value + dt;
  else g_value = g_value > dt ? g_value - dt : 0;
  if (g_lowSince && (uint32_t)(nowMs - g_lowSince) >= minMs(set.loadResetMin)) g_value = 0;  // リセット
  hotTrack(set);
}
uint32_t valueMs() {
  return g_value;
}
uint8_t level(const Settings &set) {
  // 0 の色は使わない。黄 ≥ 赤 のときは赤を優先(黄は出ない)
  if (set.loadRedMin > 0 && g_value >= minMs(set.loadRedMin)) return 2;
  if (set.loadYellowMin > 0 && g_value >= minMs(set.loadYellowMin)) return 1;
  return 0;
}
bool warned(const Settings &set) {  // 黄になったのを覚えるのは update(設定を変えた直後は、次の update の前でも色で分かる)
  return g_value > 0 && (g_hot || level(set) >= 1);
}
bool coolDown(uint32_t nowMs) {
  return g_hot && g_value > 0 && lowMs(nowMs) >= LOAD_COOL_SHOW_MS;
}
uint32_t coolLeftMs(const Settings &set, uint32_t nowMs) {
  const uint32_t r = minMs(set.loadResetMin), l = lowMs(nowMs);
  return l >= r ? 0 : r - l;
}
uint32_t lowMs(uint32_t nowMs) {
  return g_lowSince ? (uint32_t)(nowMs - g_lowSince) : 0;
}
void restore(uint32_t valueMs, uint32_t savedUtc, uint32_t lowMs) {
  g_value = valueMs > LOAD_MAX_MS ? LOAD_MAX_MS : valueMs;
  g_savedUtc = savedUtc;
  g_savedLow = lowMs;
  g_haveSaved = true;
  g_timeDone = false;
}
void onTime(uint32_t utc, const Settings &set, uint32_t nowMs) {
  if (g_timeDone) return;
  g_timeDone = true;
  if (!g_haveSaved || g_savedUtc == 0 || !enabled(set)) return;  // 前の起動で時刻が分からなかった: 引かない
  // 起動した時刻(今の時刻 − 起動してからの時間)と、前の起動で最後に書いた時刻の差 = 電源が切れていた時間(最大 1 分多い)
  const int64_t boot = (int64_t)utc - (int64_t)(nowMs / 1000);
  const int64_t offS = boot - (int64_t)g_savedUtc;
  if (offS <= 0) return;  // 差が負: 引かない
  const uint64_t offMs = (uint64_t)offS * 1000ULL;
  if ((uint64_t)g_savedLow + offMs >= (uint64_t)minMs(set.loadResetMin)) g_value = 0;  // 切る前の過負荷 回転数以下 + 切れていた時間でリセット
  else g_value = (uint64_t)g_value > offMs ? g_value - (uint32_t)offMs : 0;
  hotTrack(set);
}
void capMs(uint32_t ms) {
  if (g_value > ms) g_value = ms;
  if (g_value == 0) g_hot = false;
}
void testSet(uint32_t valueMs, bool hot) {
  g_value = valueMs > LOAD_MAX_MS ? LOAD_MAX_MS : valueMs;
  g_hot = hot && g_value > 0;
  g_lowSince = 0;
  g_started = false;
}
void testReset() {
  g_value = 0;
  g_hot = false;
  g_lowSince = 0;
  g_started = false;
  g_haveSaved = false;
  g_savedUtc = g_savedLow = 0;
  g_timeDone = false;
}

}  // namespace load

// ================================================================ perf(本体の状態の「FPS」。2.3.0。2.4.3: 1 秒ごと)
namespace perf {

// 2.4.3(本人「過去数分の FPS の値がここに出るほうがいい」): 1 秒ごとに 回数・時間の合計 をまとめる。回を数えた秒だけを覚えるので、
// 設定などを出していた間は飛ばして、サーキット・タコを出していた直近 PERF_FPS_SECS 秒になる
struct Sec {
  uint32_t frames;
  uint32_t sumUs;
};
static Sec g_sec[PERF_FPS_SECS];  // 閉じた秒(古い方から g_n 個。g_pos が次に書く所)
static int g_n = 0, g_pos = 0;
static bool g_open = false;       // 今の秒を数えている
static uint32_t g_openSec = 0;    // 今の秒(ms / 1000)
static Sec g_cur;

void pageAdd(uint32_t us, uint32_t nowMs) {
  if (!us) us = 1;
  const uint32_t sec = nowMs / 1000;
  if (g_open && sec != g_openSec) {  // 前の秒を閉じる
    g_sec[g_pos] = g_cur;
    g_pos = (g_pos + 1) % PERF_FPS_SECS;
    if (g_n < PERF_FPS_SECS) g_n++;
    g_open = false;
  }
  if (!g_open) {
    g_open = true;
    g_openSec = sec;
    g_cur.frames = 0;
    g_cur.sumUs = 0;
  }
  g_cur.frames++;
  g_cur.sumUs += us;
}
// 覚えている秒(数えている今の秒も入れて、新しい方から PERF_FPS_SECS 秒)から入れる。平均 = 全部の回の時間の平均、
// 最低 = 1 秒ごとの平均の時間が一番長い秒、fpsHist = 1 秒ごとの回/秒(古い方から)
void pageStats(PerfStats *ps) {
  ps->pageFrames = 0;
  ps->pageAvgUs = ps->pageSlowUs = 0;
  ps->histN = 0;
  const int total = g_n + (g_open ? 1 : 0);
  const int use = total > PERF_FPS_SECS ? PERF_FPS_SECS : total;
  if (!use) return;
  const int skip = total - use;  // 古い方から捨てる数(今の秒があって、閉じた秒が満杯のとき 1)
  uint64_t frames = 0, sum = 0;
  uint32_t slow = 0;
  for (int i = skip; i < total; i++) {
    const Sec &x = i < g_n ? g_sec[(g_pos - g_n + i + PERF_FPS_SECS) % PERF_FPS_SECS] : g_cur;
    frames += x.frames;
    sum += x.sumUs;
    const uint32_t avg = (uint32_t)((x.sumUs + x.frames / 2) / x.frames);
    if (avg > slow) slow = avg;
    const uint32_t fps = avg ? (1000000u + avg / 2) / avg : 255;
    ps->fpsHist[ps->histN++] = (uint8_t)(fps > 255 ? 255 : fps);
  }
  ps->pageFrames = (uint32_t)frames;
  ps->pageAvgUs = (uint32_t)((sum + frames / 2) / frames);
  ps->pageSlowUs = slow;
}
void pageClear() {
  g_n = g_pos = 0;
  g_open = false;
}

}  // namespace perf

// ================================================================ gnss
namespace gnss {

static Fix g_fix;
static bool g_have = false;
static uint32_t g_pvt = 0, g_ack = 0, g_nak = 0, g_bad = 0;
// 2.13.0: OTP の高性能設定のための受信(UBX-MON-VER の版、UBX-CFG-VALGET の応答、CFG-OTP(0x06 0x41)と VALGET の ACK/NAK の数)
static char g_monSw[31], g_monHw[11];
static uint32_t g_monVerCount = 0, g_valgetCount = 0, g_ackOtp = 0, g_nakOtp = 0, g_nakValget = 0;
static uint8_t g_valget[100];
static uint16_t g_valgetLen = 0;
// 組み立て中のメッセージ
static int g_state = 0;  // 0 = 同期 1 待ち、1 = 同期 2 待ち、2 = class、3 = id、4 = len L、5 = len H、6 = 中身、7 = ck A、8 = ck B
static uint8_t g_cls = 0, g_id = 0, g_ckA = 0, g_ckB = 0;
static uint16_t g_plen = 0, g_pos = 0;
static uint8_t g_pay[100];  // NAV-PVT(92 バイト)が入る。長いものは中身を捨てて読み飛ばす
static const uint16_t MAX_SKIP = 1024;

void begin() {
  memset(&g_fix, 0, sizeof(g_fix));
  g_have = false;
  g_pvt = g_ack = g_nak = g_bad = 0;
  g_state = 0;
  g_monSw[0] = g_monHw[0] = 0;
  g_monVerCount = g_valgetCount = g_ackOtp = g_nakOtp = g_nakValget = 0;
  g_valgetLen = 0;
}

static inline uint16_t u16(const uint8_t *p) {
  return (uint16_t)(p[0] | (p[1] << 8));
}
static inline uint32_t u32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline int32_t i32(const uint8_t *p) {
  return (int32_t)u32(p);
}

// NAV-PVT の中身(u-blox の仕様。SparkFun の UBX_NAV_PVT_data_t と同じ並び)
static void decodePvt(const uint8_t *d, uint32_t nowMs) {
  Fix f;
  memset(&f, 0, sizeof(f));
  f.iTOW = u32(d + 0);
  f.year = u16(d + 4);
  f.month = d[6];
  f.day = d[7];
  f.hour = d[8];
  f.min = d[9];
  f.sec = d[10];
  uint8_t valid = d[11];
  f.timeOk = (valid & 0x03) == 0x03;  // validDate && validTime
  f.nano = i32(d + 16);
  f.fixType = d[20];
  uint8_t flags = d[21];
  f.fixOk = (flags & 0x01) && (f.fixType == 2 || f.fixType == 3 || f.fixType == 4);
  f.diffSoln = (flags & 0x02) != 0;
  f.numSV = d[23];
  f.lon = i32(d + 24);
  f.lat = i32(d + 28);
  f.hMSL = i32(d + 36);
  f.gSpeed = i32(d + 60);
  f.headMot = i32(d + 64);
  f.pDOP = u16(d + 76);
  f.rxMs = nowMs;
  g_fix = f;
  g_have = true;
  g_pvt++;
}

int feed(const uint8_t *data, size_t n, uint32_t nowMs) {
  int got = 0;
  for (size_t i = 0; i < n; i++) {
    uint8_t b = data[i];
    switch (g_state) {
      case 0:
        if (b == 0xB5) g_state = 1;
        break;
      case 1:
        g_state = (b == 0x62) ? 2 : (b == 0xB5 ? 1 : 0);
        break;
      case 2:
        g_cls = b;
        g_ckA = b;
        g_ckB = b;
        g_state = 3;
        break;
      case 3:
        g_id = b;
        g_ckA = (uint8_t)(g_ckA + b);
        g_ckB = (uint8_t)(g_ckB + g_ckA);
        g_state = 4;
        break;
      case 4:
        g_plen = b;
        g_ckA = (uint8_t)(g_ckA + b);
        g_ckB = (uint8_t)(g_ckB + g_ckA);
        g_state = 5;
        break;
      case 5:
        g_plen |= (uint16_t)(b << 8);
        g_ckA = (uint8_t)(g_ckA + b);
        g_ckB = (uint8_t)(g_ckB + g_ckA);
        g_pos = 0;
        if (g_plen > MAX_SKIP) {
          g_state = 0;  // ありえない長さ: 同期を取り直す
        } else {
          g_state = g_plen ? 6 : 7;
        }
        break;
      case 6:
        if (g_pos < sizeof(g_pay)) g_pay[g_pos] = b;
        g_pos++;
        g_ckA = (uint8_t)(g_ckA + b);
        g_ckB = (uint8_t)(g_ckB + g_ckA);
        if (g_pos >= g_plen) g_state = 7;
        break;
      case 7:
        g_state = (b == g_ckA) ? 8 : 9;
        break;
      case 8:
      case 9:
        if (g_state == 8 && b == g_ckB) {
          if (g_cls == 0x01 && g_id == 0x07 && g_plen == 92) {
            decodePvt(g_pay, nowMs);
            got++;
          } else if (g_cls == 0x05 && g_id == 0x01) {
            g_ack++;
            if (g_plen == 2 && g_pay[0] == 0x06 && g_pay[1] == 0x41) g_ackOtp++;  // 2.13.0: CFG-OTP の ACK
          } else if (g_cls == 0x05 && g_id == 0x00) {
            g_nak++;
            if (g_plen == 2 && g_pay[0] == 0x06 && g_pay[1] == 0x41) g_nakOtp++;
            if (g_plen == 2 && g_pay[0] == 0x06 && g_pay[1] == 0x8B) g_nakValget++;
          } else if (g_cls == 0x0A && g_id == 0x04 && g_plen >= 40) {  // 2.13.0: MON-VER(swVersion 30 文字 + hwVersion 10 文字 + 拡張。拡張は使わない)
            memcpy(g_monSw, g_pay, 30);
            g_monSw[30] = 0;
            memcpy(g_monHw, g_pay + 30, 10);
            g_monHw[10] = 0;
            g_monVerCount++;
          } else if (g_cls == 0x06 && g_id == 0x8B && g_plen >= 4) {  // 2.13.0: CFG-VALGET の応答(版・層・位置 + 鍵と値の並び)
            g_valgetLen = g_plen < sizeof(g_pay) ? g_plen : (uint16_t)sizeof(g_pay);
            memcpy(g_valget, g_pay, g_valgetLen);
            g_valgetCount++;
          }
        } else {
          g_bad++;
        }
        g_state = 0;
        break;
      default:
        g_state = 0;
        break;
    }
  }
  return got;
}

bool latest(Fix *out) {
  if (!g_have) return false;
  *out = g_fix;
  return true;
}
uint32_t pvtCount() {
  return g_pvt;
}
uint32_t ackCount() {
  return g_ack;
}
uint32_t nakCount() {
  return g_nak;
}
uint32_t badCount() {
  return g_bad;
}
// ---- 2.13.0: M10 の OTP の高性能設定(SAM-M10Q 統合マニュアル 2.1.5。本人 10/29 の決定: GPS + Galileo で 20 回/秒)
// マニュアル Table 3 の 2 つの列(UBX-CFG-OTP、0x06 0x41。CPU のクロックを上げる設定を OTP に書く。永久)と、確かめる VALGET の問い合わせ。
// チェックサムは 10/29 に計算して照合した
static const uint8_t OTP_HP1[] = { 0xB5, 0x62, 0x06, 0x41, 0x10, 0x00, 0x03, 0x00, 0x04, 0x1F, 0x54, 0x5E, 0x79, 0xBF, 0x28, 0xEF, 0x12, 0x05, 0xFD, 0xFF, 0xFF, 0xFF, 0x8F, 0x0D };
static const uint8_t OTP_HP2[] = { 0xB5, 0x62, 0x06, 0x41, 0x1C, 0x00, 0x04, 0x01, 0xA4, 0x10, 0xBD, 0x34, 0xF9, 0x12, 0x28, 0xEF, 0x12, 0x05, 0x05, 0x00, 0xA4, 0x40,
                                   0x00, 0xB0, 0x71, 0x0B, 0x0A, 0x00, 0xA4, 0x40, 0x00, 0xD8, 0xB8, 0x05, 0xDE, 0xAE };
static const uint8_t OTP_VALGET_POLL[] = { 0xB5, 0x62, 0x06, 0x8B, 0x14, 0x00, 0x00, 0x04, 0x00, 0x00, 0x01, 0x00, 0xA4, 0x40, 0x03, 0x00, 0xA4, 0x40,
                                           0x05, 0x00, 0xA4, 0x40, 0x0A, 0x00, 0xA4, 0x40, 0x4C, 0x15 };
static const uint32_t OTP_KEYS[4] = { 0x40A40001, 0x40A40003, 0x40A40005, 0x40A4000A };
static const uint32_t OTP_VALUES[4] = { 0x0B71B000, 0x0B71B000, 0x0B71B000, 0x05B8D800 };  // 192 MHz × 3、96 MHz
size_t buildOtpHighPerf(int part, uint8_t *out, size_t cap) {
  const uint8_t *src = part == 0 ? OTP_HP1 : OTP_HP2;
  const size_t n = part == 0 ? sizeof(OTP_HP1) : sizeof(OTP_HP2);
  if (cap < n) return 0;
  memcpy(out, src, n);
  return n;
}
size_t buildOtpValgetPoll(uint8_t *out, size_t cap) {
  if (cap < sizeof(OTP_VALGET_POLL)) return 0;
  memcpy(out, OTP_VALGET_POLL, sizeof(OTP_VALGET_POLL));
  return sizeof(OTP_VALGET_POLL);
}
static void copyStr(char *dst, size_t cap, const char *src) {
  if (!cap) return;
  size_t i = 0;
  for (; i + 1 < cap && src[i]; i++) dst[i] = src[i];
  dst[i] = 0;
}
bool monVer(char *sw, size_t swCap, char *hw, size_t hwCap) {
  if (!g_monVerCount) return false;
  copyStr(sw, swCap, g_monSw);
  copyStr(hw, hwCap, g_monHw);
  return true;
}
uint32_t monVerCount() {
  return g_monVerCount;
}
uint32_t valgetCount() {
  return g_valgetCount;
}
uint32_t ackOtpCount() {
  return g_ackOtp;
}
uint32_t nakOtpCount() {
  return g_nakOtp;
}
uint32_t nakValgetCount() {
  return g_nakValget;
}
bool valgetValue(uint32_t key, uint32_t *val) {
  size_t pos = 4;  // 版(1)・層(1)・位置(2)の次から鍵と値
  while (pos + 4 <= g_valgetLen) {
    uint32_t k = u32(g_valget + pos);
    int size = (int)((k >> 28) & 7);
    int n = (size == 3) ? 2 : (size == 4) ? 4 : (size == 5) ? 8 : 1;
    if (pos + 4 + (size_t)n > g_valgetLen) return false;
    if (k == key) {
      uint32_t v = 0;
      for (int i = 0; i < n && i < 4; i++) v |= (uint32_t)g_valget[pos + 4 + i] << (8 * i);
      *val = v;
      return true;
    }
    pos += 4 + (size_t)n;
  }
  return false;
}
int otpHighPerfState() {
  if (!g_valgetCount) return 0;
  for (int i = 0; i < 4; i++) {
    uint32_t v;
    if (!valgetValue(OTP_KEYS[i], &v)) return 2;
    if (v != OTP_VALUES[i]) return 2;
  }
  return 1;
}

// UBX の枠(同期 2 + class + id + 長さ 2 + 中身 + チェックサム 2)を作る
static size_t ubxFrame(uint8_t *out, size_t cap, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  size_t total = (size_t)len + 8;
  if (cap < total) return 0;
  out[0] = 0xB5;
  out[1] = 0x62;
  out[2] = cls;
  out[3] = id;
  out[4] = (uint8_t)(len & 0xFF);
  out[5] = (uint8_t)(len >> 8);
  if (len) memcpy(out + 6, payload, len);  // 2.13.0: 中身なし(MON-VER の問い合わせ)も作れるように
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < 6 + (size_t)len; i++) {
    a = (uint8_t)(a + out[i]);
    b = (uint8_t)(b + a);
  }
  out[6 + len] = a;
  out[7 + len] = b;
  return total;
}

// CFG-VALSET の中身に「鍵 + 値」を足す。値の大きさは鍵の上位 3 ビット(1 = 1 バイト、2 = 1、3 = 2、4 = 4、5 = 8)
static size_t putKey(uint8_t *p, uint32_t key, uint32_t val) {
  p[0] = (uint8_t)key;
  p[1] = (uint8_t)(key >> 8);
  p[2] = (uint8_t)(key >> 16);
  p[3] = (uint8_t)(key >> 24);
  int size = (int)((key >> 28) & 7);
  int n = (size == 3) ? 2 : (size == 4) ? 4 : 1;
  for (int i = 0; i < n; i++) p[4 + i] = (uint8_t)(val >> (8 * i));
  return 4 + (size_t)n;
}

// 鍵の番号は u-blox の設定一覧(SparkFun の u-blox_config_keys.h で照合済み)
static const uint32_t KEY_RATE_MEAS = 0x30210001, KEY_RATE_NAV = 0x30210002, KEY_MSGOUT_PVT_UART1 = 0x20910007;
static const uint32_t KEY_UART1OUT_UBX = 0x10740001, KEY_UART1OUT_NMEA = 0x10740002, KEY_UART1IN_UBX = 0x10730001;
static const uint32_t KEY_DYNMODEL = 0x20110021, KEY_UART1_BAUD = 0x40520001;
static const uint32_t KEY_GPS = 0x1031001f, KEY_GPS_L1CA = 0x10310001, KEY_GAL = 0x10310021, KEY_GAL_E1 = 0x10310007;
static const uint32_t KEY_QZSS = 0x10310024, KEY_QZSS_L1CA = 0x10310012, KEY_BDS = 0x10310022, KEY_GLO = 0x10310025, KEY_SBAS = 0x10310020;

size_t buildConfig(uint8_t *out, size_t cap, int rateHz) {
  uint8_t pay[96];
  size_t n = 0;
  pay[n++] = 0x00;  // version 0
  pay[n++] = (uint8_t)RC_GPS_CFG_LAYERS;  // 層(bit0 = RAM、bit1 = BBR、bit2 = フラッシュ)
  pay[n++] = 0x00;  // reserved
  pay[n++] = 0x00;
  int meas = rateHz > 0 ? 1000 / rateHz : 1000;
  if (meas < 40) meas = 40;
  n += putKey(pay + n, KEY_RATE_MEAS, (uint32_t)meas);
  n += putKey(pay + n, KEY_RATE_NAV, 1);
  n += putKey(pay + n, KEY_MSGOUT_PVT_UART1, 1);
  n += putKey(pay + n, KEY_UART1OUT_UBX, 1);
  n += putKey(pay + n, KEY_UART1OUT_NMEA, 0);
  n += putKey(pay + n, KEY_UART1IN_UBX, 1);
  n += putKey(pay + n, KEY_DYNMODEL, 4);  // 4 = automotive
  return ubxFrame(out, cap, 0x06, 0x8A, pay, (uint16_t)n);
}

size_t buildSignals(uint8_t *out, size_t cap) {
  uint8_t pay[96];
  size_t n = 0;
  pay[n++] = 0x00;
  pay[n++] = (uint8_t)RC_GPS_CFG_LAYERS;
  pay[n++] = 0x00;
  pay[n++] = 0x00;
  n += putKey(pay + n, KEY_GPS, 1);
  n += putKey(pay + n, KEY_GPS_L1CA, 1);
  n += putKey(pay + n, KEY_GAL, 1);  // GPS + Galileo(本人 10/29 の決定。u-blox M10 の測位の上限は、既定で 10 回/秒、OTP の高性能設定で 20 回/秒)
  n += putKey(pay + n, KEY_GAL_E1, 1);
  n += putKey(pay + n, KEY_QZSS, 1);
  n += putKey(pay + n, KEY_QZSS_L1CA, 1);
  n += putKey(pay + n, KEY_BDS, 0);
  n += putKey(pay + n, KEY_GLO, 0);
  n += putKey(pay + n, KEY_SBAS, 0);
  return ubxFrame(out, cap, 0x06, 0x8A, pay, (uint16_t)n);
}

size_t buildMonVerPoll(uint8_t *out, size_t cap) {  // 2.13.0: UBX-MON-VER の問い合わせ(中身なし)
  return ubxFrame(out, cap, 0x0A, 0x04, nullptr, 0);
}
size_t buildReset(uint8_t *out, size_t cap) {  // 2.13.0: UBX-CFG-RST = ハードウェアリセット(navBbrMask 0 = ホットスタート)。OTP の設定は再起動で効く
  const uint8_t pay[4] = { 0x00, 0x00, 0x00, 0x00 };
  return ubxFrame(out, cap, 0x06, 0x04, pay, 4);
}
size_t buildBaud(uint8_t *out, size_t cap, uint32_t baud) {
  uint8_t pay[16];
  size_t n = 0;
  pay[n++] = 0x00;
  pay[n++] = (uint8_t)RC_GPS_CFG_LAYERS;
  pay[n++] = 0x00;
  pay[n++] = 0x00;
  n += putKey(pay + n, KEY_UART1_BAUD, baud);
  return ubxFrame(out, cap, 0x06, 0x8A, pay, (uint16_t)n);
}

}  // namespace gnss

// ================================================================ gtime(今の時刻。2.10.0。説明は app.h)
namespace gtime {

static bool g_ok = false;
static uint32_t g_utc = 0;  // 合わせた時刻(秒の始まり)
static uint32_t g_ms = 0;   // そのときの millis

uint32_t epoch(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
  // 1970-01-01 からの日数(グレゴリオ暦。3 月始まりの年で数える)
  const int y = (int)year - (month <= 2 ? 1 : 0);
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153u * (unsigned)(month > 2 ? month - 3 : month + 9) + 2u) / 5u + day - 1u;
  const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
  const int64_t days = (int64_t)era * 146097 + (int64_t)doe - 719468;
  return (uint32_t)(days * 86400 + (int64_t)hour * 3600 + (int64_t)min * 60 + sec);
}
void onFix(const gnss::Fix &f) {
  if (!f.timeOk || f.year < 2020 || f.month < 1 || f.month > 12 || f.day < 1 || f.day > 31 || f.hour > 23 || f.min > 59 || f.sec > 60) return;
  g_utc = epoch(f.year, f.month, f.day, f.hour, f.min, f.sec);
  g_ms = f.rxMs - (uint32_t)(f.nano / 1000000);  // 秒の始まりの millis(nano は負もある。受けるまでの遅れは見ない)
  g_ok = true;
}
uint32_t demoBase(const char *date, const char *time) {
  static const char MON[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  if (!date || !time || strlen(date) != 11 || strlen(time) != 8 || time[2] != ':' || time[5] != ':') return 0;
  int mon = 0;
  for (int i = 0; i < 12; i++)
    if (strncmp(date, MON + i * 3, 3) == 0) mon = i + 1;
  const int day = atoi(date + 4), year = atoi(date + 7), hh = atoi(time), mm = atoi(time + 3), ss = atoi(time + 6);
  if (mon == 0 || day < 1 || day > 31 || year < 2020 || year > 2099 || hh > 23 || mm > 59 || ss > 59) return 0;
  return epoch((uint16_t)year, (uint8_t)mon, (uint8_t)day, (uint8_t)hh, (uint8_t)mm, (uint8_t)ss) - (uint32_t)CLOCK_TZ_SEC;
}
bool now(uint32_t nowMs, uint32_t *utc) {
  if (!g_ok) return false;
  const int32_t d = (int32_t)(nowMs - g_ms);
  *utc = d >= 0 ? g_utc + (uint32_t)d / 1000u : g_utc - (uint32_t)(-d + 999) / 1000u;
  return true;
}
void testReset() {
  g_ok = false;
  g_utc = g_ms = 0;
}

}  // namespace gtime

// ================================================================ rcdiy
namespace rcdiy {

static inline int32_t clampI(int32_t v, int32_t lo, int32_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

uint32_t dateHourCode(const gnss::Fix &f) {
  int y = (int)f.year - 2000;
  if (y < 0) y = 0;
  int mo = f.month ? f.month - 1 : 0, d = f.day ? f.day - 1 : 0;
  return (uint32_t)(y * 8928 + mo * 744 + d * 24 + f.hour) & 0x1FFFFF;
}

void encodeGpsMain(const gnss::Fix &f, uint8_t sync, uint8_t out[20]) {
  // 時刻: 正時からの経過 = 分 × 30000 + 秒 × 500 + ms / 2(21 ビット)
  int32_t ms = (int32_t)f.min * 60000 + (int32_t)f.sec * 1000 + f.nano / 1000000;
  if (ms < 0) ms = 0;
  if (ms > 3599999) ms = 3599999;
  uint32_t t = (uint32_t)(ms / 2) & 0x1FFFFF;
  out[0] = (uint8_t)(((sync & 7) << 5) | ((t >> 16) & 0x1F));
  out[1] = (uint8_t)(t >> 8);
  out[2] = (uint8_t)t;
  // fix の種類(NMEA GGA の quality と同じ: 0 = なし、1 = GPS、2 = 補正あり)と衛星数
  uint8_t q = f.fixOk ? (f.diffSoln ? 2 : 1) : 0;
  uint8_t sv = f.numSV > 0x3E ? 0x3E : f.numSV;
  out[3] = (uint8_t)((q << 6) | (sv & 0x3F));
  uint32_t lat = f.fixOk ? (uint32_t)f.lat : 0x7FFFFFFFu, lon = f.fixOk ? (uint32_t)f.lon : 0x7FFFFFFFu;
  out[4] = (uint8_t)(lat >> 24);
  out[5] = (uint8_t)(lat >> 16);
  out[6] = (uint8_t)(lat >> 8);
  out[7] = (uint8_t)lat;
  out[8] = (uint8_t)(lon >> 24);
  out[9] = (uint8_t)(lon >> 16);
  out[10] = (uint8_t)(lon >> 8);
  out[11] = (uint8_t)lon;
  uint32_t alt = 0xFFFF, spd = 0xFFFF, brg = 0xFFFF;
  if (f.fixOk) {
    float m = f.hMSL / 1000.0f;
    if (m > 2776.7f) alt = ((uint32_t)clampI((int32_t)lroundf(m + 500.0f), 0, 0x7FFF) & 0x7FFF) | 0x8000;
    else alt = (uint32_t)clampI((int32_t)lroundf((m + 500.0f) * 10.0f), 0, 0x7FFF) & 0x7FFF;
    float kmh = f.gSpeed * 0.0036f;
    if (kmh > 327.67f) spd = ((uint32_t)clampI((int32_t)lroundf(kmh * 10.0f), 0, 0x7FFF) & 0x7FFF) | 0x8000;
    else spd = (uint32_t)clampI((int32_t)lroundf(kmh * 100.0f), 0, 0x7FFF) & 0x7FFF;
    int32_t deg100 = f.headMot / 1000;  // 1e-5 度 → 0.01 度
    while (deg100 < 0) deg100 += 36000;
    while (deg100 >= 36000) deg100 -= 36000;
    brg = (uint32_t)deg100;
  }
  out[12] = (uint8_t)(alt >> 8);
  out[13] = (uint8_t)alt;
  out[14] = (uint8_t)(spd >> 8);
  out[15] = (uint8_t)spd;
  out[16] = (uint8_t)(brg >> 8);
  out[17] = (uint8_t)brg;
  // HDOP の欄には pDOP(NAV-PVT には hDOP が無い。pDOP ≥ hDOP なので控えめな値になる)
  uint32_t dop = f.fixOk ? (uint32_t)clampI((f.pDOP + 5) / 10, 0, 0xFE) : 0xFF;
  out[18] = (uint8_t)dop;
  out[19] = 0xFF;  // VDOP: なし
}

void encodeGpsTime(const gnss::Fix &f, uint8_t sync, uint8_t out[3]) {
  uint32_t c = dateHourCode(f);
  out[0] = (uint8_t)(((sync & 7) << 5) | ((c >> 16) & 0x1F));
  out[1] = (uint8_t)(c >> 8);
  out[2] = (uint8_t)c;
}

size_t encodeCan(uint32_t id, const uint8_t *d, uint8_t dlc, uint8_t out[20]) {
  if (dlc > 16) dlc = 16;
  out[0] = (uint8_t)id;
  out[1] = (uint8_t)(id >> 8);
  out[2] = (uint8_t)(id >> 16);
  out[3] = (uint8_t)(id >> 24);
  memcpy(out + 4, d, dlc);
  return 4 + (size_t)dlc;
}

// ---- PID フィルタ
static const int MAX_PIDS = 48;
static bool g_allowAll = false;
static uint16_t g_allInterval = 0;
static int g_n = 0;
static uint32_t g_pid[MAX_PIDS];
static uint16_t g_interval[MAX_PIDS];
static uint32_t g_lastMs[MAX_PIDS];
// 全部許可のときの PID ごとの最終送信時刻(小さい表。あふれたら間隔を見ずに送る)
static uint32_t g_allPid[MAX_PIDS], g_allMs[MAX_PIDS];
static int g_allN = 0;

void filterReset() {
  g_allowAll = false;
  g_allInterval = 0;
  g_n = 0;
  g_allN = 0;
}

void filterCommand(const uint8_t *data, size_t len) {
  if (len < 1) return;
  uint8_t cmd = data[0];
  if (cmd == 0) {  // 全部拒否
    g_allowAll = false;
    g_n = 0;
  } else if (cmd == 1 && len >= 3) {  // 全部許可
    g_allowAll = true;
    g_allInterval = (uint16_t)((data[1] << 8) | data[2]);
    g_n = 0;
    g_allN = 0;
  } else if (cmd == 2 && len >= 7) {  // この PID を許可
    uint16_t iv = (uint16_t)((data[1] << 8) | data[2]);
    uint32_t pid = ((uint32_t)data[3] << 24) | ((uint32_t)data[4] << 16) | ((uint32_t)data[5] << 8) | data[6];
    for (int i = 0; i < g_n; i++) {
      if (g_pid[i] == pid) {
        g_interval[i] = iv;
        return;
      }
    }
    if (g_n < MAX_PIDS) {
      g_pid[g_n] = pid;
      g_interval[g_n] = iv;
      g_lastMs[g_n] = 0;
      g_n++;
    }
  }
}

bool filterPass(uint32_t pid, uint32_t nowMs, uint16_t minMs) {
  if (g_allowAll) {
    const uint16_t iv = g_allInterval > minMs ? g_allInterval : minMs;
    if (iv == 0) return true;
    for (int i = 0; i < g_allN; i++) {
      if (g_allPid[i] == pid) {
        if ((uint32_t)(nowMs - g_allMs[i]) < iv) return false;
        g_allMs[i] = nowMs;
        return true;
      }
    }
    if (g_allN < MAX_PIDS) {
      g_allPid[g_allN] = pid;
      g_allMs[g_allN] = nowMs;
      g_allN++;
    }
    return true;
  }
  for (int i = 0; i < g_n; i++) {
    if (g_pid[i] != pid) continue;
    const uint16_t iv = g_interval[i] > minMs ? g_interval[i] : minMs;
    if (iv && g_lastMs[i] && (uint32_t)(nowMs - g_lastMs[i]) < iv) return false;
    g_lastMs[i] = nowMs;
    return true;
  }
  return false;
}

int filterCount() {
  return g_allowAll ? -1 : g_n;
}

bool filterAllowed(uint32_t pid) {
  if (g_allowAll) return true;
  for (int i = 0; i < g_n; i++)
    if (g_pid[i] == pid) return true;
  return false;
}

}  // namespace rcdiy

// ================================================================ imu
namespace imu {

struct V3 {
  float x, y, z;
};
static inline V3 add(V3 a, V3 b) {
  return V3{ a.x + b.x, a.y + b.y, a.z + b.z };
}
static inline V3 sub(V3 a, V3 b) {
  return V3{ a.x - b.x, a.y - b.y, a.z - b.z };
}
static inline V3 mul(V3 a, float k) {
  return V3{ a.x * k, a.y * k, a.z * k };
}
static inline float dot(V3 a, V3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline V3 cross(V3 a, V3 b) {
  return V3{ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}
static inline float norm(V3 a) {
  return sqrtf(dot(a, a));
}
static const V3 DEF_UP = IMU_DEF_UP, DEF_FWD = IMU_DEF_FWD;  // 覚える前の向き(画面を立てて、背面が前)

static inline V3 unit(V3 a) {
  float n = norm(a);
  return n > 1e-6f ? mul(a, 1.0f / n) : DEF_UP;
}
// v の、up に垂直な成分(水平成分)。v が up と平行なら初期の「前」「上」で代用する
static V3 horiz(V3 v, V3 up) {
  V3 h = sub(v, mul(up, dot(v, up)));
  if (norm(h) < 1e-3f) h = sub(DEF_FWD, mul(up, dot(DEF_FWD, up)));
  if (norm(h) < 1e-3f) h = sub(DEF_UP, mul(up, dot(DEF_UP, up)));
  return unit(h);
}

static const float NEAR_UP = 0.9f;        // 覚えた「上」と 25° 以内なら同じ向きとみなす
static const float NEAR_FWD = 0.7f;       // 覚えた「前」と 45° 以内
static const float VER_UP = 0.99939f;     // 2° 以上動いたら版を進める(保存のきっかけ)
static const float VER_FWD = 0.99905f;    // 2.5°

static V3 g_up = DEF_UP, g_fwd = DEF_FWD;
static bool g_levelOk = false, g_fwdOk = false;
static uint32_t g_calVer = 0;
static V3 g_upAtVer = DEF_UP, g_fwdAtVer = DEF_FWD;  // 版を進めたときの値
static float g_g0 = 1.0f;  // 停車中に測った重力の大きさ [g](倍率のずれ。実機は 1.1 と出た)
static int g_upReject = 0, g_fwdReject = 0;  // 覚えた向きと大きく違う値が続いた回数(付け直しの検出)
static V3 g_last = { 0, 0, 0 };
static uint32_t g_lastMs = 0;
// 「上」の長期平均: 走行中(IMU_UP_EMA_MIN_KMH 以上)の加速度の指数移動平均。走っているうちに坂やカーブの影響が平均される
static V3 g_lt = { 0, 0, 1 };
// 「上」の短期平均: 停車中(車速 0 が IMU_LEVEL_STILL_MS 以上)。最初の学習と、付け直しの検出に使う
static V3 g_stillSum = { 0, 0, 0 };
static int g_stillN = 0;
static uint32_t g_stillStart = 0;  // いまの 3 秒区間の始まり
static uint32_t g_stillSince = 0;  // 静止が始まった時刻(区間をまたいで持つ。stillNow 用)
static bool g_still = false;
// 「前」: 発進(車速が IMU_FWD_FROM_KMH から IMU_FWD_DONE_KMH へ増える間)の水平方向の加速度の平均
static V3 g_fwdSum = { 0, 0, 0 };
static int g_fwdN = 0;
static bool g_fwdDone = false;  // この発進ではもう使った
static float g_prevKmh = 0;
static uint32_t g_prevKmhMs = 0;
static bool g_accel = false;

void begin() {
  g_up = DEF_UP;
  g_fwd = DEF_FWD;
  g_levelOk = g_fwdOk = false;
  g_calVer = 0;
  g_upAtVer = g_up;
  g_fwdAtVer = g_fwd;
  g_g0 = 1.0f;
  g_upReject = g_fwdReject = 0;
  g_stillSince = 0;
  g_last = V3{ 0, 0, 0 };
  g_lastMs = 0;
  g_lt = g_up;
  g_stillN = 0;
  g_still = false;
  g_fwdSum = V3{ 0, 0, 0 };
  g_fwdN = 0;
  g_fwdDone = false;
  g_accel = false;
  g_prevKmh = 0;
  g_prevKmhMs = 0;
}

void reset() {
  begin();
}

static void setUp(V3 up, bool forgetFwd) {
  g_up = up;
  g_lt = up;
  g_upAtVer = up;
  g_levelOk = true;
  g_upReject = 0;
  g_calVer++;
  if (forgetFwd) {
    g_fwdOk = false;
    g_fwdReject = 0;
    g_fwdSum = V3{ 0, 0, 0 };
    g_fwdN = 0;
  }
}
// 車速が無いとき(机の上): 置いた向きをそのまま「上」に。版は 2° 以上動いたときだけ進める(保存が続かないように)
static void refreshUp(V3 up) {
  g_up = up;
  g_lt = up;
  g_upReject = 0;
  if (dot(up, g_upAtVer) < VER_UP) {
    g_upAtVer = up;
    g_calVer++;
  }
}
static void setGravity(float m) {
  if (m < 0.8f || m > 1.2f) return;
  if (fabsf(m - g_g0) > 0.02f) g_calVer++;
  g_g0 = m;
}

void sample(float ax, float ay, float az, bool speedOk, float speedKmh, uint32_t nowMs) {
  V3 a = { ax, ay, az };
  float jump = norm(sub(a, g_last));  // 直前の値からの変化
  g_last = a;
  uint32_t dt = g_lastMs ? nowMs - g_lastMs : 20;
  if (dt > 100) dt = 100;
  g_lastMs = nowMs;
  float n = norm(a);
  bool plausible = n > 0.8f && n < 1.2f;  // 1 g 前後の値だけ使う(強い振動や手で持っているときを除く)
  // 停車の判定: 車速があればそれで。無ければ「値が動かず、大きさが 1 g に近く、立てた向き(初期の上から 45° 以内)」(机の上)
  bool quiet = jump < IMU_QUIET_G && plausible && dot(a, DEF_UP) > n * IMU_QUIET_TILT_COS;
  bool stopped = speedOk ? (speedKmh < IMU_STILL_KMH) : quiet;
  if (!speedOk) g_fwdN = 0;  // 「前」は車速が無いと覚えない
  // 上(長期): 走行中の平均。時定数 IMU_UP_TAU_MS
  if (g_levelOk && plausible && speedKmh >= IMU_UP_EMA_MIN_KMH) {
    float k = (float)dt / (float)IMU_UP_TAU_MS;
    g_lt = add(g_lt, mul(sub(a, g_lt), k));
    g_up = unit(g_lt);
    if (dot(g_up, g_upAtVer) < VER_UP) {
      g_upAtVer = g_up;
      g_calVer++;
    }
  }
  // 上(短期): 停車中の平均
  if (stopped) {
    if (!g_still) {
      g_still = true;
      g_stillSince = nowMs;
      g_stillStart = nowMs;
      g_stillSum = V3{ 0, 0, 0 };
      g_stillN = 0;
    }
    if (plausible) {
      g_stillSum = add(g_stillSum, a);
      g_stillN++;
    }
    if ((uint32_t)(nowMs - g_stillStart) >= IMU_LEVEL_STILL_MS && g_stillN >= 20) {
      V3 mean = mul(g_stillSum, 1.0f / g_stillN);
      V3 up = unit(mean);
      setGravity(norm(mean));  // 重力の大きさ(倍率のずれ)も覚える
      if (!g_levelOk) {
        setUp(up, false);  // 最初の 1 回(坂で止まっていても、走っているうちに長期平均が直す)
      } else if (!speedOk) {
        refreshUp(up);  // 机の上: 置いた向きに合わせる(「前」は残す)
      } else if (dot(up, g_up) > NEAR_UP) {
        g_upReject = 0;
      } else if (++g_upReject >= IMU_RELEARN_COUNT) {
        setUp(up, true);  // 大きく違う向きが続いた = 付け直した。「前」も覚え直す
      }
      g_stillStart = nowMs;  // 次の区間へ
      g_stillSum = V3{ 0, 0, 0 };
      g_stillN = 0;
    }
  } else {
    g_still = false;
  }
  // 前: 発進の加速。車速が IMU_FWD_FROM_KMH〜IMU_FWD_TO_KMH で増えている間の水平成分を集め、
  // IMU_FWD_DONE_KMH に達したら平均を取る(そこまで加速するのは前進。後退では普通そこまで出さない)
  if (g_levelOk && speedOk) {
    uint32_t sdt = nowMs - g_prevKmhMs;
    if (sdt >= 200) {
      g_accel = (speedKmh - g_prevKmh) > 0.5f;  // 0.2 秒で 0.5 km/h 以上の増加(約 0.07 g)
      g_prevKmh = speedKmh;
      g_prevKmhMs = nowMs;
    }
    if (speedKmh < IMU_FWD_FROM_KMH) {
      g_fwdSum = V3{ 0, 0, 0 };
      g_fwdN = 0;
      g_fwdDone = false;
    } else if (speedKmh > IMU_FWD_TO_KMH || !g_accel) {
      g_fwdSum = V3{ 0, 0, 0 };
      g_fwdN = 0;
    } else if (!g_fwdDone) {
      V3 h = sub(a, mul(g_up, dot(a, g_up)));  // 水平成分
      if (norm(h) > 0.08f) {
        g_fwdSum = add(g_fwdSum, h);
        g_fwdN++;
      }
      if (speedKmh >= IMU_FWD_DONE_KMH && g_fwdN >= IMU_FWD_SAMPLES) {
        V3 f = unit(mul(g_fwdSum, 1.0f / g_fwdN));
        if (!g_fwdOk) {
          g_fwd = f;
          g_fwdAtVer = f;
          g_fwdOk = true;
          g_fwdReject = 0;
          g_calVer++;
        } else if (dot(f, g_fwd) > NEAR_FWD) {
          g_fwd = unit(add(mul(g_fwd, 0.75f), mul(f, 0.25f)));  // 少しずつ寄せる
          g_fwdReject = 0;
          if (dot(g_fwd, g_fwdAtVer) < VER_FWD) {
            g_fwdAtVer = g_fwd;
            g_calVer++;
          }
        } else if (++g_fwdReject >= IMU_RELEARN_COUNT) {
          g_fwd = f;
          g_fwdAtVer = f;
          g_fwdReject = 0;
          g_calVer++;
        }
        g_fwdSum = V3{ 0, 0, 0 };
        g_fwdN = 0;
        g_fwdDone = true;
      }
    }
  }
}

bool levelOk() {
  return g_levelOk;
}
bool forwardOk() {
  return g_fwdOk;
}
bool stillNow() {
  return g_still && (uint32_t)(g_lastMs - g_stillSince) >= 1000;
}
float gravity() {
  return g_g0;
}

void toCar(float x, float y, float z, float *lon, float *lat, float *vert) {
  V3 a = { x, y, z };
  V3 up = g_up;
  V3 fwd = horiz(g_fwd, up);  // 前を水平に直す
  V3 right = cross(fwd, up);
  *lon = dot(a, fwd);
  *lat = dot(a, right) * (float)IMU_LAT_SIGN;
  *vert = dot(a, up);
}

void car(float *lon, float *lat, float *vert) {
  toCar(g_last.x, g_last.y, g_last.z, lon, lat, vert);
  float k = 1.0f / g_g0;  // 倍率のずれを直す(停車中に 1 g になるように)
  *lon *= k;
  *lat *= k;
  *vert = *vert * k - 1.0f;  // 重力を引く
}

uint32_t calVersion() {
  return g_calVer;
}

void exportCal(float out[9]) {
  out[0] = g_up.x;
  out[1] = g_up.y;
  out[2] = g_up.z;
  out[3] = g_fwd.x;
  out[4] = g_fwd.y;
  out[5] = g_fwd.z;
  out[6] = g_levelOk ? 1.0f : 0.0f;
  out[7] = g_fwdOk ? 1.0f : 0.0f;
  out[8] = g_g0;
}

void importCal(const float in[9]) {
  V3 up = { in[0], in[1], in[2] }, fwd = { in[3], in[4], in[5] };
  float nu = norm(up), nf = norm(fwd);
  if (in[6] > 0.5f && nu > 0.9f && nu < 1.1f) {
    g_up = up;  // 保存したときの値をそのまま(長さは 1 に近い)
    g_lt = up;
    g_upAtVer = up;
    g_levelOk = true;
    g_upReject = 0;
  }
  if (in[7] > 0.5f && nf > 0.9f && nf < 1.1f) {
    g_fwd = fwd;
    g_fwdAtVer = fwd;
    g_fwdOk = true;
    g_fwdReject = 0;
  }
  g_g0 = (in[8] > 0.8f && in[8] < 1.2f) ? in[8] : 1.0f;
}

}  // namespace imu

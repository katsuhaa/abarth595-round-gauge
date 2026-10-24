// app.cpp - 受信データの解釈(mlink)、車の CAN から値を作る(veh)、CAN の表(cantab)、全開と走行の時間(trip)。説明は app.h
#include "app.h"
#include <string.h>
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
  uint8_t boostRaw;       // 0A18A001 b4(そのまま)
  bool reverse;           // R に入っている
  uint32_t rpmZeroSince;  // 回転数 0 が続き始めた時刻(0 = 回っている)
  uint32_t rpmMs, speedMs, clutchMs, engMs;  // 最後に受けた時刻(0 = まだ)
  // 燃料
  uint32_t fuelSeenMs;    // 0C014003 を最後に受けた時刻(b5 = 0 も含む)
  uint32_t fuelMs;        // 最後に使った(正しい)値の時刻(0 = この起動でまだ。途切れたあとの判定にも使う)
  uint8_t fuelRaw;        // 最後に使った生の値
  uint8_t fuelLastB5;     // 最後に受けた b5(0 も含む)
  float fuelHp;           // 短い指数平均(VEH_FUEL_HP_S)
  float fuelLp;           // 長い指数平均(VEH_FUEL_LP_S)= 表示に使う値
  bool fuelHave;          // fuelLp に値がある(前回の値、または正しい生の値)
  bool fuelStable;        // 直近の値が安定だったか
  bool fuelArmed;         // エンジン停止を見たので、再始動後の燃料の値で給油を判定する
  bool fuelRestored;      // この起動で前回の値から始めた
  int fuelRestoredTenths;
  uint32_t fuelZero;      // 捨てた b5 = 0 のフレームの数
  uint32_t fuelRefuels;   // 給油とみなした回数
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

void fuelRestore(int tenths) {
  if (tenths <= 0) return;
  platLock();
  g.fuelHp = g.fuelLp = tenths / 10.0f;
  g.fuelHave = true;
  g.fuelRestored = true;
  g.fuelRestoredTenths = tenths;
  platUnlock();
}

bool fuelSaveValue(int *tenths) {
  platLock();
  bool have = g.fuelHave;
  float lp = g.fuelLp;
  platUnlock();
  if (!have) return false;
  long v = lroundf(lp * 10.0f);
  if (v <= 0) return false;  // 0 以下は「無い」と区別できないので書かない
  *tenths = (int)v;
  return true;
}

// 正しい生の値(b5)を 1 つ入れる。platLock の中で呼ぶ
static void fuelSample(float v, uint32_t now) {
  // エンジン停止(回転数 0 が続く、または回転数のフレームが来ない)を見たら、再始動後の給油の判定を待ち受ける
  bool engineOff = (g.rpmMs == 0) || (uint32_t)(now - g.rpmMs) > VEH_FUEL_ENGINE_OFF_MS
                   || (g.rpmZeroSince && (uint32_t)(now - g.rpmZeroSince) >= VEH_FUEL_ENGINE_OFF_MS);
  if (engineOff) g.fuelArmed = true;
  if (g.fuelMs == 0 || (uint32_t)(now - g.fuelMs) > VEH_FUEL_TIMEOUT_MS) {
    // 起動後の最初、または途切れたあと。前の値(前回保存した値、または途切れる前の値)があれば、それより
    // VEH_FUEL_REFUEL_L 以上多いときだけ給油とみなしてこの値から始め直す。そうでなければ前の値を続ける(hp だけ今の値から)
    if (g.fuelHave && v < g.fuelLp + VEH_FUEL_REFUEL_L) {
      g.fuelHp = v;
    } else {
      if (g.fuelHave) g.fuelRefuels++;
      g.fuelHp = g.fuelLp = v;
      g.fuelHave = true;
    }
    g.fuelStable = true;
    g.fuelArmed = false;  // 再始動後の判定はここで済んだ
  } else if (g.fuelArmed && !engineOff && v >= g.fuelLp + VEH_FUEL_REFUEL_L) {
    g.fuelHp = g.fuelLp = v;  // 給油(電源を切らずにエンジンを止め、再び回ってからの最初の値で判定)
    g.fuelStable = true;
    g.fuelArmed = false;
    g.fuelRefuels++;
  } else {
    float dt = (uint32_t)(now - g.fuelMs) / 1000.0f;
    g.fuelHp += dt / (VEH_FUEL_HP_S + dt) * (v - g.fuelHp);
    float dev = v - g.fuelHp;
    g.fuelStable = (dev < VEH_FUEL_STABLE_L && dev > -VEH_FUEL_STABLE_L);
    if (g.fuelStable || v < g.fuelLp) g.fuelLp += dt / (VEH_FUEL_LP_S + dt) * (v - g.fuelLp);
    if (!engineOff) g.fuelArmed = false;  // 回っているのに給油の判定が成立しなかった: 待ち受けを解く
  }
  g.fuelMs = stamp(now);
}

void onFrame(uint8_t bus, uint8_t ext, uint32_t id, uint8_t dlc, const uint8_t *d, uint32_t now) {
  if (!ext) return;  // 使う値はどれも 29 ビット ID
  if (bus == 0) {
    if (id == VEH_ID_RPM_ACCEL && dlc >= 8) {
      platLock();
      g.rpm = (uint16_t)(((uint16_t)d[2] << 8) | d[3]);
      g.accel = d[7];
      g.rpmMs = stamp(now);
      if (g.rpm == 0) {
        if (g.rpmZeroSince == 0) g.rpmZeroSince = stamp(now);
      } else {
        g.rpmZeroSince = 0;
      }
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
      g.boostRaw = d[4];
      g.reverse = (d[7] & VEH_REVERSE_MASK) != 0;
      g.engMs = stamp(now);
      platUnlock();
    }
  } else if (id == VEH_ID_FUEL && dlc >= 6) {
    platLock();
    g.fuelSeenMs = stamp(now);
    g.fuelLastB5 = d[5];
    if (d[5] == 0) {
      g.fuelZero++;  // IG ON 直後の「まだ測れていない」値(b4 = 0x88、b5 = 0)。使わない
    } else {
      g.fuelRaw = d[5];
      fuelSample(d[5], now);
    }
    platUnlock();
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
  bool engOk = within(r.engMs, now, VEH_TIMEOUT_MS);  // 0A18A001(ブースト・リバース)が届いている
  bool fuelOk = r.fuelHave && within(r.fuelSeenMs, now, VEH_FUEL_TIMEOUT_MS);
  updateGear(rpmOk, speedOk, clutchOk, r);
  char gear = (engOk && r.reverse) ? 'R' : g_gear;  // R のビットは推定より優先

  // 値の刻みは 1.x の c6_can_link が値のフレームで送っていたときと同じにする(表示と判定が変わらないように)
  out->rpmOk = rpmOk;
  out->throttleOk = rpmOk;
  out->speedOk = speedOk;
  out->waterOk = waterOk;
  out->boostOk = engOk;
  out->fuelOk = fuelOk;
  out->gearOk = gear != 0;
  out->rpm = r.rpm;
  out->speedKmh = (uint16_t)((uint32_t)r.speedRaw * 100 / 128) / 100.0f;
  out->throttlePct = (uint8_t)(((uint32_t)r.accel * 200 + 127) / 255) / 2.0f;
  int w = r.water < -3276 ? -3276 : (r.water > 3276 ? 3276 : r.water);
  out->waterC = (int16_t)(w * 10) / 10.0f;
  // 0 L = VEH_FUEL_EMPTY_RAW、35 L = VEH_FUEL_FULL_RAW の直線で % に(0 L 側は仮)
  float pct = (r.fuelLp - VEH_FUEL_EMPTY_RAW) * 100.0f / (VEH_FUEL_FULL_RAW - VEH_FUEL_EMPTY_RAW);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  out->fuelPct = (uint8_t)(pct * 2.0f + 0.5f) / 2.0f;
  int bc = (int)(r.boostRaw * VEH_BOOST_BAR_PER_LSB * 100.0f + 0.5f);
  out->boostBar = (int16_t)(bc > 32767 ? 32767 : bc) / 100.0f;
  out->gear = gear;
}

void fuelInfo(FuelInfo *out, uint32_t now) {
  platLock();
  out->have = g.fuelHave;
  out->restored = g.fuelRestored;
  out->restoredTenths = g.fuelRestoredTenths;
  out->seen = within(g.fuelSeenMs, now, VEH_FUEL_TIMEOUT_MS);
  out->raw = g.fuelRaw;
  out->lastB5 = g.fuelLastB5;
  out->hp = g.fuelHp;
  out->lp = g.fuelLp;
  out->stable = g.fuelStable;
  out->armed = g.fuelArmed;
  out->zeroFrames = g.fuelZero;
  out->refuels = g.fuelRefuels;
  platUnlock();
}

float lastRatio() {
  return g_lastRatio;
}

}  // namespace veh

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

void begin() {
  if (!g_e) g_e = (Entry *)platAlloc(sizeof(Entry) * CAN_MAX_IDS);
  g_n = 0;
  g_histPos = 0;
  g_histCount = 0;
  g_rec = 0;
  g_recHistN = 0;
}

void onFrame(uint8_t bus, uint8_t ext, uint32_t id, uint8_t dlc, const uint8_t *data, uint32_t nowMs) {
  if (!g_e) return;
  if (dlc > 8) dlc = 8;
  platLock();
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
  (void)nowMs;
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

}  // namespace cantab

// ================================================================ trip
namespace trip {

static uint32_t g_lastMs = 0;
static bool g_started = false;
static uint32_t g_moving = 0, g_wot = 0;
static uint32_t g_above = 0, g_below = 0;

void update(const Signals &s, uint32_t nowMs) {
  uint32_t dt = g_started ? (uint32_t)(nowMs - g_lastMs) : 0;
  g_started = true;
  g_lastMs = nowMs;
  if (dt > 500) dt = 500;  // 画面の切り替えなどで間が空いても、数えすぎない
  if (s.speedOk && s.speedKmh >= MOVING_SPEED_KMH) {
    g_moving += dt;
    if (s.throttleOk && s.throttlePct >= WOT_THROTTLE_PCT) g_wot += dt;
  }
  if (s.rpmOk) {  // 停止・走行に関係なく数える
    if (s.rpm > TIME_RPM) g_above += dt;
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
          } else if (g_cls == 0x05 && g_id == 0x00) {
            g_nak++;
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
  memcpy(out + 6, payload, len);
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
  n += putKey(pay + n, KEY_GAL, 1);
  n += putKey(pay + n, KEY_GAL_E1, 1);
  n += putKey(pay + n, KEY_QZSS, 1);
  n += putKey(pay + n, KEY_QZSS_L1CA, 1);
  n += putKey(pay + n, KEY_BDS, 0);
  n += putKey(pay + n, KEY_GLO, 0);
  n += putKey(pay + n, KEY_SBAS, 0);
  return ubxFrame(out, cap, 0x06, 0x8A, pay, (uint16_t)n);
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

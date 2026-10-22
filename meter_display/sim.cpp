// sim.cpp - 表示器のデバッグ用の模擬データ。説明は sim.h
//
// ここで作る値はすべて模擬で、車から読んだ値ではない。
// CAN の ID・長さ・周期・変わらないバイトは 2026-09-07 の実車ログ(C-CAN、27 個)から写した。
// 中身の動かし方は、確定している所(回転数・アクセル・車速・輪速・ブレーキ・クラッチ)だけ実車と同じ形で、
// ほかは見た目に動くようにしただけのもの。水温(0A18A001 の b3 − 40)は候補の段階(未確定)。
#include "sim.h"
#include <string.h>

namespace sim {

// ---------------------------------------------------------------- 場面
struct Scene {
  const char *name;
  const char *expect;
  uint32_t durMs;
};
static const Scene SCENES[] = {
  { "4200rpm 3速 水温92℃ 燃料62%", "キャンバス「1 走行画面」と同じ。輪=緑・片側8マス、背景=緑", 6000 },
  { "850rpm N 水温88℃ 燃料12%", "輪=一番下だけ橙、背景=紺、燃料が赤", 6000 },
  { "5600rpm 4速 水温112℃ 燃料45%", "輪=黄・片側11マス、背景=黄、水温が赤。点滅(点灯 100 ms / 消灯 80 ms。点滅は輪が黄になる 5400 rpm から)", 6000 },
  { "6300rpm 3速 水温97℃ 燃料41%", "輪=赤・全部、背景=赤。6000rpm 以上なので点滅せず、画面の明るさが最大", 6000 },
  { "3400rpm 2速 水温と燃料は信号なし", "輪=青・片側6マス、背景=紺、水温と燃料が「– –」", 6000 },
  { "回転数 800→6600→800rpm", "輪が下から伸び、灰→青→緑→黄→赤と変わる。黄になる 5400rpm から点滅し(消灯 80 ms は一定、点灯 150 ms)、5800rpm に近づくほど点灯が短くなり速くなる。6000rpm 以上は点滅せず明るさ最大", 18000 },
  { "走行の模擬(発進→1〜4速で全開→減速→停止を2回)", "ギアと輪が動く。データ画面の全開の割合と時間が増える", 45000 },
  { "水温 30→125→30℃、燃料 100→0→100%", "円弧が伸び縮みする。警告温度以上と15%未満で赤", 16000 },
  { "ブースト −0.5→2.0→−0.5 bar(3000rpm 3速 全開)", "アナログ計のページ: 内側のブースト計の帯(水色、1.6 から赤)が 0 から 2.0 まで伸びて縮む(0 より下は 0 のまま)。回転計の針は 3000", 8000 },
  { "送信を止める(4秒)", "約1秒で輪が全部消え、ギアが「–」、水温と燃料が「– –」、アナログ計の針と帯が消える", 4000 },
};
static const int SCENE_LINK_OFF = 9;  // 送信を止める場面(0 から数えて)
static const int N_SCENES = sizeof(SCENES) / sizeof(SCENES[0]);

// 模擬用の「1 km/h あたりの回転数」。変速比 3.909/2.238/1.520/1.156/0.872、最終減速 3.353、タイヤ外周 1.87 m と
// 仮に置いて計算した値(この車で確かめた値ではない。模擬の中でしか使わない)
static const float RPM_PER_KMH[6] = { 0, 116.8f, 66.9f, 45.4f, 34.5f, 26.1f };
static const float ACCEL_KMH_S[6] = { 0, 25, 15, 9, 6, 4 };

static struct {
  int scene;            // 0 から
  uint32_t sceneStart;
  uint32_t lastMs;
  bool started;
  // 車の状態(模擬)
  float rpm, speed, throttle, water, fuel;
  float boost;          // ブースト圧 [bar](大気圧との差)
  int gear;             // 0 = N
  bool brake, clutch, waterOk, fuelOk;
  // 走行の模擬
  int phase;
  uint32_t phaseStart;
} g;

static void enterScene(int s, uint32_t now) {
  g.scene = s;
  g.sceneStart = now;
  g.phase = 0;
  g.phaseStart = now;
  g.brake = false;
  g.clutch = false;
  g.waterOk = true;
  g.fuelOk = true;
  if (s == 6) {  // 走行の模擬の始まり
    g.speed = 0;
    g.gear = 0;
    g.rpm = 850;
    g.throttle = 0;
    g.water = 60;
    g.fuel = 62;
  }
}

void begin() {
  memset(&g, 0, sizeof(g));
  g.water = 90;
  g.fuel = 50;
  g.rpm = 850;
  g.boost = -0.5f;
}

// 全開のときのブースト圧の模擬 [bar](この車の値ではない。見た目に動くようにしただけ)
static float wotBoost(float rpm) {
  if (rpm < 1500) return 0.1f * rpm / 1500;
  if (rpm < 3000) return 0.1f + 1.3f * (rpm - 1500) / 1500;  // 3000rpm で 1.4
  if (rpm < 5500) return 1.4f;
  return 1.4f - 0.3f * (rpm - 5500) / 1100;                  // 6600rpm で 1.1
}
// アクセルと回転数から、ブースト圧を目標に向けて動かす(過給の遅れ 0.25 秒)。閉じているときは負圧 −0.5 bar
static void boostStep(float dt) {
  float th = g.throttle / 100.0f;
  float target = th * wotBoost(g.rpm) - (1.0f - th) * 0.5f;
  float k = dt / 0.25f;
  if (k > 1) k = 1;
  g.boost += (target - g.boost) * k;
}

static void fixedScene(float rpm, int gear, float water, float fuel, float throttle) {
  g.rpm = rpm;
  g.gear = gear;
  g.water = water;
  g.fuel = fuel;
  g.throttle = throttle;
  g.speed = gear ? rpm / RPM_PER_KMH[gear] : 0;
}

static void driveStep(uint32_t now, float dt) {
  uint32_t pt = now - g.phaseStart;
  switch (g.phase) {
    case 0:  // 停車(2 秒)
      g.gear = 0;
      g.rpm = 850;
      g.speed = 0;
      g.throttle = 0;
      g.brake = false;
      g.clutch = false;
      if (pt >= 2000) {
        g.phase = 1;
        g.gear = 1;
        g.phaseStart = now;
      }
      break;
    case 1:  // 全開で加速
      g.throttle = 100;
      g.clutch = false;
      g.speed += ACCEL_KMH_S[g.gear] * dt;
      g.rpm = g.speed * RPM_PER_KMH[g.gear];
      if (g.rpm < 1100) g.rpm = 1100;
      if (g.gear < 4 && g.rpm >= 6300) {
        g.phase = 2;
        g.phaseStart = now;
      } else if (g.gear == 4 && g.speed >= 150) {
        g.phase = 3;
        g.phaseStart = now;
      }
      break;
    case 2: {  // シフトアップ(0.3 秒、クラッチを切ってアクセルを戻す)
      g.throttle = 0;
      g.clutch = true;
      float target = g.speed * RPM_PER_KMH[g.gear + 1];
      g.rpm += (target - g.rpm) * (dt / 0.12f > 1 ? 1 : dt / 0.12f);
      if (pt >= 300) {
        g.gear++;
        g.phase = 1;
        g.phaseStart = now;
      }
      break;
    }
    case 3:  // ブレーキで減速
      g.throttle = 0;
      g.brake = true;
      g.speed -= 28 * dt;
      if (g.speed < 0) g.speed = 0;
      if (g.gear > 2 && g.speed * RPM_PER_KMH[g.gear] < 2200) g.gear--;
      if (g.speed < 12) {
        g.gear = 0;
        g.clutch = true;
        g.rpm += (850 - g.rpm) * (dt / 0.3f > 1 ? 1 : dt / 0.3f);
      } else {
        g.rpm = g.speed * RPM_PER_KMH[g.gear];
      }
      if (g.speed <= 0) {
        g.phase = 4;
        g.phaseStart = now;
      }
      break;
    default:  // 停車(3 秒)してもう一度
      g.gear = 0;
      g.rpm = 850;
      g.brake = false;
      g.clutch = false;
      if (pt >= 3000) {
        g.phase = 0;
        g.phaseStart = now;
      }
      break;
  }
  g.water += 0.6f * dt;
  if (g.water > 98) g.water = 98;
  g.fuel -= 0.02f * dt;
}

void step(uint32_t now, bool hold) {
  if (!g.started) {
    g.started = true;
    g.lastMs = now;
    enterScene(0, now);
  }
  float dt = (now - g.lastMs) / 1000.0f;
  if (dt > 0.2f) dt = 0.2f;
  g.lastMs = now;
  uint32_t t = now - g.sceneStart;
  if (t >= SCENES[g.scene].durMs) {
    if (hold) {
      g.sceneStart = now;  // 同じ場面を頭からもう一度
      g.phaseStart = now;
      t = 0;
    } else {
      enterScene((g.scene + 1) % N_SCENES, now);
      t = 0;
    }
  }
  switch (g.scene) {
    case 0: fixedScene(4200, 3, 92, 62, 55); break;
    case 1: fixedScene(850, 0, 88, 12, 0); break;
    case 2: fixedScene(5600, 4, 112, 45, 100); break;
    case 3: fixedScene(6300, 3, 97, 41, 100); break;
    case 4:
      fixedScene(3400, 2, 0, 0, 40);
      g.waterOk = false;
      g.fuelOk = false;
      break;
    case 5: {  // 回転数: 12 秒で上げ、6 秒で下げる
      float rpm = (t < 12000) ? 800 + 5800.0f * t / 12000.0f : 6600 - 5800.0f * (t - 12000) / 6000.0f;
      fixedScene(rpm, 3, 90, 50, t < 12000 ? 100 : 0);
      break;
    }
    case 6: driveStep(now, dt); break;
    case 7: {  // 水温と燃料: 8 秒で片道
      float k = (t < 8000) ? t / 8000.0f : (16000 - t) / 8000.0f;
      fixedScene(850, 0, 30 + 95 * k, 100 - 100 * k, 0);
      break;
    }
    case 8: {  // ブースト: 4 秒で片道。模擬の式を使わず、値を直接動かす
      float k = (t < 4000) ? t / 4000.0f : (8000 - t) / 4000.0f;
      fixedScene(3000, 3, 90, 50, 100);
      g.boost = -0.5f + 2.5f * k;
      return;
    }
    default: fixedScene(850, 0, 90, 50, 0); break;
  }
  boostStep(dt);
}

int scene() {
  return g.scene + 1;
}
int sceneCount() {
  return N_SCENES;
}
const char *sceneName() {
  return SCENES[g.scene].name;
}
const char *sceneExpect() {
  return SCENES[g.scene].expect;
}
bool linkOn() {
  return g.scene != SCENE_LINK_OFF;
}

void signals(MlSignals *out, uint32_t nowMs) {
  memset(out, 0, sizeof(*out));
  out->valid = ML_V_RPM | ML_V_SPEED | ML_V_THROTTLE | ML_V_GEAR | ML_V_BOOST | (g.waterOk ? ML_V_WATER : 0) | (g.fuelOk ? ML_V_FUEL : 0);
  out->flags = ML_F_SIM | ML_F_C_CAN | ML_F_B_CAN;
  out->rpm = (uint16_t)(g.rpm + 0.5f);
  out->speed_c = (uint16_t)(g.speed * 100 + 0.5f);
  out->throttle_h = (uint8_t)(g.throttle * 2 + 0.5f);
  float f = g.fuel < 0 ? 0 : (g.fuel > 100 ? 100 : g.fuel);
  out->fuel_h = (uint8_t)(f * 2 + 0.5f);
  out->water_d = (int16_t)(g.water * 10 + (g.water >= 0 ? 0.5f : -0.5f));
  out->gear = g.gear ? (char)('0' + g.gear) : 'N';
  out->scene = (uint8_t)(g.scene + 1);
  out->t_ms = nowMs;
  float b = g.boost < -3 ? -3 : (g.boost > 3 ? 3 : g.boost);
  out->boost_c = (int16_t)(b * 100 + (b >= 0 ? 0.5f : -0.5f));
}

// ---------------------------------------------------------------- CAN の生データ(模擬)
struct CanDef {
  uint32_t id;
  uint8_t bus;        // 0 = C-CAN、1 = B-CAN
  uint8_t dlc;
  uint16_t periodMs;  // 実車ログの周期
  uint8_t base[8];    // 実車ログの中身(2026-09-07、アイドリング中)
};
static const CanDef DEFS[] = {
  { 0x0010A006, 0, 8, 10, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },    // ブレーキ圧 b1-2
  { 0x0028A00F, 0, 8, 10, { 0xD5, 0x00, 0xD8, 0x01, 0x7F, 0xF8, 0x01, 0x00 } },
  { 0x0030A002, 0, 8, 10, { 0xB1, 0xA3, 0xFF, 0x01, 0xCE, 0xD1, 0x3E, 0x8E } },    // 舵角 b3-5
  { 0x0210A006, 0, 8, 10, { 0xFF, 0x06, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00 } },    // 車速 b4-5 / 128
  { 0x0218A006, 0, 8, 10, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },    // 4 輪速 / 16
  { 0x0220A006, 0, 8, 10, { 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
  { 0x0618A001, 0, 8, 10, { 0x00, 0x19, 0x03, 0x16, 0x07, 0x18, 0x46, 0x00 } },    // 回転数 b2-3、アクセル b7
  { 0x0628A001, 0, 8, 10, { 0xFF, 0x24, 0x00, 0x80, 0x0B, 0x00, 0x00, 0x20 } },    // クラッチ b5 bit5
  { 0x0810A000, 0, 4, 10, { 0x01, 0xFE, 0x1C, 0x00, 0, 0, 0, 0 } },                // ブレーキ SW b2 上位 4 ビット
  { 0x0818A000, 0, 8, 100, { 0x00, 0x64, 0x00, 0xD8, 0x9C, 0x40, 0x00, 0x00 } },
  { 0x0A18A000, 0, 8, 100, { 0x20, 0x0D, 0x40, 0x7F, 0x00, 0x49, 0x20, 0x02 } },
  { 0x0A18A001, 0, 8, 50, { 0x01, 0x00, 0xC0, 0x7C, 0x01, 0xAB, 0x18, 0x00 } },    // 水温の候補 b3 − 40(未確定)
  { 0x0A18A002, 0, 2, 100, { 0x01, 0x00, 0, 0, 0, 0, 0, 0 } },
  { 0x0A18A006, 0, 8, 100, { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
  { 0x0A18A01A, 0, 4, 500, { 0x04, 0x00, 0x00, 0x00, 0, 0, 0, 0 } },
  { 0x0A1CA001, 0, 8, 50, { 0x00, 0x01, 0x00, 0x48, 0x31, 0x10, 0x01, 0x80 } },
  { 0x0A28A000, 0, 8, 100, { 0x00, 0x00, 0x00, 0x02, 0x05, 0x48, 0x50, 0x00 } },
  { 0x0A30A000, 0, 4, 100, { 0xFF, 0x80, 0xBF, 0xF8, 0, 0, 0, 0 } },
  { 0x0B9AFF1A, 0, 4, 1000, { 0x04, 0xB0, 0x04, 0xCC, 0, 0, 0, 0 } },
  { 0x0C1CA000, 0, 8, 250, { 0x80, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 } },
  { 0x0C1CA006, 0, 8, 250, { 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
  { 0x0C20A000, 0, 8, 2000, { 0x00, 0x00, 0x00, 0x10, 0x05, 0x00, 0x84, 0x4F } },
  { 0x0C24A000, 0, 8, 2000, { 0x46, 0x31, 0x02, 0x51, 0x00, 0x00, 0x00, 0x04 } },
  { 0x0C28A000, 0, 6, 1000, { 0x07, 0x14, 0x08, 0x09, 0x20, 0x26, 0, 0 } },        // 時計らしい(07:14 08/09/2026)
  { 0x0C2CA000, 0, 8, 2000, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
  { 0x0C3CA000, 0, 2, 1000, { 0x0B, 0x97, 0, 0, 0, 0, 0, 0 } },
  { 0x0C7CA002, 0, 8, 2000, { 0xC0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 } },
  // B-CAN は 10/12 の試験で受信できている。ID で分かっているのは 04214001 だけで、周期と中身は模擬
  { 0x04214001, 1, 8, 100, { 0x01, 0x00, 0xC0, 0x7C, 0x01, 0xAB, 0x18, 0x00 } },
};
static const int N_DEFS = sizeof(DEFS) / sizeof(DEFS[0]);

static uint8_t g_sent[N_DEFS][8];
static uint32_t g_sentMs[N_DEFS];
static bool g_sentOnce[N_DEFS];
static int g_cursor = 0;

static uint8_t bcd(unsigned v) {
  return (uint8_t)(((v / 10) << 4) | (v % 10));
}

static void gen(const CanDef &d, uint32_t now, uint8_t *o) {
  memcpy(o, d.base, 8);
  uint16_t rpm = (uint16_t)(g.rpm + 0.5f);
  switch (d.id) {
    case 0x0618A001: {
      o[2] = (uint8_t)(rpm >> 8);
      o[3] = (uint8_t)rpm;
      o[7] = (uint8_t)(g.throttle * 255.0f / 100.0f + 0.5f);
      o[1] = (uint8_t)(0x19 + (int)(g.throttle / 8));
      o[5] = (uint8_t)(0x18 + ((now / 100) & 3));
      o[6] = (uint8_t)(0x46 + ((now / 40) & 1));
      break;
    }
    case 0x0210A006: {
      uint16_t v = (uint16_t)(g.speed * 128.0f + 0.5f);
      o[4] = (uint8_t)(v >> 8);
      o[5] = (uint8_t)v;
      o[1] = (uint8_t)(0x06 ^ ((now / 200) & 1));
      break;
    }
    case 0x0218A006: {
      uint16_t w = (uint16_t)(g.speed * 16.0f + 0.5f);
      for (int i = 0; i < 4; i++) {
        o[i * 2] = (uint8_t)(w >> 8);
        o[i * 2 + 1] = (uint8_t)w;
      }
      break;
    }
    case 0x0010A006: {
      uint16_t p = g.brake ? 0x0090 : 0;
      o[1] = (uint8_t)(p >> 8);
      o[2] = (uint8_t)p;
      break;
    }
    case 0x0028A00F:
      o[0] = (uint8_t)(0xD4 + (now / 30) % 3);
      o[2] = (uint8_t)(0xD7 + (now / 70) % 3);
      o[4] = (uint8_t)(0x7F + (now / 50) % 2);
      o[5] = (uint8_t)(0xF7 + (now / 90) % 3);
      break;
    case 0x0030A002:
      o[7] = (uint8_t)(now / 10);
      break;
    case 0x0220A006:
      o[1] = (uint8_t)((now / 20) & 1);
      break;
    case 0x0628A001:
      o[5] = g.clutch ? 0x20 : 0x00;
      break;
    case 0x0810A000:
      o[2] = (uint8_t)((g.brake ? 0x70 : 0x10) | ((now / 10) & 0x0F));
      break;
    case 0x0A18A001:
    case 0x04214001: {
      int w = (int)(g.water + 40.5f);
      o[3] = (uint8_t)(w < 0 ? 0 : (w > 255 ? 255 : w));
      o[5] = (uint8_t)(0xAB + (now / 500) % 2);
      break;
    }
    case 0x0A30A000:
      o[2] = (uint8_t)(0xBF ^ ((now / 300) & 1));
      break;
    case 0x0C1CA006:
      o[1] = (uint8_t)((now / 2000) & 1);
      break;
    case 0x0C28A000:
      o[1] = bcd(14 + (now / 60000) % 46);
      break;
    default: break;
  }
}

bool canPoll(uint32_t now, MlCanHead *head, uint8_t *data) {
  for (int n = 0; n < N_DEFS; n++) {
    int i = (g_cursor + n) % N_DEFS;
    const CanDef &d = DEFS[i];
    uint32_t el = now - g_sentMs[i];
    if (g_sentOnce[i] && el < d.periodMs) continue;  // 受けていないフレームは送れない
    uint8_t cur[8];
    gen(d, now, cur);
    bool changed = memcmp(cur, g_sent[i], 8) != 0;
    if (!g_sentOnce[i] || (changed && el >= 50) || el >= 1000) {
      memcpy(g_sent[i], cur, 8);
      g_sentMs[i] = now;
      g_sentOnce[i] = true;
      head->info = (uint8_t)((d.bus ? 0x80 : 0) | 0x40 | d.dlc);  // どれも 29 ビット ID
      head->id = d.id;
      memcpy(data, cur, 8);
      g_cursor = (i + 1) % N_DEFS;
      return true;
    }
  }
  return false;
}

void canRates(uint16_t fps[2], uint16_t ids[2]) {
  fps[0] = fps[1] = ids[0] = ids[1] = 0;
  for (int i = 0; i < N_DEFS; i++) {
    fps[DEFS[i].bus] += (uint16_t)(1000 / DEFS[i].periodMs);
    ids[DEFS[i].bus]++;
  }
}

}  // namespace sim

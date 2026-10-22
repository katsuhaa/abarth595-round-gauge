// ui.cpp - 画面(サーキット・タコ・データ・設定・CAN確認)
//
// 座標・大きさ・色は ui_spec.md(キャンバス版 23)の値。文字の位置(ベースライン)は、キャンバスの絵を
// ブラウザーで描いて測った値。章番号は ui_spec.md のもの。
// 仕様書のあとで変えた所: 枠をなくした(2026-10-13)、水温と燃料を輪の内側の円弧にした、ブースト計のページを足した(2026-10-14)。
// 10/23: 画面の名前をサーキット(走行)・タコ(ブースト計)・データにした。3 つは長押しで切り替え、タップで設定。
// 設定は 1 画面 1 項目(10 項目)で、上の帯・下の帯・真ん中の左右の 4 つの範囲で操作する(config.h の ZONE_*)。
#include "ui.h"
#include "gfx.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace ui {

using gfx::rgb;

// ---------------------------------------------------------------- 色(9.1)
// 背景は仕様書の紺(#0A1E46)から黒に変えた(2026-10-14。AMOLED では黒が最もコントラストが高い)。
// 走行画面の背景は回転数で緑・黄・赤になる(4 章)。その手前(灰・青の範囲)は黒。
static const uint16_t C_BG = rgb(0x000000);           // 背景(黒)
static const uint16_t C_TRACK = rgb(0x1F3A70);        // 走行画面の弧の地
static const uint16_t C_HBAR = rgb(0x274F9C);         // データ画面の横棒の地
static const uint16_t C_WHITE = rgb(0xFFFFFF);
static const uint16_t C_LABEL = rgb(0xA9C0E8);        // 名前、単位、アイコン
static const uint16_t C_WATER_BLUE = rgb(0x2F8BFF);   // 水温の 70℃ 未満
static const uint16_t C_WARN_TRACK = rgb(0x6A1C26);   // 水温の警告温度から上の地
static const uint16_t C_ALARM = rgb(0xE3241B);        // 異常のときの枠と下地
static const uint16_t C_ALARM_TRACK = rgb(0x8E130D);  // 異常のときのバーの地
static const uint16_t C_ALARM_UNIT = rgb(0xFFE3DC);   // 異常のときの単位
static const uint16_t C_ACCENT = rgb(0x5FD0FF);       // 押せる所の印
static const uint16_t C_ORANGE = rgb(0xFF4600);
static const uint16_t C_PEAK = rgb(0xFFD000);         // 回転計のピークホールドの印(黄)

// 輪の色と背景の色(3.3、4 章)。並びは 灰・青・緑・黄・赤
struct Rgb8 {
  uint8_t r, g, b;
};
static const Rgb8 RING_C[5] = { { 0x8D, 0x97, 0xAD }, { 0x2B, 0x7B, 0xFF }, { 0x22, 0xE3, 0x6A }, { 0xFF, 0xDC, 0x00 }, { 0xFF, 0x2A, 0x2A } };
static const Rgb8 BACK_C[5] = { { 0x00, 0x00, 0x00 }, { 0x00, 0x00, 0x00 }, { 0x0E, 0x7A, 0x3C }, { 0xC4, 0xA0, 0x00 }, { 0xA3, 0x15, 0x15 } };

static inline uint16_t to565(Rgb8 c) {
  return rgb(((uint32_t)c.r << 16) | ((uint32_t)c.g << 8) | c.b);
}
static uint8_t mix8(uint8_t a, uint8_t b, int num, int den) {
  return (uint8_t)((a * (den - num) + b * num + den / 2) / den);
}
// 背景の明るさで、文字とアイコンの色を変える(枠をなくしたので、背景の上に直接描く)
static int lum565(uint16_t c) {  // 0〜255
  int r = ((c >> 11) & 31) * 255 / 31, g = ((c >> 5) & 63) * 255 / 63, b = (c & 31) * 255 / 31;
  return (r * 299 + g * 587 + b * 114) / 1000;
}
static bool isBright(uint16_t bg) {
  return lum565(bg) > 120;
}
static uint16_t fgOn(uint16_t bg) {  // 数字
  return isBright(bg) ? rgb(0x06132E) : rgb(0xFFFFFF);
}
static uint16_t labelOn(uint16_t bg) {  // 名前・単位・アイコン
  if (bg == C_BG) return C_LABEL;  // 黒の上ではほかの画面と同じ色
  return isBright(bg) ? gfx::blend(bg, 0x0000, 190) : gfx::blend(bg, 0xFFFF, 170);
}
// 異常の色: 背景が赤っぽいときは赤が見えないので白
static bool isReddish(uint16_t bg) {
  int r = (bg >> 11) & 31, g = (bg >> 5) & 63;
  return r > 16 && g < 24;
}
static uint16_t alarmOn(uint16_t bg) {
  return isReddish(bg) ? rgb(0xFFFFFF) : C_ALARM;
}
// 異常のときの弧の地と、点滅の消灯側の色(暗い赤。赤っぽい背景では少し明るい赤)
static uint16_t alarmDimOn(uint16_t bg) {
  return isReddish(bg) ? gfx::blend(bg, 0xFFFF, 70) : C_ALARM_TRACK;
}
static uint16_t segOffOn(uint16_t bg) {  // 消えているマスは描かない(背景と同じ色)
  return bg;
}

// 仕様書 4.2 の計算。sp はシフト点
static Rgb8 colorAt(const Rgb8 table[5], int rpm, int sp) {
  const int thr[4] = { sp + RING_BLUE_OFFSET, sp + RING_GREEN_OFFSET, sp + RING_YELLOW_OFFSET, sp + RING_RED_OFFSET };
  int i = 0;
  while (i < 4 && rpm >= thr[i]) i++;
  if (i == 4) return table[4];
  int start = thr[i] - RING_BLEND_RPM;
  if (rpm < start) return table[i];
  Rgb8 a = table[i], b = table[i + 1];
  int num = rpm - start;
  return Rgb8{ mix8(a.r, b.r, num, RING_BLEND_RPM), mix8(a.g, b.g, num, RING_BLEND_RPM), mix8(a.b, b.b, num, RING_BLEND_RPM) };
}
// 片側で点くマスの数。0 のときは RING_IDLE_MARK が 1 なら一番下のマスだけを橙で点ける
static int litPerSide(int rpm, int sp) {
  int first = sp - 11 * RING_STEP_RPM;
  int n = 0;
  for (int k = 0; k < 12; k++) {
    if (rpm >= first + RING_STEP_RPM * k) n++;
  }
  return n;
}

// ---------------------------------------------------------------- 共通
static Screen g_sc = SC_DRIVE;
static bool g_full = true;       // 次の update で画面全体を描き直す
static Settings *g_set = nullptr;
static uint32_t g_lastActMs = 0; // 最後に操作した時刻(自動で戻る用)
static uint32_t g_nextMs = 0;    // 走行以外の画面を次に見直す時刻
static bool g_waterAlarm = false, g_fuelAlarm = false;
static bool g_alarmOff = false;  // 異常の表示の点滅の消灯側(update で時刻から決める)
static bool g_simShown = false;
static bool g_blank = false;  // 画面全体の点滅の消灯の間
static uint8_t g_level = BL_NORMAL;  // 画面全体の明るさの段階(BlinkLevel)
static bool g_tagsDirty = false;  // 走行画面の背景を塗り直したので「SIM」を描き直す
static bool g_tagsLost = false;   // 「SIM」の印の所が一部消されたかもしれない(updTags が消して描き直す)
static uint8_t g_sceneShown = 0;

static const float PI_F = 3.14159265f;

static float textC(const FontData &f, float cx, float base, const char *s, uint16_t c) {
  return gfx::drawText(f, cx - gfx::textWidth(f, s) * 0.5f, base, s, c);
}
static void textR(const FontData &f, float xr, float base, const char *s, uint16_t c) {
  gfx::drawText(f, xr - gfx::textWidth(f, s), base, s, c);
}

// ASCII は fa、それ以外(日本語)は fj で描く
static bool isAsciiLike(const char *s) {
  return ((uint8_t)*s) < 0x80;
}
static int runLen(const char *s) {
  bool a = isAsciiLike(s);
  int n = 0;
  while (s[n] && isAsciiLike(s + n) == a) {
    uint8_t c = (uint8_t)s[n];
    n += (c < 0x80) ? 1 : (c >= 0xF0) ? 4 : (c >= 0xE0) ? 3 : 2;
  }
  return n;
}
static float mixWidth(const FontData &fa, const FontData &fj, const char *s) {
  float w = 0;
  char buf[48];
  while (*s) {
    int n = runLen(s);
    if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
    memcpy(buf, s, n);
    buf[n] = 0;
    w += gfx::textWidth(isAsciiLike(s) ? fa : fj, buf);
    s += n;
  }
  return w;
}
static float mixDraw(const FontData &fa, const FontData &fj, float x, float base, const char *s, uint16_t c) {
  char buf[48];
  while (*s) {
    int n = runLen(s);
    if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
    memcpy(buf, s, n);
    buf[n] = 0;
    x = gfx::drawText(isAsciiLike(s) ? fa : fj, x, base, buf, c);
    s += n;
  }
  return x;
}


static bool g_holdShown = false;  // 長押しの表示(画面を暗くして次の画面名)を出している
static Screen g_homeSaved = SC_DRIVE;  // 覚えた画面(次の起動・時間で戻るとき)
static Screen g_homeCur = SC_DRIVE;    // 今の(最後に見た)サーキット・タコ・データ(設定の「設定」で戻る先)
static uint32_t g_homeSinceMs = 0;  // 今のサーキット・タコ・データを出し始めた時刻(覚える画面の判定)
static bool g_homeRestart = false;  // 次の update で g_homeSinceMs を数え直す(試験の testShow)
static void go(Screen sc) {
  g_sc = sc;
  g_full = true;
  g_nextMs = 0;
  g_holdShown = false;
}
Screen screen() {
  return g_sc;
}
bool blankNow() {
  return g_blank;
}

// ---------------------------------------------------------------- 点滅(回転数が「シフト点 + RING_BLINK_OFFSET」以上)
// 消灯・点灯の切り替えの時刻をここで管理する。消灯の時間は一定(BLINK_OFF_MS)、点灯の時間は回転数で決まる(始まりで
// BLINK_ON_SLOW_MS、シフト点以上で BLINK_ON_FAST_MS、その間は比例)。次の切り替えは前の切り替えの時刻に、その側の時間を足して
// 決める(速さが変わっても位相が飛ばない)。
// 始めるときは点灯側から始め、最初の切り替えは半周期後(途中の位相から始めると、最初の消灯が一瞬だけになることがある)。
// 「シフト点 + RING_SOLID_OFFSET」以上では点滅せず、明るさを最大にする(BL_MAX)。
static bool g_blinkActive = false;
static bool g_blinkOff = false;      // 消灯の側
static uint32_t g_blinkEdgeMs = 0;   // 次に切り替える時刻
static uint32_t g_blinkOn = 0;       // いまの点灯の時間(点滅していなければ 0)
static uint32_t g_blinkToggles = 0, g_blinkLateMax = 0, g_blinkLateOver = 0;

static uint32_t blinkOnFor(int rpm, int sp) {
  const int start = sp + RING_BLINK_OFFSET;
  if (rpm >= sp || sp <= start) return BLINK_ON_FAST_MS;
  if (rpm <= start) return BLINK_ON_SLOW_MS;
  return (uint32_t)(BLINK_ON_SLOW_MS + (BLINK_ON_FAST_MS - BLINK_ON_SLOW_MS) * (rpm - start) / (sp - start));
}
static void setLevel(uint8_t level) {
  if (level == g_level) return;
  g_level = level;
  g_blank = level == BL_OFF;
  platBlink(level);
}
void blinkPoll(const Signals &s, uint32_t now) {
  if (g_sc == SC_OPENING) return;  // オープニングの間は、オープニングが明るさの段階を決める
  const int sp = g_set->shiftRpm;
  const bool onPage = g_sc == SC_DRIVE || g_sc == SC_BOOST;
  const bool solid = s.rpmOk && onPage && s.rpm >= sp + RING_SOLID_OFFSET;  // 点滅せず最大輝度
  const bool active = s.rpmOk && onPage && !solid && s.rpm >= sp + RING_BLINK_OFFSET;
  if (!active) {
    g_blinkActive = false;
    g_blinkOff = false;
    g_blinkOn = 0;
    setLevel(solid ? BL_MAX : BL_NORMAL);
    return;
  }
  const uint32_t on = blinkOnFor(s.rpm, sp);
  g_blinkOn = on;
  if (!g_blinkActive) {
    g_blinkActive = true;
    g_blinkOff = false;
    g_blinkEdgeMs = now + on;
  } else if ((int32_t)(now - g_blinkEdgeMs) >= 0) {
    uint32_t late = now - g_blinkEdgeMs;
    g_blinkOff = !g_blinkOff;
    g_blinkToggles++;
    if (late > g_blinkLateMax) g_blinkLateMax = late;
    if (late * 5 > BLINK_OFF_MS) g_blinkLateOver++;
    const uint32_t next = g_blinkOff ? BLINK_OFF_MS : on;  // これから続く側の時間
    g_blinkEdgeMs = (late < next) ? g_blinkEdgeMs + next : now + next;  // 大きく遅れたら、いまから数え直す
  }
  if (BLINK_WHOLE_SCREEN) setLevel(g_blinkOff ? BL_OFF : BL_NORMAL);
  else setLevel(BL_NORMAL);
}
uint8_t blinkLevel() {
  return g_level;
}
bool blinkDue(uint32_t now) {
  return g_blinkActive && (int32_t)(now - g_blinkEdgeMs) >= 0;
}
uint32_t blinkOnMs() {
  return g_blinkOn;
}
uint32_t blinkOffMs() {
  return g_blinkOn ? BLINK_OFF_MS : 0;
}
void blinkStats(BlinkStats *out, bool reset) {
  out->onMs = g_blinkOn;
  out->offMs = g_blinkOn ? BLINK_OFF_MS : 0;
  out->toggles = g_blinkToggles;
  out->lateMaxMs = g_blinkLateMax;
  out->lateOver = g_blinkLateOver;
  if (reset) g_blinkToggles = g_blinkLateMax = g_blinkLateOver = 0;
}

// ================================================================ 走行画面(3〜5 章)
// 輪のマス: 片側 12 マス(k = 0〜11、下から上。右側は左右反転)+ 真下(k = 12)と真上(k = 13)の 1 マスずつ = 全周 26 マス
// (10/21: 真上と真下の切れ目を埋めた。シフト点で全周が点く。真下は 1 マス目と、真上は 12 マス目と一緒に点く)。
// 26 マスを等間隔に並べる(間隔 360/26°、すき間 RING_GAP_DEG)。片側の 12 マスは、仕様書 3.1 の「97+14k° から 12°」とほぼ同じ位置
static const int RING_SEGS = 14;             // 形の数(片側 12 + 真下 + 真上)
static const int RING_SEG_BOTTOM = 12, RING_SEG_TOP = 13;
static const float RING_PITCH_DEG = 360.0f / 26.0f;
static uint8_t *g_segMask[RING_SEGS];
static gfx::Rect g_segRect[RING_SEGS];
static uint16_t g_segCol[RING_SEGS];
static uint16_t g_bgCol = 0;
static uint32_t g_bgMs = 0;
static uint32_t g_waterKey = 0, g_fuelKey = 0;
static int g_gearKey = -1;

// マス k の中心の角度(3 時が 0°、時計回り)。k = 0〜11 は左側(真下から上へ)、12 = 真下、13 = 真上
static float segCenterDeg(int k) {
  if (k == RING_SEG_BOTTOM) return 90.0f;
  if (k == RING_SEG_TOP) return 270.0f;
  return 90.0f + RING_PITCH_DEG * (k + 1);
}
// マス k(片側 n マス点いているとき)が点いているか
static bool segLit(int k, int n) {
  if (k == RING_SEG_BOTTOM) return n >= 1;
  if (k == RING_SEG_TOP) return n >= 12;
  return k < n;
}
// マスの形: 中心 (233, 233)、半径 RING_R_IN〜RING_R_OUT、中心の角度 ± (間隔 − すき間) / 2
static const int RING_POOL_BYTES = RING_SEGS * 80 * 80;
static void buildRing() {
  uint8_t *pool = (uint8_t *)platAlloc(RING_POOL_BYTES);
  uint8_t *poolEnd = pool + RING_POOL_BYTES;
  const float w = RING_PITCH_DEG - RING_GAP_DEG;
  for (int k = 0; k < RING_SEGS; k++) {
    g_segMask[k] = nullptr;
    if (!pool) continue;
    gfx::Pt p[20];
    int n = 0;
    float a0 = segCenterDeg(k) - w * 0.5f;
    for (int i = 0; i <= 8; i++) {
      float a = (a0 + w * i / 8) * PI_F / 180.0f;
      p[n++] = { 233 + RING_R_OUT * cosf(a), 233 + RING_R_OUT * sinf(a) };
    }
    for (int i = 8; i >= 0; i--) {
      float a = (a0 + w * i / 8) * PI_F / 180.0f;
      p[n++] = { 233 + RING_R_IN * cosf(a), 233 + RING_R_IN * sinf(a) };
    }
    float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
    for (int i = 0; i < n; i++) {
      if (p[i].x < x0) x0 = p[i].x;
      if (p[i].y < y0) y0 = p[i].y;
      if (p[i].x > x1) x1 = p[i].x;
      if (p[i].y > y1) y1 = p[i].y;
    }
    gfx::Rect r = { (int16_t)floorf(x0), (int16_t)floorf(y0), (int16_t)ceilf(x1), (int16_t)ceilf(y1) };
    g_segRect[k] = r;
    if (pool + (r.x1 - r.x0) * (r.y1 - r.y0) > poolEnd) continue;
    g_segMask[k] = pool;
    pool += (r.x1 - r.x0) * (r.y1 - r.y0);
    gfx::polyBegin(r);
    gfx::polyAdd(p, n);
    gfx::polyToMask(g_segMask[k]);
  }
}

static void drawSeg(int k, uint16_t c) {
  g_segCol[k] = c;
  if (!g_segMask[k]) return;
  gfx::maskReplace(g_segMask[k], g_segRect[k], false, c, g_bgCol);  // 縁は今の背景の色と混ぜる
  if (k < 12) gfx::maskReplace(g_segMask[k], g_segRect[k], true, c, g_bgCol);  // 右側は左右反転(x → 466−x)。真上・真下は 1 つ
}

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// 水温と燃料の円弧(config.h の ARC_*)。弧の先の位置を 1/4 度刻みで求める(この刻みで変わったときだけ描き直す)
static int waterArcQ(const Signals &s) {
  return (int)(4.0f * ARC_SWEEP_DEG * (clampf(s.waterC, WATER_BAR_MIN_C, WATER_BAR_MAX_C) - WATER_BAR_MIN_C) / (WATER_BAR_MAX_C - WATER_BAR_MIN_C) + 0.5f);
}
static int fuelArcQ(const Signals &s) {
  return (int)(4.0f * ARC_SWEEP_DEG * clampf(s.fuelPct, 0, 100) / 100.0f + 0.5f);
}
static void arc(float a0, float a1, uint16_t c) {
  gfx::fillArc(233, 233, ARC_R_IN, ARC_R_OUT, a0, a1, c);
}
// 弧のある側(上 = 水温、下 = 燃料)を、半径 ALARM_FILL_R の円の中で横一直線に切った形で塗る(10/22)。
// 弧・アイコン・「– –」はこの中に収まるので、描き直すときはこれで消す。異常の点滅の点灯側は赤で塗る。
// clear: なめらかにせず、縁から 2 px 外まで c 一色にする(前に赤く塗った縁の画素が残らない。輪の内側 RING_R_IN には届かない)。
// それ以外: 円の縁は画素の途中で切れるので、1 行ずつ fillRectF で塗る(左右の端がなめらかになる。下地は消した直後の背景)
static void fillSide(bool top, uint16_t c, bool clear) {
  const float R = clear ? ALARM_FILL_R + 2.0f : ALARM_FILL_R;
  const int y0 = top ? (int)(233 - R) : ALARM_FUEL_Y0;
  const int y1 = top ? ALARM_WATER_Y1 : (int)(233 + R) + 1;
  for (int y = y0; y < y1; y++) {
    float dy = (float)y + 0.5f - 233.0f;
    float d2 = R * R - dy * dy;
    if (d2 <= 0) continue;
    float hw = sqrtf(d2);
    if (clear) {
      int x0 = (int)floorf(233.0f - hw), x1 = (int)ceilf(233.0f + hw);
      gfx::fillRect(x0, y, x1 - x0, 1, c);
    } else {
      gfx::fillRectF(233.0f - hw, (float)y, 233.0f + hw, (float)(y + 1), c);
    }
  }
}
// 異常の点灯側で塗る色(赤。背景が赤っぽいときは黒にして、赤い背景との差をつける)
static uint16_t alarmFillOn(uint16_t bg) {
  return isReddish(bg) ? C_BG : C_ALARM;
}

// 水温(5.4、5.6、5.7 を円弧にしたもの)。上の弧: 左端(220°)から時計回りに伸びる。アイコンは弧の内側の上(48 px)。
// 異常(10/22): 点滅の点灯側は上側を赤く塗り(fillSide)、弧の地は暗い赤・伸びた所とアイコンは白。消灯側は背景のまま、伸びた所とアイコンが赤。
// clean = true: 背景を塗った直後(消す必要がない)
static const int ICON_WATER_X = 209, ICON_WATER_Y = 109, ICON_FUEL_X = 209, ICON_FUEL_Y = 309;
static void drawWaterPanel(const Signals &s, bool clean) {
  const float A0 = WATER_ARC_START_DEG, A1 = WATER_ARC_START_DEG + ARC_SWEEP_DEG;
  bool alarm = s.waterOk && g_waterAlarm;
  bool filled = alarm && !g_alarmOff;  // 点灯側: 赤く塗る
  if (!clean) fillSide(true, g_bgCol, true);
  if (filled) fillSide(true, alarmFillOn(g_bgCol), false);
  uint16_t alarmC = filled ? C_WHITE : alarmOn(g_bgCol);
  gfx::drawIcon(ICON_WATER_L, ICON_WATER_X, ICON_WATER_Y, alarm ? alarmC : labelOn(g_bgCol));
  if (!s.waterOk) {
    textC(FONT_NUM600_34, 233, 99, "– –", labelOn(g_bgCol));  // 弧の真ん中(半径 146)
    return;
  }
  const float span = WATER_BAR_MAX_C - WATER_BAR_MIN_C;
  float ae = A0 + waterArcQ(s) * 0.25f;
  if (alarm) {
    arc(A0, A1, filled ? C_ALARM_TRACK : alarmDimOn(g_bgCol));
    arc(A0, ae, alarmC);
    return;
  }
  float aw = A0 + ARC_SWEEP_DEG * (clampf((float)g_set->warnTempC, WATER_BAR_MIN_C, WATER_BAR_MAX_C) - WATER_BAR_MIN_C) / span;
  float ab = A0 + ARC_SWEEP_DEG * (WATER_BLUE_END_C - WATER_BAR_MIN_C) / span;
  if (aw >= A1 - 0.1f) {
    arc(A0, A1, C_TRACK);
  } else {
    arc(A0, A1, C_WARN_TRACK);
    arc(A0, aw, C_TRACK);
  }
  arc(A0, ae, C_WATER_BLUE);
  if (ae > ab) arc(ab, ae, C_WHITE);
}

// 燃料(5.5、5.6、5.7 を円弧にしたもの)。下の弧: 左端(140°)から反時計回り(角度が減る向き)に伸びる。アイコンは弧の内側の下。
// 0〜FUEL_ALARM_PCT の地は暗い赤(10/22)。異常の描き方は水温と同じ(下側を塗る)
static void drawFuelPanel(const Signals &s, bool clean) {
  const float A1 = FUEL_ARC_START_DEG, A0 = FUEL_ARC_START_DEG - ARC_SWEEP_DEG;
  bool alarm = s.fuelOk && g_fuelAlarm;
  bool filled = alarm && !g_alarmOff;
  if (!clean) fillSide(false, g_bgCol, true);
  if (filled) fillSide(false, alarmFillOn(g_bgCol), false);
  uint16_t alarmC = filled ? C_WHITE : alarmOn(g_bgCol);
  gfx::drawIcon(ICON_FUEL_L, ICON_FUEL_X, ICON_FUEL_Y, alarm ? alarmC : labelOn(g_bgCol));
  if (!s.fuelOk) {
    textC(FONT_NUM600_34, 233, 391, "– –", labelOn(g_bgCol));
    return;
  }
  float as = A1 - fuelArcQ(s) * 0.25f;
  if (alarm) {
    arc(A0, A1, filled ? C_ALARM_TRACK : alarmDimOn(g_bgCol));
  } else {
    const float aw = A1 - ARC_SWEEP_DEG * FUEL_ALARM_PCT / 100.0f;  // 異常になる残量の所
    arc(A0, aw, C_TRACK);
    arc(aw, A1, C_WARN_TRACK);
  }
  arc(as, A1, alarm ? alarmC : C_WHITE);
}

// 3000rpm 以下の経過時間(config.h の IDLE_*)。update のたびに記録する(どの画面でも)
static uint32_t g_lowSinceMs = 0;  // 回転数が IDLE_RPM 以下になった時刻(0 = 今は超えている、信号なし)
static uint32_t g_nSinceMs = 0;    // ギアが N になった時刻(0 = N ではない)
static void idleTrack(const Signals &s, uint32_t now) {
  if (s.rpmOk && s.rpm <= IDLE_RPM) {
    if (!g_lowSinceMs) g_lowSinceMs = now ? now : 1;
  } else {
    g_lowSinceMs = 0;
  }
  if (s.gearOk && s.gear == 'N') {
    if (!g_nSinceMs) g_nSinceMs = now ? now : 1;
  } else {
    g_nSinceMs = 0;
  }
}
// ギアの所に経過時間を出すときは、その秒数。出さないときは -1
static int idleShowSec(uint32_t now) {
  if (!g_lowSinceMs || !g_nSinceMs || (uint32_t)(now - g_nSinceMs) < IDLE_SHOW_AFTER_N_MS) return -1;
  uint32_t sec = (uint32_t)(now - g_lowSinceMs) / 1000;
  return sec > 359999 ? 359999 : (int)sec;
}

// ギア(5.3)。枠なし。文字の大きさは config.h の GEAR_FONT(10/22: 160 px。弧の内側に入る)。
// 10/23: N が IDLE_SHOW_AFTER_N_MS 続いたら、ギアの代わりに「▼ 3000rpm」と 3000rpm 以下の経過時間(分:秒)。
// 消す範囲は、上下の塗る所(ALARM_WATER_Y1 より上、ALARM_FUEL_Y0 から下)と、左右の「SIM」の印に掛からないようにする
static const int GEAR_CLR_X0 = 120, GEAR_CLR_X1 = 346, GEAR_CLR_Y0 = 170, GEAR_CLR_Y1 = 293;
static void triDown(float cx, float cy, float size, uint16_t c) {  // 下向きの正三角形(中心 cx, cy、一辺 size)
  const float h = size * 0.866f;
  gfx::Pt p[3] = { { cx - size * 0.5f, cy - h * 0.5f }, { cx + size * 0.5f, cy - h * 0.5f }, { cx, cy + h * 0.5f } };
  gfx::polyBegin(gfx::Rect{ (int16_t)(cx - size), (int16_t)(cy - size), (int16_t)(cx + size + 1), (int16_t)(cy + size + 1) });
  gfx::polyAdd(p, 3);
  gfx::polyFill(c);
}
static void triUp(float cx, float cy, float size, uint16_t c) {
  const float h = size * 0.866f;
  gfx::Pt p[3] = { { cx, cy - h * 0.5f }, { cx + size * 0.5f, cy + h * 0.5f }, { cx - size * 0.5f, cy + h * 0.5f } };
  gfx::polyBegin(gfx::Rect{ (int16_t)(cx - size), (int16_t)(cy - size), (int16_t)(cx + size + 1), (int16_t)(cy + size + 1) });
  gfx::polyAdd(p, 3);
  gfx::polyFill(c);
}
static void drawGearPanel(const Signals &s, int idleSec) {
  gfx::fillRect(GEAR_CLR_X0, GEAR_CLR_Y0, GEAR_CLR_X1 - GEAR_CLR_X0, GEAR_CLR_Y1 - GEAR_CLR_Y0, g_bgCol);
  if (idleSec >= 0) {
    const uint16_t lc = labelOn(g_bgCol);
    char cap[12];
    snprintf(cap, sizeof(cap), "%drpm", IDLE_RPM);
    const float wc = gfx::textWidth(FONT_NUM600_30, cap);
    const float x0 = 233 - (24 + 8 + wc) * 0.5f;
    triDown(x0 + 12, 190, 24, lc);
    gfx::drawText(FONT_NUM600_30, x0 + 32, 200, cap, lc);
    char t[16];
    if (idleSec < 3600) snprintf(t, sizeof(t), "%d:%02d", idleSec / 60, idleSec % 60);
    else snprintf(t, sizeof(t), "%d:%02d:%02d", idleSec / 3600, idleSec / 60 % 60, idleSec % 60);
    const FontData &f = gfx::textWidth(FONT_NUM600_96, t) <= (GEAR_CLR_X1 - GEAR_CLR_X0 - 6) ? FONT_NUM600_96 : FONT_NUM600_64;
    textC(f, 233, 282, t, fgOn(g_bgCol));
    return;
  }
  char t[4] = "–";
  if (s.gearOk) {
    t[0] = s.gear;
    t[1] = 0;
  }
  textC(GEAR_FONT, 233, GEAR_BASELINE, t, fgOn(g_bgCol));
}

// 回転数のピークホールド(10/23): 輪の両側、画面の端に付く黄色の三角形(config.h の PEAK_TRI_*)。
// 位置はピークの回転数が輪のマスのどこに当たるか(1 マス目〜12 マス目、マスの間は比例)。左側の角度 a、右側は 180 − a
static int peakTriKey();
static gfx::Rect ptsRect(const gfx::Pt *p, int n);
static bool rectsOverlap(const gfx::Rect &a, const gfx::Rect &b);
static void peakTriPoints(int key, bool right, gfx::Pt p[3]) {
  float a = key * 0.25f;
  if (right) a = 180.0f - a;
  const float ar = a * PI_F / 180.0f, ca = cosf(ar), sa = sinf(ar);
  const float nx = -sa, ny = ca;
  const float bx = 233 + PEAK_TRI_R_BASE * ca, by = 233 + PEAK_TRI_R_BASE * sa;
  p[0] = { bx + nx * PEAK_TRI_HALF_W, by + ny * PEAK_TRI_HALF_W };
  p[1] = { bx - nx * PEAK_TRI_HALF_W, by - ny * PEAK_TRI_HALF_W };
  p[2] = { 233 + PEAK_TRI_R_TIP * ca, 233 + PEAK_TRI_R_TIP * sa };
}
static gfx::Rect peakTriRect(int key, bool right) {
  gfx::Pt p[3];
  peakTriPoints(key, right, p);
  return ptsRect(p, 3);
}
static void drawPeakTris(int key) {
  for (int r = 0; r < 2; r++) {
    gfx::Pt p[3];
    peakTriPoints(key, r != 0, p);
    gfx::polyBegin(ptsRect(p, 3));
    gfx::polyAdd(p, 3);
    gfx::polyFill(C_PEAK);
  }
}
static int g_triKey = -1;  // 描いてある三角形の角度 [1/4 度]。-1 = なし

// 描いた内容を表す数(これが変わったら描き直す)
static uint32_t waterKey(const Signals &s) {
  if (!s.waterOk) return 1;
  return 2 | (g_waterAlarm ? 4 : 0) | ((g_waterAlarm && g_alarmOff) ? 8 : 0) | ((uint32_t)waterArcQ(s) << 4) | ((uint32_t)g_set->warnTempC << 16);
}
static uint32_t fuelKey(const Signals &s) {
  if (!s.fuelOk) return 1;
  return 2 | (g_fuelAlarm ? 4 : 0) | ((g_fuelAlarm && g_alarmOff) ? 8 : 0) | ((uint32_t)fuelArcQ(s) << 4);
}
static int gearKey(const Signals &s, int idleSec) {
  if (idleSec >= 0) return 0x10000 + idleSec;
  return s.gearOk ? s.gear : 0;
}

static const uint16_t SEG_OFF_MARK = 0x0001;  // 「消えているマス」の印(色は背景から決める)

static void updDrive(const Signals &s, uint32_t now) {
  const int sp = g_set->shiftRpm;
  uint16_t want[RING_SEGS];  // 点けるマスの色。SEG_OFF_MARK = 消す
  bool blinking = false;
  if (!s.rpmOk) {
    for (int k = 0; k < RING_SEGS; k++) want[k] = SEG_OFF_MARK;
  } else {
    int n = litPerSide(s.rpm, sp);
    blinking = g_blinkActive;  // 時刻の管理は blinkPoll(update の最初で呼んである)
    // 画面全体を消す設定(BLINK_WHOLE_SCREEN 1)では輪は点いたまま(消灯は明るさか一色で)。0 のときは、消灯の間は全部のマスを消す(3.4)
    bool off = BLINK_WHOLE_SCREEN ? false : g_blinkOff;
    uint16_t lit = to565(colorAt(RING_C, s.rpm, sp));
    for (int k = 0; k < RING_SEGS; k++) {
      if (off) want[k] = SEG_OFF_MARK;
      else if (n == 0) want[k] = (RING_IDLE_MARK && k == 0) ? C_ORANGE : SEG_OFF_MARK;  // 点き始めより下: RING_IDLE_MARK なら一番下だけ橙(3.2)、既定は全部消す
      else want[k] = segLit(k, n) ? lit : SEG_OFF_MARK;
    }
  }
  uint16_t bgT = s.rpmOk ? to565(colorAt(BACK_C, s.rpm, sp)) : C_BG;

  // 枠なし: 背景の色は画面の端まで。背景が変わったら全部を描き直す
  bool ringChanged = false;
  for (int k = 0; k < RING_SEGS; k++) {
    uint16_t c = want[k] == SEG_OFF_MARK ? segOffOn(g_bgCol) : want[k];
    if (c != g_segCol[k]) ringChanged = true;
  }
  bool bgNow = g_full || (bgT != g_bgCol && (uint32_t)(now - g_bgMs) >= BG_MIN_INTERVAL_MS && !(blinking && ringChanged));
  if (bgNow) {
    g_full = false;
    g_bgCol = bgT;
    g_bgMs = now;
    // 全部を描き直すのは 30 ms 近くかかるので、合間に点滅の切り替えの時刻を見る(基板では platBlinkTick が blinkPoll を呼ぶ)
    gfx::fillAll(g_bgCol);
    platBlinkTick();
    for (int k = 0; k < RING_SEGS; k++) drawSeg(k, want[k] == SEG_OFF_MARK ? segOffOn(g_bgCol) : want[k]);
    g_triKey = peakTriKey();
    if (g_triKey >= 0) drawPeakTris(g_triKey);
    platBlinkTick();
    drawWaterPanel(s, true);
    platBlinkTick();
    const int is = idleShowSec(now);
    drawGearPanel(s, is);
    platBlinkTick();
    drawFuelPanel(s, true);
    platBlinkTick();
    g_waterKey = waterKey(s);
    g_fuelKey = fuelKey(s);
    g_gearKey = gearKey(s, is);
    g_tagsDirty = true;
    return;
  }
  // 輪のマスとピークの三角形。三角形はマスに重なるので、三角形が動いたとき・重なるマスを描き直すときは、
  // 三角形を包む四角を背景で消し、そこに掛かるマスを全部描き直してから三角形を描く(最初から描いたものと同じになる)
  const int tk = peakTriKey();
  bool redo[RING_SEGS];
  for (int k = 0; k < RING_SEGS; k++) {
    uint16_t c = want[k] == SEG_OFF_MARK ? segOffOn(g_bgCol) : want[k];
    redo[k] = c != g_segCol[k];
  }
  bool triRedo = tk != g_triKey;
  gfx::Rect tr[4];
  int nt = 0;
  if (g_triKey >= 0) {
    tr[nt++] = peakTriRect(g_triKey, false);
    tr[nt++] = peakTriRect(g_triKey, true);
  }
  if (!triRedo && g_triKey >= 0) {
    for (int k = 0; k < RING_SEGS && !triRedo; k++) {
      if (!redo[k]) continue;
      gfx::Rect sr = g_segRect[k], sm = { (int16_t)(gfx::W - sr.x1), sr.y0, (int16_t)(gfx::W - sr.x0), sr.y1 };
      for (int i = 0; i < nt; i++) triRedo = triRedo || rectsOverlap(sr, tr[i]) || (k < 12 && rectsOverlap(sm, tr[i]));
    }
  }
  if (triRedo) {
    if (tk >= 0) {
      tr[nt++] = peakTriRect(tk, false);
      tr[nt++] = peakTriRect(tk, true);
    }
    for (int i = 0; i < nt; i++) gfx::fillRect(tr[i].x0, tr[i].y0, tr[i].x1 - tr[i].x0, tr[i].y1 - tr[i].y0, g_bgCol);
    for (int k = 0; k < RING_SEGS; k++) {
      gfx::Rect sr = g_segRect[k], sm = { (int16_t)(gfx::W - sr.x1), sr.y0, (int16_t)(gfx::W - sr.x0), sr.y1 };
      for (int i = 0; i < nt; i++) redo[k] = redo[k] || rectsOverlap(sr, tr[i]) || (k < 12 && rectsOverlap(sm, tr[i]));
    }
  }
  for (int k = 0; k < RING_SEGS; k++) {
    if (redo[k]) drawSeg(k, want[k] == SEG_OFF_MARK ? segOffOn(g_bgCol) : want[k]);
  }
  if (triRedo) {
    if (tk >= 0) drawPeakTris(tk);
    g_triKey = tk;
  }
  uint32_t wk = waterKey(s), fk = fuelKey(s);
  const int is = idleShowSec(now);
  int gk = gearKey(s, is);
  if (wk != g_waterKey) {
    drawWaterPanel(s, false);
    g_waterKey = wk;
  }
  if (gk != g_gearKey) {
    drawGearPanel(s, is);
    g_gearKey = gk;
  }
  if (fk != g_fuelKey) {
    drawFuelPanel(s, false);
    g_fuelKey = fk;
  }
}

// ================================================================ アナログ計(仕様書に無いページ。2026-10-14 にブースト計として追加、
// 2026-10-21 に回転計を主役にした。2026-10-22 に遠くから見やすく描き直した(遠目の案 C))。
// 回転計: 色を抑えた目盛線(1000・500・250)、数字(60 px、太さ 600)、白い針(同じ太さ、黒い縁と影)、
// ピークホールドの黄色い長い印。ブースト: 中央の輪(10/23: 太さ 46。0 bar から今の値までトーンを落とした白、外に 0.5 bar ごとの目盛線)と、
// 輪の右に今の値の大きい数字(太さ 500、白)。水温・燃料: 異常のときだけ数字の下に赤い札(純正のメーターにあるので普段は出さない)。
// 回転計は 6000 rpm が真上(config.h の TACH_*)。
// 描き方: 針・帯・ピークの印を除いた絵(文字盤 = 目盛・数字・帯の地・ブーストの数字・札)を別の画面メモリー g_face に取っておき、
// 動くものは「前の範囲を g_face から写して消す → 描き直す」(最初から描いたものと 1 画素も違わない)。
// ブーストの数字と札が変わったときは、針を消してから描き直し、その範囲を g_face にも写す。
// 何かを消すのに写した四角が、ほかの動くもの(帯・ピークの印)に掛かったら、それも描き直す。針と中心の丸は毎回最後に描く。
// 10/23: 針とブースト以外を少しだけ暗くした(目盛線は 10/22 夜の色 × 0.93、数字 × 0.82、ピークの印 × 0.88)
static const uint16_t C_TICK = rgb(0x777C84);      // 1000 ごとの目盛線(抑えた灰色)
static const uint16_t C_TICK_M = rgb(0x535962);    // 500 ごと
static const uint16_t C_TICK_S = rgb(0x353A42);    // 250 ごと
static const uint16_t C_TICK_R = rgb(0x8F3530);    // 赤の範囲の目盛線(1000 ごと)
static const uint16_t C_TICK_RM = rgb(0x662723);   // 赤の範囲(500 ごと)
static const uint16_t C_TICK_RS = rgb(0x441F1D);   // 赤の範囲(250 ごと)
static const uint16_t C_DIAL = rgb(0xD1D1D1);      // 目盛の数字
static const uint16_t C_DIAL_RED = rgb(0xBA1D16);  // 目盛の数字(赤の範囲)
static const uint16_t C_NEEDLE = rgb(0xFFFFFF);    // 回転計の針(白。10/22 夜)
static const uint16_t C_PEAK_TACH = rgb(0xE0B700); // 回転計のピークの印(黄 × 0.88)
static const uint16_t C_RIM = rgb(0x161E2A);       // ブーストの輪の地(10/23)
static const uint16_t C_RIM_RED = rgb(0x2E1A1C);   // ブーストの輪の地(赤の範囲)
static const uint16_t C_RING_FILL = rgb(0xA9AFBA); // ブーストの輪の伸びた所(トーンを落とした白。10/23)
static const uint16_t C_RING_RED = rgb(0x9E3A33);  // ブーストの輪の伸びた所(赤の範囲)
static const uint16_t C_BOOST = rgb(0xFFFFFF);     // ブーストの数字(10/23: 水色 → 白)

static uint16_t *g_face = nullptr;  // 文字盤(針・帯・ピークの印なし)
static bool g_faceOk = false;       // g_face に今の文字盤が入っている
static int g_needleKey = -1;        // 描いてあるブーストの帯の先の角度 [1/4 度]。-1 = 帯なし(信号なし)
static int g_tachKey = -1;          // 描いてある回転計の針の角度 [1/4 度]。-1 = 針なし
static int g_peakKey = -1;          // 描いてあるピークの印の角度 [1/4 度]。-1 = 印なし
static int g_boostNumKey = -2;      // 描いてあるブーストの数字(0.01 bar 単位)。INT32_MIN 相当 = 「– –」
static uint32_t g_badgeKey = 0;     // 描いてある札の内容
// ピークホールド(どの画面でも記録する)
static float g_peakRpm = -1.0f;     // 最高の回転数(下がっていく値)。-1 = なし
static uint32_t g_peakSetMs = 0;    // 最高を記録した時刻
static uint32_t g_peakLastMs = 0;   // 前に計算した時刻

static float tachAngle(float rpm) {
  float v = clampf(rpm, 0.0f, (float)TACH_MAX_RPM);
  return TACH_START_DEG + TACH_DEG_PER_1000 * v / 1000.0f;
}
static float boostAngle(float bar) {  // ブーストの輪の角度(0 bar が TACH_START_DEG、増えると時計回り)
  float v = clampf(bar, BOOST_SCALE_MIN_BAR, BOOST_SCALE_MAX_BAR);
  return TACH_START_DEG + BOOST_RING_DEG_PER_BAR * v;
}
static int needleKey(const Signals &s) {
  return s.boostOk ? (int)lroundf(boostAngle(s.boostBar) * 4.0f) : -1;
}
static int tachKey(const Signals &s) {
  return s.rpmOk ? (int)(tachAngle((float)s.rpm) * 4.0f + 0.5f) : -1;
}
// ピークホールドの記録(update のたびに呼ぶ)。回転数が上がれば最高を更新。TACH_PEAK_HOLD_MS 止まってから
// TACH_PEAK_FALL_RPM_PER_S で今の回転数まで下がる。同じ時刻で 2 回呼んでも結果は変わらない(最初から描いたものと比べるため)
static void peakTrack(const Signals &s, uint32_t now) {
  if (!s.rpmOk) {
    g_peakRpm = -1.0f;
    g_peakLastMs = now;
    return;
  }
  float rpm = (float)s.rpm;
  if (g_peakRpm < 0 || rpm >= g_peakRpm) {
    g_peakRpm = rpm;
    g_peakSetMs = now;
  } else if ((uint32_t)(now - g_peakSetMs) > TACH_PEAK_HOLD_MS) {
    uint32_t from = (uint32_t)(g_peakLastMs - g_peakSetMs) > TACH_PEAK_HOLD_MS ? g_peakLastMs : g_peakSetMs + TACH_PEAK_HOLD_MS;
    g_peakRpm -= TACH_PEAK_FALL_RPM_PER_S * (float)(uint32_t)(now - from) / 1000.0f;
    if (g_peakRpm < rpm) g_peakRpm = rpm;
  }
  g_peakLastMs = now;
}
static int peakKey() {
  return g_peakRpm >= 0 ? (int)(tachAngle(g_peakRpm) * 4.0f + 0.5f) : -1;
}
static int peakTriKey() {  // サーキットの三角形の左側の角度 [1/4 度]
  if (g_peakRpm < 0) return -1;
  const float first = (float)(g_set->shiftRpm - 11 * RING_STEP_RPM);
  const float pos = clampf((g_peakRpm - first) / RING_STEP_RPM + 1.0f, 1.0f, 12.0f);
  return (int)lroundf((90.0f + RING_PITCH_DEG * pos) * 4.0f);
}
// 先細りの針: 角度 key [1/4 度]、根元 rIn(幅 wIn)から先 rOut(幅 wOut)まで。4 点の多角形
static void needlePoints(int key, float rIn, float rOut, float wIn, float wOut, gfx::Pt p[4]) {
  float a = key * 0.25f * PI_F / 180.0f;
  float ca = cosf(a), sa = sinf(a);
  float nx = -sa, ny = ca;  // 針に直角な向き
  p[0] = { 233 + rIn * ca + nx * wIn * 0.5f, 233 + rIn * sa + ny * wIn * 0.5f };
  p[1] = { 233 + rOut * ca + nx * wOut * 0.5f, 233 + rOut * sa + ny * wOut * 0.5f };
  p[2] = { 233 + rOut * ca - nx * wOut * 0.5f, 233 + rOut * sa - ny * wOut * 0.5f };
  p[3] = { 233 + rIn * ca - nx * wIn * 0.5f, 233 + rIn * sa - ny * wIn * 0.5f };
}
static gfx::Rect ptsRect(const gfx::Pt *p, int n) {  // 点を包む四角(消すときに写す四角)。縁の画素のぶん 2 px 広げる
  float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
  for (int i = 0; i < n; i++) {
    if (p[i].x < x0) x0 = p[i].x;
    if (p[i].y < y0) y0 = p[i].y;
    if (p[i].x > x1) x1 = p[i].x;
    if (p[i].y > y1) y1 = p[i].y;
  }
  gfx::Rect r = { (int16_t)(floorf(x0) - 2), (int16_t)(floorf(y0) - 2), (int16_t)(ceilf(x1) + 2), (int16_t)(ceilf(y1) + 2) };
  if (r.x0 < 0) r.x0 = 0;
  if (r.y0 < 0) r.y0 = 0;
  if (r.x1 > gfx::W) r.x1 = gfx::W;
  if (r.y1 > gfx::H) r.y1 = gfx::H;
  return r;
}
static bool rectsOverlap(const gfx::Rect &a, const gfx::Rect &b) {
  return a.x0 < b.x1 && b.x0 < a.x1 && a.y0 < b.y1 && b.y0 < a.y1;
}
// 環状のおうぎ形(半径 rIn〜rOut、角度 a0〜a1)を包む四角(2 px 広げる)。外周は 5° ごと、内周は両端の点で包む
static gfx::Rect arcRect(float rIn, float rOut, float a0, float a1) {
  gfx::Pt p[48];
  int n = 0;
  int steps = (int)((a1 - a0) / 5.0f) + 2;
  if (steps > 40) steps = 40;
  for (int i = 0; i <= steps; i++) {
    float a = (a0 + (a1 - a0) * i / steps) * PI_F / 180.0f;
    p[n++] = { 233 + rOut * cosf(a), 233 + rOut * sinf(a) };
  }
  p[n++] = { 233 + rIn * cosf(a0 * PI_F / 180.0f), 233 + rIn * sinf(a0 * PI_F / 180.0f) };
  p[n++] = { 233 + rIn * cosf(a1 * PI_F / 180.0f), 233 + rIn * sinf(a1 * PI_F / 180.0f) };
  return ptsRect(p, n);
}
// 針の黒い縁(針を TACH_NEEDLE_EDGE だけ太く・長くした形)
static void needleEdgePoints(int key, gfx::Pt p[4]) {
  needlePoints(key, TACH_NEEDLE_R_IN - TACH_NEEDLE_EDGE, TACH_NEEDLE_R_OUT + TACH_NEEDLE_EDGE, TACH_NEEDLE_W_IN + 2 * TACH_NEEDLE_EDGE, TACH_NEEDLE_W_OUT + 2 * TACH_NEEDLE_EDGE, p);
}
// 影の形(針を右下へずらし、grow だけ太く・長くした形)
static void shadowPoints(int key, float grow, gfx::Pt p[4]) {
  needlePoints(key, TACH_NEEDLE_R_IN - grow * 0.5f, TACH_NEEDLE_R_OUT + grow * 0.5f, TACH_NEEDLE_W_IN + grow, TACH_NEEDLE_W_OUT + grow, p);
  for (int i = 0; i < 4; i++) {
    p[i].x += TACH_SHADOW_DX;
    p[i].y += TACH_SHADOW_DY;
  }
}
static gfx::Rect tachRect(int key) {  // 消すときに写す四角: 縁と影を包む
  gfx::Pt p[8];
  needleEdgePoints(key, p);
  shadowPoints(key, TACH_SHADOW_GROW + TACH_SHADOW_SOFT, p + 4);
  return ptsRect(p, 8);
}
// ブーストの輪(TACH_START_DEG から key まで)と先端の目印を包む四角
static float barDeg(int key) {
  return key * 0.25f;
}
static gfx::Rect barRect(int key) {
  const float a = barDeg(key);
  return arcRect(BOOST_MARK_R_IN, BOOST_MARK_R_OUT, TACH_START_DEG - 0.5f, a + BOOST_MARK_HALF_DEG);
}
static gfx::Rect peakRect(int key) {
  float a = key * 0.25f;
  return arcRect(TACH_PEAK_R_IN, TACH_TICK_R_OUT, a - TACH_PEAK_HALF_DEG, a + TACH_PEAK_HALF_DEG);
}
// 影: 黒を濃さ alpha で重ねる(多角形の縁のなめらかさも濃さに掛ける)
static void shadowFill(const gfx::Pt p[4], int alpha) {
  static uint8_t mask[240 * 240];
  gfx::Rect r = ptsRect(p, 4);
  if ((r.x1 - r.x0) * (r.y1 - r.y0) > (int)sizeof(mask)) return;  // 針の形ならこれより小さい
  gfx::polyBegin(r);
  gfx::polyAdd(p, 4);
  gfx::polyToMask(mask);
  uint16_t *f = gfx::frame();
  const int w = r.x1 - r.x0;
  for (int y = r.y0; y < r.y1; y++) {
    for (int x = r.x0; x < r.x1; x++) {
      uint32_t m = mask[(y - r.y0) * w + (x - r.x0)];
      if (!m) continue;
      uint16_t &px = f[y * gfx::W + x];
      px = gfx::blend(px, 0x0000, (m * (uint32_t)alpha + 127) / 255);
    }
  }
  gfx::addDirty(r.x0, r.y0, r.x1, r.y1);
}
static void drawTachNeedle(int key) {  // 回転計の針(白。まわりに黒い縁、右下に影)
  gfx::Pt p[4];
  shadowPoints(key, TACH_SHADOW_GROW + TACH_SHADOW_SOFT, p);  // ぼけた所(薄く)
  shadowFill(p, TACH_SHADOW_ALPHA / 2);
  shadowPoints(key, TACH_SHADOW_GROW, p);  // 濃い所(重ねて濃くなる)
  shadowFill(p, TACH_SHADOW_ALPHA / 2);
  needleEdgePoints(key, p);
  gfx::polyBegin(ptsRect(p, 4));
  gfx::polyAdd(p, 4);
  gfx::polyFill(C_BG);
  needlePoints(key, TACH_NEEDLE_R_IN, TACH_NEEDLE_R_OUT, TACH_NEEDLE_W_IN, TACH_NEEDLE_W_OUT, p);
  gfx::polyBegin(ptsRect(p, 4));
  gfx::polyAdd(p, 4);
  gfx::polyFill(C_NEEDLE);
}
static void drawPeak(int key) {  // ピークホールドの印(黄)
  float a = key * 0.25f;
  gfx::fillArc(233, 233, TACH_PEAK_R_IN, TACH_TICK_R_OUT, a - TACH_PEAK_HALF_DEG, a + TACH_PEAK_HALF_DEG, C_PEAK_TACH);
}
static void drawBar(int key) {  // ブーストの輪(トーンを落とした白、赤の範囲は暗めの赤)と先端の白い目印
  const float a = barDeg(key);
  const float aRed = boostAngle(BOOST_RED_FROM_BAR);
  if (a > TACH_START_DEG) {
    gfx::fillArc(233, 233, BOOST_RING_R_IN, BOOST_RING_R_OUT, TACH_START_DEG, a < aRed ? a : aRed, C_RING_FILL);
    if (a > aRed) gfx::fillArc(233, 233, BOOST_RING_R_IN, BOOST_RING_R_OUT, aRed, a, C_RING_RED);
  }
  gfx::fillArc(233, 233, BOOST_MARK_R_IN, BOOST_MARK_R_OUT, a - BOOST_MARK_HALF_DEG, a + BOOST_MARK_HALF_DEG, C_WHITE);
}

// ---- ブーストの数字(輪の右。0.01 bar 刻み。信号なしは「– –」)
static int boostNumKey(const Signals &s) {
  if (!s.boostOk) return -100000;
  float v = clampf(s.boostBar, -9.99f, 9.99f);
  return (int)lroundf(v * 100.0f);
}
static gfx::Rect boostNumRect() {  // 上端は輪の「2」の下、左端は輪の「0」の右(どちらも消さない)
  const int h = FONT_DIAL500_76.px;
  gfx::Rect r = { (int16_t)(BOOST_NUM_X - 202), (int16_t)(BOOST_NUM_BASE - (int)(h * 0.78f)), (int16_t)(BOOST_NUM_X + 4), (int16_t)(BOOST_NUM_BASE + 6) };
  return r;
}
static void drawBoostNum(int key, bool clean) {
  const gfx::Rect r = boostNumRect();
  if (!clean) gfx::fillRect(r.x0, r.y0, r.x1 - r.x0, r.y1 - r.y0, C_BG);
  char t[32];
  if (key == -100000) snprintf(t, sizeof(t), "– –");
  else if (key < 0) snprintf(t, sizeof(t), "−%d.%02d", -key / 100, -key % 100);  // 負圧。マイナスは U+2212
  else snprintf(t, sizeof(t), "%d.%02d", key / 100, key % 100);
  textR(FONT_DIAL500_76, BOOST_NUM_X, BOOST_NUM_BASE, t, key == -100000 ? C_LABEL : C_BOOST);
}

// ---- 水温・燃料の札(異常のときだけ。config.h の AN_BADGE_*)
static gfx::Rect badgeArea() {
  gfx::Rect r = { (int16_t)AN_BADGE_X, (int16_t)AN_BADGE_Y, (int16_t)(AN_BADGE_X + AN_BADGE_W), (int16_t)(AN_BADGE_Y + AN_BADGE_PITCH + AN_BADGE_H) };
  return r;
}
static uint32_t badgeKey(const Signals &s) {
  const bool wa = s.waterOk && g_waterAlarm, fa = s.fuelOk && g_fuelAlarm;
  if (!wa && !fa) return 0;
  uint32_t k = 1 | (wa ? 2 : 0) | (fa ? 4 : 0) | (g_alarmOff ? 8 : 0);
  if (wa) k |= ((uint32_t)((int)lroundf(clampf(s.waterC, -99, 999)) + 100) & 0x7FF) << 4;
  if (fa) k |= ((uint32_t)(int)lroundf(clampf(s.fuelPct, 0, 999)) & 0x3FF) << 16;
  return k;
}
static void drawBadge(int y, const IconData &icon, int value) {
  const uint16_t bg = g_alarmOff ? C_ALARM_TRACK : C_ALARM;  // 点滅: 点灯側は赤、消灯側は暗い赤
  gfx::fillRoundRect(AN_BADGE_X, y, AN_BADGE_W, AN_BADGE_H, 6, bg);
  gfx::drawIcon(icon, AN_BADGE_X + 8, y + (AN_BADGE_H - icon.h) / 2, C_WHITE);
  char t[20];
  if (value < 0) snprintf(t, sizeof(t), "−%d", -value);
  else snprintf(t, sizeof(t), "%d", value);
  textR(FONT_DIAL500_40, AN_BADGE_X + AN_BADGE_W - 10, y + 38, t, C_WHITE);
}
static void drawBadges(const Signals &s, bool clean) {
  const gfx::Rect r = badgeArea();
  if (!clean) gfx::fillRect(r.x0, r.y0, r.x1 - r.x0, r.y1 - r.y0, C_BG);
  const bool wa = s.waterOk && g_waterAlarm, fa = s.fuelOk && g_fuelAlarm;
  int y = AN_BADGE_Y;
  if (wa) {
    drawBadge(y, ICON_WATER, (int)lroundf(clampf(s.waterC, -99, 999)));
    y += AN_BADGE_PITCH;
  }
  if (fa) drawBadge(y, ICON_FUEL, (int)lroundf(clampf(s.fuelPct, 0, 999)));
}

static void faceTake(const gfx::Rect &r) {  // 画面メモリーの四角を文字盤に写す
  if (!g_face) return;
  const uint16_t *f = gfx::frame();
  for (int y = r.y0; y < r.y1; y++) memcpy(g_face + (size_t)y * gfx::W + r.x0, f + (size_t)y * gfx::W + r.x0, (size_t)(r.x1 - r.x0) * sizeof(uint16_t));
}

// 文字盤(針・帯・ピークの印を除く)
static void drawBoostFace(const Signals &s) {
  gfx::fillAll(C_BG);
  // 回転計(0〜TACH_MAX_RPM)。数字は ×1000。シフト点 + TACH_RED_OFFSET から赤(目盛線は暗い赤、数字は赤)
  {
    const int redFrom = g_set->shiftRpm + TACH_RED_OFFSET;
    const int nTick = TACH_MAX_RPM / TACH_TICK_STEP_RPM;
    for (int i = 0; i <= nTick; i++) {
      int rpm = i * TACH_TICK_STEP_RPM;
      float a = tachAngle((float)rpm);
      bool red = rpm >= redFrom;
      if (rpm % TACH_LABEL_STEP_RPM == 0) {
        gfx::fillArc(233, 233, TACH_TICK_R_IN, TACH_TICK_R_OUT, a - TACH_TICK_HALF_DEG, a + TACH_TICK_HALF_DEG, red ? C_TICK_R : C_TICK);
        char t[16];
        snprintf(t, sizeof(t), "%d", rpm / 1000);
        float ar = a * PI_F / 180.0f;
        textC(FONT_DIAL600_60, 233 + TACH_LABEL_R * cosf(ar), 233 + TACH_LABEL_R * sinf(ar) + FONT_DIAL600_60.px * 0.36f, t, red ? C_DIAL_RED : C_DIAL);
      } else if (rpm % 500 == 0) {
        gfx::fillArc(233, 233, TACH_TICK_R_IN_M, TACH_TICK_R_OUT, a - TACH_TICK_HALF_DEG_M, a + TACH_TICK_HALF_DEG_M, red ? C_TICK_RM : C_TICK_M);
      } else {
        gfx::fillArc(233, 233, TACH_TICK_R_IN_S, TACH_TICK_R_OUT, a - TACH_TICK_HALF_DEG_S, a + TACH_TICK_HALF_DEG_S, red ? C_TICK_RS : C_TICK_S);
      }
    }
  }
  // ブーストの輪の地(赤の範囲は暗い赤)、0.5 bar ごとの目盛線(輪の外)、数字 0・1・2(輪の内側。赤の範囲は赤)
  {
    const float aRed = boostAngle(BOOST_RED_FROM_BAR), aMax = boostAngle(BOOST_SCALE_MAX_BAR);
    gfx::fillArc(233, 233, BOOST_RING_R_IN, BOOST_RING_R_OUT, TACH_START_DEG, aRed, C_RIM);
    gfx::fillArc(233, 233, BOOST_RING_R_IN, BOOST_RING_R_OUT, aRed, aMax, C_RIM_RED);
    for (float b = 0.0f; b <= BOOST_SCALE_MAX_BAR + 0.01f; b += 0.5f) {
      float a = boostAngle(b);
      gfx::fillArc(233, 233, BOOST_TICK_R_IN, BOOST_TICK_R_OUT, a - BOOST_TICK_HALF_DEG, a + BOOST_TICK_HALF_DEG, b >= BOOST_RED_FROM_BAR - 0.01f ? C_TICK_R : C_TICK);
    }
    for (int b = 0; b <= (int)BOOST_SCALE_MAX_BAR; b++) {
      char t[16];
      snprintf(t, sizeof(t), "%d", b);
      float ar = boostAngle((float)b) * PI_F / 180.0f;
      textC(FONT_DIAL500_28, 233 + BOOST_LABEL_R * cosf(ar), 233 + BOOST_LABEL_R * sinf(ar) + FONT_DIAL500_28.px * 0.36f, t, b >= BOOST_RED_FROM_BAR ? C_DIAL_RED : C_DIAL);
    }
  }
  g_boostNumKey = boostNumKey(s);
  drawBoostNum(g_boostNumKey, true);
  g_badgeKey = badgeKey(s);
  drawBadges(s, true);
}

static void updBoost(const Signals &s, uint32_t now) {
  (void)now;
  if (g_full) {
    g_full = false;
    drawBoostFace(s);
    platBlinkTick();
    if (!g_face) g_face = (uint16_t *)platAlloc((size_t)gfx::W * gfx::H * sizeof(uint16_t));
    if (g_face) {
      memcpy(g_face, gfx::frame(), (size_t)gfx::W * gfx::H * sizeof(uint16_t));
      g_faceOk = true;
    }
    g_needleKey = -1;
    g_tachKey = -1;
    g_peakKey = -1;
    g_tagsDirty = true;
  }
  int nk = needleKey(s), tk = tachKey(s), pk = peakKey(), bk = boostNumKey(s);
  uint32_t gk = badgeKey(s);
  bool faceItems = bk != g_boostNumKey || gk != g_badgeKey;
  if (nk == g_needleKey && tk == g_tachKey && pk == g_peakKey && !faceItems) return;
  if (!g_faceOk) {  // 文字盤を取っておけなかったとき: 文字盤を描き直す(遅いが正しい)
    drawBoostFace(s);
    g_tagsDirty = true;
    g_needleKey = g_tachKey = g_peakKey = -1;
    faceItems = false;
  }
  // 1. 消す(文字盤から写す)。写した四角が帯・ピークの印に掛かったら、その形も全部消して描き直す
  //    (一部だけ消して上から描くと、なめらかに塗った縁が 2 度塗りになって色が変わる)。消した四角が増えたら、もう一度見る
  gfx::Rect erased[4];
  int ne = 0;
  auto erase = [&](const gfx::Rect &r) {
    gfx::copyRectFrom(g_face, r.x0, r.y0, r.x1, r.y1);
    erased[ne++] = r;
    if (g_sc == SC_BOOST) g_tagsLost = true;  // 「SIM」の印に掛かることがある(updTags が消して描き直す)
  };
  if (g_tachKey >= 0) erase(tachRect(g_tachKey));
  bool barRedo = nk != g_needleKey, peakRedo = pk != g_peakKey;
  bool barGone = false, peakGone = false;  // 前の形を消した
  for (int pass = 0; pass < 3; pass++) {
    for (int i = 0; i < ne; i++) {
      if (g_needleKey >= 0 && rectsOverlap(erased[i], barRect(g_needleKey))) barRedo = true;
      if (g_peakKey >= 0 && rectsOverlap(erased[i], peakRect(g_peakKey))) peakRedo = true;
    }
    bool more = false;
    if (barRedo && !barGone) {
      barGone = true;
      if (g_needleKey >= 0) {
        erase(barRect(g_needleKey));
        more = true;
      }
    }
    if (peakRedo && !peakGone) {
      peakGone = true;
      if (g_peakKey >= 0) {
        erase(peakRect(g_peakKey));
        more = true;
      }
    }
    if (!more) break;
  }
  // 2. 文字盤の中の変わるもの(ブーストの数字・札)。描き直して文字盤にも写す。針は消してあり、輪とピークの印には
  //    届かない範囲(数字と札は回転計の目盛の無い右下、輪の切れている所)
  if (faceItems) {
    if (bk != g_boostNumKey) {
      drawBoostNum(bk, false);
      faceTake(boostNumRect());
      g_boostNumKey = bk;
    }
    if (gk != g_badgeKey) {
      drawBadges(s, false);
      faceTake(badgeArea());
      g_badgeKey = gk;
    }
  }
  // 3. 帯とピークの印(変わったか、消したもの)
  if (barRedo && nk >= 0) drawBar(nk);
  if (peakRedo && pk >= 0) drawPeak(pk);
  // 4. 針と中心の丸(いつも最後)
  if (tk >= 0) drawTachNeedle(tk);
  g_needleKey = nk;
  g_tachKey = tk;
  g_peakKey = pk;
}

// ================================================================ データ画面(6 章。10/23 に作り直し)
// 上: GPS・IMU・CAN の値(1 行ずつ。左の点が判定: 緑 = 正常、赤 = 異常、灰 = 待ち・切)。
// 中: 水温・燃料。下: ▲ = 回転数が TIME_RPM を超えていた時間、▼ = 以下の時間(累計。電源を切っても続く。設定の「時間の記録」でリセット)
static int g_dWaterKey, g_dFuelKey;
static uint32_t g_dAboveKey, g_dBelowKey;
static char g_dStatKey[3][64];

static void fmtTime(char *out, size_t n, uint32_t ms) {
  uint32_t t = ms / 1000;
  if (t < 3600) snprintf(out, n, "%u:%02u", (unsigned)(t / 60), (unsigned)(t % 60));
  else snprintf(out, n, "%u:%02u:%02u", (unsigned)(t / 3600), (unsigned)((t / 60) % 60), (unsigned)(t % 60));
}
static void fmtSigned(char *out, size_t n, float v, int dec) {  // マイナスは ASCII '-'(数字の字体で描くため)
  if (v < 0) snprintf(out, n, "-%.*f", dec, (double)-v);
  else snprintf(out, n, "+%.*f", dec, (double)v);
}

// 1 行: 点、名前(GPS・IMU・CAN)、そのあとに「小さい名前 値」の組を並べる。全体を中央に寄せる
struct StatPart {
  const char *label;  // 小さい名前(無ければ "")
  char value[24];
  uint16_t col;       // 値の色
};
static const uint16_t C_DOT_OFF = rgb(0x5A606A);
static const uint16_t C_DOT_OK = rgb(0x22E36A);
static void drawStatLine(int y, const char *name, uint16_t dot, const StatPart *p, int n) {
  gfx::fillRect(36, y - 28, 394, 36, C_BG);
  float w = 16 + gfx::textWidth(FONT_NUM600_28, name) + 16;
  for (int i = 0; i < n; i++) {
    if (p[i].label[0]) w += mixWidth(FONT_NUM600_24, FONT_JP500_24, p[i].label) + 6;
    w += mixWidth(FONT_NUM600_28, FONT_JP600_30, p[i].value) + (i + 1 < n ? 16 : 0);
  }
  float x = floorf(233 - w * 0.5f);
  gfx::fillCircle(x + 8, (float)(y - 10), 8, dot);
  x = gfx::drawText(FONT_NUM600_28, x + 16, (float)y, name, C_WHITE) + 16;
  for (int i = 0; i < n; i++) {
    if (p[i].label[0]) x = mixDraw(FONT_NUM600_24, FONT_JP500_24, x, (float)y, p[i].label, C_LABEL) + 6;
    x = mixDraw(FONT_NUM600_28, FONT_JP600_30, x, (float)y, p[i].value, p[i].col) + 16;
  }
}
// 3 行ぶんの中身を作る(描くかどうかは文字の並びが変わったかで決める)
static int statLines(int row, const LinkStats &ls, const SysInfo &si, const char **name, uint16_t *dot, StatPart *p) {
  const RcStats &r = si.rc;
  for (int i = 0; i < 2; i++) {
    p[i].label = "";
    p[i].value[0] = 0;
    p[i].col = C_WHITE;
  }
  *dot = C_DOT_OFF;
  if (row == 0) {
    *name = "GPS";
    if (!r.enabled) { snprintf(p[0].value, 24, "切"); p[0].col = C_LABEL; return 1; }
    if (!r.gpsOk) { snprintf(p[0].value, 24, "受信なし"); p[0].col = C_ALARM; *dot = C_ALARM; return 1; }
    p[0].label = "衛星";
    snprintf(p[0].value, 24, "%u", (unsigned)r.numSV);
    snprintf(p[1].value, 24, "%u Hz", (unsigned)r.pvtPerSec);
    if (r.fixType >= 2) *dot = C_DOT_OK;
    return 2;
  }
  if (row == 1) {
    *name = "IMU";
    if (!r.enabled) { snprintf(p[0].value, 24, "切"); p[0].col = C_LABEL; return 1; }
    if (!r.imuOk) { snprintf(p[0].value, 24, "応答なし"); p[0].col = C_ALARM; *dot = C_ALARM; return 1; }
    p[0].label = "前後";
    fmtSigned(p[0].value, 24, r.gLon, 2);
    p[1].label = "横";
    fmtSigned(p[1].value, 24, r.gLat, 2);
    strncat(p[1].value, " g", 23 - strlen(p[1].value));
    *dot = C_DOT_OK;
    return 2;
  }
  *name = "CAN";
  if (!ls.infoValid || ls.info.proto_ver != METER_LINK_PROTO_VER) {
    snprintf(p[0].value, 24, "– –");
    p[0].col = C_LABEL;
    return 1;
  }
  p[0].label = "C";
  snprintf(p[0].value, 24, "%u/s", (unsigned)ls.info.can_fps[0]);
  p[1].label = "B";
  snprintf(p[1].value, 24, "%u/s", (unsigned)ls.info.can_fps[1]);
  *dot = ls.info.can_fps[0] ? C_DOT_OK : C_ALARM;
  return 2;
}

// 水温・燃料: アイコン(48 px)、6px 空けて数字、3px 空けて単位。この並びを colX〜colX+204 の中央に寄せる(6.1)
static const int DATA_COL_W = 204;
static void drawDataCell(int colX, const IconData &icon, bool ok, int value, const char *unit, bool alarm) {
  const uint16_t ac = (alarm && g_alarmOff) ? alarmDimOn(C_BG) : C_ALARM;  // 異常: 赤で点滅(消灯側は暗い赤)
  gfx::fillRect(colX - 2, 182, DATA_COL_W + 4, 76, C_BG);
  char t[20];
  if (!ok) snprintf(t, sizeof(t), "– –");
  else if (value < 0) snprintf(t, sizeof(t), "−%d", -value);  // 氷点下。マイナスは U+2212
  else snprintf(t, sizeof(t), "%d", value);
  float wd = gfx::textWidth(FONT_NUM600_64, t), wu = gfx::textWidth(FONT_NUM600_34, unit);
  float total = icon.w + 6 + wd + 3 + wu;
  float left = floorf(colX + (DATA_COL_W - total) * 0.5f);
  // 異常(6.2): 枠なし。アイコン・数字・単位を赤で描く
  gfx::drawIcon(icon, (int)left, 196, alarm ? ac : C_LABEL);
  gfx::drawText(FONT_NUM600_64, left + icon.w + 6, 242, t, alarm ? ac : C_WHITE);
  gfx::drawText(FONT_NUM600_34, left + icon.w + 6 + wd + 3, 242, unit, alarm ? ac : C_LABEL);
}

// ▲ / ▼ の行。y = 時間の数字のベースライン
static const uint16_t C_TIME_UP = rgb(0xFFD000);
static void drawTimeRow(bool up, int y, uint32_t ms) {
  gfx::fillRect(40, y - 48, 386, 56, C_BG);
  if (up) triUp(92, (float)(y - 16), 26, C_TIME_UP);
  else triDown(92, (float)(y - 16), 26, C_LABEL);
  char t[16];
  snprintf(t, sizeof(t), "%d", TIME_RPM);
  float lx = gfx::drawText(FONT_NUM600_34, 112, (float)(y - 4), t, C_LABEL);
  fmtTime(t, sizeof(t), ms);
  // 時間は右端 400 に揃える。「3000」と 14px 以上あくように、入らなければ字を小さくする
  const float room = 400 - (lx + 14);
  const FontData *f = &FONT_NUM600_56;
  if (gfx::textWidth(*f, t) > room) f = &FONT_NUM600_48;
  if (gfx::textWidth(*f, t) > room) f = &FONT_NUM600_44;
  textR(*f, 400, (float)y, t, up ? C_TIME_UP : C_WHITE);
}

static void updData(const Signals &s, const LinkStats &ls, const SysInfo &si, uint32_t now) {
  (void)now;
  bool full = g_full;
  if (full) {
    g_full = false;
    gfx::fillAll(C_BG);
  }
  for (int row = 0; row < 3; row++) {
    const char *name;
    uint16_t dot;
    StatPart p[2];
    int n = statLines(row, ls, si, &name, &dot, p);
    char key[64];
    snprintf(key, sizeof(key), "%04X|%s|%s|%s|%s", dot, p[0].label, p[0].value, n > 1 ? p[1].label : "", n > 1 ? p[1].value : "");
    if (full || strcmp(key, g_dStatKey[row]) != 0) {
      drawStatLine(96 + 36 * row, name, dot, p, n);
      strncpy(g_dStatKey[row], key, sizeof(g_dStatKey[row]) - 1);
      g_dStatKey[row][sizeof(g_dStatKey[row]) - 1] = 0;
    }
  }
  bool wa = s.waterOk && g_waterAlarm, fa = s.fuelOk && g_fuelAlarm;
  // 数字は 3 文字まで(それより長いと枠からはみ出す)。車の値がこの範囲を出ることはない
  int wv = (int)lroundf(clampf(s.waterC, -99, 999)), fv = (int)lroundf(clampf(s.fuelPct, 0, 999));
  int wk = s.waterOk ? (wv * 8 + 4 + (wa ? 1 : 0) + ((wa && g_alarmOff) ? 2 : 0)) : 0x7FFFFFFF;
  int fk = s.fuelOk ? (fv * 8 + 4 + (fa ? 1 : 0) + ((fa && g_alarmOff) ? 2 : 0)) : 0x7FFFFFFF;
  if (full || wk != g_dWaterKey) {
    drawDataCell(26, ICON_WATER_L, s.waterOk, wv, "°C", wa);
    g_dWaterKey = wk;
  }
  if (full || fk != g_dFuelKey) {
    drawDataCell(236, ICON_FUEL_L, s.fuelOk, fv, "%", fa);
    g_dFuelKey = fk;
  }
  uint32_t ak = trip::aboveMs() / 1000, bk = trip::belowMs() / 1000;
  if (full || ak != g_dAboveKey) {
    drawTimeRow(true, 314, trip::aboveMs());
    g_dAboveKey = ak;
  }
  if (full || bk != g_dBelowKey) {
    drawTimeRow(false, 378, trip::belowMs());
    g_dBelowKey = bk;
  }
}

// 折れ線(太さ 4、角を立てる)を多角形として足し込む
static void strokePolyline(const gfx::Pt *p, int n, float hw) {
  for (int i = 0; i + 1 < n; i++) {
    float dx = p[i + 1].x - p[i].x, dy = p[i + 1].y - p[i].y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 1e-4f) continue;
    float nx = -dy / len * hw, ny = dx / len * hw;
    gfx::Pt q[4] = { { p[i].x + nx, p[i].y + ny }, { p[i + 1].x + nx, p[i + 1].y + ny }, { p[i + 1].x - nx, p[i + 1].y - ny }, { p[i].x - nx, p[i].y - ny } };
    gfx::polyAdd(q, 4);
  }
  for (int i = 1; i + 1 < n; i++) {  // 角を埋める
    float ax = p[i].x - p[i - 1].x, ay = p[i].y - p[i - 1].y, bx = p[i + 1].x - p[i].x, by = p[i + 1].y - p[i].y;
    float la = sqrtf(ax * ax + ay * ay), lb = sqrtf(bx * bx + by * by);
    if (la < 1e-4f || lb < 1e-4f) continue;
    float n1x = -ay / la, n1y = ax / la, n2x = -by / lb, n2y = bx / lb;
    float d = 1.0f + n1x * n2x + n1y * n2y;
    for (int side = -1; side <= 1; side += 2) {
      float s = side * hw;
      gfx::Pt a = { p[i].x + n1x * s, p[i].y + n1y * s }, b = { p[i].x + n2x * s, p[i].y + n2y * s };
      if (d >= 0.125f) {  // とがった先まで塗る(とがりすぎる角は平らに切る)
        gfx::Pt m = { p[i].x + (n1x + n2x) / d * s, p[i].y + (n1y + n2y) / d * s };
        gfx::Pt q[4] = { p[i], a, m, b };
        gfx::polyAdd(q, 4);
      } else {
        gfx::Pt q[3] = { p[i], a, b };
        gfx::polyAdd(q, 3);
      }
    }
  }
}

// 大きい字の書体の組(英数字、日本語)。幅に収まらなければ次の組で描く
struct FontPair {
  const FontData *a, *j;
};
static const FontPair FP_NAME[] = { { &FONT_NUM600_48, &FONT_JP600_48 }, { &FONT_NUM600_44, &FONT_JP600_44 }, { &FONT_NUM600_34, &FONT_JP600_34 } };
static const FontPair FP_VALUE[] = { { &FONT_NUM600_64, &FONT_JP600_64 }, { &FONT_NUM600_44, &FONT_JP600_44 }, { &FONT_NUM600_34, &FONT_JP600_34 } };
static void fitTextC(const FontPair *fp, int n, float maxW, float cx, float base, const char *t, uint16_t c) {
  int k = 0;
  while (k + 1 < n && mixWidth(*fp[k].a, *fp[k].j, t) > maxW) k++;
  mixDraw(*fp[k].a, *fp[k].j, cx - mixWidth(*fp[k].a, *fp[k].j, t) * 0.5f, base, t, c);
}
// 太い線の「‹」「›」(dir = -1 / 1)
// w = 横の半幅(省くと h × 0.4)、hw = 線の太さの半分
static void bigChevron(float cx, float cy, float h, int dir, uint16_t c, float w = -1, float hw = 3.0f) {
  if (w < 0) w = h * 0.4f;
  gfx::Pt p[3] = { { cx - dir * w, cy - h * 0.5f }, { cx + dir * w, cy }, { cx - dir * w, cy + h * 0.5f } };
  gfx::polyBegin(gfx::Rect{ (int16_t)(cx - h), (int16_t)(cy - h), (int16_t)(cx + h), (int16_t)(cy + h) });
  strokePolyline(p, 3, hw);
  for (int i = 0; i < 3; i += 2) {  // 端を丸める
    gfx::Pt q[12];
    for (int k = 0; k < 12; k++) q[k] = { p[i].x + hw * cosf(k * PI_F / 6), p[i].y + hw * sinf(k * PI_F / 6) };
    gfx::polyAdd(q, 12);
  }
  gfx::polyFill(c);
}
// ================================================================ 設定と CAN確認の共通の形(10/23)
// どの画面も同じ形: 上の帯(y < ZONE_TOP_Y)= 「設定」(設定の項目。タップで元の画面へ)か「‹ 戻る」(1 つ上へ)、
// その下に項目の名前(50 px)と値、左右の「‹」「›」(真ん中の左半分・右半分のタップ = 前・次)、点 = 項目の数と今の項目、
// 下の帯(y ≥ ZONE_BOT_Y)= その画面の操作。操作の無い画面には下の帯を出さない
static const uint16_t C_BAND_TOP = rgb(0x0F1724);  // 上の帯の地
static const uint16_t C_BAND_BOT = rgb(0x0E2236);  // 下の帯の地
static const uint16_t C_BAND_RED = rgb(0xB3201A);  // 消す操作(リセット・やり直す)の下の帯
static const uint16_t C_DOT = rgb(0x5A606A);       // 下の点(今でない項目)
static const uint16_t C_OK = rgb(0x22E36A);        // 判定: 正常
static const uint16_t C_LABEL2 = rgb(0xC8D6F0);    // 項目名(明るめ)
static const uint16_t C_SEL_BG = rgb(0x14233A);    // CAN確認の詳細: 選んだバイトの地

// 円の中で、y0〜y1 の行を横いっぱいに塗る(円の縁は画素の途中で切れるので、行ごとに fillRectF で塗る)
static void fillChordRows(int y0, int y1, uint16_t c) {
  for (int y = y0; y < y1; y++) {
    float dy = (float)y + 0.5f - 233.0f;
    float d2 = 233.0f * 233.0f - dy * dy;
    if (d2 <= 0) continue;
    float hw = sqrtf(d2);
    gfx::fillRectF(233.0f - hw, (float)y, 233.0f + hw, (float)(y + 1), c);
  }
}
static void drawTopSet() {  // 設定の項目の上の帯「設定」
  fillChordRows(0, ZONE_TOP_Y, C_BAND_TOP);
  textC(FONT_JP600_48, 233, 92, "設定", C_ACCENT);
  textC(FONT_NUM600_20, 233, 34, "v" METER_FW_VERSION, C_LABEL);  // 表示器の版(config.h)
}
static void drawTopBack() {  // その下の画面の上の帯「‹ 戻る」
  fillChordRows(0, ZONE_TOP_Y, C_BAND_TOP);
  float w = gfx::textWidth(FONT_JP600_48, "戻る");
  float x = 233 - (w + 30) * 0.5f;
  bigChevron(x + 8, 77, 32, -1, C_ACCENT);
  gfx::drawText(FONT_JP600_48, x + 30, 94, "戻る", C_ACCENT);
}
static void drawBottom(const char *label, uint16_t fill = C_BAND_BOT, uint16_t col = C_ACCENT) {
  fillChordRows(ZONE_BOT_Y, gfx::H, fill);
  textC(FONT_JP600_44, 233, 420, label, col);
}
static void drawSideArrows() {
  bigChevron(34, 233, 80, -1, C_ACCENT, 14.0f, 3.5f);
  bigChevron(432, 233, 80, 1, C_ACCENT, 14.0f, 3.5f);
}
// 下の点: n 個、cur が今の項目。cols があれば点ごとの色(判定。今の項目は白い輪で囲む)、無ければ今の項目だけ水色
static void drawDots(int n, int cur, const uint16_t *cols, int y) {
  const float gap = n > 8 ? 22 : 26;
  const float x0 = 233 - gap * (n - 1) * 0.5f;
  for (int i = 0; i < n; i++) {
    float x = x0 + gap * i;
    if (cols) {
      if (i == cur) gfx::fillCircle(x, (float)y, 13, C_WHITE), gfx::fillCircle(x, (float)y, 10.5f, C_BG);
      gfx::fillCircle(x, (float)y, i == cur ? 9.0f : 6.0f, cols[i]);
    } else {
      gfx::fillCircle(x, (float)y, i == cur ? 9.0f : 6.0f, i == cur ? C_ACCENT : C_DOT);
    }
  }
}
static void drawName(const char *name) {  // 項目の名前(帯の下)
  fitTextC(FP_NAME, 3, 330, 233, 184, name, C_LABEL2);
}
// 大きい値(数字 96 px と単位 44 px。入らなければ小さく)。unit = "" なら単位なし
static void drawBigValue(const char *v, const char *unit, float base, uint16_t c, float maxW = 280) {
  const bool ascii = isAsciiLike(v) && runLen(v) == (int)strlen(v);
  if (!unit[0]) {
    if (ascii && gfx::textWidth(FONT_NUM600_96, v) <= maxW) textC(FONT_NUM600_96, 233, base, v, c);
    else if (!ascii && mixWidth(FONT_NUM600_96, FONT_JP600_100, v) <= maxW) mixDraw(FONT_NUM600_96, FONT_JP600_100, 233 - mixWidth(FONT_NUM600_96, FONT_JP600_100, v) * 0.5f, base, v, c);
    else fitTextC(FP_VALUE, 3, maxW + 50, 233, base - 10, v, c);
    return;
  }
  const FontData &uf = (uint8_t)unit[0] >= 0xE0 ? FONT_JP600_44 : FONT_NUM600_44;  // 「分」「件」は日本語、「°C」は英数字の書体
  const float wu = gfx::textWidth(uf, unit);
  const FontData &vf = gfx::textWidth(FONT_NUM600_96, v) + 8 + wu <= maxW ? FONT_NUM600_96 : FONT_NUM600_64;
  const float wv = gfx::textWidth(vf, v);
  const float x = 233 - (wv + 8 + wu) * 0.5f;
  gfx::drawText(vf, x, base, v, c);
  gfx::drawText(uf, x + wv + 8, base, unit, C_LABEL2);
}
// ▲ 3000 / ▼ 3000 と累計時間の 2 行(設定の「時間の記録」とリセットの確認)
static void drawTwoTimes(int y1, int y2, uint16_t labc) {
  char t[16], n[8];
  snprintf(n, sizeof(n), "%d", TIME_RPM);
  triUp(96, (float)(y1 - 12), 24, C_TIME_UP);
  gfx::drawText(FONT_NUM600_30, 114, (float)y1, n, labc);
  fmtTime(t, sizeof(t), trip::aboveMs());
  textR(FONT_NUM600_44, 384, (float)(y1 + 4), t, C_TIME_UP);
  triDown(96, (float)(y2 - 12), 24, labc);
  gfx::drawText(FONT_NUM600_30, 114, (float)y2, n, labc);
  fmtTime(t, sizeof(t), trip::belowMs());
  textR(FONT_NUM600_44, 384, (float)(y2 + 4), t, C_WHITE);
}
// タップの範囲: 0 = 上の帯、1 = 下の帯、2 = 真ん中の左半分、3 = 真ん中の右半分
enum Zone { Z_TOP, Z_BOT, Z_LEFT, Z_RIGHT };
static Zone zoneAt(int x, int y) {
  if (y < ZONE_TOP_Y) return Z_TOP;
  if (y >= ZONE_BOT_Y) return Z_BOT;
  return x < 233 ? Z_LEFT : Z_RIGHT;
}

// ---- 受信の状態・RaceChrono の状態の項目と判定
// ---- 受信の状態・RaceChrono の状態: 1 項目の画面と、全項目の判定(下の点)
static const int INFO_N = 7;
enum Verdict : uint8_t { V_WAIT, V_OK, V_NG };
struct InfoItem {
  const char *label;
  char value[48];
  Verdict v;
};
static uint32_t g_errTotal = 0, g_errIncMs = 0;    // 誤り・欠落の合計と、最後に増えた時刻
static uint32_t g_dropTotal = 0, g_dropIncMs = 0;  // RaceChrono で捨てた数と、最後に増えた時刻
static bool g_errSeen = false, g_dropSeen = false;
static void infoTrack(const LinkStats &ls, const SysInfo &si, uint32_t now) {  // update のたびに呼ぶ
  uint32_t e = ls.crcErrors + ls.lostFrames;
  if (e != g_errTotal) {
    if (g_errSeen && e > g_errTotal) g_errIncMs = now;
    g_errTotal = e;
  }
  g_errSeen = true;
  if (si.rc.dropped != g_dropTotal) {
    if (g_dropSeen && si.rc.dropped > g_dropTotal) g_dropIncMs = now;
    g_dropTotal = si.rc.dropped;
  }
  g_dropSeen = true;
}
static bool recent(uint32_t incMs, uint32_t now) {
  return incMs && (uint32_t)(now - incMs) < INFO_RECENT_MS;
}
static void infoItems(const LinkStats &ls, const PerfStats &ps, const SysInfo &si, uint32_t now, InfoItem it[INFO_N]) {
  const char *L[INFO_N] = { "通信速度", "受信", "誤り・欠落", "送り元", "CAN", "描画", "起動" };
  for (int i = 0; i < INFO_N; i++) {
    it[i].label = L[i];
    it[i].v = V_WAIT;
  }
  if (ls.locked) snprintf(it[0].value, 48, "%u bps", (unsigned)ls.baud), it[0].v = V_OK;
  else snprintf(it[0].value, 48, "受信なし"), it[0].v = ls.bytesPerSec ? V_NG : V_WAIT;
  // 受けているとき: フレーム数。受けていないとき: 届いているバイト数(0 = 線に何も来ていない。0 でない = 来ているが読めない)
  if (ls.locked) snprintf(it[1].value, 48, "%u /s", (unsigned)ls.framesPerSec), it[1].v = ls.framesPerSec ? V_OK : V_NG;
  else snprintf(it[1].value, 48, "%u バイト/秒", (unsigned)ls.bytesPerSec);
  snprintf(it[2].value, 48, "%u / %u", (unsigned)ls.crcErrors, (unsigned)ls.lostFrames);
  it[2].v = recent(g_errIncMs, now) ? V_NG : (ls.locked ? V_OK : V_WAIT);
  if (!ls.infoValid) {
    snprintf(it[3].value, 48, "– –");
    snprintf(it[4].value, 48, "– –");
  } else if (ls.info.proto_ver != METER_LINK_PROTO_VER) {
    snprintf(it[3].value, 48, "版が違います"), it[3].v = V_NG;
    snprintf(it[4].value, 48, "– –");
  } else {
    snprintf(it[3].value, 48, "%s%s", ls.info.name, (ls.info.flags & ML_F_SIM) ? " 模擬" : ""), it[3].v = V_OK;
    snprintf(it[4].value, 48, "C %u B %u", (unsigned)ls.info.can_fps[0], (unsigned)ls.info.can_fps[1]);
    it[4].v = ls.info.can_fps[0] ? V_OK : V_NG;  // C-CAN が来ていなければ異常(B-CAN は 0 でもよい)
  }
  snprintf(it[5].value, 48, "%u fps %u ms", (unsigned)ps.fps, (unsigned)ps.loopMaxMs);
  it[5].v = ps.fps >= INFO_MIN_FPS ? V_OK : V_NG;
  // AXP2101 のレジスタ 0x20: bit0 = PWR ボタン、bit2 = VBUS(5V)が入った、bit4 = 電池をつないだ
  if (!si.axpOk) snprintf(it[6].value, 48, "不明");
  else if (si.powerOnSrc & 0x04) snprintf(it[6].value, 48, "5V 投入"), it[6].v = V_OK;
  else if (si.powerOnSrc & 0x01) snprintf(it[6].value, 48, "PWR ボタン"), it[6].v = V_OK;
  else if (si.powerOnSrc & 0x10) snprintf(it[6].value, 48, "電池"), it[6].v = V_OK;
  else snprintf(it[6].value, 48, "0x%02X", si.powerOnSrc), it[6].v = V_OK;
}
static void rcItems(const SysInfo &si, uint32_t now, InfoItem it[INFO_N]) {
  const char *L[INFO_N] = { "BLE", "送信", "GPS", "位置", "GPS 速度", "IMU", "向きの補正" };
  for (int i = 0; i < INFO_N; i++) {
    it[i].label = L[i];
    it[i].v = V_WAIT;
  }
  const RcStats &r = si.rc;
  const bool fix = r.gpsOk && r.fixType >= 2;
  if (!r.enabled) snprintf(it[0].value, 48, "切(設定)");
  else if (!r.bleOk) snprintf(it[0].value, 48, "開始できない"), it[0].v = V_NG;
  else if (r.connected) snprintf(it[0].value, 48, "接続中"), it[0].v = V_OK;
  else snprintf(it[0].value, 48, "未接続(待機中)");
  snprintf(it[1].value, 48, "CAN %u/s GPS %u/s", (unsigned)r.canPerSec, (unsigned)r.gpsPerSec);
  it[1].v = recent(g_dropIncMs, now) ? V_NG : (r.connected ? V_OK : V_WAIT);
  if (!r.gpsOk) snprintf(it[2].value, 48, "受信なし"), it[2].v = r.enabled ? V_NG : V_WAIT;
  else snprintf(it[2].value, 48, "%s 衛星 %u %u Hz", r.fixType == 3 || r.fixType == 4 ? "3D" : r.fixType == 2 ? "2D" : "測位なし", (unsigned)r.numSV, (unsigned)r.pvtPerSec), it[2].v = fix ? V_OK : V_WAIT;
  if (fix) snprintf(it[3].value, 48, "%.4f %.4f", (double)r.lat / 1e7, (double)r.lon / 1e7), it[3].v = V_OK;
  else snprintf(it[3].value, 48, "– –");
  if (fix) snprintf(it[4].value, 48, "%.1f km/h", (double)r.gpsKmh), it[4].v = V_OK;
  else snprintf(it[4].value, 48, "– –");
  if (!r.enabled) snprintf(it[5].value, 48, "切(設定)");
  else if (!r.imuOk) snprintf(it[5].value, 48, "応答なし"), it[5].v = V_NG;
  else snprintf(it[5].value, 48, "前後 %+.2f 横 %+.2f g", (double)r.gLon, (double)r.gLat), it[5].v = V_OK;
  if (!r.imuOk) snprintf(it[6].value, 48, "– –");
  else snprintf(it[6].value, 48, "上 %s 前 %s", r.imuLevel ? "済" : "未", r.imuForward ? "済" : "未"), it[6].v = (r.imuLevel && r.imuForward) ? V_OK : V_WAIT;
}
static uint16_t verdictCol(Verdict v) {
  return v == V_OK ? C_OK : v == V_NG ? C_ALARM : C_DOT;
}
// 全項目の判定のまとめ(設定の項目に出す): 異常があれば「異常 n」、無くて正常があれば「正常」、それ以外は「待ち」
static void verdictSummary(const InfoItem it[INFO_N], char *out, size_t n, uint16_t *col) {
  int ng = 0, ok = 0;
  for (int i = 0; i < INFO_N; i++) {
    ng += it[i].v == V_NG;
    ok += it[i].v == V_OK;
  }
  if (ng) snprintf(out, n, "異常 %d", ng), *col = C_ALARM;
  else if (ok) snprintf(out, n, "正常"), *col = C_WHITE;
  else snprintf(out, n, "待ち"), *col = C_LABEL2;
}

// ================================================================ 設定画面(8 章。10/23 に作り直し: 1 画面 1 項目 × 10)
enum Unit : uint8_t { U_NONE, U_RPM, U_MIN, U_PCT, U_DEGC, U_ONOFF };
enum ItemKind : uint8_t { K_VALUE, K_TIME, K_INFO, K_RC, K_CAN };
struct Item {
  const char *label;  // 項目の名前
  int Settings::*field;
  int minV, maxV, step;
  Unit unit;
  ItemKind kind;
};
static const Item ITEMS[] = {
  { "シフトポイント", &Settings::shiftRpm, SET_SHIFT_MIN, SET_SHIFT_MAX, SET_SHIFT_STEP, U_RPM, K_VALUE },
  { "自動で戻る", &Settings::autoReturnMin, SET_AUTORET_MIN, SET_AUTORET_MAX, SET_AUTORET_STEP, U_MIN, K_VALUE },
  { "明るさ", &Settings::brightnessPct, SET_BRIGHT_MIN, SET_BRIGHT_MAX, SET_BRIGHT_STEP, U_PCT, K_VALUE },
  { "水温の警告", &Settings::warnTempC, SET_WARN_MIN, SET_WARN_MAX, SET_WARN_STEP, U_DEGC, K_VALUE },
  { "デモ", &Settings::demoOn, 0, 1, 1, U_ONOFF, K_VALUE },
  { "RaceChrono 送信", &Settings::rcOn, 0, 1, 1, U_ONOFF, K_VALUE },
  { "時間の記録", nullptr, 0, 0, 0, U_NONE, K_TIME },
  { "受信の状態", nullptr, 0, 0, 0, U_NONE, K_INFO },
  { "RaceChrono", nullptr, 0, 0, 0, U_NONE, K_RC },
  { "CAN確認", nullptr, 0, 0, 0, U_NONE, K_CAN },
};
static const int N_ITEMS = sizeof(ITEMS) / sizeof(ITEMS[0]);
static int g_setItem = 0;          // 設定で見ている項目(電源を切るまで覚える。フラッシュには書かない)
static int g_infoItem = 0;         // 受信の状態・RaceChrono の中で見ている項目
static int g_editOrig = 0;         // 変更の画面に入ったときの値(「‹ 戻る」で戻す)
static bool g_saveTried = false;   // 今の値で保存を試した(失敗したときに、同じ値で繰り返し試さない)
static bool g_saveFailed = false;  // 「保存できませんでした」を出している
static uint32_t g_infoMs = 0;
static char g_setKey[64];          // 設定の項目の画面に描いてある値(変わったら描き直す)

static const char *unitText(Unit u) {
  return u == U_RPM ? "rpm" : u == U_PCT ? "%" : u == U_DEGC ? "°C" : u == U_MIN ? "分" : "";
}
static void valueText(const Item &it, int v, char *out, size_t n) {
  if (it.unit == U_ONOFF) snprintf(out, n, "%s", v ? "入" : "切");
  else snprintf(out, n, "%d", v);
}

// ---- 設定: 1 項目の画面
static void setItemValue(const Item &it, const LinkStats &ls, const PerfStats &ps, const SysInfo &si, uint32_t now, char *v, size_t n, uint16_t *col) {
  *col = C_WHITE;
  if (it.kind == K_VALUE) {
    valueText(it, g_set->*(it.field), v, n);
  } else if (it.kind == K_TIME) {
    snprintf(v, n, "%u|%u", (unsigned)(trip::aboveMs() / 1000), (unsigned)(trip::belowMs() / 1000));
  } else if (it.kind == K_INFO) {
    InfoItem items[INFO_N];
    infoItems(ls, ps, si, now, items);
    verdictSummary(items, v, n, col);
  } else if (it.kind == K_RC) {
    InfoItem items[INFO_N];
    rcItems(si, now, items);
    if (!si.rc.enabled) snprintf(v, n, "切"), *col = C_LABEL2;
    else verdictSummary(items, v, n, col);
  } else {
    snprintf(v, n, "-");
  }
}
static void updSetList(const LinkStats &ls, const PerfStats &ps, const SysInfo &si, uint32_t now) {
  if (g_setItem < 0 || g_setItem >= N_ITEMS) g_setItem = 0;
  const Item &it = ITEMS[g_setItem];
  char v[48];
  uint16_t col;
  setItemValue(it, ls, ps, si, now, v, sizeof(v), &col);
  char key[64];
  snprintf(key, sizeof(key), "%d|%04X|%s", g_setItem, col, v);
  if (!g_full && strcmp(key, g_setKey) == 0) return;
  g_full = false;
  strncpy(g_setKey, key, sizeof(g_setKey) - 1);
  g_setKey[sizeof(g_setKey) - 1] = 0;
  gfx::fillAll(C_BG);
  g_tagsDirty = true;
  drawTopSet();
  drawName(it.label);
  const char *bot = "開く";
  if (it.kind == K_VALUE) {
    drawBigValue(v, it.unit == U_ONOFF ? "" : unitText(it.unit), 290, C_WHITE);
    bot = "変更";
  } else if (it.kind == K_TIME) {
    drawTwoTimes(250, 304, C_LABEL2);
    bot = "リセット";
  } else if (it.kind == K_INFO || it.kind == K_RC) {
    drawBigValue(v, "", 290, col);
  } else {
    mixDraw(FONT_NUM600_34, FONT_JP600_34, 233 - mixWidth(FONT_NUM600_34, FONT_JP600_34, "CAN の画面を開く") * 0.5f, 280, "CAN の画面を開く", C_LABEL2);
  }
  drawSideArrows();
  drawDots(N_ITEMS, g_setItem, nullptr, 330);
  drawBottom(bot);
}

// ---- 設定: 変更の画面(項目の名前のまま、左右が − +(入・切は 切・入)、下「決定」= 保存、上「‹ 戻る」= 変えずに戻る)
static void drawEditValue() {
  const Item &it = ITEMS[g_setItem];
  gfx::fillRect(80, 206, 306, 98, C_BG);  // 〜303。下の刻みの消す範囲(304〜)と隙間を作らない
  char v[16];
  valueText(it, g_set->*(it.field), v, sizeof(v));
  drawBigValue(v, it.unit == U_ONOFF ? "" : unitText(it.unit), 290, C_WHITE, 290);
}
static void drawEditDesc() {  // 刻み(または「保存できませんでした」)
  const Item &it = ITEMS[g_setItem];
  gfx::fillRect(80, 304, 306, 38, C_BG);  // 「ずつ」の濁点の上端(305)まで
  if (g_saveFailed) {
    textC(FONT_JP600_30, 233, 334, "保存できませんでした", C_ALARM);
    return;
  }
  if (it.unit == U_ONOFF) return;  // 入・切には刻みを出さない
  char t[8];
  snprintf(t, sizeof(t), "%d", it.step);
  float w1 = gfx::textWidth(FONT_NUM600_28, t), w2 = gfx::textWidth(FONT_JP600_30, "ずつ");
  float x = 233 - (w1 + 4 + w2) * 0.5f;
  x = gfx::drawText(FONT_NUM600_28, x, 332, t, C_LABEL2) + 4;
  gfx::drawText(FONT_JP600_30, x, 332, "ずつ", C_LABEL2);
}
static void updSetEdit() {
  if (!g_full) return;
  g_full = false;
  const Item &it = ITEMS[g_setItem];
  gfx::fillAll(C_BG);
  g_tagsDirty = true;
  drawTopBack();
  drawName(it.label);
  drawEditValue();
  if (it.unit == U_ONOFF) {
    textC(FONT_JP600_44, 44, 280, "切", C_ACCENT);
    textC(FONT_JP600_44, 422, 280, "入", C_ACCENT);
  } else {
    gfx::fillRectF(28, 259, 76, 270, C_ACCENT);    // −
    gfx::fillRectF(390, 259, 438, 270, C_ACCENT);  // +
    gfx::fillRectF(408.5f, 240.5f, 419.5f, 288.5f, C_ACCENT);
  }
  drawEditDesc();
  drawBottom("決定");
}

// ---- 設定: 時間の記録のリセットの確認(上「‹ 戻る」= 消さずに戻る、下「リセット」= 消して戻る)
static uint64_t g_resetKey = 0;  // 描いてある 2 つの時間(秒)
static void updSetReset() {
  // 時間はこの画面にいる間も進むので、秒が変わったら描き直す
  const uint64_t k = ((uint64_t)(trip::aboveMs() / 1000) << 32) | (trip::belowMs() / 1000);
  if (!g_full && k == g_resetKey) return;
  g_resetKey = k;
  g_full = false;
  gfx::fillAll(C_BG);
  g_tagsDirty = true;
  drawTopBack();
  drawName("時間の記録");
  drawTwoTimes(246, 298, C_LABEL);
  mixDraw(FONT_NUM600_30, FONT_JP600_30, 233 - mixWidth(FONT_NUM600_30, FONT_JP600_30, "2 つとも 0 に戻す") * 0.5f, 334, "2 つとも 0 に戻す", C_LABEL2);
  drawBottom("リセット", C_BAND_RED, C_WHITE);
}

// ---- 受信の状態・RaceChrono の状態の中: 1 項目の画面と、全項目の判定(下の点)
static void drawInfoItem(const InfoItem it[INFO_N], bool resetOp) {
  gfx::fillAll(C_BG);
  g_tagsDirty = true;  // 全部を塗り直したので「SIM」の印も描き直す
  drawTopBack();
  if (g_infoItem < 0 || g_infoItem >= INFO_N) g_infoItem = 0;
  const InfoItem &c = it[g_infoItem];
  drawName(c.label);
  fitTextC(FP_VALUE, 3, 320, 233, 290, c.value, c.v == V_NG ? C_ALARM : (c.v == V_OK ? C_WHITE : C_LABEL2));
  drawSideArrows();
  uint16_t cols[INFO_N];
  for (int i = 0; i < INFO_N; i++) cols[i] = verdictCol(it[i].v);
  drawDots(INFO_N, g_infoItem, cols, 330);
  if (resetOp) drawBottom("向きを取り直す");  // RaceChrono の「向きの補正」: 覚えた向きを捨てる
}
// 値は 0.5 秒ごとに描き直す(項目を送ったとき、開いたときはすぐ)
static void updSetInfo(const LinkStats &ls, const PerfStats &ps, const SysInfo &si, uint32_t now) {
  if (!g_full && (uint32_t)(now - g_infoMs) < 500) return;
  g_full = false;
  g_infoMs = now;
  InfoItem it[INFO_N];
  infoItems(ls, ps, si, now, it);
  drawInfoItem(it, false);
}
static bool g_rcResetShown = false;  // 「向きを取り直す」を出している(タップの判定用)
static bool rcResetOp(const SysInfo &si) {
  return g_infoItem == 6 && si.rc.imuOk;
}
static void updSetRc(const SysInfo &si, uint32_t now) {
  if (!g_full && (uint32_t)(now - g_infoMs) < 500) return;
  g_full = false;
  g_infoMs = now;
  InfoItem it[INFO_N];
  rcItems(si, now, it);
  g_rcResetShown = rcResetOp(si);
  drawInfoItem(it, g_rcResetShown);
}

// ================================================================ CAN確認(設定の 10 項目目。10/23)
// CAN の画面: 件数(今の候補の ID の数)・記録の回数・件数の移り変わり。下「変化を記録」、右半分「一覧」、左半分「やり直す」(確認へ)。
// 一覧: 1 件ずつ(左右で ID)。候補のバイトは水色。下「詳細」。詳細: 左右でバイトを選び、値とグラフ
static const int CAN_LIST_MAX = CAN_MAX_IDS;
static uint16_t g_candE[CAN_LIST_MAX];
static uint8_t g_candM[CAN_LIST_MAX];
static int g_candN = 0, g_canIdx = 0;
static int g_detEntry = -1, g_detByte = 0;
static char g_canKey[64];
static uint32_t g_canCellKey[8];
static uint32_t g_detValKey, g_detChartMs;

static void fmtId(char *out, size_t n, uint32_t id, uint8_t ext) {
  if (ext) snprintf(out, n, "%04X %04X", (unsigned)((id >> 16) & 0xFFFF), (unsigned)(id & 0xFFFF));
  else snprintf(out, n, "%03X", (unsigned)(id & 0x7FF));
}

static void updCan() {
  const int rec = cantab::recordCount();
  g_candN = cantab::candidates(g_candE, g_candM, CAN_LIST_MAX);
  int hist[CAN_REC_HIST];
  int nh = cantab::recordHistory(hist);
  char key[64];
  snprintf(key, sizeof(key), "%d|%d|%d", rec, g_candN, nh ? hist[0] : -1);
  if (!g_full && strcmp(key, g_canKey) == 0) return;
  g_full = false;
  strncpy(g_canKey, key, sizeof(g_canKey) - 1);
  g_canKey[sizeof(g_canKey) - 1] = 0;
  gfx::fillAll(C_BG);
  g_tagsDirty = true;
  drawTopBack();
  drawName("CAN確認");
  char t[64];
  if (rec == 0) {
    snprintf(t, sizeof(t), "記録 0 回");
    mixDraw(FONT_NUM600_44, FONT_JP600_44, 233 - mixWidth(FONT_NUM600_44, FONT_JP600_44, t) * 0.5f, 256, t, C_WHITE);
    textC(FONT_JP600_30, 233, 300, "操作の前に「変化を記録」", C_LABEL);
  } else {
    snprintf(t, sizeof(t), "%d", g_candN);
    drawBigValue(t, "件", 258, C_WHITE, 200);
    snprintf(t, sizeof(t), "記録 %d 回", rec);
    mixDraw(FONT_NUM600_28, FONT_JP600_30, 233 - mixWidth(FONT_NUM600_28, FONT_JP600_30, t) * 0.5f, 298, t, C_WHITE);
    t[0] = 0;
    for (int i = 0; i < nh; i++) {
      char a[16];
      snprintf(a, sizeof(a), i ? " → %d" : "%d", hist[i]);
      strncat(t, a, sizeof(t) - 1 - strlen(t));
    }
    mixDraw(FONT_NUM600_24, FONT_JP500_24, 233 - mixWidth(FONT_NUM600_24, FONT_JP500_24, t) * 0.5f, 330, t, C_LABEL);
    textC(FONT_JP600_30, 62, 244, "やり", C_ACCENT);
    textC(FONT_JP600_30, 62, 276, "直す", C_ACCENT);
    if (g_candN > 0) {
      textC(FONT_JP600_30, 404, 244, "一覧", C_ACCENT);
      bigChevron(404, 268, 26, 1, C_ACCENT, 8.0f, 2.5f);
    }
  }
  drawBottom("変化を記録");
}

static void drawCanGrid(const cantab::View &v, uint8_t mask, int sel, int y0, bool full) {
  for (int i = 0; i < 8; i++) {
    bool have = i < v.dlc;
    uint32_t k = have ? (v.data[i] | ((mask >> i & 1) ? 0x100u : 0) | (i == sel ? 0x200u : 0)) : 0x400u;
    if (!full && k == g_canCellKey[i]) continue;
    g_canCellKey[i] = k;
    const int x = 104 + 86 * (i % 4), y = y0 + 44 * (i / 4);
    // 選択の枠(y-32〜y+8、縁の画素ぶん上下 1 px)まで消す。行の間隔 44 px の中に収め、隣の行に掛けない
    gfx::fillRect(x - 40, y - 33, 80, 43, C_BG);
    if (i == sel) {
      gfx::frameRoundRect((float)(x - 36), (float)(y - 32), 72, 40, 6, 2, C_ACCENT, C_SEL_BG);
    }
    char t[4];
    if (have) snprintf(t, sizeof(t), "%02X", v.data[i]);
    else snprintf(t, sizeof(t), "--");
    textC(FONT_MONO600_38, (float)x, (float)y, t, !have ? C_LABEL : ((mask >> i & 1) ? C_ACCENT : C_WHITE));
  }
}
static void updCanId() {
  g_candN = cantab::candidates(g_candE, g_candM, CAN_LIST_MAX);
  int shown = g_candN < CAN_LIST_MAX ? g_candN : CAN_LIST_MAX;
  if (shown == 0) {  // 候補がなくなった(やり直した): CAN の画面へ
    go(SC_CAN);
    return;
  }
  if (g_canIdx >= shown) g_canIdx = 0;
  cantab::View v;
  if (!cantab::getView(g_candE[g_canIdx], &v)) return;
  bool full = g_full;
  if (full) {
    g_full = false;
    gfx::fillAll(C_BG);
    g_tagsDirty = true;
    drawTopBack();
    char id[16], t[32];
    fmtId(id, sizeof(id), v.id, v.ext);
    snprintf(t, sizeof(t), "%s %s", v.bus ? "B-CAN" : "C-CAN", id);
    fitTextC(FP_NAME, 3, 330, 233, 184, t, C_LABEL2);
    snprintf(t, sizeof(t), "%d / %d", g_canIdx + 1, shown);
    textC(FONT_NUM600_28, 233, 336, t, C_LABEL);
    drawSideArrows();
    drawBottom("詳細");
  }
  drawCanGrid(v, g_candM[g_canIdx], -1, 250, full);
}

// 詳細のグラフ(新しい値が右端)
static void drawDetChart() {
  static uint8_t h[CAN_HIST_SAMPLES];
  static gfx::Pt p[CAN_HIST_SAMPLES + 2];
  const float X0 = 98, X1 = 368, YB = 344, YT = 326;
  gfx::fillRect(86, 320, 294, 28, C_BG);
  int n = cantab::getHistory(g_detEntry, g_detByte, h);
  if (n < 1) return;
  int lo = 255, hi = 0;
  for (int i = 0; i < n; i++) {
    if (h[i] < lo) lo = h[i];
    if (h[i] > hi) hi = h[i];
  }
  int m = 0;
  for (int i = 0; i < n; i++) {
    if (i > 0 && i + 1 < n && h[i] == h[i - 1] && h[i] == h[i + 1]) continue;  // 値が変わらない所は点を省く
    float x = X1 - (X1 - X0) * (float)(n - 1 - i) / (CAN_HIST_SAMPLES - 1);
    float y = (hi > lo) ? YB - (YB - YT) * (float)(h[i] - lo) / (float)(hi - lo) : YB;
    p[m++] = { x, y };
  }
  if (m >= 2) {
    gfx::polyBegin(gfx::Rect{ 86, 320, 380, 348 });
    strokePolyline(p, m, 2.0f);
    gfx::polyFill(rgb(0x2F8BFF));
  }
}
static void updCanDetail(uint32_t now) {
  cantab::View v;
  if (!cantab::getView(g_detEntry, &v)) {
    go(SC_CAN_ID);
    return;
  }
  uint8_t mask = 0;
  for (int i = 0; i < g_candN; i++)
    if (g_candE[i] == g_detEntry) mask = g_candM[i];
  if (g_detByte >= v.dlc) g_detByte = v.dlc ? v.dlc - 1 : 0;
  bool full = g_full;
  if (full) {
    g_full = false;
    gfx::fillAll(C_BG);
    g_tagsDirty = true;
    drawTopBack();
    drawSideArrows();
    g_detValKey = 0xFFFFFFFF;
    g_detChartMs = now - 1000;
  }
  uint32_t vk = v.dlc ? (((uint32_t)g_detByte << 8) | v.data[g_detByte]) : 0xFFFF;
  if (full || vk != g_detValKey) {
    if (!full) gfx::fillRect(60, 132, 346, 58, C_BG);
    char id[16], t[32];
    fmtId(id, sizeof(id), v.id, v.ext);
    snprintf(t, sizeof(t), "%s  b%d", id, g_detByte);
    fitTextC(FP_NAME, 3, 330, 233, 184, t, C_LABEL2);
    gfx::fillRect(130, 286, 206, 34, C_BG);  // 上は格子のマスの消す範囲(〜285)に重ねない
    if (v.dlc) {
      char a[4], b[8];
      snprintf(a, sizeof(a), "%02X", v.data[g_detByte]);
      snprintf(b, sizeof(b), "= %d", v.data[g_detByte]);
      float wa = gfx::textWidth(FONT_NUM600_44, a), wb = gfx::textWidth(FONT_NUM600_28, b);
      float x = 233 - (wa + 12 + wb) * 0.5f;
      gfx::drawText(FONT_NUM600_44, x, 319, a, C_WHITE);
      gfx::drawText(FONT_NUM600_28, x + wa + 12, 319, b, C_LABEL);
    }
    g_detValKey = vk;
    g_detChartMs = now - 1000;
  }
  drawCanGrid(v, mask, g_detByte, 230, full);
  if ((uint32_t)(now - g_detChartMs) >= 2 * CAN_HIST_INTERVAL_MS) {
    g_detChartMs = now;
    drawDetChart();
  }
}
static void updCanRedo() {
  if (!g_full) return;
  g_full = false;
  gfx::fillAll(C_BG);
  g_tagsDirty = true;
  drawTopBack();
  drawName("やり直す");
  char t[48];
  snprintf(t, sizeof(t), "記録 %d 回を消して", cantab::recordCount());
  mixDraw(FONT_NUM600_30, FONT_JP600_30, 233 - mixWidth(FONT_NUM600_30, FONT_JP600_30, t) * 0.5f, 256, t, C_LABEL2);
  mixDraw(FONT_NUM600_30, FONT_JP600_30, 233 - mixWidth(FONT_NUM600_30, FONT_JP600_30, "全部の ID を候補に戻す") * 0.5f, 298, "全部の ID を候補に戻す", C_LABEL2);
  drawBottom("やり直す", C_BAND_RED, C_WHITE);
}

// ================================================================ オープニング(config.h の OPENING_*)
// 時刻だけで進む。描くのは変わった所だけ(輪は 1 マスずつ、自己診断は変わった行だけ)。背景の色が変わるときだけ全部を描き直す。
static uint32_t g_openT0 = 0;       // 始めた時刻
static int g_openLit = 0;           // 点いているマスの数(片側、0〜12)
static int g_openPhase = 0;         // 0 = 点けていく、1 = 止まる(最大輝度)、2 = 消していく + 自己診断
static int g_openRows = 0;          // 出した自己診断の行の数(0〜4)
static uint8_t g_openState[4];      // 出してある行の状態(変わったら描き直す)
static bool g_openNg = false;
static uint16_t g_openBg = 0;
static bool g_openArcs = false;     // 弧(とアイコン)が描いてある
static uint32_t g_openWaterKey = 0, g_openFuelKey = 0;  // 描いてある弧の内容(走行画面と同じ waterKey / fuelKey)

// 仮の回転数 rpmV の色で、下から lit マスを点ける。背景は withBg のときだけ回転数の色(全部描き直すので、点けている間は黒のまま)。
// 戻り値: 背景を塗り直した(弧とアイコンも描き直す必要がある)
static bool openRing(int rpmV, int lit, bool withBg) {
  uint16_t bgT = withBg ? to565(colorAt(BACK_C, rpmV, g_set->shiftRpm)) : C_BG;
  uint16_t litC = to565(colorAt(RING_C, rpmV, g_set->shiftRpm));
  bool refilled = false;
  if (bgT != g_openBg) {
    g_openBg = bgT;
    g_bgCol = bgT;
    gfx::fillAll(bgT);
    for (int k = 0; k < RING_SEGS; k++) g_segCol[k] = 0;
    g_openArcs = false;
    refilled = true;
  }
  for (int k = 0; k < RING_SEGS; k++) {
    uint16_t c = segLit(k, lit) ? litC : bgT;
    if (c != g_segCol[k]) drawSeg(k, c);
  }
  return refilled;
}
// 水温と燃料の弧とアイコンを、走行画面と同じ描き方(drawWaterPanel / drawFuelPanel)で描く。p = 0〜1 で、水温は
// WATER_BAR_MIN_C〜WATER_BAR_MAX_C、燃料は 0〜100 % の仮の値にする。異常の表示(水温が警告温度以上、燃料が FUEL_ALARM_PCT 未満)
// も走行画面と同じに出る(赤で点滅。10/21: 本人の指示「実際と同じように」)。本物の信号の異常の判定(g_waterAlarm など)は壊さない
static void openPanels(float p) {
  Signals v;
  memset(&v, 0, sizeof(v));
  v.waterOk = v.fuelOk = true;
  v.waterC = WATER_BAR_MIN_C + (WATER_BAR_MAX_C - WATER_BAR_MIN_C) * p;
  v.fuelPct = 100.0f * p;
  bool wa = g_waterAlarm, fa = g_fuelAlarm;
  g_waterAlarm = v.waterC >= g_set->warnTempC;
  g_fuelAlarm = v.fuelPct < FUEL_ALARM_PCT;
  uint32_t wk = waterKey(v), fk = fuelKey(v);
  bool clean = !g_openArcs;  // 背景を塗った直後は消す必要がない
  if (clean || wk != g_openWaterKey) {
    drawWaterPanel(v, clean);
    g_openWaterKey = wk;
  }
  if (clean || fk != g_openFuelKey) {
    drawFuelPanel(v, clean);
    g_openFuelKey = fk;
  }
  g_openArcs = true;
  g_waterAlarm = wa;
  g_fuelAlarm = fa;
}
static void openPanelsClear() {  // 弧とアイコンを消す(縮み切ったあと。自己診断の行は弧の範囲に掛かるので、消してから出す)
  if (!g_openArcs) return;
  fillSide(true, C_BG, true);
  fillSide(false, C_BG, true);
  g_openArcs = false;
}
// 自己診断の行 i(0 = CAN、1 = GPS、2 = IMU、3 = タッチ)の状態。下位 4 ビット = 種類(0 = OK 緑、1 = 待ち 灰、2 = NG 赤、3 = 切 灰)
static uint8_t openRowState(int i, const LinkStats &ls, const SysInfo &si, const char **name, const char **state) {
  uint8_t kind = 0;
  switch (i) {
    case 0: *name = "CAN"; *state = "OK"; if (!ls.locked) { *state = "待ち"; kind = 1; } break;
    case 1: *name = "GPS"; *state = "測位";
      if (!si.rc.enabled) { *state = "切"; kind = 3; }
      else if (si.rc.fixType >= 2) { kind = 0; }
      else if (si.rc.gpsOk) { *state = "衛星待ち"; kind = 1; }
      else { *state = "待ち"; kind = 1; }
      break;
    case 2: *name = "IMU"; *state = "OK"; if (!si.rc.enabled) { *state = "切"; kind = 3; } else if (!si.rc.imuOk) { *state = "NG"; kind = 2; } break;
    default: *name = "タッチ"; *state = "OK"; if (!si.touchOk) { *state = "NG"; kind = 2; } break;
  }
  return kind;
}
static void openRow(int i, const LinkStats &ls, const SysInfo &si) {  // 1 行描く(大きい文字。4 行を中央にまとめる)
  const char *name, *state;
  uint8_t kind = openRowState(i, ls, si, &name, &state);
  g_openState[i] = kind;
  if (kind == 2) g_openNg = true;
  const uint16_t dot = kind == 0 ? rgb(0x22E36A) : kind == 2 ? C_ALARM : gfx::blend(C_BG, C_LABEL, 110);
  const uint16_t fg = kind == 2 ? C_ALARM : (kind == 0 ? C_WHITE : gfx::blend(C_BG, C_LABEL, 190));
  const int base = 176 + 50 * i;  // 4 行で y 140〜340 あたり
  gfx::fillRect(96, base - 38, 274, 48, C_BG);
  gfx::fillArc(112, base - 12, 0, 7, 0, 360, dot);
  mixDraw(FONT_NUM600_34, FONT_JP600_34, 134, base, name, fg);
  float sw = mixWidth(FONT_NUM600_34, FONT_JP600_34, state);
  mixDraw(FONT_NUM600_34, FONT_JP600_34, 362 - sw, base, state, fg);
}
static void updOpening(const LinkStats &ls, const SysInfo &si, uint32_t now) {
  if (g_full) {
    g_full = false;
    g_openT0 = now;
    g_openLit = 0;
    g_openPhase = 0;
    g_openRows = 0;
    g_openNg = false;
    g_openBg = 0xFFFF;
    g_openArcs = false;
    g_bgCol = C_BG;
    openRing(800, 0, false);
    openPanels(0.0f);
  }
  const uint32_t t = now - g_openT0;
  const uint32_t tSweep = 12u * OPENING_STEP_MS;       // 点けていく時間
  const uint32_t tHoldEnd = tSweep + OPENING_HOLD_MS;  // 止まる時間の終わり
  const uint32_t tOff = 12u * OPENING_OFF_STEP_MS;     // 消していく時間
  const int sp = g_set->shiftRpm;
  if (g_openPhase == 0) {
    int lit = (int)(t / OPENING_STEP_MS) + 1;
    if (lit > 12) lit = 12;
    if (lit != g_openLit) {
      g_openLit = lit;
      openRing(sp - 11 * RING_STEP_RPM + (lit - 1) * RING_STEP_RPM, lit, false);  // マス k が点く回転数の色
    }
    openPanels(t < tSweep ? (float)t / tSweep : 1.0f);  // 弧は輪と同じ時間で一杯まで
    if (t >= tSweep) {
      g_openPhase = 1;
      openRing(sp + RING_RED_OFFSET, 12, true);  // 赤(背景も。全部描き直すので弧も)
      openPanels(1.0f);
      setLevel(BL_MAX);
    }
  } else if (g_openPhase == 1) {
    if (t >= tSweep + OPENING_HOLD_MS / 2) setLevel(BL_NORMAL);
    openPanels(1.0f);  // 異常の点滅が進む
    if (t >= tHoldEnd) {
      g_openPhase = 2;
      g_openBg = 0xFFFF;
      g_openArcs = false;
      g_bgCol = C_BG;
      gfx::fillAll(C_BG);  // 背景を黒に。輪は点いたまま
      for (int k = 0; k < RING_SEGS; k++) g_segCol[k] = 0;
      for (int k = 0; k < RING_SEGS; k++) drawSeg(k, to565(colorAt(RING_C, sp + RING_RED_OFFSET, sp)));
      g_openLit = 12;
      openPanels(1.0f);
    }
  } else {
    uint32_t t2 = t - tHoldEnd;
    int lit = 12 - (int)(t2 / OPENING_OFF_STEP_MS);  // 輪を上から消す
    if (lit < 0) lit = 0;
    if (lit != g_openLit) {
      for (int k = 0; k < RING_SEGS; k++) if (!segLit(k, lit) && g_segCol[k] != C_BG) drawSeg(k, C_BG);
      g_openLit = lit;
    }
    if (t2 < tOff) openPanels((float)(tOff - t2) / tOff);  // 弧は輪が消える時間で縮む
    else openPanelsClear();  // 縮み切ったら地とアイコンも消す
    int rows = t2 >= tOff ? (int)((t2 - tOff) / OPENING_ROW_MS) + 1 : 0;  // 自己診断の行(弧が消えてから OPENING_ROW_MS ごとに 1 行)
    if (rows > 4) rows = 4;
    while (g_openRows < rows) {
      if (g_openRows == 0) textC(FONT_NUM600_24, 233, 392, "v" METER_FW_VERSION, C_LABEL);  // 表示器の版(config.h)。自己診断の下
      openRow(g_openRows, ls, si);
      g_openRows++;
    }
    for (int i = 0; i < g_openRows; i++) {  // 出してある行は、状態が変わったら描き直す(受信が始まった、衛星をつかんだ)
      const char *n, *st;
      if (openRowState(i, ls, si, &n, &st) != g_openState[i]) openRow(i, ls, si);
    }
  }
  uint32_t total = OPENING_MS + (g_openNg ? OPENING_NG_EXTRA_MS : 0);
  if (t >= total) {
    setLevel(BL_NORMAL);
    go(g_homeSaved);
  }
}

// ================================================================ 「SIM」の印
// 模擬データを受けている間、「SIM」と場面番号を出す(仕様書に無い。車で模擬の値を本物と取り違えないため)。
// 置く所(42×21 の四角): データ画面など = 下に「SIM」、上に場面番号。走行画面 = 輪の真上・真下を 10/21 にマスで埋めたので、
// 弧の無い左右の真ん中(ギアと輪の間): 左に「SIM」、右に場面番号。アナログ計 = 針の通らない右(ブーストの数字と札の右下)に「SIM」だけ
static void updTags(const Signals &s, bool redrawn) {
  // 設定・CAN確認の画面は上下が帯なので印は出さない(帯の上を背景色で消してしまうため)
  if (g_sc != SC_DRIVE && g_sc != SC_BOOST && g_sc != SC_DATA) {
    g_tagsDirty = false;
    g_tagsLost = false;
    g_simShown = false;
    g_sceneShown = 0;
    return;
  }
  bool sim = s.linkUp && s.sim;
  uint8_t scene = (sim && g_sc != SC_BOOST) ? s.scene : 0;
  if (g_tagsDirty) {
    redrawn = true;
    g_tagsDirty = false;
  }
  bool lost = g_tagsLost;  // 印の所が一部消されたかもしれない(アナログ計の帯を消したとき): 消して描き直す
  g_tagsLost = false;
  if (!redrawn && !lost && sim == g_simShown && scene == g_sceneShown) return;
  uint16_t bg = (g_sc == SC_DRIVE) ? g_bgCol : C_BG;
  uint16_t fg = (g_sc == SC_DRIVE) ? labelOn(g_bgCol) : C_LABEL;
  int simX = 212, simY = 437, sceneX = 212, sceneY = 8;  // 四角の左上
  if (g_sc == SC_DRIVE) {
    simX = 76; simY = 223; sceneX = 348; sceneY = 223;
  } else if (g_sc == SC_BOOST) {
    simX = 384; simY = 300;
  }
  if (!redrawn || sim) {
    gfx::fillRect(simX, simY, 42, 21, bg);
    if (g_sc != SC_BOOST) gfx::fillRect(sceneX, sceneY, 42, 21, bg);
  }
  if (sim) {
    textC(FONT_NUM600_20, simX + 21, simY + 17, "SIM", fg);
    if (scene) {
      char t[4];
      snprintf(t, sizeof(t), "%u", scene);
      textC(FONT_NUM600_20, sceneX + 21, sceneY + 18, t, fg);
    }
  }
  g_simShown = sim;
  g_sceneShown = scene;
}

// ================================================================ 長押しの表示(サーキット・タコ・データ。config.h の HOLD_*)
// 押している間: 画面を暗くし、外周が HOLD_MS で一周していき、上に次の画面名。表示の間は画面の描き直しを止める
static const uint16_t C_HOLD_TRACK = rgb(0x1A2433);
static const uint16_t C_HOLD_PILL = rgb(0x0B1220);
static struct {
  bool down;
  int x, y;
  uint32_t t0;      // 触れた時刻(画面を切り替えたら、その時刻)
  bool switched;    // 長押しで画面を切り替えた(離してもタップにしない)
} g_touch;
static float g_holdDeg = 0;  // 外周に描いた角度

static bool isMain(Screen sc) {
  return sc == SC_DRIVE || sc == SC_BOOST || sc == SC_DATA;
}
static Screen nextMain(Screen sc) {
  return sc == SC_DRIVE ? SC_BOOST : sc == SC_BOOST ? SC_DATA : SC_DRIVE;
}
static const char *mainName(Screen sc) {
  return sc == SC_DRIVE ? "サーキット" : sc == SC_BOOST ? "タコ" : "データ";
}
static void drawHoldShow() {  // 暗くして、外周の地と次の画面名
  uint16_t *f = gfx::frame();
  for (int i = 0; i < gfx::W * gfx::H; i++) f[i] = gfx::blend(f[i], 0x0000, 153);
  gfx::addDirty(0, 0, gfx::W, gfx::H);
  gfx::fillArc(233, 233, 221, 233, 0, 360, C_HOLD_TRACK);
  const char *nm = mainName(nextMain(g_sc));
  const FontData &fn = strlen(nm) <= 9 ? FONT_JP600_56 : FONT_JP600_48;  // 3 文字まで 56 px(「サーキット」は 48 px)
  const float w = gfx::textWidth(fn, nm) + 96;
  const float x = 233 - w * 0.5f;
  gfx::frameRoundRect(x, 74, w, 78, 39, 3, C_ACCENT, C_HOLD_PILL);
  gfx::Pt p[3] = { { x + 30, 95 }, { x + 54, 113 }, { x + 30, 131 } };
  gfx::polyBegin(gfx::Rect{ (int16_t)(x + 26), 90, (int16_t)(x + 58), 136 });
  gfx::polyAdd(p, 3);
  gfx::polyFill(C_ACCENT);
  gfx::drawText(fn, x + 66, 113 + fn.px * 0.36f, nm, C_WHITE);
  g_holdDeg = 0;
}
static void drawHoldProgress(float frac) {
  float deg = 360.0f * (frac > 1 ? 1 : frac);
  if (deg <= g_holdDeg + 0.5f) return;
  gfx::fillArc(233, 233, 221, 233, 270 + g_holdDeg, 270 + deg, C_ACCENT);
  g_holdDeg = deg;
}
// update の最初で呼ぶ。長押しで画面を切り替える
static void holdStep(uint32_t now) {
  if (!g_touch.down || !isMain(g_sc)) return;
  if ((uint32_t)(now - g_touch.t0) >= HOLD_MS) {
    g_lastActMs = now;
    go(nextMain(g_sc));
    g_touch.t0 = now;
    g_touch.switched = true;
  }
}
// 画面を描いたあとで呼ぶ。押している間の表示
static void holdShow(uint32_t now) {
  if (!g_touch.down || !isMain(g_sc)) return;
  const uint32_t el = now - g_touch.t0;
  if (!g_holdShown) {
    if (el < HOLD_SHOW_MS && !g_touch.switched) return;
    drawHoldShow();
    g_holdShown = true;
  }
  drawHoldProgress((float)el / HOLD_MS);
}

// ================================================================ 入口
void begin(Settings *settings, uint8_t home) {
  g_set = settings;
  gfx::setTabularDigits(DIGITS_TABULAR != 0);
  buildRing();
  for (int k = 0; k < RING_SEGS; k++) g_segCol[k] = 0;
  g_homeSaved = isMain((Screen)home) ? (Screen)home : SC_DRIVE;
  g_homeCur = g_homeSaved;
  go(OPENING_MS > 0 ? SC_OPENING : g_homeSaved);
}
uint8_t homeScreen() {
  return g_homeSaved;
}

static void leaveEdit() {  // 保存して項目の画面へ
  const Item &it = ITEMS[g_setItem];
  if (it.field && g_set->*(it.field) != g_editOrig && !g_saveTried) {
    g_saveTried = true;
    if (!platSaveSettings(*g_set)) {  // 保存に失敗: 1 回目はここに留まって知らせる。もう一度「決定」で抜ける
      g_saveFailed = true;
      drawEditDesc();
      return;
    }
  }
  go(SC_SET_LIST);
}
static void cancelEdit() {  // 変えずに項目の画面へ
  const Item &it = ITEMS[g_setItem];
  if (it.field && g_set->*(it.field) != g_editOrig) {
    g_set->*(it.field) = g_editOrig;
    if (it.field == &Settings::brightnessPct) platSetBrightness(g_editOrig);
  }
  go(SC_SET_LIST);
}

void tap(int x, int y, uint32_t nowMs) {
  g_lastActMs = nowMs;
  const Zone z = zoneAt(x, y);
  switch (g_sc) {
    case SC_OPENING:  // オープニングを飛ばす
      setLevel(BL_NORMAL);
      go(g_homeSaved);
      break;
    case SC_DRIVE:
    case SC_BOOST:
    case SC_DATA:  // タップ = 設定(前回見ていた項目)
      g_homeCur = g_sc;
      go(SC_SET_LIST);
      break;
    case SC_SET_LIST: {
      const Item &it = ITEMS[g_setItem];
      if (z == Z_TOP) {
        go(g_homeCur);
      } else if (z == Z_LEFT || z == Z_RIGHT) {
        g_setItem = (g_setItem + N_ITEMS + (z == Z_LEFT ? -1 : 1)) % N_ITEMS;
        g_full = true;
      } else if (it.kind == K_VALUE) {
        g_editOrig = g_set->*(it.field);
        g_saveTried = false;
        g_saveFailed = false;
        go(SC_SET_EDIT);
      } else if (it.kind == K_TIME) {
        go(SC_SET_RESET);
      } else if (it.kind == K_INFO || it.kind == K_RC) {
        g_infoItem = 0;
        go(it.kind == K_INFO ? SC_SET_INFO : SC_SET_RC);
      } else {
        go(SC_CAN);
      }
      break;
    }
    case SC_SET_EDIT: {
      const Item &it = ITEMS[g_setItem];
      if (z == Z_TOP) {
        cancelEdit();
      } else if (z == Z_BOT) {
        leaveEdit();
      } else {
        int v;
        if (it.unit == U_ONOFF) v = z == Z_RIGHT ? 1 : 0;
        else v = g_set->*(it.field) + (z == Z_RIGHT ? 1 : -1) * it.step;
        if (v < it.minV) v = it.minV;
        if (v > it.maxV) v = it.maxV;
        if (v != g_set->*(it.field)) {
          g_set->*(it.field) = v;
          g_saveTried = false;
          g_saveFailed = false;
          if (it.field == &Settings::brightnessPct) platSetBrightness(v);  // 明るさは変えたその場で反映
          drawEditValue();
          drawEditDesc();
        }
      }
      break;
    }
    case SC_SET_RESET:
      if (z == Z_TOP) {
        go(SC_SET_LIST);
      } else if (z == Z_BOT) {
        trip::resetTimes();
        platSaveTimes();
        go(SC_SET_LIST);
      }
      break;
    case SC_SET_INFO:
    case SC_SET_RC:
      if (z == Z_TOP) {
        go(SC_SET_LIST);
      } else if (z == Z_LEFT || z == Z_RIGHT) {
        g_infoItem = (g_infoItem + INFO_N + (z == Z_LEFT ? -1 : 1)) % INFO_N;
        g_full = true;
      } else if (z == Z_BOT && g_sc == SC_SET_RC && g_infoItem == 6 && g_rcResetShown) {
        platResetImuCal();
        g_infoMs = nowMs - 1000;  // 次の update で描き直す
      }
      break;
    case SC_CAN:
      if (z == Z_TOP) {
        go(SC_SET_LIST);
      } else if (z == Z_BOT) {
        cantab::record();
        g_full = true;
      } else if (z == Z_LEFT && cantab::recordCount() > 0) {
        go(SC_CAN_REDO);
      } else if (z == Z_RIGHT && cantab::recordCount() > 0 && g_candN > 0) {
        g_canIdx = 0;
        go(SC_CAN_ID);
      }
      break;
    case SC_CAN_ID: {
      int shown = g_candN < CAN_LIST_MAX ? g_candN : CAN_LIST_MAX;
      if (z == Z_TOP) {
        go(SC_CAN);
      } else if ((z == Z_LEFT || z == Z_RIGHT) && shown > 0) {
        g_canIdx = (g_canIdx + shown + (z == Z_LEFT ? -1 : 1)) % shown;
        g_full = true;
      } else if (z == Z_BOT && shown > 0 && g_canIdx < shown) {
        g_detEntry = g_candE[g_canIdx];
        g_detByte = 0;
        for (int b = 0; b < 8; b++)
          if (g_candM[g_canIdx] >> b & 1) {
            g_detByte = b;
            break;
          }
        go(SC_CAN_DETAIL);
      }
      break;
    }
    case SC_CAN_DETAIL:
      if (z == Z_TOP) {
        go(SC_CAN_ID);
      } else if (z == Z_LEFT || z == Z_RIGHT) {
        cantab::View v;
        int n = cantab::getView(g_detEntry, &v) && v.dlc ? v.dlc : 8;
        g_detByte = (g_detByte + n + (z == Z_LEFT ? -1 : 1)) % n;
      }
      break;
    case SC_CAN_REDO:
      if (z == Z_TOP) {
        go(SC_CAN);
      } else if (z == Z_BOT) {
        cantab::clearRecord();
        go(SC_CAN);
      }
      break;
    default:
      break;
  }
}

void touchDown(int x, int y, uint32_t nowMs) {
  g_lastActMs = nowMs;
  g_touch.down = true;
  g_touch.x = x;
  g_touch.y = y;
  g_touch.t0 = nowMs;
  g_touch.switched = false;
}
void touchUp(uint32_t nowMs) {
  if (!g_touch.down) return;
  g_touch.down = false;
  g_lastActMs = nowMs;
  if (g_holdShown) {  // 暗くした表示を消す(今の画面を描き直す)
    g_holdShown = false;
    g_full = true;
  }
  if (!g_touch.switched) tap(g_touch.x, g_touch.y, nowMs);
}

void update(const Signals &s, const LinkStats &ls, const PerfStats &ps, const SysInfo &si, uint32_t now) {
  // 異常の判定(5.6)。境目でちらつかないよう、戻る側に少し幅を持たせる
  if (!s.waterOk) g_waterAlarm = false;
  else if (s.waterC >= g_set->warnTempC) g_waterAlarm = true;
  else if (s.waterC < g_set->warnTempC - WATER_ALARM_HYST_C) g_waterAlarm = false;
  if (!s.fuelOk) g_fuelAlarm = false;
  else if (s.fuelPct < FUEL_ALARM_PCT) g_fuelAlarm = true;
  else if (s.fuelPct >= FUEL_ALARM_PCT + FUEL_ALARM_HYST_PCT) g_fuelAlarm = false;

  // 設定から時間で戻る(2.3)。戻る先は覚えている画面。CAN確認は戻らない(操作を待っていることがある)
  if (g_sc == SC_SET_LIST || g_sc == SC_SET_EDIT || g_sc == SC_SET_INFO || g_sc == SC_SET_RC || g_sc == SC_SET_RESET) {
    if (!g_touch.down && (uint32_t)(now - g_lastActMs) >= (uint32_t)g_set->autoReturnMin * 60000u) {
      if (g_sc == SC_SET_EDIT && ITEMS[g_setItem].field && g_set->*(ITEMS[g_setItem].field) != g_editOrig) platSaveSettings(*g_set);
      g_homeCur = g_homeSaved;
      go(g_homeSaved);
    }
  }
  holdStep(now);
  // 覚える画面: サーキット・タコ・データのどれかを HOME_CONFIRM_MS 続けて見ていたら、それを覚える
  if (isMain(g_sc)) {
    if (g_sc != g_homeCur || g_homeRestart) {
      g_homeRestart = false;
      g_homeCur = g_sc;
      g_homeSinceMs = now;
    } else if ((uint32_t)(now - g_homeSinceMs) >= HOME_CONFIRM_MS) {
      g_homeSaved = g_sc;
    }
  } else {
    g_homeSinceMs = now;  // 設定から戻ったら数え直す
  }

  g_alarmOff = ((now / ALARM_BLINK_HALF_MS) & 1) != 0;  // 異常の表示の点滅
  bool redrawn = g_full;
  blinkPoll(s, now);  // 回転数の点滅(サーキットとタコだけ。ほかの画面では点灯側に戻る)
  peakTrack(s, now);  // 回転数のピークホールド(どの画面でも記録する)
  idleTrack(s, now);  // 3000rpm 以下の経過時間(どの画面でも)
  infoTrack(ls, si, now);  // 受信の状態・RaceChrono の判定のための記録(どの画面でも)
  if (g_sc == SC_OPENING) {
    updOpening(ls, si, now);
    if (g_sc == SC_OPENING) return;  // 「SIM」の印は出さない
    redrawn = g_full;  // 終わって次の画面に移った。同じ回にその画面を描く(オープニングの絵を 1 回ぶん残さない)
    blinkPoll(s, now);
  }
  if (g_holdShown) {  // 長押しの表示の間は描き直さない(外周だけ進める)
    holdShow(now);
    return;
  }
  if (g_sc == SC_DRIVE) {
    updDrive(s, now);
  } else if (g_sc == SC_BOOST) {
    updBoost(s, now);
  } else if (g_full || (int32_t)(now - g_nextMs) >= 0) {  // サーキットとタコ以外の画面は 20 回/秒まで
    g_nextMs = now + 50;
    switch (g_sc) {
      case SC_DATA: updData(s, ls, si, now); break;
      case SC_CAN: updCan(); break;
      case SC_CAN_ID: updCanId(); break;
      case SC_CAN_DETAIL: updCanDetail(now); break;
      case SC_CAN_REDO: updCanRedo(); break;
      case SC_SET_LIST: updSetList(ls, ps, si, now); break;
      case SC_SET_EDIT: updSetEdit(); break;
      case SC_SET_RESET: updSetReset(); break;
      case SC_SET_INFO: updSetInfo(ls, ps, si, now); break;
      case SC_SET_RC: updSetRc(si, now); break;
      default: break;
    }
  }
  if (g_full) return;  // update の中で画面が切り替わった。次の回に描く
  updTags(s, redrawn);
  holdShow(now);
}

void testRedraw() {
  g_full = true;
}

void testShow(Screen sc, int a, int b) {
  if (sc == SC_CAN_DETAIL) {
    g_detEntry = a;
    g_detByte = b;
  } else if (sc == SC_SET_EDIT) {
    g_setItem = a;
    g_editOrig = ITEMS[a].field ? g_set->*(ITEMS[a].field) : 0;
    g_saveTried = false;
    g_saveFailed = false;
  } else if (sc == SC_SET_LIST) {
    g_setItem = a;
  } else if (sc == SC_SET_INFO || sc == SC_SET_RC) {
    g_infoItem = a;
  } else if (sc == SC_CAN_ID) {
    g_canIdx = a;
  }
  if (isMain(sc)) {
    g_homeCur = sc;
    g_homeRestart = true;
  }
  g_touch.down = false;
  go(sc);
}

void testTouch(uint32_t nowMs) {
  g_lastActMs = nowMs;
}
int testSetItem() {
  return g_setItem;
}
bool testHolding() {
  return g_holdShown;
}

}  // namespace ui

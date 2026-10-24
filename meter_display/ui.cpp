// ui.cpp - 画面(サーキット・タコ・データ・設定・CAN確認)
//
// 座標・大きさ・色は ui_spec.md(キャンバス版 23)の値。文字の位置(ベースライン)は、キャンバスの絵を
// ブラウザーで描いて測った値。章番号は ui_spec.md のもの。
// 仕様書のあとで変えた所: 枠をなくした(2026-10-13)、水温と燃料を輪の内側の円弧にした、ブースト計のページを足した(2026-10-14)。
// 10/23: 画面の名前をサーキット(走行)・タコ(ブースト計)・データにした。3 つはスワイプで切り替え、長押しで設定(2.1.0)。
// 設定は 1 画面 1 項目(2.3.0: 7 項目。RaceChrono と CAN確認は受信の状態の中)で、上の帯・下の帯・真ん中の左右の 4 つの範囲で
// 操作する(config.h の ZONE_*)。
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

// ASCII は fa、それ以外(日本語)は fj で描く。「°」(U+00B0)は英数字の書体で描く(2.4.3: 温度の「°C」。日本語の書体だと間が空く)
static bool isAsciiLike(const char *s) {
  return ((uint8_t)s[0]) < 0x80 || ((uint8_t)s[0] == 0xC2 && (uint8_t)s[1] == 0xB0);
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


static bool g_holdShown = false;  // 長押しの途中の表示(案内のまま、スワイプの札を灰にして外周。2.4.0)を出している
static bool g_guideShown = false;  // 触れたときの案内(上の「‹ スワイプ ›」、下の「長押し ▶ 設定」)を出している(2.3.0)
static bool g_pageFrame = false;  // 直前の update が、サーキット・タコをふだんどおり描いた回(pageFrame)
static Screen g_homeSaved = SC_DRIVE;  // 覚えた画面(次の起動・時間で戻るとき)
static Screen g_homeCur = SC_DRIVE;    // 今の(最後に見た)サーキット・タコ・データ(設定の「設定」で戻る先)
static uint32_t g_homeSinceMs = 0;  // 今のサーキット・タコ・データを出し始めた時刻(覚える画面の判定)
static bool g_homeRestart = false;  // 次の update で g_homeSinceMs を数え直す(試験の testShow)
static float g_hideA = -1.0f;    // タコのオープニング: 文字盤のうち、この角度 [度] から OPT_HIDE_A1 までを隠している(-1 = 隠していない。2.5.0)
static bool g_barForce = false;  // 次の updBoost で、ブーストの帯を変わっていなくても消して描き直す(タコのオープニングで文字盤を写したとき)
static void go(Screen sc) {
  g_sc = sc;
  g_full = true;
  g_nextMs = 0;
  g_holdShown = false;
  g_guideShown = false;
  g_hideA = -1.0f;  // 2.5.0: 画面が替わったら文字盤は隠さない(タコのオープニングを飛ばしたとき)
  g_barForce = false;
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
// 回転数 rpm(ok = 信号あり)から、点滅・最大輝度の段階を決める。onPage = サーキット・タコ(ほかの画面では点灯側に戻る)。
// 2.5.0: オープニングは仮の回転数でこれを呼ぶ(本物と同じ決まりで点滅・最大輝度にする)
static void blinkRun(bool ok, int rpm, bool onPage, uint32_t now) {
  const int sp = g_set->shiftRpm;
  const bool solid = ok && onPage && rpm >= sp + RING_SOLID_OFFSET;  // 点滅せず最大輝度
  const bool active = ok && onPage && !solid && rpm >= sp + RING_BLINK_OFFSET;
  if (!active) {
    g_blinkActive = false;
    g_blinkOff = false;
    g_blinkOn = 0;
    setLevel(solid ? BL_MAX : BL_NORMAL);
    return;
  }
  const uint32_t on = blinkOnFor(rpm, sp);
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
void blinkPoll(const Signals &s, uint32_t now) {
  if (g_sc == SC_OPENING) return;  // オープニングの間は、オープニングが仮の回転数で blinkRun を呼ぶ
  blinkRun(s.rpmOk, s.rpm, g_sc == SC_DRIVE || g_sc == SC_BOOST, now);
}
static void blinkReset() {  // 点滅をやめて設定の明るさに戻す(オープニングの終わり。次の画面の段階は次の blinkPoll で決まる)
  g_blinkActive = false;
  g_blinkOff = false;
  g_blinkOn = 0;
  setLevel(BL_NORMAL);
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
static int peakTriKey(const Signals &s);
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
    g_triKey = peakTriKey(s);
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
  const int tk = peakTriKey(s);
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
// 2.4.0(本人の指示 10/24「シフトポイントを超えたときはメモリと針を全て赤に」): 回転数がシフト点以上の間は、回転計の目盛線・数字・針を
// 全部この赤にする(シフト点 − TACH_ALL_RED_HYST_RPM 未満で元に戻す)
static const uint16_t C_TICK_AR = rgb(0xE5281E);   // 1000 ごとの目盛線
static const uint16_t C_TICK_ARM = rgb(0xB01F17);  // 500 ごと
static const uint16_t C_TICK_ARS = rgb(0x7E1712);  // 250 ごと
static const uint16_t C_DIAL_AR = rgb(0xFF3B30);   // 数字
static const uint16_t C_NEEDLE_AR = rgb(0xFF2A2A); // 針

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
static bool g_peakShowOk = false;   // 印を出す時間を数え始めたことがある(g_peakShowMs が使える。2.4.4)
static uint32_t g_peakShowMs = 0;   // 印を出す時間を最後に数え始めた時刻(ここから TACH_PEAK_SHOW_MS 印を出す。2.4.4)
static bool g_peakShow = false;     // 印を出す(peakTrack が決める。2.4.4)
static float g_peakLow = -1.0f;     // 前に数え始めてからの最低の回転数(2.4.5。-1 = まだ)
static bool g_tachRed = false;      // 回転数がシフト点以上(回転計の目盛と針を赤にする。2.4.0)
static bool g_faceRed = false;      // g_face(と画面)の回転計を赤で描いた
// 回転計を赤にするか(update のたびに呼ぶ。どの画面でも)。シフト点以上で赤、シフト点 − TACH_ALL_RED_HYST_RPM 未満で戻す
static void tachRedTrack(const Signals &s) {
  const int sp = g_set->shiftRpm;
  if (!s.rpmOk) g_tachRed = false;
  else if (s.rpm >= sp) g_tachRed = true;
  else if (s.rpm < sp - TACH_ALL_RED_HYST_RPM) g_tachRed = false;
}

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
// ピークホールドの記録(update のたびに呼ぶ)。回転数が最高以上なら最高を更新(印は針・輪の先といっしょに動く)。最後に更新してから
// TACH_PEAK_HOLD_MS 止まり、そのあと TACH_PEAK_FALL_RPM_PER_S で今の回転数まで下がる。同じ時刻で 2 回呼んでも結果は変わらない
// (最初から描いたものと比べるため)。
// 2.4.4: 印を出すのは、回転が上がるのが終わってから(回転が落ち始めてから)TACH_PEAK_SHOW_MS の間。2.4.5: 印が PEAK_MIN_RPM を切ったら消す
// (peakVisible)。
// 2.4.5(本人「今度ピークホールドが消えていません」): 出す時間を数え始めるのは、回転数がはっきり上がったとき(PEAK_MIN_RPM 以上で、
// 前に数え始めてからの最低より PEAK_RISE_RPM 以上高い)だけにした。2.4.4 はピークを付け直すたび(回転数が少しでも上がるたび)に
// 数え始めていて、3000 rpm 以上で走っている間は、ゆらぎで印が消えなかった
static void peakTrack(const Signals &s, uint32_t now) {
  if (!s.rpmOk) {
    g_peakRpm = -1.0f;
    g_peakLastMs = now;
    g_peakShowOk = false;
    g_peakShow = false;
    g_peakLow = -1.0f;
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
  if (g_peakLow < 0 || rpm < g_peakLow) g_peakLow = rpm;
  if (rpm >= (float)PEAK_MIN_RPM && rpm >= g_peakLow + (float)PEAK_RISE_RPM) {
    g_peakShowOk = true;
    g_peakShowMs = now;
    g_peakLow = rpm;
  }
  g_peakShow = g_peakShowOk && (uint32_t)(now - g_peakShowMs) < TACH_PEAK_SHOW_MS;
}
// ピークホールドの印を出すか。2.4.0(本人の指示 10/24「回転数が高い状態では上がっても下がっても表示し続けて」): 回転数が上がっていても
// 出す(2.2.0〜2.3.x は、下がっている間だけ出していた)。2.4.4: 回転が上がるのが終わってから TACH_PEAK_SHOW_MS の間(peakTrack)。
// 2.4.5(本人「ピークホールドが 3000 以下に下がった場合は消してしまっていいです」): 印が PEAK_MIN_RPM を切ったら消す
static bool peakVisible(const Signals &s) {
  return s.rpmOk && g_peakRpm >= (float)PEAK_MIN_RPM && g_peakShow;
}
static int peakKey(const Signals &s) {  // 回転計の印の角度 [1/4 度]。-1 = 出さない(印は針の下に描く)
  return peakVisible(s) ? (int)(tachAngle(g_peakRpm) * 4.0f + 0.5f) : -1;
}
static int peakTriKey(const Signals &s) {  // サーキットの三角形の左側の角度 [1/4 度]。-1 = 出さない
  if (!peakVisible(s)) return -1;
  const int sp = g_set->shiftRpm;
  const float first = (float)(sp - 11 * RING_STEP_RPM);
  const float pos = clampf((g_peakRpm - first) / RING_STEP_RPM + 1.0f, 1.0f, 12.0f);
  // 2.4.0: 点いたマスの上にも出す(2.2.0〜2.3.x は、三角形の所のマスが点いている間は出さなかった)。
  // 2.4.3: 11 マス目から先は 2 マスぶん進め、シフト点(12 マス目)で真上に着く(真上のマスは 12 マス目と一緒に点くので、前は 12 マス目で
  // 止まって輪の天井まで行かないように見えた。本人「ピークホールドが天井までいかないように見える」)。左右の三角形は真上で重なる
  const float at = pos <= 11.0f ? pos : 11.0f + 2.0f * (pos - 11.0f);
  return (int)lroundf((90.0f + RING_PITCH_DEG * at) * 4.0f);
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
  gfx::polyFill(g_faceRed ? C_NEEDLE_AR : C_NEEDLE);
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

// ---- タコのオープニングで文字盤を隠す(2.5.0)。中心から見た向きで、隠す範囲・出す範囲を決める
// (3 時が 0°、時計回り。g_face はいつも文字盤の全部で、隠すのは画面メモリーだけ)
static const float OPT_HIDE_A0 = 100.0f, OPT_HIDE_A1 = 340.0f;  // 隠す範囲(0 の数字の左から 8 の数字の右まで。目盛とブーストの輪が入る)
struct Wedge {  // 向き a0〜a1(時計回り)。180° 以下に分けて持つ
  int n;
  float ux0[3], uy0[3], ux1[3], uy1[3];
};
static Wedge wedgeMake(float a0, float a1) {
  Wedge w;
  w.n = 0;
  while (a1 > a0 && w.n < 3) {
    const float b = a1 - a0 > 170.0f ? a0 + 170.0f : a1;
    const float r0 = a0 * PI_F / 180.0f, r1 = b * PI_F / 180.0f;
    w.ux0[w.n] = cosf(r0);
    w.uy0[w.n] = sinf(r0);
    w.ux1[w.n] = cosf(r1);
    w.uy1[w.n] = sinf(r1);
    w.n++;
    a0 = b;
  }
  return w;
}
// 中心から (dx, dy) の向きが範囲に入るか(始まりの向きは入る、終わりの向きは入らない)
static bool wedgeHas(const Wedge &w, float dx, float dy) {
  for (int i = 0; i < w.n; i++) {
    if (w.ux0[i] * dy - w.uy0[i] * dx >= 0.0f && dx * w.uy1[i] - dy * w.ux1[i] > 0.0f) return true;
  }
  return false;
}
// 中心から画面の端まで、向き a0〜a1 の範囲を包む四角
static gfx::Rect wedgeRect(float a0, float a1) {
  gfx::Pt p[8];
  int n = 0;
  p[n++] = { 233.0f, 233.0f };
  auto add = [&](float a) {
    const float r = a * PI_F / 180.0f;
    p[n++] = { 233.0f + 236.0f * cosf(r), 233.0f + 236.0f * sinf(r) };
  };
  add(a0);
  add(a1);
  for (float q = ceilf(a0 / 90.0f) * 90.0f; q < a1 && n < 8; q += 90.0f) add(q);
  return ptsRect(p, n);
}
// 画面メモリーの r の中で、隠している範囲(g_hideA〜OPT_HIDE_A1)を黒にする
static void hideApply(const gfx::Rect &r) {
  if (g_hideA < 0) return;
  const Wedge w = wedgeMake(g_hideA, OPT_HIDE_A1);
  uint16_t *f = gfx::frame();
  for (int y = r.y0; y < r.y1; y++) {
    const float dy = (float)y + 0.5f - 233.0f;
    for (int x = r.x0; x < r.x1; x++) {
      if (wedgeHas(w, (float)x + 0.5f - 233.0f, dy)) f[y * gfx::W + x] = C_BG;
    }
  }
}
// 向き a0〜a1 の文字盤を g_face から画面メモリーへ写す(隠していた所を出す)
static void wedgeReveal(float a0, float a1) {
  if (a1 <= a0 || !g_face) return;
  const Wedge w = wedgeMake(a0, a1);
  const gfx::Rect r = wedgeRect(a0, a1);
  uint16_t *f = gfx::frame();
  for (int y = r.y0; y < r.y1; y++) {
    const float dy = (float)y + 0.5f - 233.0f;
    for (int x = r.x0; x < r.x1; x++) {
      if (wedgeHas(w, (float)x + 0.5f - 233.0f, dy)) f[y * gfx::W + x] = g_face[y * gfx::W + x];
    }
  }
  gfx::addDirty(r.x0, r.y0, r.x1, r.y1);
}

static void faceTake(const gfx::Rect &r) {  // 画面メモリーの四角を文字盤に写す
  if (!g_face) return;
  const uint16_t *f = gfx::frame();
  if (g_hideA >= 0) {  // 文字盤を隠している間: 隠している画素は写さない(g_face は文字盤のまま)
    const Wedge w = wedgeMake(g_hideA, OPT_HIDE_A1);
    for (int y = r.y0; y < r.y1; y++) {
      const float dy = (float)y + 0.5f - 233.0f;
      for (int x = r.x0; x < r.x1; x++) {
        if (!wedgeHas(w, (float)x + 0.5f - 233.0f, dy)) g_face[y * gfx::W + x] = f[y * gfx::W + x];
      }
    }
    return;
  }
  for (int y = r.y0; y < r.y1; y++) memcpy(g_face + (size_t)y * gfx::W + r.x0, f + (size_t)y * gfx::W + r.x0, (size_t)(r.x1 - r.x0) * sizeof(uint16_t));
}

// 文字盤(針・帯・ピークの印を除く)
static void drawBoostFace(const Signals &s) {
  gfx::fillAll(C_BG);
  // 回転計(0〜TACH_MAX_RPM)。数字は ×1000。シフト点 + TACH_RED_OFFSET から赤(目盛線は暗い赤、数字は赤)。
  // 2.4.0: 回転数がシフト点以上の間(g_tachRed)は、目盛線と数字を全部赤(C_TICK_AR など)
  g_faceRed = g_tachRed;
  {
    const int redFrom = g_set->shiftRpm + TACH_RED_OFFSET;
    const int nTick = TACH_MAX_RPM / TACH_TICK_STEP_RPM;
    const bool all = g_faceRed;
    for (int i = 0; i <= nTick; i++) {
      int rpm = i * TACH_TICK_STEP_RPM;
      float a = tachAngle((float)rpm);
      bool red = rpm >= redFrom;
      if (rpm % TACH_LABEL_STEP_RPM == 0) {
        gfx::fillArc(233, 233, TACH_TICK_R_IN, TACH_TICK_R_OUT, a - TACH_TICK_HALF_DEG, a + TACH_TICK_HALF_DEG, all ? C_TICK_AR : red ? C_TICK_R : C_TICK);
        char t[16];
        snprintf(t, sizeof(t), "%d", rpm / 1000);
        float ar = a * PI_F / 180.0f;
        textC(FONT_DIAL600_60, 233 + TACH_LABEL_R * cosf(ar), 233 + TACH_LABEL_R * sinf(ar) + FONT_DIAL600_60.px * 0.36f, t, all ? C_DIAL_AR : red ? C_DIAL_RED : C_DIAL);
      } else if (rpm % 500 == 0) {
        gfx::fillArc(233, 233, TACH_TICK_R_IN_M, TACH_TICK_R_OUT, a - TACH_TICK_HALF_DEG_M, a + TACH_TICK_HALF_DEG_M, all ? C_TICK_ARM : red ? C_TICK_RM : C_TICK_M);
      } else {
        gfx::fillArc(233, 233, TACH_TICK_R_IN_S, TACH_TICK_R_OUT, a - TACH_TICK_HALF_DEG_S, a + TACH_TICK_HALF_DEG_S, all ? C_TICK_ARS : red ? C_TICK_RS : C_TICK_S);
      }
    }
  }
  platBlinkTick();  // 2.4.0: 赤にする・戻すときはシフト点の近く(点滅の途中)で全部を描き直すので、合間に点滅の切り替えの時刻を見る
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
  if (g_tachRed != g_faceRed) g_full = true;  // 回転計を赤に・赤から戻す: 文字盤から描き直す(2.4.0)
  if (g_full) {
    g_full = false;
    drawBoostFace(s);
    platBlinkTick();
    if (!g_face) g_face = (uint16_t *)platAlloc((size_t)gfx::W * gfx::H * sizeof(uint16_t));
    if (g_face) {
      memcpy(g_face, gfx::frame(), (size_t)gfx::W * gfx::H * sizeof(uint16_t));
      g_faceOk = true;
      platBlinkTick();
    }
    g_needleKey = -1;
    g_tachKey = -1;
    g_peakKey = -1;
    g_tagsDirty = true;
  }
  int nk = needleKey(s), tk = tachKey(s), pk = peakKey(s), bk = boostNumKey(s);
  uint32_t gk = badgeKey(s);
  bool faceItems = bk != g_boostNumKey || gk != g_badgeKey;
  if (nk == g_needleKey && tk == g_tachKey && pk == g_peakKey && !faceItems && !g_barForce) return;
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
    hideApply(r);  // タコのオープニングで文字盤を隠している所は黒のまま(2.5.0)
    erased[ne++] = r;
    if (g_sc == SC_BOOST) g_tagsLost = true;  // 「SIM」の印に掛かることがある(updTags が消して描き直す)
  };
  if (g_tachKey >= 0) erase(tachRect(g_tachKey));
  bool barRedo = nk != g_needleKey || g_barForce, peakRedo = pk != g_peakKey;
  g_barForce = false;
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
// 中: 水温・燃料。下: ▲ = 回転数が TIME_RPM を超えていた時間、▼ = 以下の時間(累計。電源を切っても続く。設定の「時間の記録」でリセット)。
// 外周: 回転数の輪(2.5.0。config.h の DATA_RING_*)。中の絵は輪の内側に収め、消すときも輪に掛からないようにする(fillRectData)
static int g_dWaterKey, g_dFuelKey;
static uint32_t g_dAboveKey, g_dBelowKey;
static char g_dStatKey[3][64];

// ---- 回転数の輪(2.5.0。本人「実際の画面にも回転数のグラフを追加したい」「アニメーションと同じ幅で」)。真上から時計回りに
// 0〜DATA_RING_MAX_RPM で一周。点いている所の色はサーキットの輪と同じ決まり(colorAt)、残りは輪の地。画素の色は今の状態だけで決まる
// (gfx::ringPaint)ので、変わった角度の所だけを塗り直せばよい。色が変わったら点いている所を全部塗り直す。書き換えた範囲を小さく
// するため(画面へ送る量を減らす)、DR_STEP_DEG ずつに分けて塗る
static const uint16_t C_DR_TRACK = rgb(0x1A2433);  // 輪の地(データのオープニングの輪・長押しの外周の地と同じ)
static const float DR_STEP_DEG = 10.0f;
static const int DR_Q_PER_DEG = 4;  // 点いている角度は 1/4 度きざみ(1 きざみ = 約 22 rpm)
static int g_drQ = -1;              // 描いてある点いている角度(1/DR_Q_PER_DEG 度の数。-1 = まだ描いていない)
static uint16_t g_drCol = 0;        // 描いてある点いている色
static int drQ(const Signals &s) {  // 今の点いている角度(信号なしは 0 = 全部が地)
  if (!s.rpmOk || s.rpm <= 0) return 0;
  const long q = lroundf((float)s.rpm * (360.0f * DR_Q_PER_DEG) / (float)DATA_RING_MAX_RPM);
  return q > 360L * DR_Q_PER_DEG ? 360 * DR_Q_PER_DEG : (int)q;
}
// 真上から時計回りの d0〜d1 [度](負・360 超えも可)を、点いている角度 q と色 col で塗る
static void drPaint(float d0, float d1, int q, uint16_t col) {
  if (d1 - d0 > 360.0f) d1 = d0 + 360.0f;
  const float lit = (float)q / DR_Q_PER_DEG;
  for (float a = d0; a < d1; a += DR_STEP_DEG) {
    const float b = a + DR_STEP_DEG < d1 ? a + DR_STEP_DEG : d1;
    gfx::ringPaint(233, 233, DATA_RING_R_IN, DATA_RING_R_OUT, 270.0f + a, 270.0f + b, 270.0f, 270.0f + lit, col, C_DR_TRACK, C_BG);
  }
}
static void updDataRing(const Signals &s, bool full) {
  const int q = drQ(s);
  const uint16_t col = to565(colorAt(RING_C, s.rpm, g_set->shiftRpm));
  if (full || g_drQ < 0) {
    drPaint(0.0f, 360.0f, q, col);
  } else if (col != g_drCol && (q > 0 || g_drQ > 0)) {  // 色が変わった: 点いている所(前と今の広い方)を全部
    drPaint(-1.0f, (float)(q > g_drQ ? q : g_drQ) / DR_Q_PER_DEG + 1.0f, q, col);
  } else if (q != g_drQ) {  // 先が動いた: 前の先と今の先の間(なめらかにした縁のぶん 1 度広げる)
    const int lo = q < g_drQ ? q : g_drQ, hi = q < g_drQ ? g_drQ : q;
    drPaint((float)lo / DR_Q_PER_DEG - 1.0f, (float)hi / DR_Q_PER_DEG + 1.0f, q, col);
  }
  g_drQ = q;
  g_drCol = col;
}
// データ画面で消す四角: 半径 DATA_RING_R_IN − 1.5 の円の中だけを塗る(外周の輪とその縁に掛けない。2.5.0)
static void fillRectData(int x, int y, int w, int h, uint16_t c) {
  const float R = DATA_RING_R_IN - 1.5f;
  for (int yy = y; yy < y + h; yy++) {
    const float dy = (float)yy + 0.5f - 233.0f;
    const float q = R * R - dy * dy;
    if (q <= 0) continue;
    const float half = sqrtf(q);
    int xa = (int)ceilf(233.0f - half - 0.5f), xb = (int)floorf(233.0f + half - 0.5f) + 1;  // 画素の中心が円の中
    if (xa < x) xa = x;
    if (xb > x + w) xb = x + w;
    if (xb > xa) gfx::fillRect(xa, yy, xb - xa, 1, c);
  }
}

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
  fillRectData(36, y - 28, 394, 36, C_BG);
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
  fillRectData(colX - 2, 182, DATA_COL_W + 4, 76, C_BG);
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
  fillRectData(40, y - 48, 386, 56, C_BG);
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
  updDataRing(s, full);
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
// どの画面も同じ形: 上の帯(y < ZONE_TOP_Y)= 「‹ 今いる所の名前」(タップで 1 つ上へ。設定の項目は「‹ 設定」で元の画面へ。
// 2.3.0 で「‹」を付けた。2.4.0: 受信の状態・グループの中も今いる所の名前。変更・確認の画面は「‹ 戻る」)、
// その下に項目の名前(50 px)と値、左右の「‹」「›」(真ん中の左半分・右半分のタップ = 前・次。2.4.0 からスワイプでも送れる:
// 左へ = 次、右へ = 前。2.4.1: 2.4.0 でタップをやめたのを戻した)、点 = 項目の数と今の項目、下の帯(y ≥ ZONE_BOT_Y)= その画面の操作。
// 操作の無い画面には下の帯を出さない
static const uint16_t C_BAND_TOP = rgb(0x0F1724);  // 上の帯の地
static const uint16_t C_BAND_BOT = rgb(0x0E2236);  // 下の帯の地
static const uint16_t C_BAND_RED = rgb(0xB3201A);  // 消す操作(リセット・やり直す)の下の帯
static const uint16_t C_DOT = rgb(0x5A606A);       // 下の点(今でない項目)
static const uint16_t C_OK = rgb(0x22E36A);        // 判定: 正常
static const uint16_t C_LABEL2 = rgb(0xC8D6F0);    // 項目名(明るめ)
static const uint16_t C_SEL_BG = rgb(0x14233A);    // CAN確認の詳細: 選んだバイトの地

// タップの範囲: 0 = 上の帯、1 = 下の帯、2 = 真ん中の左半分、3 = 真ん中の右半分(前・次。変更の画面では − +、CAN確認では「やり直す」「一覧」)
enum Zone { Z_TOP, Z_BOT, Z_LEFT, Z_RIGHT };
static Zone zoneAt(int x, int y) {
  if (y < ZONE_TOP_Y) return Z_TOP;
  if (y >= ZONE_BOT_Y) return Z_BOT;
  return x < 233 ? Z_LEFT : Z_RIGHT;
}
// 押している帯(2.7.0。Z_TOP・Z_BOT、-1 = なし)。drawTopBack・drawBottom はこの帯を反転して描く(水色の帯に地の色の字。赤い帯は白い帯に
// 赤の字)。決めるのは update の初めの pressStep(タッチの節)
static int g_pressBand = -1;

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
static void drawTopBack(const char *label = "戻る") {  // 上の帯「‹ 名前」(2.4.0: 英字も書ける。「‹ CAN」など)
  const bool inv = g_pressBand == Z_TOP;  // 押している(2.7.0)
  const uint16_t col = inv ? C_BG : C_ACCENT;
  fillChordRows(0, ZONE_TOP_Y, inv ? C_ACCENT : C_BAND_TOP);
  float w = mixWidth(FONT_NUM600_48, FONT_JP600_48, label);
  float x = 233 - (w + 30) * 0.5f;
  bigChevron(x + 8, 77, 32, -1, col);
  mixDraw(FONT_NUM600_48, FONT_JP600_48, x + 30, 94, label, col);
}
// 設定の項目の上の帯「‹ 設定」(2.3.0。「設定」だけでは、押すと元の画面に戻るのが分からなかった)
static void drawTopSet() {
  drawTopBack("設定");
  textC(FONT_NUM600_20, 233, 34, "v" METER_FW_VERSION, g_pressBand == Z_TOP ? C_BG : C_LABEL);  // 表示器の版(config.h)
}
static void drawBottom(const char *label, uint16_t fill = C_BAND_BOT, uint16_t col = C_ACCENT, const FontData &fj = FONT_JP600_44) {
  if (g_pressBand == Z_BOT) {  // 押している(2.7.0): 帯と字の色を入れ替える
    const uint16_t f = fill;
    fill = col;
    col = f;
  }
  fillChordRows(ZONE_BOT_Y, gfx::H, fill);
  mixDraw(FONT_NUM600_44, fj, 233 - mixWidth(FONT_NUM600_44, fj, label) * 0.5f, 420, label, col);
}
// 2.4.4: メニューに入る下の帯「長押しで開く」(本人 10/24「メニューに入るときは長押しで開くにします。下の帯にも長押しで開くと書いてください」)。
// 長いので 40 px(44 px では文字の下の角が丸い画面の縁に近すぎる)。押している間は水色の帯、取り消したら灰の字(drawGuide)
static const char *const OPEN_LABEL = "長押しで開く";
static void drawOpenBand(uint16_t fill = C_BAND_BOT, uint16_t col = C_ACCENT) {
  drawBottom(OPEN_LABEL, fill, col, FONT_JP600_40);
}
// 左右の「‹」「›」(押せる所。真ん中の左半分・右半分のタップで前・次。2.4.1: 2.4.0 は押せない印にしていた)
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
// 大きい値(数字 96 px と単位 44 px。入らなければ小さく)。unit = "" なら単位なし。
// bold = true なら値を太く(2.4.3: 横に 1 px ずつずらして重ねる。96 px 以上は 3 回、64 px は 2 回ずらす。単位は太くしない)
static void drawBigValue(const char *v, const char *unit, float base, uint16_t c, float maxW = 280, bool bold = false) {
  const bool ascii = isAsciiLike(v) && runLen(v) == (int)strlen(v);
  if (!unit[0]) {
    const int b = bold ? 3 : 0;
    for (int i = 0; i <= b; i++) {
      const float dx = (float)i - b * 0.5f;
      if (ascii && gfx::textWidth(FONT_NUM600_96, v) <= maxW) textC(FONT_NUM600_96, 233 + dx, base, v, c);
      else if (!ascii && mixWidth(FONT_NUM600_96, FONT_JP600_100, v) <= maxW) mixDraw(FONT_NUM600_96, FONT_JP600_100, 233 - mixWidth(FONT_NUM600_96, FONT_JP600_100, v) * 0.5f + dx, base, v, c);
      else fitTextC(FP_VALUE, 3, maxW + 50, 233 + dx, base - 10, v, c);
    }
    return;
  }
  const FontData &uf = (uint8_t)unit[0] >= 0xE0 ? FONT_JP600_44 : FONT_NUM600_44;  // 「分」「件」は日本語、「°C」は英数字の書体
  const float wu = gfx::textWidth(uf, unit);
  const FontData &vf = gfx::textWidth(FONT_NUM600_96, v) + 8 + wu <= maxW ? FONT_NUM600_96 : FONT_NUM600_64;
  const float wv = gfx::textWidth(vf, v);
  const float x = 233 - (wv + 8 + wu) * 0.5f;
  const int b = bold ? (vf.px >= 90 ? 3 : 2) : 0;
  for (int i = 0; i <= b; i++) gfx::drawText(vf, x + (float)i - b * 0.5f, base, v, c);
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
// ---- 受信の状態・本体の状態(2.4.0: 分け直した。本人の指示 10/24「表示内容などのカテゴライズ」)
// 受信の状態 = データの出どころと行き先ごとのグループ: CAN(車 → CANリーダー → 表示器)・GPS・IMU・RaceChrono(表示器 → スマホ)。
//   受信の状態の画面は 1 ページ 1 グループ(まとめ。下「開く」で中へ)。グループの中は 1 ページ 1 項目。
// 本体の状態 = 表示器そのもの(FPS・起動)。設定の項目から直接グループの中へ。
// 同じ値は 1 か所にだけ出す(2.3.0〜2.3.1 は GPS・IMU が 2 か所にあった)。上の帯は今いる所の名前(「‹ 受信の状態」「‹ CAN」など)
enum Group : uint8_t { G_CAN, G_GPS, G_IMU, G_RC, G_SYS, G_COUNT };
static const int N_LINK_GROUPS = 4;  // 受信の状態のグループ(CAN・GPS・IMU・RaceChrono)
static const char *const GROUP_NAME[G_COUNT] = { "CAN", "GPS", "IMU", "RaceChrono", "本体の状態" };
static const int GROUP_N[G_COUNT] = { 8, 3, 2, 3, 8 };  // グループの中のページの数(2.6.0: CAN に「前後で比べる」)
static const int GRP_MAX = 8;
enum { P_CAN_BAUD, P_CAN_RX, P_CAN_ERR, P_CAN_READER, P_CAN_BUS, P_CAN_FUEL, P_CAN_CHECK, P_CAN_PP };  // CAN の中のページ
enum { P_GPS_FIX, P_GPS_POS, P_GPS_SPEED };                                                   // GPS
enum { P_IMU_ACC, P_IMU_CAL };                                                                // IMU
enum { P_RC_SEND, P_RC_LINK, P_RC_RATE };                                                     // RaceChrono
enum { P_SYS_FPS, P_SYS_CPU, P_SYS_CLOCK, P_SYS_TEMP, P_SYS_RAM, P_SYS_PSRAM, P_SYS_FLASH, P_SYS_BOOT };  // 本体の状態
enum Verdict : uint8_t { V_WAIT, V_OK, V_NG };
enum PageAct : uint8_t { A_NONE, A_OPEN_GRP, A_OPEN_CAN, A_EDIT_RC, A_IMU_RESET, A_OPEN_CANPP };  // ページの下の帯の操作
struct InfoItem {
  const char *label;
  char value[48];
  Verdict v;
  bool core;    // まとめで、緑でなければ「正常」にしない項目(2.2.0)
  PageAct act;  // 下の帯の操作(2.3.0)
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
static void initItems(InfoItem *it, int n, const char *const *labels) {
  for (int i = 0; i < n; i++) {
    it[i].label = labels[i];
    it[i].value[0] = 0;
    it[i].v = V_WAIT;
    it[i].core = false;
    it[i].act = A_NONE;
  }
}
static const char *const RC_OFF_TEXT = "切(RaceChrono 送信)";  // RaceChrono の送信が切(GPS・IMU も動かしていない)
// CAN: 車の CAN を CANリーダー(XIAO ESP32C6)が読み、UART で表示器へ送る。燃料のほかは、まとめで緑でなければ「正常」にしない
static void canItems(const LinkStats &ls, uint32_t now, InfoItem *it) {
  static const char *const L[] = { "通信速度", "受信", "誤り・欠落", "CANリーダー", "C-CAN・B-CAN", "燃料", "CAN確認", "前後で比べる" };
  initItems(it, GROUP_N[G_CAN], L);
  for (int i = P_CAN_BAUD; i <= P_CAN_BUS; i++) it[i].core = true;
  // 受けていない: バイトは届くのに読めなければ赤。何も届かないときも、起動から INFO_LINK_GRACE_MS たったら赤(2.2.0。前は灰で、
  // まとめが「正常」になっていた)。デモ表示の間は UART を読まないので灰
  const bool linkWait = g_set->demoOn || now < INFO_LINK_GRACE_MS;
  InfoItem &b = it[P_CAN_BAUD];
  if (ls.locked) snprintf(b.value, sizeof(b.value), "%u bps", (unsigned)ls.baud), b.v = V_OK;
  else snprintf(b.value, sizeof(b.value), "受信なし"), b.v = (ls.bytesPerSec || !linkWait) ? V_NG : V_WAIT;
  // 受けているとき: フレーム数。受けていないとき: 届いているバイト数(0 = 線に何も来ていない。0 でない = 来ているが読めない)
  InfoItem &r = it[P_CAN_RX];
  if (ls.locked) snprintf(r.value, sizeof(r.value), "%u /s", (unsigned)ls.framesPerSec), r.v = ls.framesPerSec ? V_OK : V_NG;
  else snprintf(r.value, sizeof(r.value), "%u バイト/秒", (unsigned)ls.bytesPerSec);
  InfoItem &e = it[P_CAN_ERR];
  snprintf(e.value, sizeof(e.value), "%u / %u", (unsigned)ls.crcErrors, (unsigned)ls.lostFrames);
  e.v = recent(g_errIncMs, now) ? V_NG : (ls.locked ? V_OK : V_WAIT);
  // CANリーダー(2.4.0: 「送り元」から改名): ファームの名前と版(版が合わなければ赤)。
  // C-CAN・B-CAN: CANリーダーが 1 秒に受けた数(C-CAN が 0 なら赤。B-CAN は 0 でもよい)
  InfoItem &d = it[P_CAN_READER], &c = it[P_CAN_BUS];
  if (!ls.infoValid) {
    snprintf(d.value, sizeof(d.value), "– –");
    snprintf(c.value, sizeof(c.value), "– –");
  } else if (ls.info.proto_ver != METER_LINK_PROTO_VER) {
    snprintf(d.value, sizeof(d.value), "版が違います"), d.v = V_NG;
    snprintf(c.value, sizeof(c.value), "– –");
  } else {
    snprintf(d.value, sizeof(d.value), "%s%s", ls.info.name, (ls.info.flags & ML_F_SIM) ? " 模擬" : ""), d.v = V_OK;
    snprintf(c.value, sizeof(c.value), "C %u B %u", (unsigned)ls.info.can_fps[0], (unsigned)ls.info.can_fps[1]);
    c.v = ls.info.can_fps[0] ? V_OK : V_NG;
  }
  // 燃料(2.2.0。2.7.0: C-CAN 0A18A000 b5 = %。純正の燃料計(8 目盛、1 目盛 12.5 %)と比べるため): 表示に使っている値。
  // 受けていない・エンジンをかけてからまだ値がない(b5 = 0 だけ)ときは「– –」
  InfoItem &f = it[P_CAN_FUEL];
  veh::FuelInfo fi;
  veh::fuelInfo(&fi, now);
  if (fi.seen && fi.have) snprintf(f.value, sizeof(f.value), "%u %%", (unsigned)fi.pct), f.v = V_OK;
  else snprintf(f.value, sizeof(f.value), "– –");
  // CAN確認(2.4.3: CAN の最後のページ。2.6.0 からは最後の 1 つ前。2.4.4: 下「長押しで開く」。2.4.0〜2.4.2 は C-CAN・B-CAN のページの
  // 下「CAN確認」)。判定なし
  InfoItem &k = it[P_CAN_CHECK];
  const int rec = cantab::recordCount();
  int hist[CAN_REC_HIST];
  const int nh = rec ? cantab::recordHistory(hist) : 0;
  if (nh) snprintf(k.value, sizeof(k.value), "候補 %d 件 記録 %d 回", hist[nh - 1], rec);
  else snprintf(k.value, sizeof(k.value), "記録 %d 回", rec);
  k.act = A_OPEN_CAN;
  // 前後で比べる(2.6.0。CAN の最後のページ)。判定なし
  InfoItem &q = it[P_CAN_PP];
  int up = 0, down = 0;
  if (cantab::ppHasResult()) cantab::ppResults(nullptr, 0, &up, &down), snprintf(q.value, sizeof(q.value), "候補 %d 件", up + down);
  else snprintf(q.value, sizeof(q.value), "%s", cantab::ppHasPre() ? "前を記録した" : "記録なし");
  q.act = A_OPEN_CANPP;
}
// GPS(M100。RaceChrono の送信が入のときだけ動かす): 測位(届かなければ赤)・位置・速度
static void gpsItems(const SysInfo &si, InfoItem *it) {
  static const char *const L[] = { "測位", "位置", "速度" };
  initItems(it, GROUP_N[G_GPS], L);
  it[P_GPS_FIX].core = true;
  const RcStats &r = si.rc;
  const bool fix = r.gpsOk && r.fixType >= 2;
  InfoItem &x = it[P_GPS_FIX], &p = it[P_GPS_POS], &s = it[P_GPS_SPEED];
  if (!r.enabled) snprintf(x.value, sizeof(x.value), "%s", RC_OFF_TEXT);
  else if (!r.gpsOk) snprintf(x.value, sizeof(x.value), "受信なし"), x.v = V_NG;
  else snprintf(x.value, sizeof(x.value), "%s 衛星 %u %u Hz", r.fixType == 3 || r.fixType == 4 ? "3D" : r.fixType == 2 ? "2D" : "測位なし", (unsigned)r.numSV, (unsigned)r.pvtPerSec), x.v = fix ? V_OK : V_WAIT;
  if (fix) snprintf(p.value, sizeof(p.value), "%.4f %.4f", (double)r.lat / 1e7, (double)r.lon / 1e7), p.v = V_OK;
  else snprintf(p.value, sizeof(p.value), "– –");
  if (fix) snprintf(s.value, sizeof(s.value), "%.1f km/h", (double)r.gpsKmh), s.v = V_OK;
  else snprintf(s.value, sizeof(s.value), "– –");
}
// IMU(基板の QMI8658。RaceChrono の送信が入のときだけ読む): 加速度(応答が無ければ赤)・向きの補正(下「向きを取り直す」)
static void imuItems(const SysInfo &si, InfoItem *it) {
  static const char *const L[] = { "加速度", "向きの補正" };
  initItems(it, GROUP_N[G_IMU], L);
  it[P_IMU_ACC].core = true;
  const RcStats &r = si.rc;
  InfoItem &a = it[P_IMU_ACC], &c = it[P_IMU_CAL];
  if (!r.enabled) snprintf(a.value, sizeof(a.value), "%s", RC_OFF_TEXT);
  else if (!r.imuOk) snprintf(a.value, sizeof(a.value), "応答なし"), a.v = V_NG;
  else snprintf(a.value, sizeof(a.value), "前後 %+.2f 横 %+.2f g", (double)r.gLon, (double)r.gLat), a.v = V_OK;
  if (!r.imuOk) snprintf(c.value, sizeof(c.value), "– –");
  else snprintf(c.value, sizeof(c.value), "上 %s 前 %s", r.imuLevel ? "済" : "未", r.imuForward ? "済" : "未"), c.v = (r.imuLevel && r.imuForward) ? V_OK : V_WAIT;
  if (r.imuOk) c.act = A_IMU_RESET;  // 下「向きを取り直す」(覚えた向きを捨てる)
}
// RaceChrono(表示器 → スマホ。BLE): 送信の入・切(下「変更」。変えると次の起動から効く)・接続(開始できなければ赤)・
// 送った数(直近 INFO_RECENT_MS に捨てたら赤)
static void rcItems(const SysInfo &si, uint32_t now, InfoItem *it) {
  static const char *const L[] = { "送信", "接続", "送った数" };
  initItems(it, GROUP_N[G_RC], L);
  it[P_RC_LINK].core = it[P_RC_RATE].core = true;
  const RcStats &r = si.rc;
  InfoItem &s = it[P_RC_SEND], &l = it[P_RC_LINK], &n = it[P_RC_RATE];
  const bool on = g_set->rcOn != 0;
  snprintf(s.value, sizeof(s.value), "%s%s", on ? "入" : "切", on != r.enabled ? "(次の起動から)" : "");
  s.act = A_EDIT_RC;
  if (!r.enabled) snprintf(l.value, sizeof(l.value), "%s", RC_OFF_TEXT);
  else if (!r.bleOk) snprintf(l.value, sizeof(l.value), "開始できない"), l.v = V_NG;
  else if (r.connected) snprintf(l.value, sizeof(l.value), "接続中"), l.v = V_OK;
  else snprintf(l.value, sizeof(l.value), "未接続(待機中)");
  snprintf(n.value, sizeof(n.value), "CAN %u/s GPS %u/s", (unsigned)r.canPerSec, (unsigned)r.gpsPerSec);
  n.v = recent(g_dropIncMs, now) ? V_NG : (r.connected ? V_OK : V_WAIT);
}
// 本体の状態(2.4.0: 受信の状態から移した): FPS・起動。2.4.3: CPU・クロック・温度・RAM・PSRAM・フラッシュ(本人「クロック数など CPU に
// 関わるデータ、型式など」)。値は基板が SysInfo の cpu に入れる(起動時と 1 秒ごと)
static void sysItems(const PerfStats &ps, const SysInfo &si, InfoItem *it) {
  static const char *const L[] = { "FPS", "CPU", "クロック", "温度", "RAM", "PSRAM", "フラッシュ", "起動" };
  initItems(it, GROUP_N[G_SYS], L);
  // FPS(2.3.0。前は「描画」。2.4.3: 直近 PERF_FPS_SECS 秒の 1 秒ごとの値): サーキット・タコで 1 回描いて送る時間から出した回/秒。
  // 平均 = 全部の回の平均、最低 = 1 秒ごとの値で一番低い秒。平均が INFO_FPS_MIN 未満なら赤(2.3.0〜2.4.2 は最低で決めていて、
  // 背景の色を塗り直す回(全体の描き直し)があると毎回赤になっていた)。数えた回が FPS_MIN_FRAMES 未満なら「– –」の灰。下にグラフ
  InfoItem &f = it[P_SYS_FPS], &b = it[P_SYS_BOOT];
  if (ps.pageFrames >= FPS_MIN_FRAMES && ps.pageAvgUs && ps.pageSlowUs) {
    const unsigned avg = (unsigned)((1000000u + ps.pageAvgUs / 2) / ps.pageAvgUs);
    const unsigned low = (unsigned)(1000000u / ps.pageSlowUs);
    snprintf(f.value, sizeof(f.value), "平均 %u 最低 %u", avg, low);
    f.v = avg < INFO_FPS_MIN ? V_NG : V_OK;
  } else {
    snprintf(f.value, sizeof(f.value), "– –");
  }
  // CPU(型式・版・コアの数)・クロック・温度(チップの中のセンサー)・RAM(内部の空き / 全体)・PSRAM・フラッシュ(大きさと速さ)
  const CpuInfo &c = si.cpu;
  InfoItem &cpu = it[P_SYS_CPU], &clk = it[P_SYS_CLOCK], &tmp = it[P_SYS_TEMP], &ram = it[P_SYS_RAM], &ps2 = it[P_SYS_PSRAM], &fl = it[P_SYS_FLASH];
  if (!c.ok) {
    snprintf(cpu.value, sizeof(cpu.value), "– –");
    snprintf(clk.value, sizeof(clk.value), "– –");
    snprintf(tmp.value, sizeof(tmp.value), "– –");
    snprintf(ram.value, sizeof(ram.value), "– –");
    snprintf(ps2.value, sizeof(ps2.value), "– –");
    snprintf(fl.value, sizeof(fl.value), "– –");
  } else {
    snprintf(cpu.value, sizeof(cpu.value), "%s v%u.%u %u コア", c.model, (unsigned)(c.rev / 100), (unsigned)(c.rev % 100), (unsigned)c.cores), cpu.v = V_OK;
    snprintf(clk.value, sizeof(clk.value), "%u MHz", (unsigned)c.cpuMHz), clk.v = V_OK;
    if (c.tempOk) snprintf(tmp.value, sizeof(tmp.value), "%.1f °C", (double)c.tempC), tmp.v = V_OK;
    else snprintf(tmp.value, sizeof(tmp.value), "– –");
    snprintf(ram.value, sizeof(ram.value), "空き %u / %u KB", (unsigned)(c.heapFree / 1024), (unsigned)(c.heapBytes / 1024)), ram.v = V_OK;
    if (c.psramBytes) snprintf(ps2.value, sizeof(ps2.value), "空き %.1f / %.1f MB", (double)c.psramFree / 1048576.0, (double)c.psramBytes / 1048576.0), ps2.v = V_OK;
    else snprintf(ps2.value, sizeof(ps2.value), "なし");
    snprintf(fl.value, sizeof(fl.value), "%u MB %u MHz", (unsigned)(c.flashBytes >> 20), (unsigned)(c.flashHz / 1000000u)), fl.v = V_OK;
  }
  // AXP2101 のレジスタ 0x20: bit0 = PWR ボタン、bit2 = VBUS(5V)が入った、bit4 = 電池をつないだ
  if (!si.axpOk) snprintf(b.value, sizeof(b.value), "不明");
  else if (si.powerOnSrc & 0x04) snprintf(b.value, sizeof(b.value), "5V 投入"), b.v = V_OK;
  else if (si.powerOnSrc & 0x01) snprintf(b.value, sizeof(b.value), "PWR ボタン"), b.v = V_OK;
  else if (si.powerOnSrc & 0x10) snprintf(b.value, sizeof(b.value), "電池"), b.v = V_OK;
  else snprintf(b.value, sizeof(b.value), "0x%02X", si.powerOnSrc), b.v = V_OK;
}
// グループ g のページ(it[0〜GROUP_N[g] − 1])。ページの数を返す
static int groupItems(int g, const LinkStats &ls, const PerfStats &ps, const SysInfo &si, uint32_t now, InfoItem *it) {
  switch (g) {
    case G_CAN: canItems(ls, now, it); return GROUP_N[G_CAN];
    case G_GPS: gpsItems(si, it); return GROUP_N[G_GPS];
    case G_IMU: imuItems(si, it); return GROUP_N[G_IMU];
    case G_RC: rcItems(si, now, it); return GROUP_N[G_RC];
    default: sysItems(ps, si, it); return GROUP_N[G_SYS];
  }
}
static uint16_t verdictCol(Verdict v) {
  return v == V_OK ? C_OK : v == V_NG ? C_ALARM : C_DOT;
}
// 項目の判定のまとめ: 異常があれば「異常 n」。無くても、core の項目が 1 つでも正常でなければ「待ち」。core が全部正常のときだけ「正常」
// (2.2.0。前は正常が 1 つでもあれば「正常」で、通信が切れていても「正常」と出ていた)
static void verdictSummary(const InfoItem *it, int cnt, char *out, size_t n, uint16_t *col) {
  int ng = 0, coreNotOk = 0;
  for (int i = 0; i < cnt; i++) {
    ng += it[i].v == V_NG;
    coreNotOk += it[i].core && it[i].v != V_OK;
  }
  if (ng) snprintf(out, n, "異常 %d", ng), *col = C_ALARM;
  else if (coreNotOk) snprintf(out, n, "待ち"), *col = C_LABEL2;
  else snprintf(out, n, "正常"), *col = C_WHITE;
}
static Verdict colVerdict(uint16_t col) {
  return col == C_ALARM ? V_NG : (col == C_WHITE ? V_OK : V_WAIT);
}
// グループのまとめ(受信の状態のページ、設定の「本体の状態」)。GPS・IMU・RaceChrono は、送信が切なら「切」の灰
static void groupSummary(int g, const LinkStats &ls, const PerfStats &ps, const SysInfo &si, uint32_t now, char *out, size_t n, uint16_t *col) {
  if ((g == G_GPS || g == G_IMU || g == G_RC) && !si.rc.enabled) {
    snprintf(out, n, "切");
    *col = C_LABEL2;
    return;
  }
  InfoItem it[GRP_MAX];
  verdictSummary(it, groupItems(g, ls, ps, si, now, it), out, n, col);
}
// 設定の「受信の状態」のまとめ: 4 グループの項目を 1 回ずつ数える。「待ち」にするかは CAN の項目だけで決める
// (GPS・IMU・RaceChrono は赤だけを数える。スマホが未接続でも「待ち」にしない)
static void linkSummary(const LinkStats &ls, const PerfStats &ps, const SysInfo &si, uint32_t now, char *out, size_t n, uint16_t *col) {
  InfoItem it[GRP_MAX * N_LINK_GROUPS];
  int cnt = 0;
  for (int g = 0; g < N_LINK_GROUPS; g++) {
    const int k = groupItems(g, ls, ps, si, now, it + cnt);
    if (g != G_CAN)
      for (int i = 0; i < k; i++) it[cnt + i].core = false;
    cnt += k;
  }
  verdictSummary(it, cnt, out, n, col);
}
// 受信の状態のページ: 1 ページ 1 グループ(まとめ。下「開く」で中へ)
static void linkPages(const LinkStats &ls, const PerfStats &ps, const SysInfo &si, uint32_t now, InfoItem *it) {
  for (int g = 0; g < N_LINK_GROUPS; g++) {
    InfoItem &p = it[g];
    p.label = GROUP_NAME[g];
    uint16_t col;
    groupSummary(g, ls, ps, si, now, p.value, sizeof(p.value), &col);
    p.v = colVerdict(col);
    p.core = false;
    p.act = A_OPEN_GRP;
  }
}

// ================================================================ 設定画面(8 章。10/23 に作り直し: 1 画面 1 項目。2.4.0: 8 項目)
// 並び(本人の指示 10/24): 明るさ → 受信の状態 → シフトポイント → 水温の警告 → 時間の記録 → 自動で戻る → デモ。
// 2.4.0: 受信の状態の次に「本体の状態」(受信と関係ない FPS・起動を受信の状態から出した)
enum Unit : uint8_t { U_NONE, U_RPM, U_MIN, U_PCT, U_DEGC, U_ONOFF };
enum ItemKind : uint8_t { K_VALUE, K_TIME, K_LINK, K_SYS };
struct Item {
  const char *label;  // 項目の名前
  int Settings::*field;
  int minV, maxV, step;
  Unit unit;
  ItemKind kind;
};
static const Item ITEMS[] = {
  { "明るさ", &Settings::brightnessPct, SET_BRIGHT_MIN, SET_BRIGHT_MAX, SET_BRIGHT_STEP, U_PCT, K_VALUE },
  { "受信の状態", nullptr, 0, 0, 0, U_NONE, K_LINK },
  { "本体の状態", nullptr, 0, 0, 0, U_NONE, K_SYS },
  { "シフトポイント", &Settings::shiftRpm, SET_SHIFT_MIN, SET_SHIFT_MAX, SET_SHIFT_STEP, U_RPM, K_VALUE },
  { "水温の警告", &Settings::warnTempC, SET_WARN_MIN, SET_WARN_MAX, SET_WARN_STEP, U_DEGC, K_VALUE },
  { "時間の記録", nullptr, 0, 0, 0, U_NONE, K_TIME },
  { "自動で戻る", &Settings::autoReturnMin, SET_AUTORET_MIN, SET_AUTORET_MAX, SET_AUTORET_STEP, U_MIN, K_VALUE },
  { "デモ", &Settings::demoOn, 0, 1, 1, U_ONOFF, K_VALUE },
};
static const int N_ITEMS = sizeof(ITEMS) / sizeof(ITEMS[0]);
// 受信の状態 → RaceChrono の 1 ページ目「送信」(変えると次の起動から効く)
static const Item RC_SEND_ITEM = { "RaceChrono 送信", &Settings::rcOn, 0, 1, 1, U_ONOFF, K_VALUE };
static int g_setItem = 0;          // 設定で見ている項目(電源を切るまで覚える。フラッシュには書かない)
static int g_infoPage = 0;         // 受信の状態で見ているグループ
static int g_grp = G_CAN;          // グループの中の画面のグループ
static int g_grpPage = 0;          // そのページ
static Screen g_grpFrom = SC_SET_INFO;  // グループの中の画面の上の帯で戻る画面(受信の状態、本体の状態なら設定)
static const Item *g_editItem = &ITEMS[0];  // 変更の画面で変えている項目
static Screen g_editFrom = SC_SET_LIST;     // 変更の画面を出した画面(「‹ 戻る」「決定」のあとに戻る)
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
static void startEdit(const Item *it, Screen from) {
  g_editItem = it;
  g_editFrom = from;
  g_editOrig = g_set->*(it->field);
  g_saveTried = false;
  g_saveFailed = false;
  go(SC_SET_EDIT);
}
static void openGroup(int g, Screen from) {
  g_grp = g;
  g_grpPage = 0;
  g_grpFrom = from;
  go(SC_SET_GRP);
}
// ---- 設定: 1 項目の画面(2.4.4: 下の帯はどの項目も「長押しで開く」。2.4.3 は「決定」で、値を確定する「決定」と意味が 2 つになっていた)
static void setItemValue(const Item &it, const LinkStats &ls, const PerfStats &ps, const SysInfo &si, uint32_t now, char *v, size_t n, uint16_t *col) {
  *col = C_WHITE;
  if (it.kind == K_VALUE) {
    valueText(it, g_set->*(it.field), v, n);
  } else if (it.kind == K_TIME) {
    snprintf(v, n, "%u|%u", (unsigned)(trip::aboveMs() / 1000), (unsigned)(trip::belowMs() / 1000));
  } else if (it.kind == K_LINK) {
    linkSummary(ls, ps, si, now, v, n, col);
  } else {
    groupSummary(G_SYS, ls, ps, si, now, v, n, col);
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
  if (it.kind == K_VALUE) drawBigValue(v, it.unit == U_ONOFF ? "" : unitText(it.unit), 290, C_WHITE);
  else if (it.kind == K_TIME) drawTwoTimes(250, 304, C_LABEL2);
  else drawBigValue(v, "", 290, col);
  drawSideArrows();
  drawDots(N_ITEMS, g_setItem, nullptr, 330);
  drawOpenBand();  // 2.4.4: どの項目も下の帯の長押しで開く(値の項目は変更の画面、時間の記録はリセットの確認)
}

// ---- 設定: 変更の画面(項目の名前のまま、左右が − +(入・切は 切・入)、下「決定」= 保存、上「‹ 戻る」= 変えずに戻る)。
// 2.4.1: 項目の画面と見た目を分けるため、値を水色にし、下の「決定」を水色の帯にした(2.4.0 の値の枠はやめた。本人「枠で囲まれるのはダサい」)。
// 2.4.3: 「決定」で保存したら、押した所(下の「決定」の帯)を SAVED_FLASH_COUNT 回点滅させ、その間は値を太くする。点滅が終わったら
// 元の画面へ(2.3.1〜2.4.2 は戻った画面で値を緑で点滅させていた。本人「押したところにアクションがある方が直感的」「緑は意味がない」)。
// 2.4.5: 変えていなくても「決定」を押したら点滅させる(leaveEdit)
static bool g_saveFlash = false;     // 「決定」を押したあとの点滅の途中(この間は操作を受けない)
static uint32_t g_saveFlashT0 = 0;   // 点滅を始めた時刻
static int g_saveFlashPhase = -1;    // 描いてある段階(-1 = 点滅なし、偶数 = 帯を消す、奇数 = 水色の帯)
static void drawEditValue() {
  const Item &it = *g_editItem;
  gfx::fillRect(80, 206, 306, 98, C_BG);  // 〜303。下の刻みの消す範囲(304〜)と隙間を作らない
  char v[16];
  valueText(it, g_set->*(it.field), v, sizeof(v));
  drawBigValue(v, it.unit == U_ONOFF ? "" : unitText(it.unit), 290, C_ACCENT, 290, g_saveFlash);
}
static void drawEditDesc() {  // 刻み(または「保存できませんでした」)
  const Item &it = *g_editItem;
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
// 下の「決定」の帯(phase: -1 = ふだんの水色の帯、偶数 = 消す(暗い帯に水色の字)、奇数 = 水色の帯)。帯の行を背景で消してから描く
// (円の縁のなめらかな画素を 2 度塗りしない。最初から描いた絵と同じにする)
static void drawEditBottom(int phase) {
  gfx::fillRect(0, ZONE_BOT_Y, gfx::W, gfx::H - ZONE_BOT_Y, C_BG);
  if (phase >= 0 && (phase % 2) == 0) drawBottom("決定");
  else drawBottom("決定", C_ACCENT, C_BG);
}
static int saveFlashPhase(uint32_t now) {  // 今の点滅の段階。-1 = 点滅なし、2 × SAVED_FLASH_COUNT 以上 = 終わった
  return g_saveFlash ? (int)((uint32_t)(now - g_saveFlashT0) / SAVED_FLASH_MS) : -1;
}
static void updSetEdit(uint32_t now) {
  const int ph = saveFlashPhase(now);
  if (ph >= 2 * SAVED_FLASH_COUNT) {  // 点滅が終わった: 元の画面へ
    g_saveFlash = false;
    g_saveFlashPhase = -1;
    go(g_editFrom);
    return;
  }
  if (!g_full) {
    if (ph != g_saveFlashPhase) {  // 点滅が進んだ: 下の帯だけ描き直す
      drawEditBottom(ph);
      g_saveFlashPhase = ph;
    }
    return;
  }
  g_full = false;
  const Item &it = *g_editItem;
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
  drawEditBottom(ph);
  g_saveFlashPhase = ph;
}

// ---- 設定: 消す操作の確認(上「‹ 戻る」= 消さずに戻る、下の赤い帯 = 消して戻る)。2.4.3: 時間の記録のリセットと、IMU の向きの取り直し
enum ResetKind : uint8_t { RK_TIME, RK_IMU };
static ResetKind g_resetKind = RK_TIME;
static Screen g_resetFrom = SC_SET_LIST;  // 戻る画面(時間の記録 = 設定、向きの補正 = IMU の中)
static uint64_t g_resetKey = 0;  // 描いてある 2 つの時間(秒)
static void updSetReset() {
  // 時間の記録: 時間はこの画面にいる間も進むので、秒が変わったら描き直す
  const uint64_t k = g_resetKind == RK_TIME ? (((uint64_t)(trip::aboveMs() / 1000) << 32) | (trip::belowMs() / 1000)) : 0;
  if (!g_full && k == g_resetKey) return;
  g_resetKey = k;
  g_full = false;
  gfx::fillAll(C_BG);
  g_tagsDirty = true;
  drawTopBack();
  if (g_resetKind == RK_TIME) {
    drawName("時間の記録");
    drawTwoTimes(246, 298, C_LABEL);
    mixDraw(FONT_NUM600_30, FONT_JP600_30, 233 - mixWidth(FONT_NUM600_30, FONT_JP600_30, "2 つとも 0 に戻す") * 0.5f, 334, "2 つとも 0 に戻す", C_LABEL2);
    drawBottom("リセット", C_BAND_RED, C_WHITE);
  } else {
    drawName("向きの補正");
    mixDraw(FONT_NUM600_30, FONT_JP600_30, 233 - mixWidth(FONT_NUM600_30, FONT_JP600_30, "覚えた向き(上・前)を捨てて") * 0.5f, 262, "覚えた向き(上・前)を捨てて", C_LABEL2);
    mixDraw(FONT_NUM600_30, FONT_JP600_30, 233 - mixWidth(FONT_NUM600_30, FONT_JP600_30, "停車と発進で取り直す") * 0.5f, 304, "停車と発進で取り直す", C_LABEL2);
    drawBottom("取り直す", C_BAND_RED, C_WHITE);
  }
}

// ---- 受信の状態・グループの中: 1 ページの画面と、全ページの判定(下の点)。上の帯は今いる所の名前、下の帯はそのページの操作
// (2.4.4: 操作のあるページはどれも「長押しで開く」)
static void drawPageValue(const InfoItem &c, float base = 290, int fonts = 3) {
  const uint16_t vc = c.act == A_EDIT_RC ? C_WHITE : c.v == V_NG ? C_ALARM : (c.v == V_OK ? C_WHITE : C_LABEL2);
  fitTextC(FP_VALUE + (3 - fonts), fonts, 320, 233, base, c.value, vc);
}
// FPS のグラフ(2.4.3。本人「過去数分の FPS の値がここに出るほうがいい」): 直近 PERF_FPS_SECS 秒(サーキット・タコを出していた秒だけ)の
// 1 秒ごとの回/秒。右端が新しい。縦は 0〜FPS_GRAPH_MAX 回/秒、INFO_FPS_MIN の所に細い線
static const float FG_X0 = 93, FG_X1 = 373, FG_Y0 = 250, FG_Y1 = 306;
static float fpsY(int fps) {
  const float f = fps > FPS_GRAPH_MAX ? (float)FPS_GRAPH_MAX : (float)fps;
  return FG_Y1 - (FG_Y1 - FG_Y0) * f / (float)FPS_GRAPH_MAX;
}
static void drawFpsGraph(const PerfStats &ps) {
  gfx::fillRect((int)FG_X0, (int)fpsY(INFO_FPS_MIN), (int)(FG_X1 - FG_X0), 1, C_DOT);  // 異常の境目
  gfx::fillRect((int)FG_X0, (int)FG_Y1, (int)(FG_X1 - FG_X0), 2, C_DOT);              // 0
  const int n = ps.histN > PERF_FPS_SECS ? PERF_FPS_SECS : ps.histN;
  if (n < 2) return;
  static gfx::Pt p[PERF_FPS_SECS];
  const float step = (FG_X1 - FG_X0) / (float)(PERF_FPS_SECS - 1);
  for (int i = 0; i < n; i++) p[i] = { FG_X1 - step * (float)(n - 1 - i), fpsY(ps.fpsHist[i]) };
  gfx::polyBegin(gfx::Rect{ (int16_t)(FG_X0 - 4), (int16_t)(FG_Y0 - 4), (int16_t)(FG_X1 + 4), (int16_t)(FG_Y1 + 4) });
  strokePolyline(p, n, 1.5f);
  gfx::polyFill(C_WHITE);
}
static void drawPage(const char *title, const InfoItem *it, int cnt, int cur, const PerfStats *graph = nullptr) {
  gfx::fillAll(C_BG);
  g_tagsDirty = true;  // 全部を塗り直したので「SIM」の印も描き直す
  drawTopBack(title);
  const InfoItem &c = it[cur];
  drawName(c.label);
  if (graph) {  // FPS: 値を小さくして上に、下にグラフ
    drawPageValue(c, 238, 2);
    drawFpsGraph(*graph);
  } else {
    drawPageValue(c);
  }
  drawSideArrows();
  uint16_t cols[GRP_MAX];
  for (int i = 0; i < cnt; i++) cols[i] = verdictCol(it[i].v);
  drawDots(cnt, cur, cols, 330);
  if (c.act != A_NONE) drawOpenBand();  // 2.4.4: 操作のあるページは下の帯の長押しで開く
}
// 値は 0.5 秒ごとに描き直す(ページを送ったとき、開いたときはすぐ)
static void updSetInfo(const LinkStats &ls, const PerfStats &ps, const SysInfo &si, uint32_t now) {
  if (!g_full && (uint32_t)(now - g_infoMs) < 500) return;
  g_full = false;
  g_infoMs = now;
  InfoItem it[N_LINK_GROUPS];
  linkPages(ls, ps, si, now, it);
  if (g_infoPage < 0 || g_infoPage >= N_LINK_GROUPS) g_infoPage = 0;
  drawPage("受信の状態", it, N_LINK_GROUPS, g_infoPage);
}
static PageAct g_grpAct = A_NONE;  // 描いてあるページの下の帯の操作(長押しで開くもの。2.4.4)
static void updSetGrp(const LinkStats &ls, const PerfStats &ps, const SysInfo &si, uint32_t now) {
  if (g_grp < 0 || g_grp >= G_COUNT) g_grp = G_CAN;
  const int cnt = GROUP_N[g_grp];
  if (g_grpPage < 0 || g_grpPage >= cnt) g_grpPage = 0;
  if (!g_full && (uint32_t)(now - g_infoMs) < 500) return;
  g_full = false;
  g_infoMs = now;
  InfoItem it[GRP_MAX];
  groupItems(g_grp, ls, ps, si, now, it);
  g_grpAct = it[g_grpPage].act;
  const bool fps = g_grp == G_SYS && g_grpPage == P_SYS_FPS && it[g_grpPage].value[0] != 0;
  drawPage(GROUP_NAME[g_grp], it, cnt, g_grpPage, fps ? &ps : nullptr);
}

// ================================================================ CAN確認(10/23。設定 → 受信の状態 → CAN の「CAN確認」のページの下「長押しで開く」。2.4.4)
// 2.6.0: CAN の最後のページ「前後で比べる」(updCanPp)。一覧・詳細・やり直すは両方で使う(g_canPp)
// CAN の画面: 件数(今の候補の ID の数)・記録の回数・件数の移り変わり。下「変化を記録」、右半分「一覧」、左半分「やり直す」(確認へ)。
// 一覧: 1 件ずつ(左右のタップかスワイプで ID)。候補のバイトは水色。下「長押しで開く」で詳細(2.4.4)。詳細: 左右でバイトを選び、値とグラフ
static const int CAN_LIST_MAX = CAN_MAX_IDS;
static uint16_t g_candE[CAN_LIST_MAX];
static uint8_t g_candM[CAN_LIST_MAX];
static int g_candN = 0, g_canIdx = 0;
// 2.6.0: 一覧・詳細・やり直すは、「変化を記録」(CAN確認)と「前後で比べる」の両方から使う。g_canPp = 前後で比べるから来た
static bool g_canPp = false;
static cantab::PpResult g_ppList[CAN_PP_MAX];
static int g_listN = 0;  // 一覧の件数(描いたときの値。長押しで開けるか・送りに使う)
static int listCount() {  // 一覧の件数を数え直す(前後で比べる: 候補のバイト。変化を記録: 候補の ID)
  if (g_canPp) {
    g_listN = cantab::ppResults(g_ppList, CAN_PP_MAX);
    if (g_listN > CAN_PP_MAX) g_listN = CAN_PP_MAX;
  } else {
    g_candN = cantab::candidates(g_candE, g_candM, CAN_LIST_MAX);
    g_listN = g_candN < CAN_LIST_MAX ? g_candN : CAN_LIST_MAX;
  }
  return g_listN;
}
static int listEntry(int i) {  // 一覧の i 番目の表の番号
  return g_canPp ? g_ppList[i].entry : g_candE[i];
}
static uint8_t listMask(int i) {  // 一覧の i 番目の候補のバイトの印
  return g_canPp ? (uint8_t)(1u << g_ppList[i].byte) : g_candM[i];
}
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
  const int shown = listCount();
  if (shown == 0) {  // 候補がなくなった(やり直した): CAN の画面へ
    go(g_canPp ? SC_CAN_PP : SC_CAN);
    return;
  }
  if (g_canIdx >= shown) g_canIdx = 0;
  cantab::View v;
  if (!cantab::getView(listEntry(g_canIdx), &v)) return;
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
    if (g_canPp) {  // 2.6.0: 何番目か、候補のバイト、前と後の真ん中の値と差
      const cantab::PpResult &r = g_ppList[g_canIdx];
      char u[64];
      snprintf(u, sizeof(u), "%d/%d  b%d  %d → %d  %+d", g_canIdx + 1, shown, r.byte, r.pre, r.post, (int)r.post - (int)r.pre);
      mixDraw(FONT_NUM600_28, FONT_JP600_30, 233 - mixWidth(FONT_NUM600_28, FONT_JP600_30, u) * 0.5f, 336, u, C_LABEL);
    } else {
      snprintf(t, sizeof(t), "%d / %d", g_canIdx + 1, shown);
      textC(FONT_NUM600_28, 233, 336, t, C_LABEL);
    }
    drawSideArrows();
    drawOpenBand();  // 2.4.4: 長押しで詳細
  }
  drawCanGrid(v, listMask(g_canIdx), -1, 250, full);
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
  uint8_t mask = 0;  // 候補のバイト(前後で比べる: その ID の候補のバイト全部)
  for (int i = 0; i < g_listN; i++)
    if (listEntry(i) == g_detEntry) mask |= listMask(i);
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
  if (g_canPp) {  // 2.6.0
    snprintf(t, sizeof(t), "前の記録と候補を消す");
    mixDraw(FONT_NUM600_30, FONT_JP600_30, 233 - mixWidth(FONT_NUM600_30, FONT_JP600_30, t) * 0.5f, 277, t, C_LABEL2);
  } else {
    snprintf(t, sizeof(t), "記録 %d 回を消して", cantab::recordCount());
    mixDraw(FONT_NUM600_30, FONT_JP600_30, 233 - mixWidth(FONT_NUM600_30, FONT_JP600_30, t) * 0.5f, 256, t, C_LABEL2);
    mixDraw(FONT_NUM600_30, FONT_JP600_30, 233 - mixWidth(FONT_NUM600_30, FONT_JP600_30, "全部の ID を候補に戻す") * 0.5f, 298, "全部の ID を候補に戻す", C_LABEL2);
  }
  drawBottom("やり直す", C_BAND_RED, C_WHITE);
}

// ---- 前後で比べる(2.6.0。設定 → 受信の状態 → CAN の最後のページの下「長押しで開く」。config.h の CAN_PP_*)
// 下「前を記録」(前の記録が無いとき)・「後を記録」(あるとき)。前も後もエンジンがかかっている間に記録する(2.7.0。案内の字)。
// 受信が始まって CAN_PP_SETTLE_MS たつまでは、帯に残りの秒を灰で出す
// (押しても何もしない)。候補ができたら、件数と「増えた・減った」の数。左半分「やり直す」(確認へ)、右半分「一覧」(1 バイトずつ、前 → 後)
static char g_canPpKey[64];
static const uint16_t C_PP_WAIT = rgb(0x5A606A);  // 記録できるまでの下の帯の字(灰。長押しを取り消したときの帯の字と同じ色)
static void updCanPp(uint32_t now) {
  const int wait = cantab::ppWaitMs(now);
  const bool pre = cantab::ppHasPre(), done = cantab::ppHasResult();
  int up = 0, down = 0;
  const int stored = done ? cantab::ppResults(nullptr, 0, &up, &down) : 0;
  const int waitS = wait < 0 ? -1 : (wait + 999) / 1000;
  char key[64];
  snprintf(key, sizeof(key), "%d|%d|%d|%d|%d|%d", pre, done, up, down, stored, waitS);
  if (!g_full && strcmp(key, g_canPpKey) == 0) return;
  g_full = false;
  strncpy(g_canPpKey, key, sizeof(g_canPpKey) - 1);
  g_canPpKey[sizeof(g_canPpKey) - 1] = 0;
  gfx::fillAll(C_BG);
  g_tagsDirty = true;
  drawTopBack();
  drawName("前後で比べる");
  char t[64];
  if (done) {
    snprintf(t, sizeof(t), "%d", up + down);
    drawBigValue(t, "件", 258, C_WHITE, 200);
    snprintf(t, sizeof(t), "増えた %d 減った %d", up, down);
    mixDraw(FONT_NUM600_28, FONT_JP600_30, 233 - mixWidth(FONT_NUM600_28, FONT_JP600_30, t) * 0.5f, 298, t, C_WHITE);
    textC(FONT_JP500_24, 233, 330, "前と後で違うバイト", C_LABEL);
  } else if (pre) {  // 左の「やり直す」に寄らないよう 34 px。2.7.0: 後もエンジンがかかっている間に記録する
    mixDraw(FONT_NUM600_34, FONT_JP600_34, 233 - mixWidth(FONT_NUM600_34, FONT_JP600_34, "前を記録した") * 0.5f, 256, "前を記録した", C_WHITE);
    textC(FONT_JP600_30, 233, 300, "給油してエンジンを", C_LABEL);
    textC(FONT_JP600_30, 233, 334, "かけたら「後を記録」", C_LABEL);
  } else {  // 2.7.0: 前はエンジンを切る前に記録する(燃料などはエンジンが止まると 0 になり、前の 10 秒に入ると候補から外れる)
    mixDraw(FONT_NUM600_44, FONT_JP600_44, 233 - mixWidth(FONT_NUM600_44, FONT_JP600_44, "記録なし") * 0.5f, 256, "記録なし", C_WHITE);
    snprintf(t, sizeof(t), "止まって %d 秒たったら", CAN_PP_WIN_MS / 1000);
    mixDraw(FONT_NUM600_30, FONT_JP600_30, 233 - mixWidth(FONT_NUM600_30, FONT_JP600_30, t) * 0.5f, 300, t, C_LABEL);
    textC(FONT_JP600_30, 233, 334, "エンジンを切る前に記録", C_LABEL);
  }
  if (pre) {
    textC(FONT_JP600_30, 62, 244, "やり", C_ACCENT);
    textC(FONT_JP600_30, 62, 276, "直す", C_ACCENT);
  }
  if (done && stored > 0) {
    textC(FONT_JP600_30, 404, 244, "一覧", C_ACCENT);
    bigChevron(404, 268, 26, 1, C_ACCENT, 8.0f, 2.5f);
  }
  if (wait < 0) {
    drawBottom("受信なし", C_BAND_BOT, C_PP_WAIT);
  } else if (wait > 0) {
    snprintf(t, sizeof(t), "待ち %d 秒", waitS);
    drawBottom(t, C_BAND_BOT, C_PP_WAIT);
  } else {
    drawBottom(pre ? "後を記録" : "前を記録");
  }
}

// ================================================================ オープニング(config.h の OPENING_ON と OPC_*・OPT_*・OPD_*)
// 2.5.0: 覚えた画面(サーキット・タコ・データ)ごとに 3 種類(流れは config.h)。どれも時刻だけで進む。描くのは変わった所だけ
// (背景の色を変えるときと、タコの目盛を赤にする・戻すときだけ全部を描き直す)。タップ・スワイプで飛ばせる(openFinish)
enum OpenKind : uint8_t { OPK_DRIVE, OPK_TACH, OPK_DATA };
static uint8_t g_openKind = OPK_DRIVE;
static uint32_t g_openT0 = 0;       // 始めた時刻
static int g_openPhase = 0;         // 進んだ段階(全部を描き直す段階の入口を 1 回だけにする)
static bool g_openNg = false;       // データ: 自己診断に NG がある(長く見せる)
static uint16_t g_openBg = 0;       // サーキット: 塗ってある背景
static bool g_openArcs = false;     // サーキット: 弧(とアイコン)が描いてある
static uint32_t g_openWaterKey = 0, g_openFuelKey = 0;  // サーキット: 描いてある弧の内容(走行画面と同じ waterKey / fuelKey)
static int g_openGearA = -1;        // サーキット: 描いてあるギアの濃さ(0〜16。-1 = まだ)

static float easeIn(float u) {  // はじめはゆっくり
  return u * u;
}
static float easeOut(float u) {  // はじめは速く
  return 1.0f - (1.0f - u) * (1.0f - u);
}
static float easeInOut(float u) {  // ゆっくり始まり、ゆっくり止まる
  return 0.5f - 0.5f * cosf(PI_F * u);
}
// 段階の中の進み(0〜1)。t = 始めてからの時刻、from = その段階の始まり、len = 長さ
static float stepU(uint32_t t, uint32_t from, uint32_t len) {
  if (t <= from) return 0.0f;
  if (len == 0 || t - from >= len) return 1.0f;
  return (float)(t - from) / (float)len;
}
// 終わり(飛ばしたときも): 明るさを設定の値に戻して覚えた画面へ(その画面の明るさの段階は次の blinkPoll で決まる)
static void openFinish() {
  blinkReset();
  go(g_homeSaved);
}

// ---- サーキット
static const uint16_t C_SEG_GHOST = rgb(0x1E242D);  // 消えているマス(オープニングの間だけ見せる)
static const uint16_t C_SEG_TAIL = rgb(0x7A8494);   // 走る光のすぐ後ろのマス
static const uint32_t OPC_T_FLASH = OPC_SCAN_MS;    // 段階の始まりの時刻
static const uint32_t OPC_T_RISE = OPC_T_FLASH + OPC_FLASH_MS;
static const uint32_t OPC_T_LIMIT = OPC_T_RISE + OPC_RISE_MS;
static const uint32_t OPC_T_FALL = OPC_T_LIMIT + OPC_LIMIT_MS;
static const uint32_t OPC_T_GEAR = OPC_T_FALL + OPC_FALL_MS;

// 仮の回転数 rpmV の色で、下から lit マスを点ける。背景は withBg のときだけ回転数の色(背景が変わるときは全部描き直す)
static void openRing(int rpmV, int lit, bool withBg) {
  uint16_t bgT = withBg ? to565(colorAt(BACK_C, rpmV, g_set->shiftRpm)) : C_BG;
  uint16_t litC = to565(colorAt(RING_C, rpmV, g_set->shiftRpm));
  if (bgT != g_openBg) {
    g_openBg = bgT;
    g_bgCol = bgT;
    gfx::fillAll(bgT);
    for (int k = 0; k < RING_SEGS; k++) g_segCol[k] = 0;
    g_openArcs = false;
  }
  for (int k = 0; k < RING_SEGS; k++) {
    uint16_t c = segLit(k, lit) ? litC : bgT;
    if (c != g_segCol[k]) drawSeg(k, c);
  }
}
static void openSegs(const uint16_t want[RING_SEGS]) {  // 色が変わったマスだけ描く
  for (int k = 0; k < RING_SEGS; k++) {
    if (want[k] != g_segCol[k]) drawSeg(k, want[k]);
  }
}
// 水温と燃料の弧とアイコンを、走行画面と同じ描き方(drawWaterPanel / drawFuelPanel)で描く。pw・pf = 0〜1 で、水温は
// WATER_BAR_MIN_C〜WATER_BAR_MAX_C、燃料は 0〜100 % の仮の値にする。異常の表示(水温が警告温度以上、燃料が FUEL_ALARM_PCT 未満)
// も走行画面と同じに出る(赤で点滅。10/21: 本人の指示「実際と同じように」)。本物の信号の異常の判定(g_waterAlarm など)は壊さない
static void openPanels(float pw, float pf) {
  Signals v;
  memset(&v, 0, sizeof(v));
  v.waterOk = v.fuelOk = true;
  v.waterC = WATER_BAR_MIN_C + (WATER_BAR_MAX_C - WATER_BAR_MIN_C) * pw;
  v.fuelPct = 100.0f * pf;
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
static float openWaterP(const Signals &s) {  // 今の水温の弧の長さ(0〜1。信号なしは 0)
  return s.waterOk ? clampf((s.waterC - WATER_BAR_MIN_C) / (WATER_BAR_MAX_C - WATER_BAR_MIN_C), 0.0f, 1.0f) : 0.0f;
}
static float openFuelP(const Signals &s) {  // 今の燃料の弧の長さ(0〜1。信号なしは 0)
  return s.fuelOk ? clampf(s.fuelPct / 100.0f, 0.0f, 1.0f) : 0.0f;
}
static int scanSeg(int p) {  // 走る光の位置 p(0 = 真下、1〜12 = 片側の 1〜12 マス目、13 = 真上)のマス
  return p == 0 ? RING_SEG_BOTTOM : (p >= 13 ? RING_SEG_TOP : p - 1);
}
static void openDrive(const Signals &s, uint32_t t, uint32_t now) {
  const int sp = g_set->shiftRpm;
  const int rLo = sp - 11 * RING_STEP_RPM;  // 1 マス目が点く回転数
  const int rHi = sp + RING_RED_OFFSET;     // 全部が赤
  uint16_t want[RING_SEGS];
  if (t < OPC_T_RISE) {  // 1・2: 白い光が下から上へ走り(左右とも)、真上で止まる。通ったマスは暗い灰
    const int h = t < OPC_T_FLASH ? (int)(14u * t / OPC_SCAN_MS) : 13;
    for (int k = 0; k < RING_SEGS; k++) want[k] = C_BG;
    for (int p = 0; p <= h; p++) want[scanSeg(p)] = p == h ? C_WHITE : ((p == h - 1 && t < OPC_T_FLASH) ? C_SEG_TAIL : C_SEG_GHOST);
    openSegs(want);
    if (t >= OPC_T_FLASH) openPanels(0.0f, 0.0f);  // 2: 弧の地とアイコン
    return;
  }
  if (t < OPC_T_LIMIT) {  // 3: 回転数を上げるように点いていく(はじめはゆっくり)。弧も一杯まで伸びる
    const float e = easeIn(stepU(t, OPC_T_RISE, OPC_RISE_MS));
    const int rpm = rLo + (int)lroundf((float)(rHi - rLo) * e);
    const int n = litPerSide(rpm, sp);
    const uint16_t lit = to565(colorAt(RING_C, rpm, sp));
    for (int k = 0; k < RING_SEGS; k++) want[k] = segLit(k, n) ? lit : C_SEG_GHOST;
    openSegs(want);
    openPanels(e, e);
    blinkRun(true, rpm, true, now);
    return;
  }
  if (t < OPC_T_FALL) {  // 4: 全部点いて背景も赤、最大輝度(リミッター)
    if (g_openPhase < 1) {
      g_openPhase = 1;
      openRing(rHi, 12, true);  // 背景の色が変わるので全部を描き直す(弧も)
    }
    openPanels(1.0f, 1.0f);  // 水温の警告の点滅が進む
    blinkRun(true, rHi, true, now);
    return;
  }
  if (g_openPhase < 2) {  // 5 の入口: 背景を黒に戻す(全部を描き直す)
    g_openPhase = 2;
    g_openBg = C_BG;
    g_bgCol = C_BG;
    gfx::fillAll(C_BG);
    for (int k = 0; k < RING_SEGS; k++) g_segCol[k] = C_BG;
    g_openArcs = false;
  }
  // 5: 回転数を下げるように消えていく(はじめは速く。今の回転数まで。信号なしは全部消える)。弧は今の値まで縮む
  const int rEnd = s.rpmOk ? s.rpm : rLo - RING_STEP_RPM;
  const float e = easeOut(stepU(t, OPC_T_FALL, OPC_FALL_MS));
  const int rpm = rHi + (int)lroundf((float)(rEnd - rHi) * e);
  const int n = litPerSide(rpm, sp);
  const uint16_t lit = to565(colorAt(RING_C, rpm, sp));
  for (int k = 0; k < RING_SEGS; k++) want[k] = segLit(k, n) ? lit : C_BG;
  openSegs(want);
  openPanels(1.0f + (openWaterP(s) - 1.0f) * e, 1.0f + (openFuelP(s) - 1.0f) * e);
  blinkRun(true, rpm, true, now);
  if (t < OPC_T_GEAR) return;
  // 6: 真ん中にギアが浮かぶ(黒から白へ 16 段)。終わったらサーキットの画面(全部を描き直す。同じ絵になる)
  const int a = (int)(16.0f * stepU(t, OPC_T_GEAR, OPC_GEAR_MS));
  if (a != g_openGearA) {
    g_openGearA = a;
    gfx::fillRect(GEAR_CLR_X0, GEAR_CLR_Y0, GEAR_CLR_X1 - GEAR_CLR_X0, GEAR_CLR_Y1 - GEAR_CLR_Y0, C_BG);
    char g[4] = "–";
    if (s.gearOk) {
      g[0] = s.gear;
      g[1] = 0;
    }
    if (a > 0) textC(GEAR_FONT, 233, GEAR_BASELINE, g, gfx::blend(C_BG, C_WHITE, (uint32_t)(a * 255 / 16)));
  }
  if (t >= OPC_TOTAL_MS) openFinish();
}

// ---- タコ
static const uint32_t OPT_T_TOP = OPT_UP_MS;  // 段階の始まりの時刻
static const uint32_t OPT_T_DOWN = OPT_T_TOP + OPT_TOP_MS;
static const uint32_t OPT_T_SETTLE = OPT_T_DOWN + OPT_DOWN_MS;
static Signals openTachSig(int rpm, float bar) {  // 仮の値(回転数とブーストだけ。水温・燃料は無し = 札を出さない)
  Signals f;
  memset(&f, 0, sizeof(f));
  f.linkUp = true;
  f.rpmOk = true;
  f.rpm = rpm;
  f.boostOk = true;
  f.boostBar = bar;
  return f;
}
// 仮の値でタコの画面を描く(updBoost。ピークの印は出さない)。文字盤を隠している間に目盛を赤にする・戻すときは、ここで
// 文字盤から描き直して隠す所を黒にする(updBoost の描き直しでは全部が出てしまうため)
static void openTachDraw(const Signals &f, uint32_t now) {
  tachRedTrack(f);
  if (g_tachRed != g_faceRed && g_hideA >= 0 && g_face) {
    drawBoostFace(f);
    memcpy(g_face, gfx::frame(), (size_t)gfx::W * gfx::H * sizeof(uint16_t));
    hideApply(gfx::Rect{ 0, 0, (int16_t)gfx::W, (int16_t)gfx::H });
    g_needleKey = g_tachKey = g_peakKey = -1;
  }
  const float peak = g_peakRpm;
  g_peakRpm = -1.0f;
  updBoost(f, now);
  g_peakRpm = peak;
  blinkRun(true, f.rpm, true, now);
}
static void openTach(const Signals &s, uint32_t t, uint32_t now) {
  if (g_openPhase < 1) {  // 最初の回: 文字盤を描いて g_face に取り、隠す所を黒に(取っておけなかったときは隠さない)
    g_openPhase = 1;
    const Signals f0 = openTachSig(0, 0.0f);
    tachRedTrack(f0);
    if (!g_face) g_face = (uint16_t *)platAlloc((size_t)gfx::W * gfx::H * sizeof(uint16_t));
    drawBoostFace(f0);
    g_faceOk = g_face != nullptr;
    if (g_face) {
      memcpy(g_face, gfx::frame(), (size_t)gfx::W * gfx::H * sizeof(uint16_t));
      g_hideA = OPT_HIDE_A0;
      hideApply(gfx::Rect{ 0, 0, (int16_t)gfx::W, (int16_t)gfx::H });
    }
    g_needleKey = g_tachKey = g_peakKey = -1;
  }
  int rpm;
  float bar;
  if (t < OPT_T_SETTLE) {  // 1〜3: 0 → 振り切り → 止まる → 0。ブーストは回転数と同じ割合(輪の先は針と同じ向き)
    const float e = t < OPT_T_TOP ? easeInOut(stepU(t, 0, OPT_UP_MS)) : (t < OPT_T_DOWN ? 1.0f : 1.0f - easeInOut(stepU(t, OPT_T_DOWN, OPT_DOWN_MS)));
    rpm = (int)lroundf((float)TACH_MAX_RPM * e);
    bar = BOOST_SCALE_MAX_BAR * (float)rpm / (float)TACH_MAX_RPM;
  } else {  // 4: 今の値へ(信号なしは 0)
    const float e = easeOut(stepU(t, OPT_T_SETTLE, OPT_SETTLE_MS));
    rpm = (int)lroundf((s.rpmOk ? (float)s.rpm : 0.0f) * e);
    bar = (s.boostOk ? s.boostBar : 0.0f) * e;
  }
  const Signals f = openTachSig(rpm, bar);
  if (g_hideA >= 0) {  // 針の所まで文字盤を出す(振り切りで全部)
    float a = t < OPT_T_TOP ? (float)tachKey(f) * 0.25f : OPT_HIDE_A1;
    if (a > OPT_HIDE_A1) a = OPT_HIDE_A1;
    if (a > g_hideA) {
      wedgeReveal(g_hideA, a);
      g_hideA = a >= OPT_HIDE_A1 ? -1.0f : a;
      g_barForce = true;  // 写した所に帯の先が掛かっていることがある
    }
  }
  openTachDraw(f, now);
  if (t >= OPT_TOTAL_MS) {  // 描き直さずに、そのままタコの画面へ(g_face と描いたものの記録はそのまま使える)
    blinkReset();
    g_sc = SC_BOOST;
    g_full = false;
    g_nextMs = 0;
    g_holdShown = false;
    g_guideShown = false;
    g_tagsDirty = true;  // 「SIM」の印
  }
}

// ---- データ
// 自己診断の行 i(0 = CAN、1 = GPS、2 = IMU、3 = タッチ)の状態。下位 4 ビット = 種類(0 = OK 緑、1 = 待ち 灰、2 = NG 赤、3 = 切 灰)
static uint8_t g_openState[4];  // 出してある行の状態(変わったら描き直す)
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
static int odBase(int i) {  // 行 i のベースライン(4 行を中央にまとめる。字は 44 px、行の間隔 62 px)
  return 158 + 62 * i;
}
static void openRow(int i, const LinkStats &ls, const SysInfo &si) {  // 結果の行(大きい文字)
  const char *name, *state;
  uint8_t kind = openRowState(i, ls, si, &name, &state);
  g_openState[i] = kind;
  if (kind == 2) g_openNg = true;
  const uint16_t dot = kind == 0 ? rgb(0x22E36A) : kind == 2 ? C_ALARM : gfx::blend(C_BG, C_LABEL, 110);
  const uint16_t fg = kind == 2 ? C_ALARM : (kind == 0 ? C_WHITE : gfx::blend(C_BG, C_LABEL, 190));
  const int base = odBase(i);
  gfx::fillRect(70, base - 46, 330, 58, C_BG);
  gfx::fillArc(90, base - 15, 0, 9, 0, 360, dot);
  mixDraw(FONT_NUM600_44, FONT_JP600_44, 112, base, name, fg);
  float sw = mixWidth(FONT_NUM600_44, FONT_JP600_44, state);
  mixDraw(FONT_NUM600_44, FONT_JP600_44, 386 - sw, base, state, fg);
}
static const char *const OD_NAMES[4] = { "CAN", "GPS", "IMU", "タッチ" };
static void odDot(int i, bool lit) {  // 確かめている途中の点(明・暗で点滅)
  const int base = odBase(i);
  gfx::fillRect(79, base - 26, 22, 22, C_BG);
  gfx::fillArc(90, base - 15, 0, 9, 0, 360, lit ? C_LABEL : gfx::blend(C_BG, C_LABEL, 70));
}
static void odChecking(int i, bool lit) {  // 確かめている途中の行(名前を暗めに、結果はまだ出さない)
  const int base = odBase(i);
  gfx::fillRect(70, base - 46, 330, 58, C_BG);
  odDot(i, lit);
  mixDraw(FONT_NUM600_44, FONT_JP600_44, 112, base, OD_NAMES[i], gfx::blend(C_BG, C_LABEL, 190));
}
static const uint16_t C_OD_TRACK = C_DR_TRACK;  // 外周の輪の地(データ画面の回転数の輪と同じ)
static const float OD_R_IN = DATA_RING_R_IN, OD_R_OUT = DATA_RING_R_OUT;  // 外周の輪の半径(2.5.0: データ画面の回転数の輪と同じ所。214〜224 から寄せた)
static float g_odTrack = 0.0f;  // 描いた輪の地の広がり(真上から左右へ、度)
static float g_odFill = 0.0f;   // 埋めた輪(真上から時計回り、度)
static int8_t g_odPulse[4];     // 行の様子: 2 = まだ出していない、1・0 = 確かめている途中(点の明・暗)、-1 = 結果を出した
static bool g_odDone = false;   // 全部の結果を出して、輪を結果の色にした
static bool g_odDoneNg = false; // そのときの色(NG があった)
static uint32_t odRowT(int i) {  // 行 i を出す時刻
  return OPD_RING_MS + (uint32_t)i * OPD_ROW_MS;
}
static void odArc(float a0, float a1, uint16_t c) {
  gfx::fillArc(233, 233, OD_R_IN, OD_R_OUT, a0, a1, c);
}
static void openData(const LinkStats &ls, const SysInfo &si, uint32_t t) {
  // 1: 輪の地を真上から左右へ。少しずつ足す(つなぎ目は 1° 重ねて、縁のなめらかさで線が出ないようにする)
  const float tr = 180.0f * easeOut(stepU(t, 0, OPD_RING_MS));
  if (tr >= g_odTrack + 0.5f || (tr >= 180.0f && g_odTrack < 180.0f)) {
    if (g_odTrack <= 0.0f) {
      odArc(270.0f - tr, 270.0f + tr, C_OD_TRACK);
    } else {
      odArc(270.0f - tr, 271.0f - g_odTrack, C_OD_TRACK);
      odArc(269.0f + g_odTrack, 270.0f + tr + (tr >= 180.0f ? 1.0f : 0.0f), C_OD_TRACK);  // 真下で左右がつながる所も重ねる
    }
    g_odTrack = tr;
  }
  // 2: 行を出す。確かめている間は点が点滅し、輪が 1/4 ずつ埋まる。結果を出した行は、状態が変わったら描き直す
  float fill = 0.0f;
  for (int i = 0; i < 4; i++) {
    fill += 90.0f * stepU(t, odRowT(i), OPD_CHECK_MS);
    if (t < odRowT(i)) continue;
    if (t >= odRowT(i) + OPD_CHECK_MS) {
      const char *n, *st;
      if (g_odPulse[i] != -1 || openRowState(i, ls, si, &n, &st) != g_openState[i]) openRow(i, ls, si);
      g_odPulse[i] = -1;
    } else {
      const int8_t lit = (((t - odRowT(i)) / OPD_PULSE_MS) & 1) ? 0 : 1;
      if (g_odPulse[i] == 2) odChecking(i, lit != 0);
      else if (g_odPulse[i] != lit) odDot(i, lit != 0);
      g_odPulse[i] = lit;
    }
  }
  if (!g_odDone && fill >= g_odFill + 0.5f) {
    odArc(270.0f + (g_odFill > 0.0f ? g_odFill - 1.0f : 0.0f), 270.0f + fill, C_ACCENT);
    g_odFill = fill;
  }
  // 3: 全部出たら、輪を結果の色に(NG が無ければ緑、あれば赤)。版を出す
  if (t >= odRowT(3) + OPD_CHECK_MS && (!g_odDone || g_odDoneNg != g_openNg)) {
    if (!g_odDone) textC(FONT_NUM600_24, 233, 404, "v" METER_FW_VERSION, C_LABEL);  // 表示器の版(config.h)。自己診断の下
    g_odDone = true;
    g_odDoneNg = g_openNg;
    gfx::fillArcHard(233, 233, OD_R_IN - 2.0f, OD_R_OUT + 2.0f, 0, 360, C_BG);
    odArc(0, 360, g_openNg ? C_ALARM : C_OK);
  }
  if (t >= OPD_TOTAL_MS + (g_openNg ? OPENING_NG_EXTRA_MS : 0)) openFinish();
}

static void updOpening(const Signals &s, const LinkStats &ls, const SysInfo &si, uint32_t now) {
  if (g_full) {  // 始め: 覚えた画面で種類を決め、画面を黒にする(タコは openTach の最初の回で文字盤を描く)
    g_full = false;
    g_openKind = g_homeSaved == SC_BOOST ? OPK_TACH : (g_homeSaved == SC_DATA ? OPK_DATA : OPK_DRIVE);
    g_openT0 = now;
    g_openPhase = 0;
    g_openNg = false;
    blinkReset();
    if (g_openKind == OPK_DRIVE) {
      g_openBg = 0xFFFF;  // 必ず塗る
      g_openArcs = false;
      g_openGearA = -1;
      openRing(0, 0, false);  // 黒で塗る(マスは描かない)
    } else if (g_openKind == OPK_DATA) {
      gfx::fillAll(C_BG);
      g_odTrack = g_odFill = 0.0f;
      for (int i = 0; i < 4; i++) g_odPulse[i] = 2;
      g_odDone = false;
      g_odDoneNg = false;
    }
  }
  const uint32_t t = now - g_openT0;
  if (g_openKind == OPK_TACH) openTach(s, t, now);
  else if (g_openKind == OPK_DATA) openData(ls, si, t);
  else openDrive(s, t, now);
}

// ================================================================ 「SIM」の印
// 模擬データを受けている間、「SIM」と場面番号を出す(仕様書に無い。車で模擬の値を本物と取り違えないため)。
// 置く所(42×21 の四角): データ画面 = 下に「SIM」、上に場面番号(2.5.0: 外周の回転数の輪の内側に入れた)。走行画面 = 輪の真上・真下を 10/21 にマスで埋めたので、
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
  int simX = 212, simY = 430, sceneX = 212, sceneY = 15;  // 四角の左上(データ画面。2.5.0: 437・8 から回転数の輪の内側へ)
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

// ================================================================ タッチ(2.3.0。2.4.0 で作り直し。config.h の TOUCH_CONFIRM_MS・HOLD_*・SWIPE_*)
// タッチは触れてから TOUCH_CONFIRM_MS 押し続けたら決まる(それより短いタッチは何もしない)。2.4.0: どの画面も離したときに決める:
//   横に SWIPE_MIN_PX 以上(縦の SWIPE_DIR_RATIO 倍以上)動いていればスワイプ(左へ = 次、右へ = 前。2.4.2: 2.4.0〜2.4.1 の右へ = 次を戻した。
//   本人 10/24「間違えました 左にスワイプでタコでした」)、
//   動きが HOLD_CANCEL_PX 未満ならタップ(触れた所)。サーキット・タコ・データのタップは何もしない。
// サーキット・タコ・データ: 決まったら案内を出す(上に「‹ スワイプ ›」、下に「長押し ▶ 設定」。上下対称で同じ幅。本人の指示 10/24)。
//   HOLD_SHOW_MS から長押しの途中: 案内の画面のまま、スワイプの札を灰にして外周が HOLD_MS までに一周していく(暗くしない。札の文字も
//   位置も変えない。本人の指示 10/24「タイトルが変わることで認識しづらい」)。HOLD_MS で設定へ。
//   指が HOLD_CANCEL_PX 以上動いたら長押しを取り消し、案内は出したまま長押しの札を灰にする(元の画面に戻さない。指を離すまで)。
//   案内・長押しの表示の間は、画面の描き直しを止める。
// 設定の中のメニュー(2.4.4): 画面のどこかを押し続けると(2.4.5: 2.4.4 は下の帯だけ)、HOLD_SHOW_MS で下の帯「長押しで開く」を水色にして
//   (案内の代わり)外周が回り、HOLD_MS で開く(menuOpen)。2.5.0: 2.4.5 までは TOUCH_CONFIRM_MS で帯を水色にしていて、矢印のタップでも
//   帯が反転した。指が HOLD_CANCEL_PX 以上動いたら取り消し、外周を消して帯の字を灰にする(離すまで)。外周が出てから HOLD_MS の前に
//   離したら何もしない(2.7.0)。短いタップは、その所のタップ(上の帯 = 戻る、左右の半分 = 前・次。下の帯は何もしない)
static const uint16_t C_HOLD_TRACK = rgb(0x1A2433);    // 外周の地
static const uint16_t C_HOLD_PILL = rgb(0x0B1220);     // 札の中
static const uint16_t C_PILL_DIM = rgb(0x343B47);      // 灰にした札の枠
static const uint16_t C_PILL_DIM_TEXT = rgb(0x5A606A); // 灰にした札の字と印
static struct {
  bool down;
  int x, y;         // 触れた所
  int lx, ly;       // 最後に知らされた所(スワイプの判定)
  uint32_t t0;      // 触れた時刻
  bool confirmed;   // TOUCH_CONFIRM_MS 押し続けた(タッチが決まった)
  bool moved;       // HOLD_CANCEL_PX 以上動いた(長押しを取り消した。離してもタップにしない)
  bool guided;      // このタッチで案内を出した(取り消したあとも、離すまで案内を出す)
  bool switched;    // 長押しで設定に入った・下の帯の操作を待っている間に触れた(2.7.0)。離してもタップ・スワイプにしない
} g_touch;
enum GuideMode : uint8_t { GM_GUIDE, GM_HOLD, GM_CANCEL };  // 案内・長押しの途中(スワイプを灰)・長押しを取り消した(長押しを灰)
static GuideMode g_guideMode = GM_GUIDE;  // 描いてある案内の状態
static float g_holdDeg = 0;  // 外周に描いた角度

static bool isMain(Screen sc) {
  return sc == SC_DRIVE || sc == SC_BOOST || sc == SC_DATA;
}
static Screen nextMain(Screen sc) {
  return sc == SC_DRIVE ? SC_BOOST : sc == SC_BOOST ? SC_DATA : SC_DRIVE;
}
static Screen prevMain(Screen sc) {
  return sc == SC_DRIVE ? SC_DATA : sc == SC_BOOST ? SC_DRIVE : SC_BOOST;
}
// 今の指の位置で離したら、どちらへのスワイプになるか(1 = 左へ = 次、-1 = 右へ = 前、0 = スワイプにならない)
static int swipeDir() {
  const int dx = g_touch.lx - g_touch.x, dy = g_touch.ly - g_touch.y;
  const int ax = dx < 0 ? -dx : dx, ay = dy < 0 ? -dy : dy;
  if (ax >= SWIPE_MIN_PX && ax >= SWIPE_DIR_RATIO * ay) return dx < 0 ? 1 : -1;
  return 0;
}
// 札 2 つ(高さ 78、両端は半円、枠 3 px)。上 = スワイプ、下 = 長押し。画面の中心について上下対称、幅は 2 つとも同じ
static const float PILL_H = 78, PILL_R = 39;
static const float PILL_Y_TOP = 74;                         // 上の札の上端
static const float PILL_Y_BOT = 466 - PILL_Y_TOP - PILL_H;  // 下の札の上端(上下対称。314)
static float pillW() {
  const FontData &f = FONT_JP600_48;
  const float ws = gfx::textWidth(f, "スワイプ") + 140;
  const float wh = 30 + gfx::textWidth(f, "長押し") + 12 + 24 + 12 + gfx::textWidth(f, "設定") + 30;
  return ws > wh ? ws : wh;
}
static void drawPillFrame(float x, float y, float w, bool dim) {
  gfx::frameRoundRect(x, y, w, PILL_H, PILL_R, 3, dim ? C_PILL_DIM : C_ACCENT, C_HOLD_PILL);
}
static void drawPillTri(float x, float cy, uint16_t c) {  // 札の中の「▶」(幅 24、高さ 36)
  gfx::Pt p[3] = { { x, cy - 18 }, { x + 24, cy }, { x, cy + 18 } };
  gfx::polyBegin(gfx::Rect{ (int16_t)(x - 4), (int16_t)(cy - 23), (int16_t)(x + 28), (int16_t)(cy + 23) });
  gfx::polyAdd(p, 3);
  gfx::polyFill(c);
}
static void drawSwipePill(bool dim) {  // 上の札「‹ スワイプ ›」
  const FontData &f = FONT_JP600_48;
  const float w = pillW(), x = 233 - w * 0.5f, cy = PILL_Y_TOP + PILL_H * 0.5f;
  const uint16_t mark = dim ? C_PILL_DIM_TEXT : C_ACCENT, text = dim ? C_PILL_DIM_TEXT : C_WHITE;
  drawPillFrame(x, PILL_Y_TOP, w, dim);
  bigChevron(x + 42, cy, 36, -1, mark, 11.0f, 3.5f);
  bigChevron(x + w - 42, cy, 36, 1, mark, 11.0f, 3.5f);
  textC(f, 233, cy + f.px * 0.36f, "スワイプ", text);
}
static void drawHoldPill(bool dim) {  // 下の札「長押し ▶ 設定」
  const FontData &f = FONT_JP600_48;
  const float w1 = gfx::textWidth(f, "長押し"), w2 = gfx::textWidth(f, "設定");
  const float wc = w1 + 12 + 24 + 12 + w2;  // 中身の幅
  const float w = pillW(), x = 233 - w * 0.5f, cy = PILL_Y_BOT + PILL_H * 0.5f, cx = 233 - wc * 0.5f;
  const uint16_t mark = dim ? C_PILL_DIM_TEXT : C_ACCENT, text = dim ? C_PILL_DIM_TEXT : C_WHITE;
  drawPillFrame(x, PILL_Y_BOT, w, dim);
  gfx::drawText(f, cx, cy + f.px * 0.36f, "長押し", text);
  drawPillTri(cx + w1 + 12, cy, mark);
  gfx::drawText(f, cx + w1 + 12 + 24 + 12, cy + f.px * 0.36f, "設定", text);
}
// 案内(2.3.0。本人の指示 10/24「タッチした瞬間に何も反応しない」)。札は同じ形・同じ所のまま、状態で色だけを変える。
// 設定の中のメニュー(2.4.4)は、下の帯「長押しで開く」の色を変える(長押しの途中 = 水色の帯、取り消した = 灰の字。
// 2.5.0: 水色にするのは HOLD_SHOW_MS から。タップ・スワイプでは帯を変えない)
static void drawGuide(GuideMode m) {
  g_guideMode = m;
  if (!isMain(g_sc)) {
    if (m == GM_CANCEL) drawOpenBand(C_BAND_BOT, C_PILL_DIM_TEXT);
    else drawOpenBand(C_ACCENT, C_BG);
    return;
  }
  drawSwipePill(m == GM_HOLD);
  drawHoldPill(m == GM_CANCEL);
}
static void drawHoldShow() {  // 長押しの途中: スワイプの札を灰にし、外周の地を描く
  drawGuide(GM_HOLD);
  gfx::fillArc(233, 233, 221, 233, 0, 360, C_HOLD_TRACK);
  g_holdDeg = 0;
}
static void drawHoldProgress(float frac) {
  float deg = 360.0f * (frac > 1 ? 1 : frac);
  if (deg <= g_holdDeg + 0.5f) return;
  gfx::fillArc(233, 233, 221, 233, 270 + g_holdDeg, 270 + deg, C_ACCENT);
  g_holdDeg = deg;
}
// 今の画面に、下の帯「長押しで開く」があるか(2.4.4。設定の項目はどれも、受信の状態はどのグループも、グループの中は操作のあるページ、
// CAN確認の一覧は候補があるとき)
static bool menuOpenable() {
  switch (g_sc) {
    case SC_SET_LIST:
    case SC_SET_INFO:
      return true;
    case SC_SET_GRP:
      return g_grpAct != A_NONE;
    case SC_CAN_ID:
      return g_canIdx < g_listN;
    default:
      return false;
  }
}
// 下の帯「長押しで開く」の長押し(2.4.4。2.4.3 までは帯のタップで開いていた)
static void menuOpen() {
  switch (g_sc) {
    case SC_SET_LIST: {
      const Item &it = ITEMS[g_setItem];
      if (it.kind == K_VALUE) {
        startEdit(&it, SC_SET_LIST);
      } else if (it.kind == K_TIME) {
        g_resetKind = RK_TIME;
        g_resetFrom = SC_SET_LIST;
        go(SC_SET_RESET);
      } else if (it.kind == K_LINK) {
        g_infoPage = 0;
        go(SC_SET_INFO);
      } else {
        openGroup(G_SYS, SC_SET_LIST);
      }
      break;
    }
    case SC_SET_INFO:
      openGroup(g_infoPage, SC_SET_INFO);
      break;
    case SC_SET_GRP:
      if (g_grpAct == A_OPEN_CAN) {
        go(SC_CAN);
      } else if (g_grpAct == A_OPEN_CANPP) {  // 2.6.0
        go(SC_CAN_PP);
      } else if (g_grpAct == A_EDIT_RC) {
        startEdit(&RC_SEND_ITEM, SC_SET_GRP);
      } else if (g_grpAct == A_IMU_RESET) {
        g_resetKind = RK_IMU;  // 確かめてから捨てる(2.4.3。前は押すとすぐ捨てていた)
        g_resetFrom = SC_SET_GRP;
        go(SC_SET_RESET);
      }
      break;
    case SC_CAN_ID:
      g_detEntry = listEntry(g_canIdx);
      g_detByte = 0;
      for (int b = 0; b < 8; b++) {
        if (listMask(g_canIdx) >> b & 1) {
          g_detByte = b;
          break;
        }
      }
      go(SC_CAN_DETAIL);
      break;
    default:
      break;
  }
}
// 長押しを数える画面: サーキット・タコ・データと、設定の中で「長押しで開く」がある画面。どちらも画面のどこでも
// (2.4.5。本人「長押しで開くはその場所ではなく画面全体です」。2.4.4 は下の帯だけで数えていた)
static bool holdArea() {
  return isMain(g_sc) || menuOpenable();
}
// 長押しとして数えているタッチ(長押しを数える所で、決まっていて、動いていない)
static bool holdTouch() {
  return g_touch.down && holdArea() && g_touch.confirmed && !g_touch.moved && !g_touch.switched;
}
static bool holdDue(uint32_t now) {  // 長押しの表示を出す時刻を過ぎた
  return holdTouch() && (uint32_t)(now - g_touch.t0) >= HOLD_SHOW_MS;
}
// update の最初で呼ぶ。タッチが決まったか(案内は画面を描いたあと。スワイプ・タップは離したとき、長押しは holdStep)
static void touchStep(uint32_t now) {
  if (!g_touch.down || g_touch.confirmed || g_touch.switched) return;
  if ((uint32_t)(now - g_touch.t0) < TOUCH_CONFIRM_MS) return;
  g_touch.confirmed = true;
  g_lastActMs = now;
}
// update の最初で呼ぶ。長押しで設定へ(前回見ていた項目)。設定の中のメニューは開く(2.4.4)
static void holdStep(uint32_t now) {
  if (!holdTouch() || (uint32_t)(now - g_touch.t0) < HOLD_MS) return;
  g_lastActMs = now;
  if (isMain(g_sc)) {
    g_homeCur = g_sc;
    go(SC_SET_LIST);
  } else {
    menuOpen();
  }
  g_touch.switched = true;
}
// 外周を今の時刻まで進める(表示を出したところ(HOLD_SHOW_MS)から HOLD_MS までに一周する。2.2.0: 出した瞬間に半分まで飛ばない)
static void holdProgress(uint32_t now) {
  const uint32_t el = now - g_touch.t0;
  const float span = (float)(HOLD_MS - HOLD_SHOW_MS);
  drawHoldProgress(el > HOLD_SHOW_MS ? (float)(el - HOLD_SHOW_MS) / span : 0.0f);
}
// ---- 押した帯の反転(2.7.0。本人「can確認画面などでボタンを押した時に押したことがわかりにくいので、反転表示を入れてください」)。
// タップで動く帯(tapBand)は、タッチが決まってから(TOUCH_CONFIRM_MS)離すまで反転して描く。長押しの表示が出たら(HOLD_SHOW_MS。
// 設定の中)・指が HOLD_CANCEL_PX 以上動いたら元に戻す。下の帯の操作は、離してから PRESS_FLASH_MS 反転のまま見せて(指で隠れていた所が
// 見えるように)、それから行う(「決定」の点滅と同じく、押した所で分かるように)。上の帯(1 つ上へ)は離したらすぐ移る
static struct {
  bool on;
  int x, y;      // タップの位置
  uint32_t due;  // 行う時刻
  Screen sc;     // 押した画面(ほかの画面に移っていたら行わない)
} g_pressTap;
// 帯 z が今の画面でタップで動くか。上の帯 = 設定・CAN確認のどの画面も 1 つ上へ(変更の画面の「決定」の点滅の間は受けない)。
// 下の帯 = その場の操作(CAN確認の「変化を記録」、前後で比べるの「前を記録」「後を記録」(記録できるとき)、赤い帯のリセット・取り直す・
// やり直す)。「長押しで開く」はタップでは動かない。「決定」は押したあとの点滅がある(leaveEdit)ので入れない
static bool tapBand(int z, uint32_t now) {
  if (isMain(g_sc) || g_sc == SC_OPENING) return false;
  if (z == Z_TOP) return !(g_sc == SC_SET_EDIT && g_saveFlash);
  if (z != Z_BOT) return false;
  switch (g_sc) {
    case SC_CAN:
    case SC_SET_RESET:
    case SC_CAN_REDO:
      return true;
    case SC_CAN_PP:
      return cantab::ppWaitMs(now) == 0;
    default:
      return false;
  }
}
static int pressWant(uint32_t now) {  // 今、反転して描く帯
  if (g_pressTap.on) return Z_BOT;
  if (!g_touch.down || !g_touch.confirmed || g_touch.moved || g_touch.switched || holdDue(now)) return -1;
  const int z = zoneAt(g_touch.x, g_touch.y);
  return tapBand(z, now) ? z : -1;
}
// update の初めで呼ぶ。下の帯の操作を行う時刻になったら行い、反転する帯が変わったら画面を描き直す
static void pressStep(uint32_t now) {
  if (g_pressTap.on && (g_sc != g_pressTap.sc || (int32_t)(now - g_pressTap.due) >= 0)) {
    g_pressTap.on = false;
    if (g_sc == g_pressTap.sc) tap(g_pressTap.x, g_pressTap.y, now);
  }
  const int want = pressWant(now);
  if (want == g_pressBand) return;
  g_pressBand = want;
  g_full = true;
}
// 画面を描いたあとで呼ぶ。タッチが決まっていれば案内を出す(長押しの時刻を過ぎていれば長押しの途中の形で)。
// 決まる前から指が動いていた(スワイプの途中)ときは出さない。案内を出したあとに取り消したときは、長押しの札を灰にして出し直す。
// 設定の中は、長押しの時刻(HOLD_SHOW_MS)になってから出す(下の帯を水色にして外周。2.5.0)
static void guideShow(uint32_t now) {
  if (g_guideShown || !g_touch.down || !holdArea() || !g_touch.confirmed || g_touch.switched) return;
  if (g_touch.moved && !g_touch.guided) return;
  // 2.5.0: 設定の中は、長押しの表示を出す時刻(HOLD_SHOW_MS)までは何も出さない(本人「設定で左右の矢印をタップした時に長押しで開く
  // ボタンが反転してしまうので混乱します」。2.4.5 は触れて TOUCH_CONFIRM_MS で下の帯を水色にしていて、矢印のタップでも帯が変わった)
  if (!isMain(g_sc) && !g_touch.guided && !holdDue(now)) return;
  g_guideShown = true;
  g_touch.guided = true;
  if (holdDue(now)) {
    drawHoldShow();
    g_holdShown = true;
    holdProgress(now);
  } else {
    drawGuide(g_touch.moved ? GM_CANCEL : GM_GUIDE);
  }
}
// 案内・長押しの表示を出している間の update(画面は描き直さない)。true = この回はここまで(画面を描かない)
static bool guideStep(uint32_t now) {
  if (!g_guideShown) return false;
  if (g_touch.moved) {
    if (!g_holdShown) {  // 案内の間に取り消した: 長押しの札だけを灰に
      if (g_guideMode != GM_CANCEL) drawGuide(GM_CANCEL);
      return true;
    }
    // 長押しの途中で取り消した: 外周を消すため今の画面を描き直し、そのあと案内を出し直す(guideShow。長押しの札は灰)
    g_holdShown = false;
    g_guideShown = false;
    g_full = true;
    return false;
  }
  if (!g_holdShown && holdDue(now)) {
    drawHoldShow();
    g_holdShown = true;
  }
  if (g_holdShown) holdProgress(now);
  return true;
}

// ================================================================ 入口
void begin(Settings *settings, uint8_t home) {
  g_set = settings;
  gfx::setTabularDigits(DIGITS_TABULAR != 0);
  buildRing();
  for (int k = 0; k < RING_SEGS; k++) g_segCol[k] = 0;
  g_homeSaved = isMain((Screen)home) ? (Screen)home : SC_DRIVE;
  g_homeCur = g_homeSaved;
  go(OPENING_ON ? SC_OPENING : g_homeSaved);
}
uint8_t homeScreen() {
  return g_homeSaved;
}

// 「決定」: 変えていれば保存する。そのあと下の「決定」の帯を点滅させ(値は太く)、点滅が終わってから戻る(updSetEdit)。
// 2.4.5: 変えていないとき・保存に失敗したあとの 2 回目も点滅させる(本人「決定のフラッシュが消えました」。2.4.3 は保存したときだけだった)
static void leaveEdit(uint32_t now) {
  const Item &it = *g_editItem;
  if (g_set->*(it.field) != g_editOrig && !g_saveTried) {
    g_saveTried = true;
    if (!platSaveSettings(*g_set)) {  // 保存に失敗: 1 回目はここに留まって知らせる。もう一度「決定」で抜ける
      g_saveFailed = true;
      drawEditDesc();
      return;
    }
    g_editOrig = g_set->*(it.field);  // 保存した値(時間で戻るときに、もう一度保存しない)
  }
  g_saveFlash = true;
  g_saveFlashT0 = now;
  drawEditValue();  // 太く
  drawEditBottom(0);  // 押した所をすぐ消す(点滅の 1 回目)
  g_saveFlashPhase = 0;
}
static void cancelEdit() {  // 変えずに元の画面へ
  const Item &it = *g_editItem;
  if (g_set->*(it.field) != g_editOrig) {
    g_set->*(it.field) = g_editOrig;
    if (it.field == &Settings::brightnessPct) platSetBrightness(g_editOrig);
  }
  go(g_editFrom);
}

// タップ(離したとき。位置は触れた所)。真ん中の左半分・右半分は前・次(スワイプと同じ。2.4.1 で戻した)
void tap(int x, int y, uint32_t nowMs) {
  g_lastActMs = nowMs;
  const Zone z = zoneAt(x, y);
  switch (g_sc) {
    case SC_OPENING:  // オープニングを飛ばす
      openFinish();
      break;
    case SC_DRIVE:
    case SC_BOOST:
    case SC_DATA:  // 2.1.0: タップは何もしない(設定は長押し、画面の切り替えはスワイプ)
      break;
    case SC_SET_LIST:  // 下の帯「長押しで開く」はタップでは開かない(2.4.4。長押しは menuOpen)
      if (z == Z_TOP) {
        go(g_homeCur);
      } else if (z == Z_LEFT || z == Z_RIGHT) {
        swipe(z == Z_RIGHT ? 1 : -1, nowMs);
      }
      break;
    case SC_SET_EDIT: {
      const Item &it = *g_editItem;
      if (g_saveFlash) break;  // 保存したあとの点滅の間は受けない(終わったら元の画面へ)
      if (z == Z_TOP) {
        cancelEdit();
      } else if (z == Z_BOT) {
        leaveEdit(nowMs);
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
        go(g_resetFrom);
      } else if (z == Z_BOT) {
        if (g_resetKind == RK_TIME) {
          trip::resetTimes();
          platSaveTimes();
        } else {
          platResetImuCal();
        }
        go(g_resetFrom);
      }
      break;
    case SC_SET_INFO:
      if (z == Z_TOP) {
        go(SC_SET_LIST);
      } else if (z == Z_LEFT || z == Z_RIGHT) {
        swipe(z == Z_RIGHT ? 1 : -1, nowMs);
      }
      break;
    case SC_SET_GRP:
      if (z == Z_TOP) {
        if (g_grpFrom == SC_SET_INFO) g_infoPage = g_grp;  // 受信の状態の、そのグループのページへ
        go(g_grpFrom);
      } else if (z == Z_LEFT || z == Z_RIGHT) {
        swipe(z == Z_RIGHT ? 1 : -1, nowMs);
      }
      break;
    case SC_CAN:
      if (z == Z_TOP) {
        go(SC_SET_GRP);  // 受信の状態 → CAN の「CAN確認」のページへ(2.4.3)
      } else if (z == Z_BOT) {
        cantab::record();
        g_full = true;
      } else if (z == Z_LEFT && cantab::recordCount() > 0) {
        g_canPp = false;
        go(SC_CAN_REDO);
      } else if (z == Z_RIGHT && cantab::recordCount() > 0 && g_candN > 0) {
        g_canPp = false;
        g_canIdx = 0;
        go(SC_CAN_ID);
      }
      break;
    case SC_CAN_PP:  // 2.6.0: 前後で比べる
      if (z == Z_TOP) {
        go(SC_SET_GRP);  // 受信の状態 → CAN の「前後で比べる」のページへ
      } else if (z == Z_BOT) {
        if (cantab::ppHasPre()) cantab::ppRecordPost(nowMs);  // 記録できるまでは何もしない(帯に残りの秒)
        else cantab::ppRecordPre(nowMs);
        g_full = true;
      } else if (z == Z_LEFT && cantab::ppHasPre()) {
        g_canPp = true;
        go(SC_CAN_REDO);
      } else if (z == Z_RIGHT && cantab::ppHasResult() && cantab::ppResults(nullptr, 0) > 0) {
        g_canPp = true;
        g_canIdx = 0;
        go(SC_CAN_ID);
      }
      break;
    case SC_CAN_ID:  // 下の帯「長押しで開く」で詳細(2.4.4。menuOpen)
      if (z == Z_TOP) {
        go(g_canPp ? SC_CAN_PP : SC_CAN);
      } else if (z == Z_LEFT || z == Z_RIGHT) {
        swipe(z == Z_RIGHT ? 1 : -1, nowMs);
      }
      break;
    case SC_CAN_DETAIL:
      if (z == Z_TOP) {
        go(SC_CAN_ID);
      } else if (z == Z_LEFT || z == Z_RIGHT) {
        swipe(z == Z_RIGHT ? 1 : -1, nowMs);
      }
      break;
    case SC_CAN_REDO:
      if (z == Z_TOP) {
        go(g_canPp ? SC_CAN_PP : SC_CAN);
      } else if (z == Z_BOT) {
        if (g_canPp) cantab::ppClear();
        else cantab::clearRecord();
        go(g_canPp ? SC_CAN_PP : SC_CAN);
      }
      break;
    default:
      break;
  }
}

// 横のスワイプ(2.4.0。dir = 1: 次(左へのスワイプ、右の「›」)、-1: 前(右へのスワイプ、左の「‹」))。サーキット・タコ・データは画面、設定の中は項目・ページ、
// CAN確認の一覧は ID、詳細はバイト。ほかの画面では何もしない(オープニングは飛ばす)
void swipe(int dir, uint32_t nowMs) {
  g_lastActMs = nowMs;
  const int d = dir > 0 ? 1 : -1;
  switch (g_sc) {
    case SC_OPENING:
      openFinish();
      break;
    case SC_DRIVE:
    case SC_BOOST:
    case SC_DATA:
      go(d > 0 ? nextMain(g_sc) : prevMain(g_sc));
      break;
    case SC_SET_LIST:
      g_setItem = (g_setItem + N_ITEMS + d) % N_ITEMS;
      g_full = true;
      break;
    case SC_SET_INFO:
      g_infoPage = (g_infoPage + N_LINK_GROUPS + d) % N_LINK_GROUPS;
      g_full = true;
      break;
    case SC_SET_GRP: {
      const int n = GROUP_N[g_grp < G_COUNT ? g_grp : G_SYS];
      g_grpPage = (g_grpPage + n + d) % n;
      g_full = true;
      break;
    }
    case SC_CAN_ID: {
      const int shown = g_listN;
      if (shown > 0) {
        g_canIdx = (g_canIdx + shown + d) % shown;
        g_full = true;
      }
      break;
    }
    case SC_CAN_DETAIL: {
      cantab::View v;
      const int n = cantab::getView(g_detEntry, &v) && v.dlc ? v.dlc : 8;
      g_detByte = (g_detByte + n + d) % n;
      break;
    }
    default:
      break;
  }
}

void touchDown(int x, int y, uint32_t nowMs) {
  g_lastActMs = nowMs;
  g_touch.down = true;
  g_touch.x = g_touch.lx = x;
  g_touch.y = g_touch.ly = y;
  g_touch.t0 = nowMs;
  g_touch.confirmed = false;
  g_touch.moved = false;
  g_touch.guided = false;
  g_touch.switched = g_pressTap.on;  // 2.7.0: 下の帯の操作を待っている間(PRESS_FLASH_MS)のタッチは何もしない(「決定」の点滅の間と同じ)
}
void touchMove(int x, int y, uint32_t nowMs) {
  if (!g_touch.down) return;
  g_touch.lx = x;
  g_touch.ly = y;
  const int dx = x - g_touch.x, dy = y - g_touch.y;
  if (!g_touch.moved && dx * dx + dy * dy >= HOLD_CANCEL_PX * HOLD_CANCEL_PX) {
    g_touch.moved = true;  // 長押しを取り消す(案内の札は次の update で灰にする)。離してもタップにしない
    g_lastActMs = nowMs;
  }
}
void touchUp(uint32_t nowMs) {
  if (!g_touch.down) return;
  g_touch.down = false;
  g_lastActMs = nowMs;
  const bool held = g_holdShown && !g_touch.moved;  // 長押しの表示(外周)を出していて、動かしていない
  if (g_holdShown || g_guideShown) {  // 案内・長押しの表示を消す(今の画面を描き直す)
    g_holdShown = false;
    g_guideShown = false;
    g_full = true;
  }
  if (g_touch.switched) return;  // 長押しで設定に入った。離しても何もしない
  // 2.7.0: 長押しの表示(外周)が出てから HOLD_MS の前に離したのは、長押しをやめたもの。何もしない(本人「長押しを認識して、
  // アニメーションが出た後に指を離した場合は何も動作しないほうがいい」。2.6.0 までは、その所のタップになって設定の中で画面が移っていた)
  if (held) return;
  // 決まる前に離した: 短いタッチは何もしない(押していた時間は足りているのに update で確かめていなかったときは、ここで決める)
  if (!g_touch.confirmed && (uint32_t)(nowMs - g_touch.t0) < TOUCH_CONFIRM_MS) return;
  g_touch.confirmed = true;
  const int dir = swipeDir();
  if (dir) {
    swipe(dir, nowMs);
  } else if (!g_touch.moved) {
    if (zoneAt(g_touch.x, g_touch.y) == Z_BOT && tapBand(Z_BOT, nowMs)) {  // 2.7.0: 下の帯は反転を PRESS_FLASH_MS 見せてから(pressStep)
      g_pressTap.on = true;
      g_pressTap.x = g_touch.x;
      g_pressTap.y = g_touch.y;
      g_pressTap.due = nowMs + PRESS_FLASH_MS;
      g_pressTap.sc = g_sc;
    } else {
      tap(g_touch.x, g_touch.y, nowMs);
    }
  }
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
  if (g_sc == SC_SET_LIST || g_sc == SC_SET_EDIT || g_sc == SC_SET_INFO || g_sc == SC_SET_GRP || g_sc == SC_SET_RESET) {
    if (!g_touch.down && (uint32_t)(now - g_lastActMs) >= (uint32_t)g_set->autoReturnMin * 60000u) {
      if (g_sc == SC_SET_EDIT && g_set->*(g_editItem->field) != g_editOrig) platSaveSettings(*g_set);
      g_homeCur = g_homeSaved;
      go(g_homeSaved);
    }
  }
  touchStep(now);
  holdStep(now);
  pressStep(now);  // 2.7.0: 押した帯の反転(と、反転を見せたあとの下の帯の操作)
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
  g_pageFrame = false;
  blinkPoll(s, now);  // 回転数の点滅(サーキットとタコだけ。ほかの画面では点灯側に戻る)
  peakTrack(s, now);  // 回転数のピークホールド(どの画面でも記録する)
  tachRedTrack(s);    // タコの目盛と針を赤にするか(どの画面でも。2.4.0)
  idleTrack(s, now);  // 3000rpm 以下の経過時間(どの画面でも)
  infoTrack(ls, si, now);  // 受信の状態の判定のための記録(どの画面でも)
  if (g_sc == SC_OPENING) {
    updOpening(s, ls, si, now);
    if (g_sc == SC_OPENING) return;  // 「SIM」の印は出さない
    blinkPoll(s, now);  // 終わって次の画面に移った。同じ回にその画面を描く(オープニングの絵を 1 回ぶん残さない)
  }
  if (guideStep(now)) return;  // 案内・長押しの表示の間は描き直さない(札と外周だけ)
  bool redrawn = g_full;
  if (g_sc == SC_DRIVE) {
    updDrive(s, now);
  } else if (g_sc == SC_BOOST) {
    redrawn = redrawn || g_tachRed != g_faceRed;  // 目盛を赤に・赤から戻すときは全部を描き直す(updBoost)
    updBoost(s, now);
  } else if (g_full || (int32_t)(now - g_nextMs) >= 0) {  // サーキットとタコ以外の画面は 20 回/秒まで
    g_nextMs = now + 50;
    switch (g_sc) {
      case SC_DATA: updData(s, ls, si, now); break;
      case SC_CAN: updCan(); break;
      case SC_CAN_ID: updCanId(); break;
      case SC_CAN_DETAIL: updCanDetail(now); break;
      case SC_CAN_REDO: updCanRedo(); break;
      case SC_CAN_PP: updCanPp(now); break;
      case SC_SET_LIST: updSetList(ls, ps, si, now); break;
      case SC_SET_EDIT: updSetEdit(now); break;
      case SC_SET_RESET: updSetReset(); break;
      case SC_SET_INFO: updSetInfo(ls, ps, si, now); break;
      case SC_SET_GRP: updSetGrp(ls, ps, si, now); break;
      default: break;
    }
  } else if (g_sc == SC_DATA) {
    updDataRing(s, false);  // 2.5.0: データ画面の回転数の輪は毎回(なめらかに動かす。描くのは先の所だけ)。字は 20 回/秒まで
  }
  if (g_full) return;  // update の中で画面が切り替わった。次の回に描く
  updTags(s, redrawn);
  guideShow(now);
  // サーキット・タコをふだんどおり描いた回(本体の状態の「FPS」に使う)。画面を切り替えた回の全体の描き直しと、
  // 案内・長押しの表示(この回に出し始めた場合も含む)は除く
  g_pageFrame = (g_sc == SC_DRIVE || g_sc == SC_BOOST) && !redrawn && !g_guideShown;
}
bool pageFrame() {
  return g_pageFrame;
}

void testRedraw() {
  g_full = true;
}

void testShow(Screen sc, int a, int b) {
  if (sc == SC_CAN_DETAIL) {
    g_detEntry = a;
    g_detByte = b;
  } else if (sc == SC_SET_EDIT) {
    g_editItem = b == 1 ? &RC_SEND_ITEM : &ITEMS[(a >= 0 && a < N_ITEMS && ITEMS[a].field) ? a : 0];
    g_editFrom = b == 1 ? SC_SET_GRP : SC_SET_LIST;
    if (b == 1) {
      g_grp = G_RC;
      g_grpPage = P_RC_SEND;
      g_grpFrom = SC_SET_INFO;
    } else {
      g_setItem = (int)(g_editItem - ITEMS);
    }
    g_editOrig = g_set->*(g_editItem->field);
    g_saveTried = false;
    g_saveFailed = false;
    g_saveFlash = false;
  } else if (sc == SC_SET_LIST) {
    g_setItem = a;
  } else if (sc == SC_SET_RESET) {
    g_resetKind = a == 1 ? RK_IMU : RK_TIME;
    g_resetFrom = a == 1 ? SC_SET_GRP : SC_SET_LIST;
    if (a == 1) {
      g_grp = G_IMU;
      g_grpPage = P_IMU_CAL;
      g_grpFrom = SC_SET_INFO;
    }
  } else if (sc == SC_SET_INFO) {
    g_infoPage = a;
  } else if (sc == SC_SET_GRP) {
    g_grp = (b >= 0 && b < G_COUNT) ? b : G_CAN;
    g_grpPage = a;
    g_grpFrom = g_grp == G_SYS ? SC_SET_LIST : SC_SET_INFO;
  } else if (sc == SC_CAN_ID) {
    g_canIdx = a;
    g_canPp = b == 1;
    listCount();
  } else if (sc == SC_CAN_REDO) {
    g_canPp = a == 1;
  } else if (sc == SC_OPENING) {  // オープニングの種類(0 サーキット、1 タコ、2 データ)。覚えた画面もそれにする(終わったらそこへ)
    g_homeSaved = a == 1 ? SC_BOOST : (a == 2 ? SC_DATA : SC_DRIVE);
    g_homeCur = g_homeSaved;
  }
  if (isMain(sc)) {
    g_homeCur = sc;
    g_homeRestart = true;
  }
  g_touch.down = false;
  g_pressTap.on = false;
  go(sc);
}

void testTouch(uint32_t nowMs) {
  g_lastActMs = nowMs;
}
int testSetItem() {
  return g_setItem;
}
int testPage() {
  return g_sc == SC_SET_INFO ? g_infoPage : g_sc == SC_SET_GRP ? g_grpPage : g_setItem;
}
int testGroup() {
  return g_grp;
}
bool testHolding() {
  return g_holdShown;
}
bool testGuide() {
  return g_guideShown;
}
int testGuideMode() {
  return (int)g_guideMode;
}
bool testTachRed() {
  return g_tachRed;
}
bool testFlashing() {
  return g_saveFlash;
}
int testInfoSummary(int g, const LinkStats &ls, const PerfStats &ps, const SysInfo &si, uint32_t nowMs, char *out, size_t n) {
  uint16_t col = C_LABEL2;
  if (g < 0) linkSummary(ls, ps, si, nowMs, out, n, &col);
  else groupSummary(g < G_COUNT ? g : G_SYS, ls, ps, si, nowMs, out, n, &col);
  return col == C_ALARM ? 2 : (col == C_WHITE ? 1 : 0);
}
void testPeakReset() {
  g_peakRpm = -1.0f;
  g_peakShowOk = false;
  g_peakShow = false;
  g_peakLow = -1.0f;
}
float testPeakRpm() {
  return g_peakRpm;
}
bool testPeakDrawn() {
  return g_sc == SC_BOOST ? g_peakKey >= 0 : g_triKey >= 0;
}
int testInfoItem(int g, int i, const LinkStats &ls, const PerfStats &ps, const SysInfo &si, uint32_t nowMs, char *out, size_t n) {
  if (g < 0 || g >= G_COUNT) return -1;
  InfoItem it[GRP_MAX];
  const int cnt = groupItems(g, ls, ps, si, nowMs, it);
  if (i < 0 || i >= cnt) return -1;
  snprintf(out, n, "%s", it[i].value);
  return it[i].v;
}

}  // namespace ui

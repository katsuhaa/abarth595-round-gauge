// gfx.cpp - 画面メモリ(466×466、RGB565)への描画。説明は gfx.h
#include "gfx.h"
#include <math.h>
#include <string.h>

namespace gfx {

static uint16_t *g_fb = nullptr;
static float *g_acc = nullptr;
static bool g_tabular = true;
static uint32_t g_missing = 0;

void begin(uint16_t *framebuffer, float *scratch) {
  g_fb = framebuffer;
  g_acc = scratch;
  dirtyClear();
}
uint16_t *frame() {
  return g_fb;
}

// ------------------------------------------------------------ 書き換えた範囲
static const int MAX_DIRTY = 40;
static const int MERGE_SLACK = 1200;  // 2 つをまとめたときに増える画素がこれ以下なら 1 つにする
static Rect g_dirty[MAX_DIRTY];
static int g_nDirty = 0;

static inline int rectArea(const Rect &r) {
  return (int)(r.x1 - r.x0) * (int)(r.y1 - r.y0);
}
static inline Rect rectUnion(const Rect &a, const Rect &b) {
  Rect u;
  u.x0 = a.x0 < b.x0 ? a.x0 : b.x0;
  u.y0 = a.y0 < b.y0 ? a.y0 : b.y0;
  u.x1 = a.x1 > b.x1 ? a.x1 : b.x1;
  u.y1 = a.y1 > b.y1 ? a.y1 : b.y1;
  return u;
}

void addDirty(int x0, int y0, int x1, int y1) {
  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x1 > W) x1 = W;
  if (y1 > H) y1 = H;
  if (x1 <= x0 || y1 <= y0) return;
  Rect r = { (int16_t)x0, (int16_t)y0, (int16_t)x1, (int16_t)y1 };
  for (;;) {
    int hit = -1;
    for (int i = 0; i < g_nDirty; i++) {
      Rect u = rectUnion(g_dirty[i], r);
      if (rectArea(u) - rectArea(g_dirty[i]) - rectArea(r) <= MERGE_SLACK) {
        hit = i;
        r = u;
        break;
      }
    }
    if (hit < 0) break;
    g_dirty[hit] = g_dirty[--g_nDirty];  // まとめた結果を、残りともう一度比べる
  }
  if (g_nDirty == MAX_DIRTY) {
    int best = 0, bestGrow = 0x7FFFFFFF;
    for (int i = 0; i < g_nDirty; i++) {
      int grow = rectArea(rectUnion(g_dirty[i], r)) - rectArea(g_dirty[i]);
      if (grow < bestGrow) {
        bestGrow = grow;
        best = i;
      }
    }
    g_dirty[best] = rectUnion(g_dirty[best], r);
    return;
  }
  g_dirty[g_nDirty++] = r;
}
int dirtyCount() {
  return g_nDirty;
}
Rect dirtyAt(int i) {
  return g_dirty[i];
}
void dirtyClear() {
  g_nDirty = 0;
}
uint32_t dirtyPixels() {
  uint32_t n = 0;
  for (int i = 0; i < g_nDirty; i++) n += (uint32_t)rectArea(g_dirty[i]);
  return n;
}

// ------------------------------------------------------------ 画素
static inline void fill16(uint16_t *p, int n, uint16_t c) {
  while (n-- > 0) *p++ = c;
}
// 1 行の [xa, xb) を濃さ a(0〜255)で塗る
static inline void span(int xa, int xb, int y, uint16_t c, int a) {
  if (y < 0 || y >= H || a <= 0) return;
  if (xa < 0) xa = 0;
  if (xb > W) xb = W;
  if (xb <= xa) return;
  uint16_t *p = g_fb + y * W + xa;
  if (a >= 255) {
    fill16(p, xb - xa, c);
  } else {
    for (int n = xb - xa; n > 0; n--, p++) *p = blend(*p, c, (uint32_t)a);
  }
}
static bool g_onBase = false;  // true の間、縁の画素は g_base と混ぜる(画面にある色を読まない)
static uint16_t g_base = 0;
static inline void plot(int x, int y, uint16_t c, float cov) {
  if (x < 0 || x >= W || y < 0 || y >= H) return;
  int a = (int)(cov * 255.0f + 0.5f);
  if (a <= 0) return;
  uint16_t *p = g_fb + y * W + x;
  *p = (a >= 255) ? c : blend(g_onBase ? g_base : *p, c, (uint32_t)a);
}

void fillAll(uint16_t c) {
  fill16(g_fb, W * H, c);
  addDirty(0, 0, W, H);
}

void fillRect(int x, int y, int w, int h, uint16_t c) {
  for (int iy = y; iy < y + h; iy++) span(x, x + w, iy, c, 255);
  addDirty(x, y, x + w, y + h);
}

void copyRectFrom(const uint16_t *src, int x0, int y0, int x1, int y1) {
  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x1 > W) x1 = W;
  if (y1 > H) y1 = H;
  if (x1 <= x0 || y1 <= y0) return;
  for (int y = y0; y < y1; y++) memcpy(g_fb + y * W + x0, src + y * W + x0, (size_t)(x1 - x0) * sizeof(uint16_t));
  addDirty(x0, y0, x1, y1);
}

void fillRectF(float x0, float y0, float x1, float y1, uint16_t c) {
  if (x1 <= x0 || y1 <= y0) return;
  int ix0 = (int)floorf(x0), ix1 = (int)ceilf(x1);
  int iy0 = (int)floorf(y0), iy1 = (int)ceilf(y1);
  for (int iy = iy0; iy < iy1; iy++) {
    float vy = fminf(y1, (float)(iy + 1)) - fmaxf(y0, (float)iy);
    if (vy <= 0) continue;
    int fullA = ix0, fullB = ix1;
    float vxl = fminf(x1, (float)(ix0 + 1)) - x0;
    if (vxl < 0.999f) {
      plot(ix0, iy, c, vxl * vy);
      fullA = ix0 + 1;
    }
    if (ix1 - 1 >= fullA) {
      float vxr = x1 - (float)(ix1 - 1);
      if (vxr < 0.999f) {
        plot(ix1 - 1, iy, c, vxr * vy);
        fullB = ix1 - 1;
      }
    }
    span(fullA, fullB, iy, c, (int)(vy * 255.0f + 0.5f));
  }
  addDirty(ix0, iy0, ix1, iy1);
}

// 角丸の四角までの距離(中が負)。角ごとに丸みを変えられる
struct RRect {
  float cx, cy, hw, hh, rTL, rTR, rBR, rBL;
};
static inline float sdRRect(const RRect &q, float px, float py) {
  float dx = px - q.cx, dy = py - q.cy;
  float r = (dx < 0) ? (dy < 0 ? q.rTL : q.rBL) : (dy < 0 ? q.rTR : q.rBR);
  float ax = fabsf(dx) - q.hw + r, ay = fabsf(dy) - q.hh + r;
  float mx = ax > 0 ? ax : 0, my = ay > 0 ? ay : 0;
  float inside = (ax > ay ? ax : ay);
  if (inside > 0) inside = 0;
  return sqrtf(mx * mx + my * my) + inside - r;
}
static inline float covFromDist(float d) {
  float c = 0.5f - d;
  return c < 0 ? 0 : (c > 1 ? 1 : c);
}

void fillRoundRect(float x, float y, float w, float h, float r, uint16_t c, uint8_t corners) {
  if (w <= 0 || h <= 0) return;
  float rmax = (w < h ? w : h) * 0.5f;
  if (r > rmax) r = rmax;
  if (r < 0) r = 0;
  RRect q;
  q.cx = x + w * 0.5f;
  q.cy = y + h * 0.5f;
  q.hw = w * 0.5f;
  q.hh = h * 0.5f;
  q.rTL = (corners & CORNER_TL) ? r : 0;
  q.rTR = (corners & CORNER_TR) ? r : 0;
  q.rBR = (corners & CORNER_BR) ? r : 0;
  q.rBL = (corners & CORNER_BL) ? r : 0;
  int iy0 = (int)floorf(y), iy1 = (int)ceilf(y + h);
  for (int iy = iy0; iy < iy1; iy++) {
    if (iy < 0 || iy >= H) continue;
    float yc = iy + 0.5f;
    bool top = yc < q.cy;
    float rl = top ? q.rTL : q.rBL, rr = top ? q.rTR : q.rBR;
    // 左端: [pl, fl) は 1 画素ずつ計算し、fl から右は全部塗る
    int pl, fl, fr, pr;
    float tl = top ? (y + rl) - yc : yc - (y + h - rl);
    if (rl > 0 && tl > 0) {
      float o = (rl + 0.5f) * (rl + 0.5f) - tl * tl;
      float in = (rl > 0.5f) ? (rl - 0.5f) * (rl - 0.5f) - tl * tl : -1;
      pl = (int)floorf(x + (o > 0 ? rl - sqrtf(o) : rl)) - 1;
      fl = (int)ceilf(x + (in > 0 ? rl - sqrtf(in) : rl)) + 1;
    } else {
      pl = (int)floorf(x) - 1;
      fl = (int)ceilf(x) + 1;
    }
    float tr = top ? (y + rr) - yc : yc - (y + h - rr);
    if (rr > 0 && tr > 0) {
      float o = (rr + 0.5f) * (rr + 0.5f) - tr * tr;
      float in = (rr > 0.5f) ? (rr - 0.5f) * (rr - 0.5f) - tr * tr : -1;
      pr = (int)ceilf(x + w - (o > 0 ? rr - sqrtf(o) : rr)) + 1;
      fr = (int)floorf(x + w - (in > 0 ? rr - sqrtf(in) : rr)) - 1;
    } else {
      pr = (int)ceilf(x + w) + 1;
      fr = (int)floorf(x + w) - 1;
    }
    if (fl > fr) fl = fr = (pl + pr) / 2;
    for (int ix = pl; ix < fl; ix++) plot(ix, iy, c, covFromDist(sdRRect(q, ix + 0.5f, yc)));
    float covMid = covFromDist(fabsf(yc - q.cy) - q.hh);  // 上下の辺が画素の途中にあるとき
    span(fl, fr, iy, c, (int)(covMid * 255.0f + 0.5f));
    for (int ix = fr; ix < pr; ix++) plot(ix, iy, c, covFromDist(sdRRect(q, ix + 0.5f, yc)));
  }
  addDirty((int)floorf(x), iy0, (int)ceilf(x + w), iy1);
}

void fillCircle(float cx, float cy, float r, uint16_t c) {
  fillRoundRect(cx - r, cy - r, 2 * r, 2 * r, r, c, CORNER_ALL);
}

void fillCircleOn(float cx, float cy, float r, uint16_t c, uint16_t base) {
  g_onBase = true;
  g_base = base;
  fillRoundRect(cx - r, cy - r, 2 * r, 2 * r, r, c, CORNER_ALL);
  g_onBase = false;
}

void frameRoundRect(float x, float y, float w, float h, float r, float border, uint16_t borderColor, uint16_t insideColor) {
  fillRoundRect(x, y, w, h, r, borderColor, CORNER_ALL);
  float ri = r - border;
  fillRoundRect(x + border, y + border, w - 2 * border, h - 2 * border, ri > 0 ? ri : 0, insideColor, CORNER_ALL);
}

// ------------------------------------------------------------ 環状のおうぎ形
// 画素ごとに「形の縁までの距離」(中が負)を求めて塗る。
// 半径の方向: max(rIn − r, r − rOut)。角度の方向: 両端の線までの距離(開きが 180° 以下なら 2 本の線の内側、
// 180° より大きければどちらかの線の内側)。環の上では、線までの距離がそのまま縁までの距離になる。
static void arcFill(float cx, float cy, float rIn, float rOut, float a0, float a1, uint16_t c, bool hard) {
  float sweep = a1 - a0;
  if (sweep <= 0 || rOut <= 0 || rOut <= rIn) return;
  if (rIn < 0) rIn = 0;
  const float DEG = 3.14159265f / 180.0f;
  bool full = sweep >= 360.0f;
  float d0x = cosf(a0 * DEG), d0y = sinf(a0 * DEG), d1x = cosf(a1 * DEG), d1y = sinf(a1 * DEG);
  // 範囲: 外側の弧の点と、内側の両端の点を包む四角(縁の画素のぶん 2 px 広げる)
  float bx0 = 1e9f, by0 = 1e9f, bx1 = -1e9f, by1 = -1e9f;
  auto take = [&](float x, float y) {
    if (x < bx0) bx0 = x;
    if (y < by0) by0 = y;
    if (x > bx1) bx1 = x;
    if (y > by1) by1 = y;
  };
  if (full) {
    take(cx - rOut, cy - rOut);
    take(cx + rOut, cy + rOut);
  } else {
    int steps = (int)(sweep / 2.0f) + 2;
    for (int i = 0; i <= steps; i++) {
      float a = (a0 + sweep * i / steps) * DEG;
      take(cx + rOut * cosf(a), cy + rOut * sinf(a));
    }
    take(cx + rIn * d0x, cy + rIn * d0y);
    take(cx + rIn * d1x, cy + rIn * d1y);
  }
  int ix0 = (int)floorf(bx0) - 2, iy0 = (int)floorf(by0) - 2, ix1 = (int)ceilf(bx1) + 2, iy1 = (int)ceilf(by1) + 2;
  if (ix0 < 0) ix0 = 0;
  if (iy0 < 0) iy0 = 0;
  if (ix1 > W) ix1 = W;
  if (iy1 > H) iy1 = H;
  const float ro = rOut + 1.0f, ri = rIn - 1.0f;
  const float inSq = (rIn + 1.0f) * (rIn + 1.0f), outSq = (rOut - 1.0f) * (rOut - 1.0f);  // この間は半径の方向の縁から 1 px 以上離れている
  for (int iy = iy0; iy < iy1; iy++) {
    float dy = iy + 0.5f - cy;
    float oo = ro * ro - dy * dy;
    if (oo <= 0) continue;
    float xo = sqrtf(oo);
    float ii = (ri > 0) ? ri * ri - dy * dy : -1;
    float xi = ii > 0 ? sqrtf(ii) : 0;
    // 行の中で環に重なる区間: [cx − xo, cx − xi] と [cx + xi, cx + xo](xi = 0 なら 1 つ)
    for (int side = 0; side < (xi > 0 ? 2 : 1); side++) {
      float xa = (xi > 0) ? (side ? cx + xi : cx - xo) : cx - xo;
      float xb = (xi > 0) ? (side ? cx + xo : cx - xi) : cx + xo;
      int sx0 = (int)floorf(xa) - 1, sx1 = (int)ceilf(xb) + 1;
      if (sx0 < ix0) sx0 = ix0;
      if (sx1 > ix1) sx1 = ix1;
      for (int ix = sx0; ix < sx1; ix++) {
        float px = ix + 0.5f - cx, py = dy;
        float rr = px * px + py * py;
        float sd;
        if (rr > inSq && rr < outSq) {
          sd = -1.0f;  // 環の中ほど: 平方根を省く
        } else {
          float r = sqrtf(rr);
          sd = rIn - r;
          if (r - rOut > sd) sd = r - rOut;
        }
        if (!full) {
          float s0 = d0y * px - d0x * py;  // a0 の線: 中が負
          float s1 = d1x * py - d1y * px;  // a1 の線: 中が負
          float sa = (sweep <= 180.0f) ? (s0 > s1 ? s0 : s1) : (s0 < s1 ? s0 : s1);
          if (sa > sd) sd = sa;
        }
        if (hard) {
          if (sd < 0) plot(ix, iy, c, 1.0f);
        } else {
          plot(ix, iy, c, covFromDist(sd));
        }
      }
    }
  }
  addDirty(ix0, iy0, ix1, iy1);
}

void fillArc(float cx, float cy, float rIn, float rOut, float a0, float a1, uint16_t c) {
  arcFill(cx, cy, rIn, rOut, a0, a1, c, false);
}

void fillArcHard(float cx, float cy, float rIn, float rOut, float a0, float a1, uint16_t c) {
  arcFill(cx, cy, rIn, rOut, a0, a1, c, true);
}

// ------------------------------------------------------------ 多角形
// 各画素に「辺が通ったぶんの面積の変化」を足し込み、行ごとに左から足し合わせると塗る割合になる方式。
static Rect g_pb = { 0, 0, 0, 0 };
static int g_pw = 0, g_ph = 0;  // g_pw は右に 2 列の余白を含む

void polyBegin(Rect b) {
  if (b.x1 - b.x0 > W) b.x1 = (int16_t)(b.x0 + W);
  if (b.y1 - b.y0 > H) b.y1 = (int16_t)(b.y0 + H);
  g_pb = b;
  g_pw = (b.x1 - b.x0) + 2;
  g_ph = (b.y1 - b.y0);
  if (g_pw < 2 || g_ph < 0) {
    g_pw = 2;
    g_ph = 0;
  }
  memset(g_acc, 0, sizeof(float) * (size_t)g_pw * (size_t)g_ph);
}

static void accLine(float ax, float ay, float bx, float by) {
  if (ay == by) return;
  float dir = 1.0f;
  if (ay > by) {
    dir = -1.0f;
    float t = ax;
    ax = bx;
    bx = t;
    t = ay;
    ay = by;
    by = t;
  }
  const float xmax = (float)(g_pw - 2);
  float dxdy = (bx - ax) / (by - ay);
  float x = ax;
  int ys = (int)floorf(ay);
  if (ay < 0) {
    x -= ay * dxdy;
    ys = 0;
  }
  int ye = (int)ceilf(by);
  if (ye > g_ph) ye = g_ph;
  for (int yy = ys; yy < ye; yy++) {
    float *row = g_acc + (size_t)yy * g_pw;
    float dy = fminf((float)(yy + 1), by) - fmaxf((float)yy, ay);
    float xnext = x + dxdy * dy;
    float d = dy * dir;
    float x0 = x < xnext ? x : xnext, x1 = x < xnext ? xnext : x;
    if (x0 < 0) x0 = 0;
    if (x1 < 0) x1 = 0;
    if (x0 > xmax) x0 = xmax;
    if (x1 > xmax) x1 = xmax;
    float x0floor = floorf(x0);
    int x0i = (int)x0floor;
    float x1ceil = ceilf(x1);
    int x1i = (int)x1ceil;
    if (x1i <= x0i + 1) {
      float xmf = 0.5f * (x0 + x1) - x0floor;
      row[x0i] += d - d * xmf;
      row[x0i + 1] += d * xmf;
    } else {
      float s = 1.0f / (x1 - x0);
      float x0f = x0 - x0floor;
      float a0 = 0.5f * s * (1.0f - x0f) * (1.0f - x0f);
      float x1f = x1 - x1ceil + 1.0f;
      float am = 0.5f * s * x1f * x1f;
      row[x0i] += d * a0;
      if (x1i == x0i + 2) {
        row[x0i + 1] += d * (1.0f - a0 - am);
      } else {
        float a1 = s * (1.5f - x0f);
        row[x0i + 1] += d * (a1 - a0);
        for (int xi = x0i + 2; xi < x1i - 1; xi++) row[xi] += d * s;
        float a2 = a1 + (float)(x1i - x0i - 3) * s;
        row[x1i - 1] += d * (1.0f - a2 - am);
      }
      row[x1i] += d * am;
    }
    x = xnext;
  }
}

void polyAdd(const Pt *p, int n) {
  if (n < 3) return;
  float area2 = 0;
  for (int i = 0; i < n; i++) {
    const Pt &a = p[i], &b = p[(i + 1) % n];
    area2 += a.x * b.y - b.x * a.y;
  }
  float ox = (float)g_pb.x0, oy = (float)g_pb.y0;
  for (int i = 0; i < n; i++) {
    const Pt &a = p[i], &b = p[(i + 1) % n];
    if (area2 >= 0) accLine(a.x - ox, a.y - oy, b.x - ox, b.y - oy);  // 向きをそろえて、重なりを足し算にする
    else accLine(b.x - ox, b.y - oy, a.x - ox, a.y - oy);
  }
}

void polyFill(uint16_t c) {
  int bw = g_pw - 2;
  for (int yy = 0; yy < g_ph; yy++) {
    int y = g_pb.y0 + yy;
    const float *row = g_acc + (size_t)yy * g_pw;
    float acc = 0;
    if (y < 0 || y >= H) continue;
    for (int xx = 0; xx < bw; xx++) {
      acc += row[xx];
      float v = fabsf(acc);
      if (v < 0.002f) continue;
      int x = g_pb.x0 + xx;
      if (x < 0 || x >= W) continue;
      uint16_t *px = g_fb + y * W + x;
      if (v >= 0.998f) *px = c;
      else *px = blend(*px, c, (uint32_t)(v * 255.0f + 0.5f));
    }
  }
  addDirty(g_pb.x0, g_pb.y0, g_pb.x1, g_pb.y1);
}

void polyToMask(uint8_t *mask) {
  int bw = g_pw - 2;
  for (int yy = 0; yy < g_ph; yy++) {
    const float *row = g_acc + (size_t)yy * g_pw;
    float acc = 0;
    for (int xx = 0; xx < bw; xx++) {
      acc += row[xx];
      float v = fabsf(acc);
      if (v > 1.0f) v = 1.0f;
      mask[yy * bw + xx] = (uint8_t)(v * 255.0f + 0.5f);
    }
  }
}

void maskReplace(const uint8_t *mask, Rect r, bool mirrorX, uint16_t c, uint16_t base) {
  int bw = r.x1 - r.x0;
  for (int y = r.y0; y < r.y1; y++) {
    if (y < 0 || y >= H) continue;
    const uint8_t *m = mask + (y - r.y0) * bw;
    for (int i = 0; i < bw; i++) {
      uint8_t a = m[i];
      if (!a) continue;
      int x = mirrorX ? (W - 1 - (r.x0 + i)) : (r.x0 + i);
      if (x < 0 || x >= W) continue;
      g_fb[y * W + x] = (a == 255) ? c : blend(base, c, a);
    }
  }
  if (mirrorX) addDirty(W - r.x1, r.y0, W - r.x0, r.y1);
  else addDirty(r.x0, r.y0, r.x1, r.y1);
}

// ------------------------------------------------------------ 文字とアイコン
void setTabularDigits(bool tabular) {
  g_tabular = tabular;
}
uint32_t missingGlyphs() {
  return g_missing;
}

static uint32_t nextCode(const char *&s) {
  uint8_t c = (uint8_t)*s;
  if (c == 0) return 0;
  s++;
  if (c < 0x80) return c;
  int extra = (c >= 0xF0) ? 3 : (c >= 0xE0) ? 2 : (c >= 0xC0) ? 1 : 0;
  uint32_t code = c & (0x3F >> extra);
  if (extra == 0) return '?';
  while (extra-- > 0) {
    uint8_t d = (uint8_t)*s;
    if ((d & 0xC0) != 0x80) return '?';
    s++;
    code = (code << 6) | (d & 0x3F);
  }
  return code;
}

static const GlyphInfo *lookup(const FontData &f, uint32_t code) {
  int lo = 0, hi = (int)f.count - 1;
  while (lo <= hi) {
    int mid = (lo + hi) >> 1;
    uint32_t c = f.glyphs[mid].code;
    if (c == code) return &f.glyphs[mid];
    if (c < code) lo = mid + 1;
    else hi = mid - 1;
  }
  return nullptr;
}
static const GlyphInfo *findGlyph(const FontData &f, uint32_t code) {
  if (!g_tabular && code >= '0' && code <= '9') {
    const GlyphInfo *g = lookup(f, FONT_PROP_DIGIT_BASE + (code - '0'));
    if (g) return g;
  }
  return lookup(f, code);
}

// 4bit の濃さを RLE でたたんだ絵を、色 c で (x0, y0) から描く
static void blitRle(const uint8_t *p, int w, int h, int x0, int y0, uint16_t c) {
  int total = w * h, i = 0, col = 0, row = 0;
  while (i < total) {
    uint8_t b = *p++;
    int v = b & 15, run = (b >> 4) + 1;
    if ((b >> 4) == 15) run = 16 + *p++;
    i += run;
    if (v == 0) {
      col += run;
      while (col >= w) {
        col -= w;
        row++;
      }
      continue;
    }
    uint32_t a = (uint32_t)v * 17;
    while (run-- > 0) {
      int x = x0 + col, y = y0 + row;
      if (x >= 0 && x < W && y >= 0 && y < H) {
        uint16_t *px = g_fb + y * W + x;
        *px = (v == 15) ? c : blend(*px, c, a);
      }
      if (++col >= w) {
        col = 0;
        row++;
      }
    }
  }
}

float textWidth(const FontData &f, const char *utf8) {
  float wsum = 0;
  const char *s = utf8;
  for (uint32_t code; (code = nextCode(s)) != 0;) {
    const GlyphInfo *g = findGlyph(f, code);
    if (g) wsum += g->adv16 / 16.0f;
  }
  return wsum;
}

float drawText(const FontData &f, float x, float baseline, const char *utf8, uint16_t c) {
  const char *s = utf8;
  int by = (int)lroundf(baseline);
  int dx0 = W, dy0 = H, dx1 = 0, dy1 = 0;
  for (uint32_t code; (code = nextCode(s)) != 0;) {
    const GlyphInfo *g = findGlyph(f, code);
    if (!g) {
      g_missing++;
      continue;
    }
    if (g->w && g->h) {
      int gx = (int)lroundf(x) + g->xoff, gy = by - g->yoff;
      blitRle(f.rle + g->offset, g->w, g->h, gx, gy, c);
      if (gx < dx0) dx0 = gx;
      if (gy < dy0) dy0 = gy;
      if (gx + g->w > dx1) dx1 = gx + g->w;
      if (gy + g->h > dy1) dy1 = gy + g->h;
    }
    x += g->adv16 / 16.0f;
  }
  addDirty(dx0, dy0, dx1, dy1);
  return x;
}

void drawIcon(const IconData &ic, int x, int y, uint16_t c) {
  blitRle(ic.rle, ic.w, ic.h, x, y, c);
  addDirty(x, y, x + ic.w, y + ic.h);
}

}  // namespace gfx

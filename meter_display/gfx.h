// gfx.h - 画面メモリ(466×466、RGB565)への描画
//
// ここは ESP32 に依存しない。同じコードを PC でも動かして、キャンバスの絵と照合している。
// 描いた範囲は「書き換えた範囲」として覚え、meter_display.ino がその範囲だけを画面へ送る。
#pragma once
#include <stdint.h>
#include "src/fonts_data.h"

namespace gfx {

const int W = 466;
const int H = 466;

struct Rect {
  int16_t x0, y0, x1, y1;  // x1, y1 は含まない
};
struct Pt {
  float x, y;
};

// framebuffer: W*H 個の uint16_t。scratch: W*(H+2) 個の float(多角形の計算用)
void begin(uint16_t *framebuffer, float *scratch);
uint16_t *frame();
// 描く先の画面メモリー(W×H、同じ並び)を替える(2.10.0。サーキットの、時計の針を除いた絵に描くのに使う)。nullptr で begin の
// 画面メモリーへ戻す。画面メモリー以外に描いている間は「書き換えた範囲」を覚えない。frame() は今の描く先を返す
void setTarget(uint16_t *fb);

// 0xRRGGBB → RGB565(四捨五入)
static constexpr uint16_t rgb(uint32_t c) {
  return (uint16_t)((((((c >> 16) & 0xFF) * 31 + 127) / 255) << 11) | (((((c >> 8) & 0xFF) * 63 + 127) / 255) << 5) | (((c & 0xFF) * 31 + 127) / 255));
}
// bg と fg を a(0〜255)で混ぜる
static inline uint16_t blend(uint16_t bg, uint16_t fg, uint32_t a) {
  uint32_t k = (a + 2) >> 2;  // 0〜64
  uint32_t ik = 64 - k;
  uint32_t rb = ((((uint32_t)bg & 0xF81F) * ik + ((uint32_t)fg & 0xF81F) * k) >> 6) & 0xF81F;
  uint32_t g = ((((uint32_t)bg & 0x07E0) * ik + ((uint32_t)fg & 0x07E0) * k) >> 6) & 0x07E0;
  return (uint16_t)(rb | g);
}

// ---- 塗り ----
void fillAll(uint16_t c);
// 別の画面メモリー(W×H、同じ並び)の [x0, x1) × [y0, y1) を写す(ブースト計の針を消すのに使う)
void copyRectFrom(const uint16_t *src, int x0, int y0, int x1, int y1);
// [x0, x1) × [y0, y1) のうち、画素の中心が中心 (cx, cy) から半径 r 以内の画素だけを写す(2.10.0。サーキットの時計の針を消すのに使う。
// 輪のマスには掛からない)
void copyDiscFrom(const uint16_t *src, int x0, int y0, int x1, int y1, float cx, float cy, float r);
void fillRect(int x, int y, int w, int h, uint16_t c);
void fillRectF(float x0, float y0, float x1, float y1, uint16_t c);  // 端が画素の途中でもなめらかに塗る

enum : uint8_t { CORNER_TL = 1, CORNER_TR = 2, CORNER_BR = 4, CORNER_BL = 8, CORNER_ALL = 15 };
void fillRoundRect(float x, float y, float w, float h, float r, uint16_t c, uint8_t corners = CORNER_ALL);
void fillCircle(float cx, float cy, float r, uint16_t c);
// 円を塗り直す。縁の画素は「base と c を混ぜた色」にする(前の色が縁に残らない)。円のまわりが base 一色のときに使う
void fillCircleOn(float cx, float cy, float r, uint16_t c, uint16_t base);
// 枠線つきの角丸。枠線は内側に取る(CSS の border-box と同じ)
void frameRoundRect(float x, float y, float w, float h, float r, float border, uint16_t borderColor, uint16_t insideColor);
// 環状のおうぎ形(中心 (cx, cy)、半径 rIn〜rOut、角度 a0〜a1 [度]。3 時の方向が 0°、時計回り。a1 − a0 ≥ 360 で輪全体)。
// 縁をなめらかに塗る(縁の画素は今ある色と混ぜる)
void fillArc(float cx, float cy, float rIn, float rOut, float a0, float a1, uint16_t c);
// 同じ形を、縁をなめらかにせずに塗る(画素の中心が中にある画素だけ)。fillArc で描いたものを消すときに、
// 半径と角度を少し広げて呼ぶ(なめらかに塗った縁の画素まで消える)
void fillArcHard(float cx, float cy, float rIn, float rOut, float a0, float a1, uint16_t c);
// 輪の一部を塗り直す(2.5.0。データ画面の回転数の輪)。中心 (cx, cy)、半径 rIn〜rOut の輪のうち、角度 a0〜a1 [度] の範囲にある画素を、
// 「litA0〜litA1 は lit、残りは track」の色で塗る(角度は fillArc と同じ。litA1 − litA0 ≥ 360 で全部 lit、≤ 0 で全部 track)。
// 縁はなめらかにし、半径の方向の縁は base と混ぜる。今ある色とは混ぜないので、同じ所を何度塗っても、少しずつ塗っても、
// 全部を一度に塗ったのと同じ絵になる(輪の縁の外は base の一色であること)
void ringPaint(float cx, float cy, float rIn, float rOut, float a0, float a1, float litA0, float litA1, uint16_t lit, uint16_t track, uint16_t base);

// ---- 多角形(縁をなめらかに塗る) ----
// polyBegin → polyAdd(何個でも。重なった所は 1 回ぶんとして塗る)→ polyFill か polyToMask
void polyBegin(Rect bounds);
void polyAdd(const Pt *p, int n);
void polyFill(uint16_t c);
void polyToMask(uint8_t *mask);  // bounds の幅×高さ。0〜255 の濃さ

// 濃さの表で塗る(回転数の輪)。画素 = base と c を濃さで混ぜた色。濃さ 0 の画素は触らない
void maskReplace(const uint8_t *mask, Rect r, bool mirrorX, uint16_t c, uint16_t base);

// ---- 文字とアイコン ----
// tabular = false にすると、数字を幅のそろっていない形(キャンバスの絵と同じ)で描く
void setTabularDigits(bool tabular);
float textWidth(const FontData &f, const char *utf8);
float drawText(const FontData &f, float x, float baseline, const char *utf8, uint16_t c);  // 戻り値は次の書き出し位置
void drawIcon(const IconData &ic, int x, int y, uint16_t c);
uint32_t missingGlyphs();  // 書体データに無かった文字の数(0 が正常)

// ---- 書き換えた範囲 ----
void addDirty(int x0, int y0, int x1, int y1);
int dirtyCount();
Rect dirtyAt(int i);
void dirtyClear();
uint32_t dirtyPixels();  // いま覚えている範囲の画素数の合計

}  // namespace gfx

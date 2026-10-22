// fonts_data.h - 書体とアイコンのデータ(gen_assets.py が作る。手で書き換えない)
// 書体: Saira / Saira Semi Condensed (The Saira Project Authors), IBM Plex Sans JP / IBM Plex Mono (IBM Corp.)。
// どれも SIL Open Font License 1.1。ここにあるのは、決まった大きさで描いた文字の濃さのデータ。
#pragma once
#include <stdint.h>

struct GlyphInfo {
  uint16_t code;    // 文字コード(Unicode、U+FFFF まで)
  uint8_t w, h;     // 絵の幅と高さ [px]
  int16_t xoff;     // 書き出し位置から絵の左端まで [px]
  int16_t yoff;     // ベースラインから絵の上端まで(上が正)[px]
  uint16_t adv16;   // 次の文字までの送り [1/16 px]
  uint32_t offset;  // rle の中の位置
};
struct FontData {
  const GlyphInfo *glyphs;  // code の昇順
  uint16_t count;
  uint16_t px;
  const uint8_t *rle;
  const char *name;
};
struct IconData {
  uint8_t w, h;
  const uint8_t *rle;
};
#define FONT_PROP_DIGIT_BASE 0xE030  // 幅をそろえていない数字(キャンバスの絵と同じ)の文字コード

extern const FontData FONT_GEAR_160;
extern const FontData FONT_DIAL600_60;
extern const FontData FONT_DIAL500_76;
extern const FontData FONT_DIAL500_40;
extern const FontData FONT_DIAL500_28;
extern const FontData FONT_NUM600_100;
extern const FontData FONT_NUM600_96;
extern const FontData FONT_NUM600_64;
extern const FontData FONT_NUM600_56;
extern const FontData FONT_NUM600_48;
extern const FontData FONT_NUM600_44;
extern const FontData FONT_NUM600_34;
extern const FontData FONT_NUM600_30;
extern const FontData FONT_NUM600_28;
extern const FontData FONT_NUM600_24;
extern const FontData FONT_NUM600_20;
extern const FontData FONT_JP600_100;
extern const FontData FONT_JP600_64;
extern const FontData FONT_JP600_56;
extern const FontData FONT_JP600_48;
extern const FontData FONT_JP600_44;
extern const FontData FONT_JP600_34;
extern const FontData FONT_JP600_30;
extern const FontData FONT_JP500_24;
extern const FontData FONT_MONO600_34;
extern const FontData FONT_MONO600_38;
extern const FontData FONT_MONO700_38;
extern const FontData FONT_MONO500_28;

extern const IconData ICON_WATER;
extern const IconData ICON_FUEL;
extern const IconData ICON_WATER_L;
extern const IconData ICON_FUEL_L;
extern const IconData ICON_CHEV_RIGHT;
extern const IconData ICON_CHEV_LEFT;
extern const IconData ICON_CHEV_DOWN;
extern const IconData ICON_CHEV_UP;
extern const IconData ICON_CHEV_DOWN_S;

// pc_util.h - PC 上の試験で使う小物: 画面メモリー(RGB565)を BMP ファイルに書き出す
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string>

// w×h の RGB565 を 24 ビットの BMP にする(Mac のプレビューでそのまま開ける)
static inline bool saveBmp(const std::string &path, const uint16_t *fb, int w, int h) {
  FILE *f = fopen(path.c_str(), "wb");
  if (!f) return false;
  const int pad = (4 - (w * 3) % 4) % 4;
  const uint32_t dataSize = (uint32_t)(w * 3 + pad) * (uint32_t)h, fileSize = 54 + dataSize;
  uint8_t hd[54] = { 0 };
  hd[0] = 'B';
  hd[1] = 'M';
  for (int i = 0; i < 4; i++) {
    hd[2 + i] = (uint8_t)(fileSize >> (8 * i));
    hd[18 + i] = (uint8_t)((uint32_t)w >> (8 * i));
    hd[22 + i] = (uint8_t)((uint32_t)h >> (8 * i));
    hd[34 + i] = (uint8_t)(dataSize >> (8 * i));
  }
  hd[10] = 54;
  hd[14] = 40;
  hd[26] = 1;
  hd[28] = 24;
  fwrite(hd, 1, sizeof(hd), f);
  const uint8_t zero[3] = { 0, 0, 0 };
  for (int y = h - 1; y >= 0; y--) {  // BMP は下の行から
    for (int x = 0; x < w; x++) {
      uint16_t c = fb[y * w + x];
      uint8_t bgr[3] = { (uint8_t)(((c & 31) * 255 + 15) / 31), (uint8_t)((((c >> 5) & 63) * 255 + 31) / 63), (uint8_t)((((c >> 11) & 31) * 255 + 15) / 31) };
      fwrite(bgr, 1, 3, f);
    }
    fwrite(zero, 1, (size_t)pad, f);
  }
  fclose(f);
  return true;
}

// sim.h - 表示器のデバッグ用の模擬データ(車の値ではない)
//
// このファイルと sim.cpp は c6_display_debug と meter_display(デモ表示)の両方に、同じ内容で置く。
//
// 場面を順に流し、表示に使う値(MlSignals)と、CAN の生データ(CAN確認画面用)を作る。
// ESP32 に依存しない。同じコードを PC でも動かして、表示器のコードと突き合わせている。
#pragma once
#include <stdint.h>
#include "meter_link.h"

namespace sim {

void begin();
// 場面を進める。hold = true の間は、今の場面に留まる(場面の中の動きは続く)
void step(uint32_t nowMs, bool hold);

int scene();               // 1 から始まる場面番号
int sceneCount();
const char *sceneName();   // 場面の内容
const char *sceneExpect(); // 表示器でどう見えるはずか
bool linkOn();             // false の場面では何も送らない(受信が止まったときの表示を見る)
void signals(MlSignals *out, uint32_t nowMs);

// 送るべき CAN フレームがあれば 1 つ取り出す(無ければ false)。data は 8 バイト
// 送る決まり: 中身が変わった ID は最短 50 ms 間隔、変わらない ID は 1 秒ごと
bool canPoll(uint32_t nowMs, MlCanHead *head, uint8_t *data);
void canRates(uint16_t fps[2], uint16_t ids[2]);  // 模擬している受信数と ID の数

}  // namespace sim

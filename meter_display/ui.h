// ui.h - 画面(サーキット・タコ・データ・設定・CAN確認)。配置と色は ui_spec.md(キャンバス版 23)と、キャンバス「遠目の案」の 10/23 の決定による
#pragma once
#include "app.h"

namespace ui {

enum Screen : uint8_t {
  SC_DRIVE,       // サーキット(走行画面)
  SC_BOOST,       // タコ(アナログ計。仕様書に無い)
  SC_DATA,        // データ
  SC_CAN,         // CAN確認(件数・記録の回数。設定の 10 項目目の「開く」)
  SC_CAN_DETAIL,  // CAN確認:詳細(バイトを選んで値とグラフ)
  SC_SET_LIST,    // 設定(1 画面 1 項目、10 項目)
  SC_SET_EDIT,    // 設定:値の変更
  SC_SET_INFO,    // 設定:受信の状態の中
  SC_SET_RC,      // 設定:RaceChrono の状態の中(BLE・GPS・IMU。仕様書に無い)
  SC_OPENING,     // オープニング(電源を入れた直後。config.h の OPENING_*。仕様書に無い)
  SC_CAN_ID,      // CAN確認:一覧(1 件ずつ)
  SC_CAN_REDO,    // CAN確認:やり直すの確認
  SC_SET_RESET,   // 設定:時間の記録のリセットの確認
  SC_COUNT
};

// home = 前回「覚えた画面」(サーキット・タコ・データ。NVS から)。オープニングのあとと、設定から時間で戻るときに出す
void begin(Settings *settings, uint8_t home = SC_DRIVE);
// タッチ(config.h の HOLD_* と ZONE_*)。基板は触れた瞬間に touchDown、離したら touchUp を呼ぶ。
// サーキット・タコ・データでは、HOLD_MS より前に離すとタップ(設定へ)、押し続けると HOLD_MS ごとに次の画面。
// ほかの画面は離したときにタップとして扱う(位置は触れた所)
void touchDown(int x, int y, uint32_t nowMs);
void touchUp(uint32_t nowMs);
// タップを 1 つ(試験用にも使う。touchUp が短いタッチのときに呼ぶ)
void tap(int x, int y, uint32_t nowMs);
// 覚えた画面(HOME_CONFIRM_MS 続けて見ていた サーキット・タコ・データ)。基板はこれを TIME_SAVE_MS ごとに NVS へ書く
uint8_t homeScreen();
// 変わった所だけを画面メモリに描く。画面へ送るのは呼び出し側(gfx の「書き換えた範囲」を使う)
void update(const Signals &s, const LinkStats &ls, const PerfStats &ps, const SysInfo &si, uint32_t nowMs);
Screen screen();
// 走行画面とブースト計で、画面全体の点滅(config.h の BLINK_WHOLE_SCREEN)の消灯の間なら true。
// BLINK_BY_BRIGHTNESS 0 のとき: true の間、呼び出し側は画面メモリーの代わりに一色を送り、false に戻ったら画面全体を送り直す。
// BLINK_BY_BRIGHTNESS 1 のとき: 切り替わるたびに platBlink が呼ばれるので、呼び出し側はこれを見なくてよい
bool blankNow();
uint8_t blinkLevel();  // いまの明るさの段階(BlinkLevel)

// 点滅(回転数が「シフト点 + RING_BLINK_OFFSET」以上「シフト点 + RING_SOLID_OFFSET」未満、走行画面かブースト計)の時刻の管理。
// 消灯の時間は一定(config.h の BLINK_OFF_MS)、点灯の時間は回転数で決まる(BLINK_ON_SLOW_MS → BLINK_ON_FAST_MS)。
// 次に切り替える時刻は前の切り替えから点灯・消灯の時間を足して決めるので、速さが変わっても位相は飛ばない。
// 切り替えがその時間以上遅れたときは、その時刻から数え直す。
// 「シフト点 + RING_SOLID_OFFSET」以上では点滅せず、明るさを最大(BL_MAX)にする。
// update() の中でも呼ぶが、描画や画面へ送る合間(数 ms ごと)にも呼ぶと、切り替えの遅れが小さくなる。
// 明るさの段階が変わると platBlink(段階) を呼ぶ
void blinkPoll(const Signals &s, uint32_t nowMs);
bool blinkDue(uint32_t nowMs);  // 点滅中で、次の切り替えの時刻を過ぎている(blinkPoll を呼ぶ値打ちがある)
uint32_t blinkOnMs();           // いまの点灯の時間 [ms]。点滅していなければ 0
uint32_t blinkOffMs();          // いまの消灯の時間 [ms](BLINK_OFF_MS)。点滅していなければ 0
struct BlinkStats {
  uint32_t onMs;       // いまの点灯の時間(点滅していなければ 0)
  uint32_t offMs;      // いまの消灯の時間(点滅していなければ 0)
  uint32_t toggles;    // 切り替えた回数
  uint32_t lateMaxMs;  // 切り替えの予定の時刻からの遅れの最大
  uint32_t lateOver;   // 遅れが消灯の時間の 2 割を超えた回数(0 が正常)
};
void blinkStats(BlinkStats *out, bool reset);  // reset = true なら回数と最大を 0 に戻す

// 試験用(PC 上で各画面を描いてキャンバスの絵と比べるのに使う)
void testShow(Screen sc, int a = 0, int b = 0);
void testTouch(uint32_t nowMs);  // 試験用: 「操作した」ことにして、自動で戻るのを先延ばしにする
void testRedraw();  // 次の update で、今の画面を最初から描き直す(少しずつ描いた結果と比べるため)
int testSetItem();  // 試験用: 設定で見ている項目
bool testHolding(); // 試験用: 長押しの表示を出している

}  // namespace ui

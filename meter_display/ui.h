// ui.h - 画面(サーキット・タコ・データ・設定・CAN確認)。配置と色は ui_spec.md(キャンバス版 23)と、キャンバス「遠目の案」の 10/23 の決定による
#pragma once
#include "app.h"

namespace ui {

enum Screen : uint8_t {
  SC_DRIVE,       // サーキット(走行画面)
  SC_BOOST,       // タコ(アナログ計。仕様書に無い)
  SC_DATA,        // データ
  SC_CAN,         // CAN確認(件数・記録の回数。2.4.0: 受信の状態 → CAN の「C-CAN・B-CAN」の下「CAN確認」)
  SC_CAN_DETAIL,  // CAN確認:詳細(バイトを選んで値とグラフ)
  SC_SET_LIST,    // 設定(1 画面 1 項目、8 項目。2.4.0)
  SC_SET_EDIT,    // 設定:値の変更
  SC_SET_INFO,    // 設定:受信の状態の中(2.4.0: 1 ページ 1 グループ。CAN・GPS・IMU・RaceChrono のまとめ)
  SC_SET_GRP,     // 設定:グループの中(2.4.0: 受信の状態の CAN・GPS・IMU・RaceChrono と、設定の本体の状態。1 ページ 1 項目)
  SC_OPENING,     // オープニング(電源を入れた直後。2.5.0: 覚えた画面ごとに 3 種類。config.h の OPENING_ON・OPC_*・OPT_*・OPD_*。仕様書に無い)
  SC_CAN_ID,      // CAN確認:一覧(1 件ずつ)
  SC_CAN_REDO,    // CAN確認:やり直すの確認
  SC_SET_RESET,   // 設定:リセットの確認(時間の記録。2.4.3: IMU の向きの補正も)
  SC_CAN_PP,      // CAN確認:前後で比べる(2.6.0。給油などの前後で、ゆっくり変わる値のバイトを見つける)
  SC_COUNT
};

// home = 前回「覚えた画面」(サーキット・タコ・データ。NVS から)。オープニングのあとと、設定から時間で戻るときに出す
void begin(Settings *settings, uint8_t home = SC_DRIVE);
// タッチ(config.h の TOUCH_CONFIRM_MS・HOLD_*・SWIPE_* と ZONE_*)。基板は触れた瞬間に touchDown、押している間の知らせごとに
// touchMove、離したら touchUp を呼ぶ。時間は update で進む(触れてから TOUCH_CONFIRM_MS でタッチが決まる。それより短いタッチは何もしない)。
// 2.4.0: どの画面も離したときに決める。左へスワイプ = 次、右へスワイプ = 前(サーキット・タコ・データは画面、設定の中は項目・ページ)。
// 動きが HOLD_CANCEL_PX 未満ならタップ(位置は触れた所。設定の中の左半分・右半分は前・次)。サーキット・タコ・データのタップは何もしない。
// 長押し HOLD_MS = 設定。設定の中で「長押しで開く」がある画面は、画面のどこかの長押し HOLD_MS で開く(2.4.4。2.4.5: 画面全体。
// タップでは開かない)。サーキット・タコ・データでは、タッチが決まったら案内を出す(2.3.0)。設定の中は HOLD_SHOW_MS から下の帯
// 「長押しで開く」を水色にして外周(2.5.0。タップ・スワイプでは帯を変えない)。外周が出てから HOLD_MS の前に離したら何もしない(2.7.0)。
// 2.7.0: 設定・CAN確認の上の帯と、タップで動く下の帯は、押している間は反転して描く。下の帯の操作は、離して PRESS_FLASH_MS あと
// (update の中)に行う
void touchDown(int x, int y, uint32_t nowMs);
void touchMove(int x, int y, uint32_t nowMs);
void touchUp(uint32_t nowMs);
// タップを 1 つ、すぐに(試験用。離したときにも呼ぶ。サーキット・タコ・データと、設定の中の下の帯「長押しで開く」では何もしない。反転は見せない)
void tap(int x, int y, uint32_t nowMs);
// 横のスワイプを 1 つ、すぐに(試験用。離したときにも呼ぶ)。dir = 1: 次(左へのスワイプ)、-1: 前(右へのスワイプ)
void swipe(int dir, uint32_t nowMs);
// 覚えた画面(HOME_CONFIRM_MS 続けて見ていた サーキット・タコ・データ)。基板はこれを TIME_SAVE_MS ごとに NVS へ書く
uint8_t homeScreen();
// 変わった所だけを画面メモリに描く。画面へ送るのは呼び出し側(gfx の「書き換えた範囲」を使う)
void update(const Signals &s, const LinkStats &ls, const PerfStats &ps, const SysInfo &si, uint32_t nowMs);
// 直前の update が、サーキット・タコをふだんどおり描いた回なら true。画面を切り替えた回・タコの目盛を赤にする(戻す)回の全体の描き直しと、
// 案内・長押しの表示の回は false。
// 基板は、この回に描いて送るのにかかった時間を perf::pageAdd に渡す(本体の状態の「FPS」)
bool pageFrame();
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
// a: 設定・変更 = 設定の項目、受信の状態・グループの中 = ページ(グループの中は b = グループ: 0 CAN、1 GPS、2 IMU、3 RaceChrono、
// 4 本体の状態)、CAN確認の一覧 = 何件目、CAN確認の詳細 = 表の番号(b = バイト)。変更で b = 1 なら、RaceChrono の「送信」(a は使わない)。
// リセットの確認は a = 1 なら IMU の向きの補正(2.4.3。0 = 時間の記録)。オープニングは a = 種類(0 サーキット、1 タコ、2 データ。
// 覚えた画面もそれにする。2.5.0)。CAN確認の一覧は b = 1、やり直すは a = 1 なら「前後で比べる」の候補(2.6.0)
void testShow(Screen sc, int a = 0, int b = 0);
void testTouch(uint32_t nowMs);  // 試験用: 「操作した」ことにして、自動で戻るのを先延ばしにする
void testRedraw();  // 次の update で、今の画面を最初から描き直す(少しずつ描いた結果と比べるため)
int testSetItem();  // 試験用: 設定で見ている項目
int testPage();     // 試験用: 受信の状態・グループの中で見ているページ(ほかの画面では設定の項目)
int testGroup();    // 試験用: グループの中の画面のグループ(0 CAN、1 GPS、2 IMU、3 RaceChrono、4 本体の状態)
bool testHolding(); // 試験用: 長押しの表示を出している
bool testGuide();   // 試験用: 触れたときの案内を出している(2.3.0)
int testGuideMode(); // 試験用: 案内の状態(0 = ふつう、1 = 長押しの途中(スワイプを灰)、2 = 長押しを取り消した(長押しを灰))
bool testTachRed();  // 試験用: タコの目盛と針を赤にしている(回転数がシフト点以上。2.4.0)
bool testFlashing();  // 試験用: 「決定」を押したあとの点滅の途中(2.4.3: 変更の画面のまま。2.4.5: 変えていなくても点滅。update を呼んだあとの状態)
// 試験用: グループ g(0 CAN、1 GPS、2 IMU、3 RaceChrono、4 本体の状態)のページ i の値の文字を out に入れ、
// 判定を返す(0 = 待ち、1 = 正常、2 = 異常。無いページは -1)
int testInfoItem(int g, int i, const LinkStats &ls, const PerfStats &ps, const SysInfo &si, uint32_t nowMs, char *out, size_t n);
// 試験用: まとめ(g = -1 なら設定の「受信の状態」、0〜4 ならそのグループ)の文字を out に入れ、0 = 待ち、1 = 正常、2 = 異常を返す
int testInfoSummary(int g, const LinkStats &ls, const PerfStats &ps, const SysInfo &si, uint32_t nowMs, char *out, size_t n);
bool testPeakDrawn();  // 試験用: ピークホールドの印(タコは回転計の印、サーキットは三角形)を描いてある
void testPeakReset();  // 試験用: ピークホールドの記録を消す(前の絵・試験のピークを残さない)
float testPeakRpm();   // 試験用: ピークホールドの印の回転数(止まっている・下がっている値。-1 = なし。2.4.4)

}  // namespace ui

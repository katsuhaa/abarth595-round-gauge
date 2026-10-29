// app.h - 表示器の中で受け渡すデータと、各部の入口
//
// mlink  : UART で届いたバイト列をフレームに分け、最新の値と受信の状態を持つ
// veh    : 車の CAN の生データから表示に使う値(回転数・車速・水温・ブースト・燃料・ギアなど)を作る(2026-10-23 に c6_can_link から移した)。
//          2.9.0: ライトのスイッチ(lightInfo)
// bright : 画面の明るさ(ライトのスイッチで「明るさ」と「イルミネーションの明るさ」を選ぶ。2.9.0)
// cantab : CAN の生データの表(CAN確認画面用)。変化を記録するたびに候補を絞り込む、値の履歴
// trip   : 全開の時間と走行時間(電源 ON からの累計)、3000rpm 超・以下の累計時間(NVS に保存)
// ui     : 画面(ui.h)
// plat   : 基板に依存する部分。meter_display.ino にある(PC 上の試験では別の実装を使う)
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "config.h"
#include "meter_link.h"

// 表示に使う値。*Ok が false の値は「信号がない」
struct Signals {
  bool linkUp;  // LINK_TIMEOUT_MS 以内に値のフレームを受けた
  bool rpmOk, speedOk, throttleOk, waterOk, fuelOk, gearOk, boostOk;
  int rpm;            // [rpm]
  float speedKmh;     // [km/h]
  float throttlePct;  // [%]
  float waterC;       // [℃]
  float fuelPct;      // [%]
  float boostBar;     // ブースト圧(大気圧との差)[bar]。負は負圧
  char gear;          // '1'〜'5'、'N'、'R'
  bool sim;           // 模擬データ
  uint8_t scene;      // 模擬データの場面番号
  bool sportOk, sport;  // スポーツモード(2.11.0。車の CAN の生データから作るときだけ。C-CAN 0A18A000 b4 の 0x10)
};

struct Settings {
  int shiftRpm;       // シフト点 [rpm]
  int autoReturnMin;  // 自動で戻る [分]
  int brightnessPct;  // 明るさ [%]
  int warnTempC;      // 水温の警告温度 [℃]
  int demoOn;         // デモ表示(1 = 入)
  int rcOn;           // RaceChrono への送信(1 = 入)。次の起動から効く
  int illumPct;       // イルミネーションの明るさ [%](ライトのスイッチが入の間のふだんの明るさ。2.9.0。初期化の並びを変えないよう最後に置く)
  // 2.10.0: 過負荷時間(load。2.12.0 で高負荷時間から改名)。分。黄・赤は 0 で「切」(黄・赤とも 0 で機能を切)。初期化の並びを変えないよう最後に置く。
  // 2.12.0: 「クールダウン時間」はなくした
  int loadYellowMin;  // 過負荷時間 黄
  int loadRedMin;     // 過負荷時間 赤
  int loadResetMin;   // 過負荷がリセットされる時間(回転数が過負荷 回転数未満でこれだけ続いたら 0)
  int ringOne;        // 回転数の輪(2.12.0。0 = 両側、1 = 片側)。初期化の並びを変えないよう最後に置く
  // 2.12.0: 過負荷(本人「過負荷は短期的な上昇によりクールダウンが必要になるという考え方と、それの累積によりメンテナンスの期間を推測する」)。
  // 初期化の並びを変えないよう最後に置く
  int loadRpm;        // 過負荷 回転数(過負荷時間・累積時間・サーキットの ▼ の境目)
  int maintHours;     // メンテナンスまでの累積時間 [時間](累積時間がこれに達したらメンテナンスの時期。0 = 切)
};
// 2.10.0: 設定を全部初期値(config.h の SET_*_DEFAULT)にする(設定の「デフォルトに戻す」と、起動時の初期値)
void settingsDefaults(Settings *s);

struct LinkStats {
  uint32_t baud;         // いま受信に使っている速度
  bool locked;           // この速度で正しいフレームを受けている(LINK_TIMEOUT_MS 以内に受けた)
  uint32_t framesOk;     // 正しく受けたフレームの累計
  uint32_t crcErrors;    // CRC が合わなかった回数
  uint32_t lostFrames;   // 連番の飛びから数えた、届かなかったフレーム数
  uint32_t junkBytes;    // フレームにならずに捨てたバイト数(XIAO の起動ログなど)
  uint32_t framesPerSec; // 直近 1 秒に受けたフレーム数
  uint32_t bytesPerSec;  // 直近 1 秒に届いたバイト数(フレームにならなかったものも含む)
  bool infoValid;        // 送る側の状態(MlInfo)を受けている
  MlInfo info;
};

struct PerfStats {
  uint32_t fps;         // 直近 1 秒に画面へ送った回数
  uint32_t renderMaxMs; // 直近 1 秒で、描画にかかった最長の時間
  uint32_t flushMaxMs;  // 直近 1 秒で、画面へ送るのにかかった最長の時間
  uint32_t loopMaxMs;   // 直近 1 秒で、1 周にかかった最長の時間
  // 前にサーキット・タコを出していた間(今出していればその間)に、1 回描いて送るのにかかった最長の時間 [µs]。0 = まだ測っていない。
  // 画面を切り替えた回の全体の描き直しと、案内・長押しの表示は数えない(ui::pageFrame)。シリアルに出す(2.2.0 までは受信の状態の「描画」)
  uint32_t pageMaxUs;
  // 本体の状態の「FPS」(2.3.0。2.4.0 で受信の状態から移した。perf::pageStats が入れる)。2.4.3: サーキット・タコで 1 回描いて送るのに
  // かかった時間を 1 秒ごとにまとめ、サーキット・タコを出していた秒だけを新しい方から PERF_FPS_SECS 秒ぶん見る
  uint32_t pageFrames;   // その間に数えた回数
  uint32_t pageAvgUs;    // 1 回の平均の時間 [µs](全部の回の平均)
  uint32_t pageSlowUs;   // 1 秒ごとの平均の時間のうち、一番長い秒の値 [µs](回/秒の「最低」)
  uint16_t histN;        // fpsHist に入っている秒の数
  uint8_t fpsHist[PERF_FPS_SECS];  // 1 秒ごとの回/秒(その秒の平均の時間から。255 で頭打ち)。古い方から histN 個
};

// 表示器の CPU とメモリ(本体の状態に出す。2.4.3)。基板が起動時に入れ、空きと温度は 1 秒ごとに入れ直す
struct CpuInfo {
  bool ok;               // 入れてある
  char model[20];        // 型式(例「ESP32-S3」)
  uint16_t rev;          // チップの版(100 × 大 + 小。例 2 = v0.2)
  uint8_t cores;         // コアの数
  uint16_t cpuMHz;       // CPU のクロック [MHz]
  uint32_t flashBytes;   // フラッシュの大きさ
  uint32_t flashHz;      // フラッシュの速さ [Hz]
  uint32_t heapBytes;    // 内部 RAM のヒープの大きさ
  uint32_t heapFree;     // その空き
  uint32_t psramBytes;   // PSRAM の大きさ(0 = 無い)
  uint32_t psramFree;    // その空き
  bool tempOk;           // 温度を読めた
  float tempC;           // チップの温度 [℃](チップの中のセンサー)
};

// RaceChrono へ送る側(BLE・GPS・IMU)の状態。受信の状態の GPS・IMU・RaceChrono に出す
struct RcStats {
  bool enabled;         // 設定が「入」で、この起動で動かしている
  bool bleOk;           // BLE を始められた
  bool connected;       // RaceChrono(スマホ)がつながっている
  uint32_t canPerSec;   // 直近 1 秒に送った CAN の通知の数
  uint32_t gpsPerSec;   // 直近 1 秒に送った GPS の通知の数
  uint32_t dropped;     // 送れずに捨てた通知の累計
  bool gpsOk;           // GPS(M100)から UBX が届いている
  uint8_t fixType;      // 0 = なし、2 = 2D、3 = 3D
  uint8_t numSV;        // 使っている衛星の数
  uint32_t pvtPerSec;   // 直近 1 秒に届いた位置(NAV-PVT)の数
  int32_t lat, lon;     // 1e-7 度
  float gpsKmh;         // GPS の速度
  bool imuOk;           // IMU(QMI8658)と通信できた
  bool imuLevel;        // 重力の向きを覚えた(停車中)
  bool imuForward;      // 前の向きを覚えた(発進の加速)
  float gLon, gLat;     // 車の座標での加速度 [g](前後、横)
  // 2.10.0: GPS(M100)を動かしている。RaceChrono の送信の入・切に関係なく動かす(時計と高負荷時間に時刻を使う)。
  // 落ちる起動が続いて RC_CRASH_LIMIT で止めたときだけ false(enabled は送信 = BLE・IMU を動かしているか)
  bool gpsOn;
  // 2.13.0: M100 の OTP の高性能設定の状態。0 = 待ち(確かめている・書いている)、1 = 済、2 = 未(確かめただけ。RC_GPS_OTP_HIGH_PERF 0)、
  // 3 = 不明(応答なし)、4 = 異常(未対応の版、書けなかった、書いたのに入っていない)
  uint8_t otp;
};

struct SysInfo {
  bool axpOk;           // 電源 IC(AXP2101)と通信できた
  uint8_t powerOnSrc;   // AXP2101 のレジスタ 0x20(起動のきっかけ)
  bool touchOk;         // タッチ IC と通信できた
  bool nvsOk;           // 設定の保存領域を開けた
  RcStats rc;
  CpuInfo cpu;          // 2.4.3
  // 2.10.0: 今の時刻(サーキットの時計と高負荷時間に使う)。GPS の時刻が一度分かったら、GPS が途切れても表示器の中の時計で進める
  bool timeOk;          // 時刻が分かっている
  uint32_t utc;         // UTC、1970-01-01 からの秒
  // 2.11.0: utc はデモの時刻(デモ表示の間、GPS の時刻が分かるまで。gtime::demoBase)。時計の表示だけに使い、高負荷時間の
  // 「電源が切れていた時間を引く」と保存する時刻には使わない
  bool timeDemo;
};

// ---------------------------------------------------------------- plat(基板に依存する部分)
void platLock();      // mlink と cantab のデータを守る
void platUnlock();
void *platAlloc(size_t bytes);         // 大きなメモリ(PSRAM)
// 設定の明るさの項目(「明るさ」「イルミネーションの明るさ」)を変更の画面で変えた・戻した。pct はその値。基板は画面の明るさを
// 決め直す(変更の画面を出している間は ui::previewPct() の値、それ以外はライトのスイッチで bright::normalPct。2.9.0)
void platSetBrightness(int pct);
// 画面全体の明るさの段階(走行画面・ブースト計の点滅と、シフト点 + RING_SOLID_OFFSET 以上の最大輝度)。ui::blinkPoll が
// 変わるたびに platBlink を呼ぶ。基板ではパネルの明るさを 0 / 設定の値 / 100 % にする(PC 上の試験では記録するだけ)
enum BlinkLevel : uint8_t {
  BL_NORMAL = 0,  // 設定の明るさ(点滅の点灯側、点滅していないとき)
  BL_OFF = 1,     // 0(点滅の消灯側)
  BL_MAX = 2,     // 100 %(シフト点 + RING_SOLID_OFFSET 以上)
};
void platBlink(uint8_t level);  // BlinkLevel の値
// 描画の長い処理の合間に ui.cpp から呼ばれる。基板では ui::blinkPoll(今の値, millis()) を呼び、点滅の切り替えの遅れを
// 小さくする(PC 上の試験では何もしない。時刻は update に渡した値だけで進む)
void platBlinkTick();
bool platSaveSettings(const Settings &s);  // 保存して読み返し、合っていれば true
// CAN の生データを 1 フレーム受けるたびに呼ばれる(受信タスク)。RaceChrono へ送るのに使う(PC 上の試験では何もしない)
void platCanFrame(uint8_t bus, uint8_t ext, uint32_t id, uint8_t dlc, const uint8_t *data, uint32_t nowMs);
// IMU の覚えた向き(取り付け角の補正)を捨てる(受信の状態 → IMU の「向きの補正」の下「向きを取り直す」)。保存した値も消す
void platResetImuCal();
// 3000rpm 超・以下の累計時間を今すぐ NVS に書く(「時間の記録」のリセットのあと)。ふだんは TIME_SAVE_MS ごとに書く
void platSaveTimes();
// CAN確認の「前後で比べる」の前の記録(2.6.0)を保存する(n = 0 なら消す)。書けたら true
bool platSaveCanPre(const void *data, size_t n);
// 保存した前の記録を読む。戻り値は読んだバイト数(無い・入らないときは 0)
size_t platLoadCanPre(void *data, size_t maxN);

// ---------------------------------------------------------------- mlink
namespace mlink {
void begin();
void feed(const uint8_t *data, size_t n, uint32_t nowMs);  // 受信タスクから呼ぶ
void tick(uint32_t nowMs);                                 // 1 秒ごとの集計。受信タスクから呼ぶ
// 受信の速度を自動で合わせる。受信タスクが 1 周ごとに呼ぶ(rxBytes = その周に届いたバイト数)。
// 「バイトは届くのに正しいフレームが 1 つも無い」区間が続いたら、次の候補の速度へ移る(config.h の LINK_BAUD_*)。
// 戻り値: 速度を切り替えるときは新しい速度 [bps](呼んだ側が UART の速度を変える)。そのままなら 0。
uint32_t autoBaud(uint32_t nowMs, size_t rxBytes);
uint32_t baud();  // いま受信に使う速度 [bps]
// 設定の保存(フラッシュへの書き込み)の間は UART の受信が遅れ、数バイト落ちることがある。
// untilMs までに見つかった CRC 誤りと欠落は、線の品質の数字(crcErrors・lostFrames)に入れない。
void graceUntil(uint32_t untilMs);
// 受信元が切り替わった(デモ表示の入・切)。次のフレームでは連番の飛びを欠落として数えない
void resyncSeq();
// 表示に使う値。値のフレーム(MlSignals)が LINK_TIMEOUT_MS 以内に届いていればその値(c6_display_debug・デモ表示・1.x の c6_can_link)、
// 届いていなければ CAN の生データから作った値(veh。2.0.0 の c6_can_link)。linkUp はどちらでも「何かのフレームを LINK_TIMEOUT_MS 以内に受けた」
void getSignals(Signals *out, uint32_t nowMs);
void getStats(LinkStats *out);
bool usingRaw(uint32_t nowMs);  // いま getSignals が生データから作った値を返す(値のフレームが来ていない)
}

// ---------------------------------------------------------------- veh(車の CAN の生データから表示に使う値を作る。config.h の VEH_*)
namespace veh {
void begin();  // 状態を全部消す
// CAN の生データを 1 フレーム受けるたびに呼ぶ(受信タスク。mlink が呼ぶ)
void onFrame(uint8_t bus, uint8_t ext, uint32_t id, uint8_t dlc, const uint8_t *data, uint32_t nowMs);
// 表示に使う値(linkUp・sim・scene 以外)を out に入れる。out は呼ぶ側で 0 にしておく
void getSignals(Signals *out, uint32_t nowMs);
// 燃料(2.7.0: C-CAN 0A18A000 b5 = %。config.h の 燃料)の中身。受信の状態 → CAN の「燃料」とシリアルに出す
struct FuelInfo {
  bool seen;         // 0A18A000 を VEH_FUEL_TIMEOUT_MS 以内に受けている(b5 = 0 も含む)
  bool have;         // 使える値(1〜100)を受けたことがある
  uint8_t pct;       // 最後に使った値 [%](表示に使っている値)
  uint8_t lastB5;    // 最後に受けた b5(使わない 0 も含む)
  uint32_t ignored;  // 捨てた b5(0 と 100 超)のフレームの数
};
void fuelInfo(FuelInfo *out, uint32_t nowMs);
float lastRatio();  // ギアの推定に使った最後の比 [rpm per km/h](シリアル用)
// ライト(2.9.0: B-CAN 06214000 b0 の 0x08 = ライトのスイッチ。config.h の ライト)。明るさの切り替え・受信の状態 → CAN の「ライト」・
// シリアルに使う
struct LightInfo {
  bool have;         // 電源を入れてから受けたことがある(false = 不明)
  bool seen;         // VEH_LIGHT_TIMEOUT_MS 以内に受けている(have で seen でなければ「受信なし」。on は最後の状態)
  bool on;           // ライトのスイッチが入(最初に受けた値はすぐ、そのあとは VEH_LIGHT_SETTLE_MS 続いた値)
  uint8_t lastB0;    // 最後に受けた b0
  uint32_t firstMs;  // 最初に受けた時刻 = 電源を入れてからの ms(have のときだけ)
  uint32_t changes;  // 切り替えた回数(最初の値は数えない)
};
void lightInfo(LightInfo *out, uint32_t nowMs);
}

// ---------------------------------------------------------------- bright(画面の明るさ。2.9.0)
namespace bright {
// ふだんの明るさ [%]: ライトのスイッチが入なら「イルミネーションの明るさ」、切・まだ受けていないなら「明るさ」
int normalPct(const Settings &s, bool lightOn);
// 明るさの段階(BlinkLevel)→ パネルに送る明るさ [%]: 点滅の消灯 0、最大 100(ライトに関係なく。本人 10/26「A」)、ふだん normal
int panelPct(uint8_t level, int normal);
}

// ---------------------------------------------------------------- cantab
namespace cantab {
struct View {
  uint32_t id;
  uint8_t bus;  // 0 = C-CAN、1 = B-CAN
  uint8_t ext;  // 1 = 29 ビット ID
  uint8_t dlc;
  uint8_t data[8];
  uint8_t base[8];
};
void begin();
void onFrame(uint8_t bus, uint8_t ext, uint32_t id, uint8_t dlc, const uint8_t *data, uint32_t nowMs);
void tick(uint32_t nowMs);  // 履歴を取る。受信タスクから呼ぶ
int count();
// 「変化を記録」(config.h の CAN_REC_*)。1 回目 = 基準を取り、全部の ID を候補にする。2 回目から = 前の候補のうち、前の記録から今までに
// 値が違っていて変わった回数が 1〜CAN_REC_MAX_CHANGES 回のバイトを持つ ID だけを残し、基準を取り直す。戻り値は残った ID の数
int record();
void clearRecord();      // 記録を消す(やり直す)
int recordCount();       // 記録した回数
// 件数の移り変わり(古い順、最大 CAN_REC_HIST 個)。戻り値は入れた個数
int recordHistory(int *out);
// 今の候補(ID の並び = 表の順)。entries に表の番号、masks に候補のバイトの印(bit b = バイト b)。戻り値は全部の件数
int candidates(uint16_t *entries, uint8_t *masks, int maxOut);
bool getView(int entry, View *out);
// 古い順に並べた値を out(CAN_HIST_SAMPLES 個)へ入れる。戻り値は入れた個数
int getHistory(int entry, int byteIndex, uint8_t *out);
// ---- 前後で比べる(2.6.0。config.h の CAN_PP_*)。前の記録は platSaveCanPre で保存し、begin で読み戻す
struct PpResult {
  uint16_t entry;  // 表の番号
  uint8_t byte;    // バイトの位置
  uint8_t pre, post;  // 前と後の真ん中の値
};
int ppWaitMs(uint32_t nowMs);      // 記録できるまでの残り [ms](0 = 今できる。-1 = まだ何も受けていない)
bool ppRecordPre(uint32_t nowMs);  // 直前 CAN_PP_WIN_MS を「前」にして保存する(候補は消す)。記録できないときは false
bool ppHasPre();
int ppRecordPost(uint32_t nowMs);  // 直前 CAN_PP_WIN_MS を「後」にして、前と比べた候補を作る。戻り値 = 候補の数(-1 = 記録できない)
bool ppHasResult();
// 候補(増えたもの → 減ったもの、それぞれ差の大きい順)。戻り値は全部の数。up・down に増えた・減ったの数
int ppResults(PpResult *out, int maxOut, int *up = nullptr, int *down = nullptr);
void ppClear();  // 前の記録(保存も)と候補を消す
}

// ---------------------------------------------------------------- trip
namespace trip {
void update(const Signals &s, uint32_t nowMs, int rpm = TIME_RPM);  // rpm = ▲▼ を分ける回転数(2.12.0: 設定の過負荷 回転数)
uint32_t movingMs();  // 走行時間
uint32_t wotMs();     // 全開の時間
uint32_t aboveMs();   // 回転数が rpm 以上の時間の累計(電源を切っても続く。2.12.0: 過負荷時間とそろえて「超える」→「以上」)
uint32_t belowMs();   // rpm 未満の時間の累計
void setTimes(uint32_t above, uint32_t below);  // 起動時に NVS から戻す
void resetTimes();    // 0 に戻す(設定の「累積時間 リセット」。2.11.0 までは「時間の記録」のリセット)
}

// ---------------------------------------------------------------- load(過負荷時間。2.10.0。仕様書 circuit_screen_spec.md 第 3 版 3 章。config.h の LOAD_*)
// 回転数が過負荷 回転数(設定の loadRpm。2.12.0。初期値 LOAD_RPM)以上で 1 秒に 1 秒増え、未満で 1 秒に 1 秒減る(0 より下にならない。
// 上限 LOAD_MAX_MS)。過負荷 回転数未満が「過負荷がリセットされる時間」続いたら 0。回転数の信号がない間は変わらない(リセットの数えも 0 から)。
// 黄・赤とも 0 なら数えず 0。表示器の電源が切れていた間も減る: 1 分ごとに値と時刻を NVS に書き(snapshot)、次の起動で戻し(restore)、
// GPS の時刻が分かったら(onTime)切れていた時間ぶん引く。保存した「過負荷 回転数未満の経過」+ 切れていた時間 が リセットの時間以上なら 0。
// 2.12.0: クールダウン = リセットのあとに一度でも黄(黄が切なら赤)になっていて、過負荷 回転数未満が LOAD_COOL_SHOW_MS 続いている間
// (タイトルを「クールダウン」にして、リセットまでの残りを出す。config.h の LOAD_COOL_SHOW_MS)
namespace load {
void update(const Signals &s, const Settings &set, uint32_t nowMs);  // update のたびに呼ぶ(どの画面でも。同じ時刻で 2 回呼んでも同じ)
bool enabled(const Settings &set);    // 黄・赤のどちらかが 0 でない
uint32_t valueMs();                   // 今の値 [ms]
uint8_t level(const Settings &set);   // 色: 0 = 白、1 = 黄、2 = 赤
// 2.12.1: 黄(黄が切なら赤)になってから 0 に戻るまで true(サーキットの真ん中に過負荷時間を出す条件。黄になる前は出さない。
// 本人 10/29「過負荷回転数を超えたとき警告になっていないときに一瞬過負荷時間が表示されます」)。「黄になった」は電源を切ると忘れる
bool warned(const Settings &set);
bool coolDown(uint32_t nowMs);        // クールダウン(タイトルを「クールダウン」にして、リセットまでの残りを出す)
uint32_t coolLeftMs(const Settings &set, uint32_t nowMs);  // リセットまでの残り [ms]
uint32_t lowMs(uint32_t nowMs);       // 過負荷 回転数未満が続いている時間 [ms](0 = 続いていない)
// 起動時に NVS から戻す。savedUtc = 前の起動で最後に書いた時刻(0 = その起動では時刻が分からなかった)
void restore(uint32_t valueMs, uint32_t savedUtc, uint32_t lowMs);
// 今の時刻(SysInfo の timeOk のとき)。起動後に最初に呼ばれたときだけ、電源が切れていた時間を引く
void onTime(uint32_t utc, const Settings &set, uint32_t nowMs);
// 2.12.0: 過負荷時間をこの値以下にする(累積時間 リセットのとき。過負荷時間は累積時間を超えない。本人 10/28「累積時間が０なのに過負荷４９：３３と
// 表示されています」)。0 になったら「黄になった」も忘れる
void capMs(uint32_t ms);
void testSet(uint32_t valueMs, bool hot);  // 試験用: 値と「リセットのあとに黄になった」を決める(数えた記録は消す)
void testReset();                               // 試験用: 全部 0 に
}

// ---------------------------------------------------------------- perf(本体の状態の「FPS」。2.3.0。2.4.3: 1 秒ごとにまとめる)
// サーキット・タコで 1 回描いて送るのにかかった時間を 1 秒ごとにまとめ、回を数えた秒だけを新しい方から PERF_FPS_SECS 秒ぶん覚える
// (loop() だけから呼ぶ)
namespace perf {
void pageAdd(uint32_t us, uint32_t nowMs);  // 1 回ぶん(ui::pageFrame が true の回)
void pageStats(PerfStats *ps);  // 覚えている秒(今の秒も)から pageFrames・pageAvgUs・pageSlowUs・histN・fpsHist を入れる(0 回ならどれも 0)
void pageClear();               // 覚えた秒を捨てる(試験用)
}

// ---------------------------------------------------------------- gnss(GPS モジュール M100 の UBX を解析する)
// 受けるのは UBX-NAV-PVT(class 0x01、id 0x07、92 バイト)だけ。ほかの UBX は読み飛ばす(ACK は数える)。
namespace gnss {
struct Fix {
  bool fixOk;        // 位置が使える(2D か 3D で gnssFixOK)
  uint8_t fixType;   // 0 = なし、2 = 2D、3 = 3D、4 = 3D+推測航法
  uint8_t numSV;
  bool diffSoln;     // 補正(SBAS など)あり
  int32_t lat, lon;  // 1e-7 度
  int32_t hMSL;      // 平均海面からの高さ [mm]
  int32_t gSpeed;    // 対地速度 [mm/s]
  int32_t headMot;   // 進行方向 [1e-5 度]
  uint16_t pDOP;     // 0.01
  bool timeOk;       // 日時が使える(validDate && validTime)
  uint16_t year;
  uint8_t month, day, hour, min, sec;
  int32_t nano;      // 秒の端数 [ns](負もある)
  uint32_t iTOW;     // [ms]
  uint32_t rxMs;     // 受けた時刻(millis)
};
void begin();
// 届いたバイト列を解析する。戻り値は完成した NAV-PVT の数
int feed(const uint8_t *data, size_t n, uint32_t nowMs);
bool latest(Fix *out);    // 最新の NAV-PVT(1 つも無ければ false)
uint32_t pvtCount();      // 受けた NAV-PVT の累計
uint32_t ackCount();      // 受けた UBX-ACK-ACK の累計(設定が通った数)
uint32_t nakCount();      // 受けた UBX-ACK-NAK の累計
uint32_t badCount();      // チェックサムの合わなかった数
// 設定(UBX-CFG-VALSET、層は config.h の RC_GPS_CFG_LAYERS)。2 つに分けてある(1 つでも鍵が通らないと、その
// メッセージ全体が NAK になるため。無くても困らない衛星系の設定で、測位の設定まで落とさない)。戻り値はバイト数
//   buildConfig : 測位 rateHz 回/秒、NAV-PVT を UART1 へ、NMEA を止める、UBX の入力、車載モデル(out は 96 バイト以上)
//   buildSignals: 衛星系は GPS・Galileo・QZSS、BeiDou・GLONASS・SBAS は切(本人 10/29 の決定。u-blox M10 は GPS+Galileo だと測位が既定で
//                 最大 10 回/秒、OTP の高性能設定(下)で 20 回/秒。out は 96 バイト以上)
size_t buildConfig(uint8_t *out, size_t cap, int rateHz);
size_t buildSignals(uint8_t *out, size_t cap);
// UART1 の速度だけを変える(工場出荷の 9600 bps から RC_GPS_BAUD へ)
size_t buildBaud(uint8_t *out, size_t cap, uint32_t baud);
// 2.13.0: M10 の OTP の高性能設定(測位の回数の上限を上げる。SAM-M10Q 統合マニュアル 2.1.5。永久の設定で、元に戻せない。本人 10/29 の決定)
size_t buildMonVerPoll(uint8_t *out, size_t cap);         // UBX-MON-VER の問い合わせ(版を読む)
size_t buildOtpHighPerf(int part, uint8_t *out, size_t cap);  // マニュアルの設定列 2 本(part 0、1)をそのまま。UBX-CFG-OTP(0x06 0x41)
size_t buildOtpValgetPoll(uint8_t *out, size_t cap);      // 設定を確かめる UBX-CFG-VALGET の問い合わせ(鍵 4 つ)
size_t buildReset(uint8_t *out, size_t cap);              // UBX-CFG-RST(ハードウェアリセット。書いた設定は再起動で効く)
bool monVer(char *sw, size_t swCap, char *hw, size_t hwCap);  // 受けた MON-VER の swVersion(例「ROM SPG 5.10 (7b202e)」)と hwVersion。無ければ false
uint32_t monVerCount();     // 受けた MON-VER の数
uint32_t valgetCount();     // 受けた CFG-VALGET の応答の数
uint32_t ackOtpCount();     // CFG-OTP への ACK の数(設定列 2 本で 2)
uint32_t nakOtpCount();     // CFG-OTP への NAK の数
uint32_t nakValgetCount();  // CFG-VALGET への NAK の数(鍵を知らないファーム)
bool valgetValue(uint32_t key, uint32_t *val);  // 最後に受けた VALGET の応答から鍵の値を取る
int otpHighPerfState();     // 0 = 応答なし、1 = 高性能設定が入っている(4 つの鍵が期待の値)、2 = 入っていない
}

// ---------------------------------------------------------------- gtime(今の時刻。2.10.0: サーキットの時計と高負荷時間)
// GPS の日時(NAV-PVT の validDate・validTime)で合わせ、そのあとは millis で進める(GPS が途切れても続く。仕様書
// circuit_screen_spec.md 第 3 版 3.2)。NAV-PVT を受けるたびに合わせ直す。基板は SysInfo の timeOk・utc に入れる
namespace gtime {
void onFix(const gnss::Fix &f);           // 新しい NAV-PVT を受けたら呼ぶ(日時が使えないものは捨てる)
bool now(uint32_t nowMs, uint32_t *utc);  // 時刻が分かっていれば true と UTC(1970-01-01 からの秒)
uint32_t epoch(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec);  // 日時(UTC)→ 秒
// 2.11.0: デモの時計の起点。date・time は __DATE__("Oct 28 2026")・__TIME__("12:34:56")の形で、ビルドした PC の時刻(日本時間 =
// UTC + CLOCK_TZ_SEC)。UTC の秒を返す。読めなければ 0
uint32_t demoBase(const char *date, const char *time);
void testReset();                         // 試験用: 分からない状態に戻す
}

// ---------------------------------------------------------------- rcdiy(RaceChrono DIY の BLE の中身を作る。仕様: github.com/aollin/racechrono-ble-diy-device)
namespace rcdiy {
static const uint32_t PID_IMU_ACCEL = RC_PID_IMU_ACCEL;  // 表示器の IMU: 前後・横・上下の加速度 [mg](int16 × 3、LE)
static const uint32_t PID_IMU_GYRO = RC_PID_IMU_GYRO;    // 同 ヨーレート [0.01 deg/s](int16 × 3、LE。2 つ目以降は 0)
// GPS main(20 バイト)と GPS time(3 バイト)。sync は 0〜7。speedKmh は GPS の速度
void encodeGpsMain(const gnss::Fix &f, uint8_t sync, uint8_t out[20]);
void encodeGpsTime(const gnss::Fix &f, uint8_t sync, uint8_t out[3]);
uint32_t dateHourCode(const gnss::Fix &f);  // 0x0004 の 21 ビットの値(これが変わったら sync を進める)
// CAN(4 バイトの ID(LE)+ 中身 1〜16 バイト)。戻り値はバイト数
size_t encodeCan(uint32_t id, const uint8_t *d, uint8_t dlc, uint8_t out[20]);
// PID フィルタ(0x0002 に書かれる命令)。RaceChrono は「全部拒否」→「この PID を n ms 間隔で」を PID ごとに送ってくる
void filterReset();
void filterCommand(const uint8_t *data, size_t len);
// この PID を今送ってよいか(フィルタと、PID ごとの最短間隔)。送るときは lastSent を更新する。
// minMs: RaceChrono が頼む間隔がこれより短いときは、この間隔にする(車の CAN は config.h の RC_CAN_MIN_MS、IMU は 0)
bool filterPass(uint32_t pid, uint32_t nowMs, uint16_t minMs = 0);
int filterCount();        // 許可している PID の数(全部許可のときは -1)
}

// ---------------------------------------------------------------- imu(取り付け角の補正)
// 表示器はダッシュ上に傾けて付けるので、センサーの軸は車の軸と違う。停車中の重力の向きで「上」を、
// 発進の加速の向きで「前」を覚え、加速度を車の座標(前後・横・上下)に直す。
namespace imu {
void begin();
// センサーの 1 回分の値 [g] と、そのときの車速。50 回/秒くらいで呼ぶ。
// 覚える前は config.h の IMU_DEF_UP / IMU_DEF_FWD(画面を立てて、背面が前)を使う。
// 車速が無いときは、立てた向き(45° 以内)で値が動かない状態が 3 秒続けば静止とみなし、その重力の向きを「上」にする(「前」は車速が要る)
void sample(float ax, float ay, float az, bool speedOk, float speedKmh, uint32_t nowMs);
bool levelOk();      // 「上」を覚えた
bool forwardOk();    // 「前」を覚えた
bool stillNow();     // いま静止している(1 秒以上)。ジャイロの零点を取るのに使う
float gravity();     // 停車中に測った重力の大きさ [g](センサーの倍率のずれ。car() はこれで割る)。覚える前は 1
// 直近の値を車の座標で返す [g]。lon = 前が正、lat = 右が正(IMU_LAT_SIGN で反転)、vert = 上が正(重力を引いた値)。gravity() で割ってある
void car(float *lon, float *lat, float *vert);
// 覚えた向きの保存と復元(NVS 用)。9 個の float(上 3、前 3、上を覚えた 1/0、前を覚えた 1/0、重力の大きさ)
void exportCal(float out[9]);
void importCal(const float in[9]);
// 覚えた向きが変わった(新しく覚えた、覚え直した)回数。保存のきっかけに使う
uint32_t calVersion();
// センサーの座標の値を車の座標に直す(ヨーレートなど、直近の値以外にも使う)
void toCar(float x, float y, float z, float *lon, float *lat, float *vert);
// 試験用: 覚えた向きを捨てる
void reset();
}

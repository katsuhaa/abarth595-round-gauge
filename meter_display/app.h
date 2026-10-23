// app.h - 表示器の中で受け渡すデータと、各部の入口
//
// mlink  : UART で届いたバイト列をフレームに分け、最新の値と受信の状態を持つ
// veh    : 車の CAN の生データから表示に使う値(回転数・車速・水温・ブースト・燃料・ギアなど)を作る(2026-10-23 に c6_can_link から移した)
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
};

struct Settings {
  int shiftRpm;       // シフト点 [rpm]
  int autoReturnMin;  // 自動で戻る [分]
  int brightnessPct;  // 明るさ [%]
  int warnTempC;      // 水温の警告温度 [℃]
  int demoOn;         // デモ表示(1 = 入)
  int rcOn;           // RaceChrono への送信(1 = 入)。次の起動から効く
};

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
};

// RaceChrono へ送る側(BLE・GPS・IMU)の状態。「RaceChrono」の画面に出す
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
};

struct SysInfo {
  bool axpOk;           // 電源 IC(AXP2101)と通信できた
  uint8_t powerOnSrc;   // AXP2101 のレジスタ 0x20(起動のきっかけ)
  bool touchOk;         // タッチ IC と通信できた
  bool nvsOk;           // 設定の保存領域を開けた
  RcStats rc;
};

// ---------------------------------------------------------------- plat(基板に依存する部分)
void platLock();      // mlink と cantab のデータを守る
void platUnlock();
void *platAlloc(size_t bytes);         // 大きなメモリ(PSRAM)
void platSetBrightness(int pct);       // 画面の明るさ
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
// IMU の覚えた向き(取り付け角の補正)を捨てる(「RaceChrono の状態」の「向きの補正」の行をタップ)。保存した値も消す
void platResetImuCal();
// 3000rpm 超・以下の累計時間を今すぐ NVS に書く(「時間の記録」のリセットのあと)。ふだんは TIME_SAVE_MS ごとに書く
void platSaveTimes();

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
void begin();  // 状態を全部消す(燃料の前回の値も)。setup() で NVS から戻す前に呼ぶ
// CAN の生データを 1 フレーム受けるたびに呼ぶ(受信タスク。mlink が呼ぶ)
void onFrame(uint8_t bus, uint8_t ext, uint32_t id, uint8_t dlc, const uint8_t *data, uint32_t nowMs);
// 表示に使う値(linkUp・sim・scene 以外)を out に入れる。out は呼ぶ側で 0 にしておく
void getSignals(Signals *out, uint32_t nowMs);
// 燃料のなましの値(lp)の保存と復元(NVS)。値は生の値の 10 倍(0.1 L 単位)の整数
void fuelRestore(int tenths);       // 起動時に、前回保存した値を入れる(0 以下は「無い」)
bool fuelSaveValue(int *tenths);    // 保存する値。なましの値がまだ無ければ false
struct FuelInfo {
  bool have;            // なましの値がある(前回の値か、正しい生の値を受けた)
  bool restored;        // この起動で前回の値から始めた
  int restoredTenths;   // そのときの値(生の値の 10 倍)
  bool seen;            // 0C014003 を VEH_FUEL_TIMEOUT_MS 以内に受けている(b5 = 0 も含む)
  uint8_t raw;          // 最後に使った生の値(b5)
  float hp, lp;
  bool stable, armed;
  uint32_t zeroFrames;  // 捨てた b5 = 0 のフレームの数
  uint32_t refuels;     // 給油とみなした回数
};
void fuelInfo(FuelInfo *out, uint32_t nowMs);
float lastRatio();  // ギアの推定に使った最後の比 [rpm per km/h](シリアル用)
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
}

// ---------------------------------------------------------------- trip
namespace trip {
void update(const Signals &s, uint32_t nowMs);
uint32_t movingMs();  // 走行時間
uint32_t wotMs();     // 全開の時間
uint32_t aboveMs();   // 回転数が TIME_RPM を超えている時間の累計(電源を切っても続く)
uint32_t belowMs();   // TIME_RPM 以下の時間の累計
void setTimes(uint32_t above, uint32_t below);  // 起動時に NVS から戻す
void resetTimes();    // 0 に戻す(「時間の記録」のリセット)
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
//   buildSignals: 衛星系は GPS・Galileo・QZSS、BeiDou・GLONASS・SBAS は切(RaceBox 互換機と同じ。out は 96 バイト以上)
size_t buildConfig(uint8_t *out, size_t cap, int rateHz);
size_t buildSignals(uint8_t *out, size_t cap);
// UART1 の速度だけを変える(工場出荷の 9600 bps から RC_GPS_BAUD へ)
size_t buildBaud(uint8_t *out, size_t cap, uint32_t baud);
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

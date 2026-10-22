/*
 * c6_display_debug.ino  -  表示器のデバッグ用: 模擬データの送信  Rev.2  (2026-10-14)
 *   Rev.2: データ形式を版 2 に(ブースト圧を追加)。場面 9(ブースト)を追加
 *
 * 目的:
 *   車につながずに、表示器(meter_display)の全部の状態を机の上で出す。
 *   XIAO ESP32C6 が、本番と同じ UART のデータ形式(meter_link.h)で模擬の値を送り続ける。
 *   CAN には一切つながない(CAN のコードは入っていない)。送る値はすべて模擬で、車の値ではない。
 *   模擬であることはデータの中に印を入れてあり、表示器は画面の下に「SIM」、上に場面番号を出す。
 *
 * 配線(机の上。2 本だけ):
 *   XIAO「D6」  → 表示器の 8 ピンヘッダ「IO16」
 *   XIAO「GND」 → 表示器の 8 ピンヘッダ「GND」
 *   電源は XIAO・表示器とも、それぞれの USB-C から取る(「VBUS」どうしはつながない)。
 *   表示器の USB-C を先に挿す(表示器の電源が切れたまま「D6」の信号だけが入る状態を避ける)。
 *   試験機のトランシーバーは、つないだままでも外してもよい(このスケッチは「D0」〜「D3」を使わない)。
 *
 * 操作: なし。書き込むと、下の場面を順に繰り返す(1 周 約 121 秒)。
 *   任意: XIAO の BOOT ボタンを押すと、今の場面で止まる。もう一度押すと次へ進む。
 *   (Seeed の資料ではボタンの印字は「B」。実物の印字は未確認。「R」はリセットなので押さない)
 *
 * 場面(表示器の画面の上に出る番号):
 *   1  4200rpm 3速 水温92℃ 燃料62%      キャンバス「1 走行画面」
 *   2  850rpm N 水温88℃ 燃料12%          「850rpm(紺)・燃料が少ない」
 *   3  5600rpm 4速 水温112℃ 燃料45%      「5600rpm(黄)・水温が警告温度以上」
 *   4  6300rpm 3速 水温97℃ 燃料41%       「6300rpm(赤)・点滅せず明るさ最大」
 *   5  3400rpm 2速 水温・燃料は信号なし    「3400rpm(紺)・水温と燃料の信号がない」
 *   6  回転数 800→6600→800rpm             輪の伸び方・色の変わり方・点滅の始まり(シフト点の 1000 rpm 手前)と速くなり方・6000rpm からの最大輝度
 *   7  走行の模擬(発進→全開→減速→停止)  ギア、全開の割合と時間(データ画面)
 *   8  水温 30→125→30℃、燃料 100→0→100%  円弧と、異常のときの赤
 *   9  ブースト −0.5→2.0→−0.5 bar          アナログ計のページの内側の針(走行画面は変わらない)
 *  10  送信を止める(4 秒)                 信号がないときの表示(約 1 秒で切り替わる)
 *   場面 1〜8 でも、回転数とアクセルからブースト圧の模擬値を送る(全開で 1.4 bar くらい。閉じていると負圧)。
 *   どの場面でも、CAN確認画面用の CAN の生データ(実車ログの ID 27 個 + B-CAN 1 個の模擬)を送る。
 *
 * LED(基板の LED。GPIO15、Low で点灯):
 *   1 秒に 1 回短く光る = 送信中      1 秒に 2 回光る = 場面を止めている      速い点滅 = UART の初期化に失敗
 *
 * シリアル(USB): 場面が変わるたびに内容と「表示器でどう見えるはずか」を出す。1 秒ごとに送信数。
 *   シリアルモニターはいつ開いてもよい(開くのを待たずに送り始め、最初の説明は 60 秒ごとに出し直す)。
 *
 * ボード: Seeed XIAO ESP32C6、arduino-esp32 3.3.11、追加ライブラリなし。
 *   Arduino IDE のボードは「XIAO_ESP32C6」。「USB CDC On Boot」は Enabled(このボードの既定値)。
 */

#include <Arduino.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "meter_link.h"
#include "sim.h"

#if !ARDUINO_USB_CDC_ON_BOOT
#error "「USB CDC On Boot」を Enabled にしてください(XIAO_ESP32C6 の既定値)。Disabled だとログが USB に出ません。"
#endif

// ---- 配線(XIAO の印字名) ----
static const int PIN_LINK_TX = 16;           // 「D6」 → 表示器「IO16」
static const uint8_t LED_PIN = LED_BUILTIN;  // GPIO15
static const uint8_t LED_ON = LOW;
static const uint8_t LED_OFF = HIGH;
static const int PIN_BOOT = 9;               // BOOT ボタン(ESP32-C6 の起動モードのピンは GPIO9)。押すと Low

// ---- 設定 ----
static const uint32_t SIGNALS_MS = 20;       // 値のフレームの間隔(50 回/秒)
static const uint32_t INFO_MS = 1000;        // 状態のフレームの間隔
static const int CAN_PER_LOOP = 8;           // 1 周(1 ms)で送る CAN フレームの上限
static const char FW_NAME[12] = "c6dbg r1";
// 1 にすると、場面 1 に入るたびに「CRC の壊れたフレームを 5 個」送り、「連番を 3 つ」飛ばす。
// 表示器の「受信の状態」の「誤り・欠落」が 1 周ごとに 5 と 8 ずつ増えれば、数え方は正しい
// (欠落 8 = 壊した 5 個 + 飛ばした 3 個。壊れたフレームは「届かなかったフレーム」にも数える)。
// 線の品質を見るときは 0 のままにする(0 なら、数字が増えるのは線が原因)。
#define INJECT_FAULTS 0

static const uart_port_t LINK_UART = UART_NUM_1;
static bool g_uartOk = false;
static uint8_t g_seq = 0;
static uint32_t g_txFrames = 0, g_txBytes = 0, g_txDrop = 0;
static bool g_hold = false;

// フレームを 1 つ送る。UART の送信バッファーに入りきらないときは捨てて数える
static bool sendFrame(uint8_t type, const void *payload, uint8_t len, bool breakCrc = false) {
  uint8_t buf[ML_MAX_PAYLOAD + ML_OVERHEAD];
  size_t n = mlBuild(buf, type, g_seq, payload, len);
  if (breakCrc) buf[n - 1] ^= 0x5A;
  size_t room = 0;
  uart_get_tx_buffer_free_size(LINK_UART, &room);
  if (room < n) {
    g_txDrop++;
    return false;
  }
  uart_write_bytes(LINK_UART, buf, n);
  g_seq++;
  g_txFrames++;
  g_txBytes += n;
  return true;
}

static bool startUart() {
  uart_config_t cfg = {};
  cfg.baud_rate = METER_LINK_BAUD;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  if (uart_driver_install(LINK_UART, 256, 4096, 0, nullptr, 0) != ESP_OK) return false;
  if (uart_param_config(LINK_UART, &cfg) != ESP_OK) return false;
  if (uart_set_pin(LINK_UART, PIN_LINK_TX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return false;
  return true;
}

static void ledUpdate(uint32_t now) {
  bool on;
  if (!g_uartOk) on = ((now / 100) % 2) == 0;
  else if (g_hold) on = (now % 1000) < 60 || ((now % 1000) >= 200 && (now % 1000) < 260);
  else on = (now % 1000) < 60;
  digitalWrite(LED_PIN, on ? LED_ON : LED_OFF);
}

// BOOT ボタン: 押すたびに「止める」と「進める」を切り替える
static void buttonUpdate(uint32_t now) {
  static bool prev = false;
  static uint32_t lastMs = 0;
  bool down = digitalRead(PIN_BOOT) == LOW;
  if (down != prev && now - lastMs >= 40) {
    lastMs = now;
    prev = down;
    if (down) {
      g_hold = !g_hold;
      Serial.printf("ボタン: %s\n", g_hold ? "今の場面で止める" : "次の場面へ進める");
    }
  }
}

// 最初の説明を出す。setup() の最後と、そのあと 60 秒ごとに呼ぶ(シリアルモニターをいつ開いても見られるように)
static uint32_t g_bannerMs = 0;
static void printBanner() {
  g_bannerMs = millis();
  Serial.println();
  Serial.println("==== c6_display_debug Rev.1  表示器のデバッグ用の模擬データ(車の値ではない) ====");
  Serial.printf("UART: XIAO「D6」(GPIO%d) → 表示器「IO16」  %d bps  データ形式 版 %d  → %s\n", PIN_LINK_TX, METER_LINK_BAUD, METER_LINK_PROTO_VER,
                g_uartOk ? "送信中" : "初期化に失敗(送っていない)");
  Serial.println("表示器は画面の下に「SIM」、上に場面番号を出す。場面は自動で進む(BOOT ボタンで止める/進める)。");
  Serial.println();
}

static void printScene() {
  Serial.printf("場面 %d/%d  %s\n        → 表示器: %s\n", sim::scene(), sim::sceneCount(), sim::sceneName(), sim::sceneExpect());
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_ON);
  pinMode(PIN_BOOT, INPUT_PULLUP);

  // シリアルモニターがつながるのは待たない(電源を入れたらすぐ送り始める)。
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // Mac が読んでいなくても止まらない

  g_uartOk = startUart();
  sim::begin();
  printBanner();
}

void loop() {
  static uint32_t lastSig = 0, lastInfo = 0, lastLog = 0;
  static uint32_t logFrames = 0, logBytes = 0;
  static int lastScene = 0;
  uint32_t now = millis();

  ledUpdate(now);
  buttonUpdate(now);
  if (now - g_bannerMs >= 60000) {
    printBanner();
    if (g_uartOk && lastScene) printScene();
  }
  if (!g_uartOk) {
    delay(10);
    return;
  }
  sim::step(now, g_hold);

  if (sim::scene() != lastScene) {
    lastScene = sim::scene();
    printScene();
#if INJECT_FAULTS
    if (lastScene == 1) {
      MlSignals s;
      sim::signals(&s, now);
      for (int i = 0; i < 5; i++) sendFrame(ML_TYPE_SIGNALS, &s, sizeof(s), true);
      g_seq += 3;
      Serial.println("        (試験: CRC の壊れたフレームを 5 個送り、連番を 3 つ飛ばした。表示器の「誤り・欠落」は 5 と 8 増える)");
    }
#endif
  }

  if (sim::linkOn()) {
    if (now - lastSig >= SIGNALS_MS) {
      lastSig = now;
      MlSignals s;
      sim::signals(&s, now);
      sendFrame(ML_TYPE_SIGNALS, &s, sizeof(s));
    }
    MlCanHead head;
    uint8_t data[8];
    for (int i = 0; i < CAN_PER_LOOP && sim::canPoll(now, &head, data); i++) {
      uint8_t p[sizeof(MlCanHead) + 8];
      uint8_t dlc = head.info & 0x0F;
      memcpy(p, &head, sizeof(head));
      memcpy(p + sizeof(head), data, dlc);
      sendFrame(ML_TYPE_CAN, p, (uint8_t)(sizeof(head) + dlc));
    }
    if (now - lastInfo >= INFO_MS) {
      lastInfo = now;
      MlInfo info = {};
      info.proto_ver = METER_LINK_PROTO_VER;
      info.flags = ML_F_SIM | ML_F_C_CAN | ML_F_B_CAN;
      uint16_t fps[2], ids[2];
      sim::canRates(fps, ids);
      memcpy(info.can_fps, fps, sizeof(fps));
      memcpy(info.can_ids, ids, sizeof(ids));
      info.tx_drop = g_txDrop;
      info.uptime_s = now / 1000;
      memcpy(info.name, FW_NAME, sizeof(info.name));
      sendFrame(ML_TYPE_INFO, &info, sizeof(info));
    }
  }

  if (now - lastLog >= 1000) {
    lastLog = now;
    Serial.printf("  t=%lus 場面%d%s 送信 %lu フレーム/秒 %lu バイト/秒(回線の %lu%%) 捨てた数 %lu\n", (unsigned long)(now / 1000), sim::scene(), g_hold ? "(止めている)" : "",
                  (unsigned long)(g_txFrames - logFrames), (unsigned long)(g_txBytes - logBytes), (unsigned long)((g_txBytes - logBytes) * 1000 / METER_LINK_BAUD),
                  (unsigned long)g_txDrop);
    logFrames = g_txFrames;
    logBytes = g_txBytes;
  }
  delay(1);
}

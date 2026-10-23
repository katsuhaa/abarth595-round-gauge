/*
 * c6_can_link.ino  -  本番用: 車の CAN を読んで、生データをそのまま表示器へ UART で送る  版は下の FW_VERSION
 *
 * 役割:
 *   XIAO ESP32C6 の CAN コントローラー 2 つで C-CAN(500 kbps)と B-CAN(50 kbps)を送信せずに受け、
 *   CAN の生データ(ID と中身)を UART で表示器(meter_display)へ送る。
 *   値への換算(回転数・車速・水温・ブースト・燃料など)、ギアの推定、燃料のなましはしない。2.0.0 で表示器へ移した
 *   (meter_display の config.h の VEH_* と app.cpp の veh。値の出どころの表もそちら)。
 *   1.x が送っていた値のフレーム(MlSignals)は送らないので、表示器は 2.0.0 以降を使う。データ形式は meter_link.h(版 2)のまま。
 *   送る決まり: 中身が変わった ID は最短 CAN_MIN_GAP_MS 間隔(回転数・車速は 100 回/秒で来るので、50 回/秒で届く)、
 *   変わらない ID は CAN_REFRESH_MS ごと。CAN_STALE_MS 以上受けていない ID は送らない(表示器はそれで「信号なし」にする)。
 *
 * 配線(XIAO の印字名。「595 CAN試験機 配線図」Rev.0.1 = can_dual_test と同じ + 表示器への 1 本):
 *   トランシーバー A(C-CAN)「CRX」 → XIAO「D0」(GPIO0)
 *   トランシーバー B(B-CAN)「CRX」 → XIAO「D2」(GPIO2)
 *   XIAO「D1」(GPIO1)、「D3」(GPIO21) は CAN の TX の割り当て先。配線しない。
 *   トランシーバーの「CTX」は 2 個とも電源ピン(3.3V)に固定。LISTEN_ONLY なので送信も ACK もしない。
 *   XIAO「D6」(GPIO16) → 表示器の 8 ピンヘッダ「IO16」(UART、921600 bps)。GND は共通。
 *
 * 操作: なし。書き込んで車につなぎ、IG ON にするだけ。
 *
 * LED(基板の LED。GPIO15、Low で点灯):
 *   ゆっくり点滅(1 秒点灯・1 秒消灯) = C-CAN・B-CAN とも受信中
 *   短い点滅 N 回のあと休み: 1 回 = C-CAN 受信なし  2 回 = B-CAN 受信なし  3 回 = 両方なし  4 回 = C-CAN エラー  5 回 = B-CAN エラー
 *   速い点滅(連続) = CAN か UART の初期化に失敗(理由はシリアル)
 *
 * シリアル(USB、115200 bps。開くのを待たず、最初の説明は 60 秒ごとに出し直す):
 *   HB 行   1 秒ごと。バスごとの状態、受信数/秒、ID の数、エラー、表示器への送信数。値は出さない(表示器のシリアルの
 *           「車の値(CAN から)」の行に出る)。
 *   NEWID 行  ID を初めて受けたとき。  ID 一覧  30 秒ごと(ID 探しに使う)。
 *   L 行(Rev.2)  CAN の中身が変わるたび(同じ ID は最短 LOG_MIN_GAP_MS 間隔)と、変わらなくても LOG_REFRESH_MS ごと。
 *     形式: L <C|B> <millis> <ID 16進8桁> <len> <データ 16進>   例: L C 123456 0618A001 8 00 00 0C 80 00 00 00 1F
 *     Mac で  cat /dev/cu.usbmodem… | tee ログ  で記録し、pc_test/can_id_find.py で ID を探す(手順は can_id_confirm_procedure.md)。
 *     Mac がつないでいなければ USB の送信バッファーで捨てられるだけ(表示器への UART には影響しない)。
 *
 * ボード: Seeed XIAO ESP32C6、arduino-esp32 3.3.11(ESP-IDF 5.5)、追加ライブラリなし。
 *   Arduino IDE のボードは「XIAO_ESP32C6」。「USB CDC On Boot」は Enabled(このボードの既定値)。
 */

#include <Arduino.h>
#include <string.h>
#include "driver/twai.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "meter_link.h"

#if !ARDUINO_USB_CDC_ON_BOOT
#error "「USB CDC On Boot」を Enabled にしてください(XIAO_ESP32C6 の既定値)。Disabled だとログが USB に出ません。"
#endif

// ---- 配線(XIAO の印字名) ----
static const gpio_num_t C_RX = GPIO_NUM_0;   // 「D0」 ← トランシーバー A「CRX」
static const gpio_num_t C_TX = GPIO_NUM_1;   // 「D1」 TX の割り当て先。配線しない
static const gpio_num_t B_RX = GPIO_NUM_2;   // 「D2」 ← トランシーバー B「CRX」
static const gpio_num_t B_TX = GPIO_NUM_21;  // 「D3」 TX の割り当て先。配線しない
static const int PIN_LINK_TX = 16;           // 「D6」 → 表示器「IO16」
static const uint8_t LED_PIN = LED_BUILTIN;  // GPIO15
static const uint8_t LED_ON = LOW;
static const uint8_t LED_OFF = HIGH;

// ---- 送信の設定 ----
static const uint32_t INFO_MS = 1000;         // 状態のフレームの間隔
// 同じ ID の生データを送る最短の間隔(中身が変わったとき)。2.0.0 で 50 → 20 ms: 表示器が回転数・車速などを生データから作るので、
// 1.x の値のフレームと同じ 50 回/秒で届ける(回転数 0618A001・車速 0210A006 は車から 100 回/秒で来る)。
// 回線の使用は 1.x の 7 % 前後から、多くて 15 % 前後に増える(10/22 のログの ID 一覧から計算した上限: C-CAN 30 ID・約 1000 回/秒、
// B-CAN 47 ID・約 145 回/秒が全部毎回変わるとして約 710 フレーム/秒・14 KB/秒)。
// RaceChrono へ送る量は表示器の RC_CAN_MIN_MS(50 ms)で 1.x と同じに保つ
static const uint32_t CAN_MIN_GAP_MS = 20;
static const uint32_t CAN_REFRESH_MS = 1000;  // 変わらなくてもこの間隔で送る(CAN確認画面で ID が見えるように)
// これ以上受けていない ID は送らない(送り直しもしない)。表示器は届かなくなった値を「信号なし」にする(meter_display の VEH_TIMEOUT_MS)
static const uint32_t CAN_STALE_MS = 1500;
static const int CAN_PER_LOOP = 8;            // 1 周で送る生データの上限
// 版(2026-10-23 から git で管理。Rev.5 の次が 1.0.0。1.1.0 で燃料のなましを入れ替えた。2.0.0 で値への換算をやめ、生データだけを送る)。
// 形式は「大.中.小」: 小 = 直しだけ、中 = 値の追加や変更、大 = 表示器とのデータ形式(meter_link.h)や送るものが変わるとき。
// 表示器の 受信の状態 の「送り元」に FW_NAME が出る
#define FW_VERSION "2.0.0"
static const char FW_NAME[12] = "c6can " FW_VERSION;  // 11 文字まで(最後の 0 を入れて 12)
static const uart_port_t LINK_UART = UART_NUM_1;

// ---- 受信の設定(can_dual_test と同じ) ----
static const uint32_t TICK_MS = 1000;
static const uint32_t TABLE_MS = 30000;
static const uint32_t NONE_SECONDS = 2;
static const int MAX_IDS = 128;
static const uint32_t RX_QUEUE_LEN = 128;
static const int NEWID_PER_TICK = 24;

// ---- USB への変化ログ(L 行、Rev.2) ----
#define CAN_LOG_USB 1                           // 1 = L 行を出す。0 で出さない(ID 探しが終わったら 0 にしてよい)。受信・表示器への送信には影響しない(10/22 実測: fps・miss・捨てた が変わらず)
static const uint32_t LOG_MIN_GAP_MS = 100;     // 同じ ID の L 行の最短間隔(変わったとき)。10 行/秒/ID まで
static const uint32_t LOG_REFRESH_MS = 5000;    // 変わらなくてもこの間隔で出す(状態の記録用)
static const int LOG_PER_LOOP = 16;             // 1 周で出す L 行の上限

enum BusState { ST_WAIT, ST_OK, ST_NONE, ST_ERR, ST_INIT_FAIL };

struct IdEntry {
  uint32_t id;
  uint8_t ext;
  uint8_t dlc;
  uint8_t data[8];
  uint32_t count;
  uint32_t countAtTable;
  uint32_t rxMs;  // 最後に受けた時刻(CAN_STALE_MS の判定)
  bool announced;
  // 表示器へ送った内容(loop() 側だけが使う)
  uint8_t sentData[8];
  uint8_t sentDlc;
  uint32_t sentMs;
  bool sentOnce;
  // USB の L 行に出した内容(loop() 側だけが使う)
  uint8_t logData[8];
  uint8_t logDlc;
  uint32_t logMs;
  bool logOnce;
};

struct Bus {
  const char *name;
  const char *rxLabel;
  int controller;
  gpio_num_t rx;
  gpio_num_t tx;
  uint32_t kbps;
  bool isB;
  twai_handle_t h;
  bool ready;
  esp_err_t installErr;
  esp_err_t startErr;
  portMUX_TYPE mux;
  uint32_t total;
  IdEntry ids[MAX_IDS];
  int nIds;
  uint32_t idOverflow;
  uint32_t prevTotal;
  uint32_t prevBusErr;
  uint32_t busErr;
  uint32_t noneSecs;
  uint32_t fps;
  BusState state;
};

static Bus g_bus[2] = {
  { "C-CAN", "D0", 0, C_RX, C_TX, 500, false, nullptr, false, ESP_OK, ESP_OK, portMUX_INITIALIZER_UNLOCKED },
  { "B-CAN", "D2", 1, B_RX, B_TX, 50, true, nullptr, false, ESP_OK, ESP_OK, portMUX_INITIALIZER_UNLOCKED },
};

static uint8_t g_seq = 0;
static uint32_t g_txFrames = 0, g_txBytes = 0, g_txDrop = 0;
static bool g_uartOk = false;
static uint32_t g_lastTick = 0, g_lastTable = 0, g_tableBase = 0, g_bannerMs = 0;
static int g_ledCode = 8;
static uint32_t g_ledStart = 0;

// ---------------------------------------------------------------- 関数(型の定義より後に置く)
static const char *stateName(BusState s) {
  switch (s) {
    case ST_OK: return "OK  ";
    case ST_NONE: return "NONE";
    case ST_ERR: return "ERR ";
    case ST_INIT_FAIL: return "FAIL";
    default: return "WAIT";
  }
}

// ---------------------------------------------------------------- LED(can_dual_test と同じ決まり)
static void setLedCode(int code) {
  if (code != g_ledCode) {
    g_ledCode = code;
    g_ledStart = millis();
  }
}
static void ledUpdate(uint32_t now) {
  bool on = false;
  if (g_ledCode == 8) {
    on = true;
  } else if (g_ledCode == 0) {
    on = ((now / 1000) % 2) == 0;
  } else if (g_ledCode == 9) {
    on = ((now / 100) % 2) == 0;
  } else if (g_ledCode >= 1 && g_ledCode <= 5) {
    uint32_t blinks = (uint32_t)g_ledCode * 400;
    uint32_t t = (now - g_ledStart) % (blinks + 1500);
    on = (t < blinks) && ((t % 400) < 150);
  }
  digitalWrite(LED_PIN, on ? LED_ON : LED_OFF);
}

// ---------------------------------------------------------------- 受信(値への換算はしない。表示器が生データから作る)
static void rxTask(void *arg) {
  Bus *b = (Bus *)arg;
  twai_message_t m;
  for (;;) {
    if (twai_receive_v2(b->h, &m, pdMS_TO_TICKS(200)) != ESP_OK) continue;
    uint8_t dlc = m.data_length_code > 8 ? 8 : m.data_length_code;
    uint32_t now = millis();
    portENTER_CRITICAL(&b->mux);
    b->total++;
    int i = 0;
    while (i < b->nIds && !(b->ids[i].id == m.identifier && b->ids[i].ext == m.extd)) i++;
    if (i == b->nIds) {
      if (b->nIds < MAX_IDS) {
        IdEntry &n = b->ids[b->nIds++];
        memset(&n, 0, sizeof(n));
        n.id = m.identifier;
        n.ext = m.extd;
      } else {
        b->idOverflow++;
        i = -1;
      }
    }
    if (i >= 0) {
      IdEntry &e = b->ids[i];
      e.dlc = dlc;
      memset(e.data, 0, sizeof(e.data));
      if (!m.rtr) memcpy(e.data, m.data, dlc);
      e.count++;
      e.rxMs = now ? now : 1;
    }
    portEXIT_CRITICAL(&b->mux);
  }
}

static bool startBus(Bus &b) {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT_V2(b.controller, b.tx, b.rx, TWAI_MODE_LISTEN_ONLY);
  g.tx_queue_len = 0;
  g.rx_queue_len = RX_QUEUE_LEN;
  twai_timing_config_t t500 = TWAI_TIMING_CONFIG_500KBITS();
  twai_timing_config_t t50 = TWAI_TIMING_CONFIG_50KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  b.installErr = twai_driver_install_v2(&g, b.kbps == 500 ? &t500 : &t50, &f, &b.h);
  if (b.installErr != ESP_OK) return false;
  b.startErr = twai_start_v2(b.h);
  if (b.startErr != ESP_OK) return false;
  gpio_pullup_en(b.rx);
  char taskName[12];
  snprintf(taskName, sizeof(taskName), "rx_%s", b.isB ? "B" : "C");
  xTaskCreate(rxTask, taskName, 4096, &b, 5, nullptr);
  return true;
}

// ---------------------------------------------------------------- UART(c6_display_debug と同じ)
static bool sendFrame(uint8_t type, const void *payload, uint8_t len) {
  uint8_t buf[ML_MAX_PAYLOAD + ML_OVERHEAD];
  size_t n = mlBuild(buf, type, g_seq, payload, len);
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

// ---------------------------------------------------------------- 表示器へ送る
// CAN の生データ: 中身が変わった ID は最短 CAN_MIN_GAP_MS 間隔、変わらない ID は CAN_REFRESH_MS ごと。
// CAN_STALE_MS 以上受けていない ID は送らない(古い値を送り直さない)
static int g_canCursor[2] = { 0, 0 };
static void sendCanFrames(uint32_t now) {
  int sent = 0;
  for (int k = 0; k < 2 && sent < CAN_PER_LOOP; k++) {
    Bus &b = g_bus[k];
    if (!b.ready) continue;
    int n;
    portENTER_CRITICAL(&b.mux);
    n = b.nIds;
    portEXIT_CRITICAL(&b.mux);
    for (int c = 0; c < n && sent < CAN_PER_LOOP; c++) {
      int i = (g_canCursor[k] + c) % n;
      IdEntry e;
      portENTER_CRITICAL(&b.mux);
      e = b.ids[i];
      portEXIT_CRITICAL(&b.mux);
      if ((int32_t)(now - e.rxMs) > (int32_t)CAN_STALE_MS) continue;  // 受信タスクの時刻が now より新しいことがある(差が負)
      uint32_t el = now - e.sentMs;
      bool changed = e.dlc != e.sentDlc || memcmp(e.data, e.sentData, 8) != 0;
      if (!(!e.sentOnce || (changed && el >= CAN_MIN_GAP_MS) || el >= CAN_REFRESH_MS)) continue;
      uint8_t p[sizeof(MlCanHead) + 8];
      MlCanHead head;
      head.info = (uint8_t)((b.isB ? 0x80 : 0) | (e.ext ? 0x40 : 0) | (e.dlc & 0x0F));
      head.id = e.id;
      memcpy(p, &head, sizeof(head));
      memcpy(p + sizeof(head), e.data, e.dlc);
      if (!sendFrame(ML_TYPE_CAN, p, (uint8_t)(sizeof(head) + e.dlc))) return;  // 送信バッファーが一杯: 次の周に回す
      portENTER_CRITICAL(&b.mux);
      memcpy(b.ids[i].sentData, e.data, 8);
      b.ids[i].sentDlc = e.dlc;
      b.ids[i].sentMs = now;
      b.ids[i].sentOnce = true;
      portEXIT_CRITICAL(&b.mux);
      sent++;
      g_canCursor[k] = (i + 1) % n;
    }
  }
}

// USB への変化ログ(L 行): 中身が変わった ID は最短 LOG_MIN_GAP_MS 間隔、変わらない ID は LOG_REFRESH_MS ごと
static int g_logCursor[2] = { 0, 0 };
static uint32_t g_logLines = 0;
static void logCanChanges(uint32_t now) {
#if !CAN_LOG_USB
  (void)now;
  return;
#endif
  int sent = 0;
  for (int k = 0; k < 2 && sent < LOG_PER_LOOP; k++) {
    Bus &b = g_bus[k];
    if (!b.ready) continue;
    int n;
    portENTER_CRITICAL(&b.mux);
    n = b.nIds;
    portEXIT_CRITICAL(&b.mux);
    for (int c = 0; c < n && sent < LOG_PER_LOOP; c++) {
      int i = (g_logCursor[k] + c) % n;
      IdEntry e;
      portENTER_CRITICAL(&b.mux);
      e = b.ids[i];
      portEXIT_CRITICAL(&b.mux);
      uint32_t el = now - e.logMs;
      bool changed = e.dlc != e.logDlc || memcmp(e.data, e.logData, 8) != 0;
      if (!(!e.logOnce || (changed && el >= LOG_MIN_GAP_MS) || el >= LOG_REFRESH_MS)) continue;
      if (Serial.availableForWrite() < 2048) return;  // USB のバッファーに余裕がない(Mac が読んでいない、または HB 行・ID 一覧の分を残す): 次の周に回す
      char line[64];
      int len = snprintf(line, sizeof(line), "L %c %lu %08lX %u", b.isB ? 'B' : 'C', (unsigned long)now, (unsigned long)e.id, e.dlc);
      for (int j = 0; j < e.dlc && len < (int)sizeof(line) - 4; j++) len += snprintf(line + len, sizeof(line) - len, " %02X", e.data[j]);
      line[len++] = '\n';
      Serial.write((const uint8_t *)line, len);
      g_logLines++;
      portENTER_CRITICAL(&b.mux);
      memcpy(b.ids[i].logData, e.data, 8);
      b.ids[i].logDlc = e.dlc;
      b.ids[i].logMs = now;
      b.ids[i].logOnce = true;
      portEXIT_CRITICAL(&b.mux);
      sent++;
      g_logCursor[k] = (i + 1) % n;
    }
  }
}

static void sendInfo(uint32_t now) {
  MlInfo info = {};
  info.proto_ver = METER_LINK_PROTO_VER;
  info.flags = (g_bus[0].state == ST_OK ? ML_F_C_CAN : 0) | (g_bus[1].state == ST_OK ? ML_F_B_CAN : 0);
  for (int k = 0; k < 2; k++) {
    info.can_fps[k] = (uint16_t)(g_bus[k].fps > 65535 ? 65535 : g_bus[k].fps);
    info.can_ids[k] = (uint16_t)g_bus[k].nIds;
    info.can_err[k] = g_bus[k].busErr;
  }
  info.tx_drop = g_txDrop;
  info.uptime_s = now / 1000;
  memcpy(info.name, FW_NAME, sizeof(info.name));
  sendFrame(ML_TYPE_INFO, &info, sizeof(info));
}

// ---------------------------------------------------------------- シリアル
static void printBanner() {
  g_bannerMs = millis();
  Serial.println();
  Serial.println("==== c6_can_link v" FW_VERSION "  車の CAN を読んで、生データを表示器へ送る(本番用) ====");
  Serial.println("C-CAN: 500 kbps  RX=「D0」  B-CAN: 50 kbps  RX=「D2」  どちらも LISTEN_ONLY(送信も ACK もしない)");
  Serial.printf("UART: 「D6」(GPIO%d) → 表示器「IO16」  %d bps  データ形式 版 %d  → %s\n", PIN_LINK_TX, METER_LINK_BAUD, METER_LINK_PROTO_VER,
                g_uartOk ? "送信中" : "初期化に失敗(送っていない)");
  Serial.printf("送るもの: CAN の生データだけ(変わった ID は %lu ms 間隔まで、変わらない ID は %lu ms ごと、%lu ms 受けていない ID は送らない)と状態(1 秒ごと)。\n",
                (unsigned long)CAN_MIN_GAP_MS, (unsigned long)CAN_REFRESH_MS, (unsigned long)CAN_STALE_MS);
  Serial.println("値への換算・ギアの推定・燃料のなましは表示器(meter_display 2.0.0 以降)で行う。値のフレームは送らない");
  Serial.println("LED: ゆっくり点滅=両方受信中 / 1回=C-CAN受信なし 2回=B-CAN受信なし 3回=両方なし 4回=C-CANエラー 5回=B-CANエラー / 速い点滅=初期化失敗");
  Serial.printf("L 行: CAN の中身が変わるたび(同じ ID は %lu ms 間隔まで)と %lu 秒ごと。形式 L <C|B> <millis> <ID> <len> <データ>。pc_test/can_id_find.py で解析\n",
                (unsigned long)LOG_MIN_GAP_MS, (unsigned long)(LOG_REFRESH_MS / 1000));
  Serial.println();
}

static void printNewIds(uint32_t now) {
  int printed = 0;
  for (int k = 0; k < 2 && printed < NEWID_PER_TICK; k++) {
    Bus &b = g_bus[k];
    if (!b.ready) continue;
    for (;;) {
      IdEntry e;
      bool found = false;
      portENTER_CRITICAL(&b.mux);
      for (int i = 0; i < b.nIds; i++) {
        if (!b.ids[i].announced) {
          b.ids[i].announced = true;
          e = b.ids[i];
          found = true;
          break;
        }
      }
      portEXIT_CRITICAL(&b.mux);
      if (!found) break;
      Serial.printf("NEWID t=%.1fs %s %08lX %s len%u", now / 1000.0, b.name, (unsigned long)e.id, e.ext ? "29bit" : "11bit", e.dlc);
      for (int j = 0; j < e.dlc; j++) Serial.printf(" %02X", e.data[j]);
      Serial.println();
      if (++printed >= NEWID_PER_TICK) break;
    }
  }
}

static IdEntry g_snap[MAX_IDS];
static float g_snapHz[MAX_IDS];
static void printTables(uint32_t now) {
  float elapsed = (now - g_tableBase) / 1000.0f;
  if (elapsed < 0.5f) elapsed = 0.5f;
  g_tableBase = now;
  for (int k = 0; k < 2; k++) {
    Bus &b = g_bus[k];
    if (!b.ready) continue;
    int n;
    uint32_t total, overflow;
    portENTER_CRITICAL(&b.mux);
    n = b.nIds;
    total = b.total;
    overflow = b.idOverflow;
    for (int i = 0; i < n; i++) {
      g_snap[i] = b.ids[i];
      g_snapHz[i] = (b.ids[i].count - b.ids[i].countAtTable) / elapsed;
      b.ids[i].countAtTable = b.ids[i].count;
    }
    portEXIT_CRITICAL(&b.mux);
    for (int i = 1; i < n; i++) {  // ID の昇順
      IdEntry e = g_snap[i];
      float hz = g_snapHz[i];
      int j = i - 1;
      while (j >= 0 && g_snap[j].id > e.id) {
        g_snap[j + 1] = g_snap[j];
        g_snapHz[j + 1] = g_snapHz[j];
        j--;
      }
      g_snap[j + 1] = e;
      g_snapHz[j + 1] = hz;
    }
    Serial.printf("\n==== %s ID一覧 t=%lus ids=%d frames=%lu %lukbps RX=%s%s ====\n", b.name, (unsigned long)(now / 1000), n, (unsigned long)total,
                  (unsigned long)b.kbps, b.rxLabel, overflow ? " (ID 表があふれた)" : "");
    Serial.println("  ID        type   Hz     len  data");
    for (int i = 0; i < n; i++) {
      Serial.printf("  %08lX  %s  %6.1f  %u   ", (unsigned long)g_snap[i].id, g_snap[i].ext ? "29bit" : "11bit", g_snapHz[i], g_snap[i].dlc);
      for (int j = 0; j < g_snap[i].dlc; j++) Serial.printf("%02X ", g_snap[i].data[j]);
      Serial.println();
    }
  }
  Serial.println();
}

static void tick(uint32_t now) {
  static uint32_t logFrames = 0, logBytes = 0;
  char seg[2][96];
  for (int k = 0; k < 2; k++) {
    Bus &b = g_bus[k];
    if (!b.ready) {
      b.state = ST_INIT_FAIL;
      snprintf(seg[k], sizeof(seg[k]), "%s FAIL install=%s start=%s", b.name, esp_err_to_name(b.installErr), esp_err_to_name(b.startErr));
      continue;
    }
    uint32_t total;
    int ids, stale = 0;
    portENTER_CRITICAL(&b.mux);
    total = b.total;
    ids = b.nIds;
    for (int i = 0; i < ids; i++) {
      if ((int32_t)(now - b.ids[i].rxMs) > (int32_t)CAN_STALE_MS) stale++;  // 止まっていて表示器へ送っていない ID
    }
    portEXIT_CRITICAL(&b.mux);
    twai_status_info_t st;
    memset(&st, 0, sizeof(st));
    twai_get_status_info_v2(b.h, &st);
    uint32_t frames = total - b.prevTotal;
    uint32_t errDelta = st.bus_error_count - b.prevBusErr;
    b.prevTotal = total;
    b.prevBusErr = st.bus_error_count;
    b.busErr = st.bus_error_count;
    b.fps = frames;
    b.noneSecs = frames ? 0 : b.noneSecs + 1;
    if (errDelta > 0 || st.state == TWAI_STATE_BUS_OFF) b.state = ST_ERR;
    else if (b.noneSecs >= NONE_SECONDS) b.state = ST_NONE;
    else if (frames > 0) b.state = ST_OK;
    else b.state = ST_WAIT;
    snprintf(seg[k], sizeof(seg[k]), "%s %s fps=%lu ids=%d 止まった %d err+%lu miss=%lu", b.name, stateName(b.state), (unsigned long)frames, ids, stale,
             (unsigned long)errDelta, (unsigned long)(st.rx_missed_count + st.rx_overrun_count));
  }
  BusState c = g_bus[0].state, bb = g_bus[1].state;
  int code;
  if (!g_uartOk || c == ST_INIT_FAIL || bb == ST_INIT_FAIL) code = 9;
  else if (c == ST_NONE && bb == ST_NONE) code = 3;
  else if (c == ST_NONE) code = 1;
  else if (bb == ST_NONE) code = 2;
  else if (c == ST_ERR) code = 4;
  else if (bb == ST_ERR) code = 5;
  else if (c == ST_OK && bb == ST_OK) code = 0;
  else code = g_ledCode;
  setLedCode(code);

  static uint32_t logLines = 0;
  Serial.printf("HB t=%lus | %s | %s | 送信 %lu フレーム/秒 %lu バイト/秒(回線の %lu%%) 捨てた %lu | L行 %lu/秒 | LED=", (unsigned long)(now / 1000), seg[0], seg[1],
                (unsigned long)(g_txFrames - logFrames), (unsigned long)(g_txBytes - logBytes), (unsigned long)((g_txBytes - logBytes) * 1000 / METER_LINK_BAUD),
                (unsigned long)g_txDrop, (unsigned long)(g_logLines - logLines));
  logFrames = g_txFrames;
  logBytes = g_txBytes;
  logLines = g_logLines;
  if (code == 0) Serial.println("ゆっくり点滅(両方OK)");
  else if (code == 9) Serial.println("速い点滅(初期化失敗)");
  else if (code == 8) Serial.println("点灯(判定前)");
  else Serial.printf("%d回点滅\n", code);
  printNewIds(now);
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_ON);
  Serial.setTxBufferSize(8192);
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // Mac が読んでいなくても止まらない
  g_uartOk = startUart();
  for (int k = 0; k < 2; k++) g_bus[k].ready = startBus(g_bus[k]);
  printBanner();
  for (int k = 0; k < 2; k++) {
    Bus &b = g_bus[k];
    Serial.printf("%s: install=%s start=%s → %s\n", b.name, esp_err_to_name(b.installErr), esp_err_to_name(b.startErr), b.ready ? "受信開始" : "失敗");
  }
  uint32_t now = millis();
  g_lastTick = now;
  g_lastTable = now;
  g_tableBase = now;
}

void loop() {
  static uint32_t lastInfo = 0;
  uint32_t now = millis();
  ledUpdate(now);
  if (now - g_bannerMs >= 60000) printBanner();
  if (g_uartOk) {
    sendCanFrames(now);
    if (now - lastInfo >= INFO_MS) {
      lastInfo = now;
      sendInfo(now);
    }
  }
  logCanChanges(now);
  if (now - g_lastTick >= TICK_MS) {
    g_lastTick += TICK_MS;
    tick(now);
  }
  if (now - g_lastTable >= TABLE_MS) {
    g_lastTable = now;
    printTables(now);
  }
  delay(1);
}

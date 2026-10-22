/*
 * can_dual_test.ino  -  C-CAN と B-CAN の同時受信試験  Rev.1  (2026-10-08)
 *
 * 目的:
 *   XIAO ESP32C6 の CAN コントローラー 2 つで、C-CAN(OBD 6/14 番、500 kbps)と
 *   B-CAN(OBD 1/9 番、50 kbps)を、送信せずに同時に受けられるかを確かめる。
 *   判定は XIAO 自身が行い、基板の LED と USB シリアルに出す。
 *
 * 配線: 「595 CAN試験機 配線図」Rev.0.1 のとおり
 *   トランシーバー A(C-CAN)「CRX」 → XIAO「D0」(GPIO0)
 *   トランシーバー B(B-CAN)「CRX」 → XIAO「D2」(GPIO2)
 *   XIAO「D1」(GPIO1)、「D3」(GPIO21) は CAN の TX の割り当て先。配線しない。
 *   (ESP-IDF の TWAI ドライバーは TX ピンの指定が必須。LISTEN_ONLY では常に H のまま)
 *   トランシーバーの「CTX」は 2 個とも電源ピン(3.3V)に固定。
 *   LISTEN_ONLY なので送信も ACK もしない。車のバスへは何も出さない。
 *
 * 操作: なし。書き込んで OBD に挿し、IG ON にするだけ。
 *   USB をつないだ Mac では screen などでログを残せる。モバイルバッテリーなら LED だけで判定する。
 *
 * 判定(1 秒ごと、バスごと):
 *   OK   = 直近 1 秒にフレームあり、かつバスエラーの増加なし
 *   NONE = フレームなしが 2 秒続いた
 *   ERR  = バスエラーが増えた(フレームの有無は問わない)
 *   両方 OK が 10 秒続いたら「VERDICT PASS」を 1 回出す。その後も監視は続ける。
 *
 * LED(基板の LED。GPIO15、Low で点灯):
 *   点灯したまま = 起動直後(最初の判定まで)
 *   ゆっくり点滅(1 秒点灯・1 秒消灯) = 両方 OK
 *   短い点滅 N 回のあと休み、の繰り返し:
 *     1 回 = C-CAN 受信なし   2 回 = B-CAN 受信なし   3 回 = 両方とも受信なし
 *     4 回 = C-CAN エラー     5 回 = B-CAN エラー
 *   速い点滅(連続) = CAN の初期化に失敗(理由はシリアル)
 *
 * シリアル(USB):
 *   HB 行     1 秒ごと。バスごとの状態、受信数/秒、ID の数、エラー。
 *   切り分け   受信なしのとき、RX ピンの生レベルと 20 ms 間のエッジ数から原因の目安を出す。
 *   NEWID 行  ID を初めて受けたとき。
 *   ID 一覧   30 秒ごと。バスごとに ID・周期・長さ・最新データ(B-CAN の ID 探しに使う)。
 *
 * ボード: Seeed XIAO ESP32C6、arduino-esp32 3.3.11(ESP-IDF 5.5)、追加ライブラリなし。
 *   Arduino IDE のボードは「XIAO_ESP32C6」。「USB CDC On Boot」は Enabled(このボードの既定値)。
 */

#include <Arduino.h>
#include <string.h>
#include "driver/twai.h"
#include "driver/gpio.h"

#if !ARDUINO_USB_CDC_ON_BOOT
#error "「USB CDC On Boot」を Enabled にしてください(XIAO_ESP32C6 の既定値)。Disabled だとログが USB に出ません。"
#endif

// ---- 配線(XIAO の印字名) ----
static const gpio_num_t C_RX = GPIO_NUM_0;   // 「D0」 ← トランシーバー A「CRX」
static const gpio_num_t C_TX = GPIO_NUM_1;   // 「D1」 TX の割り当て先。配線しない
static const gpio_num_t B_RX = GPIO_NUM_2;   // 「D2」 ← トランシーバー B「CRX」
static const gpio_num_t B_TX = GPIO_NUM_21;  // 「D3」 TX の割り当て先。配線しない
static const uint8_t LED_PIN = LED_BUILTIN;  // GPIO15
static const uint8_t LED_ON = LOW;           // Low で点灯(RIOT の XIAO ESP32C6 定義: LED0_ACTIVE 0)
static const uint8_t LED_OFF = HIGH;

// ---- 判定の設定 ----
static const uint32_t TICK_MS = 1000;      // 判定と HB 行の間隔
static const uint32_t TABLE_MS = 30000;    // ID 一覧の間隔
static const uint32_t PASS_SECONDS = 10;   // 両方 OK がこれだけ続いたら PASS
static const uint32_t NONE_SECONDS = 2;    // これだけフレームが無ければ「受信なし」
static const int MAX_IDS = 128;            // バスごとに覚える ID の数
static const uint32_t RX_QUEUE_LEN = 128;  // ドライバーの受信キュー
static const int NEWID_PER_TICK = 24;      // 1 回の判定で出す NEWID 行の上限

enum BusState { ST_WAIT, ST_OK, ST_NONE, ST_ERR, ST_INIT_FAIL };

struct IdEntry {
  uint32_t id;
  uint8_t ext;
  uint8_t dlc;
  uint8_t data[8];
  uint32_t count;
  uint32_t countAtTable;
  bool announced;
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
  // 受信タスクが更新する(mux で保護)
  portMUX_TYPE mux;
  uint32_t total;
  IdEntry ids[MAX_IDS];
  int nIds;
  uint32_t idOverflow;
  // loop() 側だけが使う
  uint32_t prevTotal;
  uint32_t prevBusErr;
  uint32_t noneSecs;
  BusState state;
};

static Bus g_bus[2] = {
  { "C-CAN", "D0", 0, C_RX, C_TX, 500, false, nullptr, false, ESP_OK, ESP_OK, portMUX_INITIALIZER_UNLOCKED },
  { "B-CAN", "D2", 1, B_RX, B_TX, 50, true, nullptr, false, ESP_OK, ESP_OK, portMUX_INITIALIZER_UNLOCKED },
};

static uint32_t g_lastTick = 0;
static uint32_t g_lastTable = 0;
static uint32_t g_tableBase = 0;
static uint32_t g_passStreak = 0;
static bool g_passShown = false;

// 関数はすべて型の定義より後に置く(Arduino が先頭に関数の宣言を自動で入れるため)
static const char *stateName(BusState s) {
  switch (s) {
    case ST_OK: return "OK  ";
    case ST_NONE: return "NONE";
    case ST_ERR: return "ERR ";
    case ST_INIT_FAIL: return "FAIL";
    default: return "WAIT";
  }
}

// ---------------------------------------------------------------- LED
static int g_ledCode = 8;  // 8 起動直後(点灯) / 0 両方OK / 1..5 点滅回数 / 9 初期化失敗
static uint32_t g_ledStart = 0;

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
    uint32_t blinks = (uint32_t)g_ledCode * 400;  // 1 回 = 150 ms 点灯 + 250 ms 消灯
    uint32_t t = (now - g_ledStart) % (blinks + 1500);
    on = (t < blinks) && ((t % 400) < 150);
  }
  digitalWrite(LED_PIN, on ? LED_ON : LED_OFF);
}

// ---------------------------------------------------------------- 受信
static void rxTask(void *arg) {
  Bus *b = (Bus *)arg;
  twai_message_t m;
  for (;;) {
    if (twai_receive_v2(b->h, &m, pdMS_TO_TICKS(200)) != ESP_OK) {
      continue;
    }
    uint8_t dlc = m.data_length_code > 8 ? 8 : m.data_length_code;
    portENTER_CRITICAL(&b->mux);
    b->total++;
    int i = 0;
    while (i < b->nIds && !(b->ids[i].id == m.identifier && b->ids[i].ext == m.extd)) {
      i++;
    }
    if (i == b->nIds) {
      if (b->nIds < MAX_IDS) {
        IdEntry &n = b->ids[b->nIds++];
        n.id = m.identifier;
        n.ext = m.extd;
        n.count = 0;
        n.countAtTable = 0;
        n.announced = false;
      } else {
        b->idOverflow++;
        i = -1;
      }
    }
    if (i >= 0) {
      IdEntry &e = b->ids[i];
      e.dlc = dlc;
      memset(e.data, 0, sizeof(e.data));
      if (!m.rtr) {
        memcpy(e.data, m.data, dlc);
      }
      e.count++;
    }
    portEXIT_CRITICAL(&b->mux);
  }
}

static bool startBus(Bus &b) {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT_V2(b.controller, b.tx, b.rx, TWAI_MODE_LISTEN_ONLY);
  g.tx_queue_len = 0;  // 送信しない
  g.rx_queue_len = RX_QUEUE_LEN;
  twai_timing_config_t t500 = TWAI_TIMING_CONFIG_500KBITS();
  twai_timing_config_t t50 = TWAI_TIMING_CONFIG_50KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  b.installErr = twai_driver_install_v2(&g, b.kbps == 500 ? &t500 : &t50, &f, &b.h);
  if (b.installErr != ESP_OK) {
    return false;
  }
  b.startErr = twai_start_v2(b.h);
  if (b.startErr != ESP_OK) {
    return false;
  }
  gpio_pullup_en(b.rx);  // トランシーバーが駆動していないときは H(レセッシブ)に見せる
  char taskName[12];
  snprintf(taskName, sizeof(taskName), "rx_%s", b.isB ? "B" : "C");
  xTaskCreate(rxTask, taskName, 4096, &b, 5, nullptr);
  return true;
}

// RX ピンを 20 ms 見て、最後のレベルとエッジ数を返す(受信なしのときの切り分け用)
static void samplePin(gpio_num_t pin, int &level, int &edges) {
  int last = gpio_get_level(pin);
  int e = 0;
  for (int i = 0; i < 1000; i++) {
    delayMicroseconds(20);
    int v = gpio_get_level(pin);
    if (v != last) {
      e++;
      last = v;
    }
  }
  level = last;
  edges = e;
}

static const char *hintFor(const Bus &b, int level, int edges, bool framesSeen) {
  if (framesSeen) {
    return b.isB ? "受信しながらエラーが増えている(配線・撚り・120Ω の外し忘れ・1/9 番の向きを確認)"
                 : "受信しながらエラーが増えている(配線・撚り・120Ω の外し忘れを確認)";
  }
  if (edges > 0) {
    return b.isB ? "信号は来ているが復号できない → 1 番と 9 番を入れ替えて試す"
                 : "信号は来ているが復号できない(ビットレートか CANH/CANL の向き)";
  }
  if (level == 1) {
    return b.isB ? "バスに通信なし(IG OFF、または 1/9 番が届いていない・端子が無い)"
                 : "バスに通信なし(IG OFF、または 6/14 番が届いていない)";
  }
  return b.isB ? "CRX が L のまま(トランシーバーの電源・CTX の 3.3V 固定・配線、または 1/9 番が逆)"
               : "CRX が L のまま(トランシーバーの電源・CTX の 3.3V 固定・配線を確認)";
}

// ---------------------------------------------------------------- 表示
static void printNewIds(uint32_t now) {
  int printed = 0;
  for (int k = 0; k < 2 && printed < NEWID_PER_TICK; k++) {
    Bus &b = g_bus[k];
    if (!b.ready) {
      continue;
    }
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
      if (!found) {
        break;
      }
      Serial.printf("NEWID t=%.1fs %s %08lX %s len%u", now / 1000.0, b.name, (unsigned long)e.id, e.ext ? "29bit" : "11bit", e.dlc);
      for (int j = 0; j < e.dlc; j++) {
        Serial.printf(" %02X", e.data[j]);
      }
      Serial.println();
      if (++printed >= NEWID_PER_TICK) {
        break;
      }
    }
  }
}

static IdEntry g_snap[MAX_IDS];
static float g_snapHz[MAX_IDS];

static void printTables(uint32_t now) {
  float elapsed = (now - g_tableBase) / 1000.0f;
  if (elapsed < 0.5f) {
    elapsed = 0.5f;
  }
  g_tableBase = now;
  for (int k = 0; k < 2; k++) {
    Bus &b = g_bus[k];
    if (!b.ready) {
      continue;
    }
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
    // ID の昇順に並べる
    for (int i = 1; i < n; i++) {
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
      for (int j = 0; j < g_snap[i].dlc; j++) {
        Serial.printf("%02X ", g_snap[i].data[j]);
      }
      Serial.println();
    }
  }
  Serial.println();
}

static void tick(uint32_t now) {
  char seg[2][96];
  const char *hint[2] = { nullptr, nullptr };
  int hintLevel[2] = { 0, 0 };
  int hintEdges[2] = { 0, 0 };
  bool sampled[2] = { false, false };

  for (int k = 0; k < 2; k++) {
    Bus &b = g_bus[k];
    if (!b.ready) {
      b.state = ST_INIT_FAIL;
      snprintf(seg[k], sizeof(seg[k]), "%s FAIL install=%s start=%s", b.name, esp_err_to_name(b.installErr), esp_err_to_name(b.startErr));
      continue;
    }
    uint32_t total;
    int ids;
    portENTER_CRITICAL(&b.mux);
    total = b.total;
    ids = b.nIds;
    portEXIT_CRITICAL(&b.mux);
    twai_status_info_t st;
    memset(&st, 0, sizeof(st));
    twai_get_status_info_v2(b.h, &st);

    uint32_t frames = total - b.prevTotal;
    uint32_t errDelta = st.bus_error_count - b.prevBusErr;
    b.prevTotal = total;
    b.prevBusErr = st.bus_error_count;
    b.noneSecs = frames ? 0 : b.noneSecs + 1;

    if (errDelta > 0 || st.state == TWAI_STATE_BUS_OFF) {
      b.state = ST_ERR;
    } else if (b.noneSecs >= NONE_SECONDS) {
      b.state = ST_NONE;
    } else if (frames > 0) {
      b.state = ST_OK;
    } else {
      b.state = ST_WAIT;
    }

    snprintf(seg[k], sizeof(seg[k]), "%s %s fps=%lu ids=%d err+%lu rec=%lu miss=%lu", b.name, stateName(b.state), (unsigned long)frames, ids,
             (unsigned long)errDelta, (unsigned long)st.rx_error_counter, (unsigned long)(st.rx_missed_count + st.rx_overrun_count));

    if (b.state == ST_NONE || b.state == ST_ERR) {
      if (frames == 0) {
        samplePin(b.rx, hintLevel[k], hintEdges[k]);
        sampled[k] = true;
      }
      hint[k] = hintFor(b, hintLevel[k], hintEdges[k], frames > 0);
    }
  }

  // LED と判定
  BusState c = g_bus[0].state;
  BusState bb = g_bus[1].state;
  int code;
  if (c == ST_INIT_FAIL || bb == ST_INIT_FAIL) {
    code = 9;
  } else if (c == ST_NONE && bb == ST_NONE) {
    code = 3;
  } else if (c == ST_NONE) {
    code = 1;
  } else if (bb == ST_NONE) {
    code = 2;
  } else if (c == ST_ERR) {
    code = 4;
  } else if (bb == ST_ERR) {
    code = 5;
  } else if (c == ST_OK && bb == ST_OK) {
    code = 0;
  } else {
    code = g_ledCode;  // 起動直後など WAIT のときは表示を変えない
  }
  setLedCode(code);

  bool bothOk = (c == ST_OK && bb == ST_OK);
  g_passStreak = bothOk ? g_passStreak + 1 : 0;

  Serial.printf("HB t=%lus | %s | %s | LED=", (unsigned long)(now / 1000), seg[0], seg[1]);
  if (code == 0) {
    Serial.println("ゆっくり点滅(両方OK)");
  } else if (code == 9) {
    Serial.println("速い点滅(初期化失敗)");
  } else if (code == 8) {
    Serial.println("点灯(判定前)");
  } else {
    Serial.printf("%d回点滅\n", code);
  }
  for (int k = 0; k < 2; k++) {
    if (hint[k]) {
      Serial.printf("  %s: ピン「%s」", g_bus[k].name, g_bus[k].rxLabel);
      if (sampled[k]) {
        Serial.printf(" レベル=%s エッジ=%d/20ms", hintLevel[k] ? "H" : "L", hintEdges[k]);
      }
      Serial.printf(" → %s\n", hint[k]);
    }
  }

  if (!g_passShown && g_passStreak >= PASS_SECONDS) {
    g_passShown = true;
    Serial.printf("\n#### VERDICT PASS t=%lus: C-CAN・B-CAN とも「受信あり・エラー増加なし」が %lu 秒続いた ####\n\n", (unsigned long)(now / 1000),
                  (unsigned long)PASS_SECONDS);
  } else if (g_passShown && !bothOk) {
    g_passShown = false;
    Serial.printf("\n#### PASS 状態が途切れた t=%lus(上の HB 行を参照)。再び 10 秒続けば PASS を出し直す ####\n\n", (unsigned long)(now / 1000));
  }

  printNewIds(now);
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_ON);  // 起動の合図(最初の判定まで点灯したまま)

  Serial.setTxBufferSize(8192);
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // Mac が読んでいなくても止まらない(あふれた分は捨てる)
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
    delay(10);
  }

  Serial.println();
  Serial.println("==== can_dual_test Rev.1  C-CAN と B-CAN の同時受信試験 ====");
  Serial.println("C-CAN: 500 kbps  RX=「D0」(GPIO0)  TX=「D1」(GPIO1、配線しない)  LISTEN_ONLY");
  Serial.println("B-CAN:  50 kbps  RX=「D2」(GPIO2)  TX=「D3」(GPIO21、配線しない)  LISTEN_ONLY");
  Serial.printf("判定: 両方とも「受信あり・エラー増加なし」が %lu 秒続けば VERDICT PASS\n", (unsigned long)PASS_SECONDS);
  Serial.println("LED: ゆっくり点滅=両方OK / 1回=C-CAN受信なし 2回=B-CAN受信なし 3回=両方なし 4回=C-CANエラー 5回=B-CANエラー / 速い点滅=初期化失敗");

  for (int k = 0; k < 2; k++) {
    Bus &b = g_bus[k];
    b.ready = startBus(b);
    Serial.printf("%s: install=%s start=%s → %s\n", b.name, esp_err_to_name(b.installErr), esp_err_to_name(b.startErr), b.ready ? "受信開始" : "失敗");
  }
  Serial.println();

  uint32_t now = millis();
  g_lastTick = now;
  g_lastTable = now;
  g_tableBase = now;
}

void loop() {
  uint32_t now = millis();
  ledUpdate(now);
  if (now - g_lastTick >= TICK_MS) {
    g_lastTick += TICK_MS;
    tick(now);
  }
  if (now - g_lastTable >= TABLE_MS) {
    g_lastTable = now;
    printTables(now);
  }
  delay(5);
}

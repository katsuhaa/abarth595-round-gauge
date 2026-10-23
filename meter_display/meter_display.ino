/*
 * meter_display.ino  -  595 メーター表示器  版は config.h の METER_FW_VERSION
 *
 * 目的:
 *   アバルト 595 の純正ブースト計の位置に付ける丸型表示器。XIAO ESP32C6(c6_can_link)から UART で届く CAN の生データから
 *   表示に使う値(回転数・車速・水温・ブースト・燃料・ギアなど)を作り(app.cpp の veh。2.0.0 で c6_can_link から移した)、
 *   ui_spec.md(キャンバス版 23)の画面で表示する。CAN には直接つながない(受信するのは UART だけ)。
 *   燃料のなましの値は、累計時間と同じく TIME_SAVE_MS ごとに保存し、次の起動ではその値から始める。
 *   あわせて RaceChrono DIY の BLE 機器になり、GPS(M100)・CAN の生データ(XIAO から)・IMU(基板の QMI8658)を
 *   1 台でスマホの RaceChrono へ送る(2026-10-14 に追加。仕様は github.com/aollin/racechrono-ble-diy-device)。
 *
 * 配線: 「595表示器 配線図」Rev.0.7 のとおり
 *   8 ピンヘッダ「IO16」 ← XIAO「D6」(UART。速度は XIAO 側で決まり、こちらは自動で合わせる)
 *   8 ピンヘッダ「IO18」 ← M100「TX」(GPS の UBX を受ける)【未確認】配線図の向きと合わせる。逆なら config.h の PIN_GPS_RX/TX を入れ替える
 *   8 ピンヘッダ「IO17」 → M100「RX」(M100 に設定を送る。25 回/秒、NMEA を止めて UBX だけ)
 *   8 ピンヘッダ「GND」  ← XIAO「GND」、M100「GND」
 *   8 ピンヘッダ「VBUS」 ← 5V(机の上で試すときは USB-C だけでよい。「VBUS」と USB-C は基板上で直結)
 *   M100 の電源は配線図のとおり(この版では触れていない)。
 *   ヘッダのピンは、端から数えた位置ではなく基板の印字で確かめること(公式の HARDWARE_REFERENCE.md と回路図で、
 *   ピン番号と信号の対応が食い違っている。回路図では 6 番 = GPIO16、7 番 = GPIO17、8 番 = GPIO18)。
 *
 * 操作(2.1.0): サーキット・タコ・データは、左へスワイプで次の画面、右へスワイプで前の画面(サーキット → タコ → データ → サーキット)。
 *   長押し(0.8 秒。2.2.0)で設定。短いタップは何もしない。設定の中はタップ(上・下の帯と左右の半分)。しきい値は config.h の HOLD_*・SWIPE_*。
 *   電源を入れた直後はオープニング(輪が点く → 自己診断。config.h の OPENING_MS。タップで飛ばせる)。2026-10-21
 *
 * デモ表示: 設定 →「⌄」→「デモ」を「入」にすると、XIAO をつながずに模擬の値(c6_display_debug と同じ場面 1〜9)を出す。
 *   入の間は UART を読まない。画面の下に「SIM」、上に場面番号が出る。設定は保存され、電源を入れ直しても続く。
 *
 * RaceChrono: 設定 → 3 ページ目「RaceChrono」で、BLE の接続・送った数・GPS・IMU の状態が見られる。
 *   スマホの RaceChrono では「設定 → Add other device → Add other device → RaceChrono DIY」で「595 meter」を 1 つ登録する
 *   (GPS 機器の一覧には出ない。GPS も CAN もこの 1 つの登録で届く)。
 *   IMU の値は CAN の 1 つ(PID は config.h の RC_PID_IMU_ACCEL / RC_PID_IMU_GYRO = 0x0618A0F1 / F2)として送る。
 *   RaceChrono の加速度のチャンネルは内部が m/s² なので、式は bytesToIntLe(raw, 0, 2) * 0.00980665(mg → m/s²)。
 *   IMU の取り付け角は自動で覚える(停車 3 秒で「上」、発進の加速で「前」)。覚えた向きは設定と同じ保存領域に残る。
 *   覚える前は「画面を立てて運転者に向け、背面が車の前」(config.h の IMU_DEF_UP / IMU_DEF_FWD)として扱う。
 *   車速が無い机の上でも、立てた向き(45° 以内)で動かさずに 3 秒置けば、その向きを「上」にする(置き直すたびに更新)。
 *   「前」は車で発進したときに覚える。停車中の重力の大きさ(倍率のずれ)も覚え、1 g になるように割る。
 *   設定 → 3 ページ目「RaceChrono 送信」を切にすると、次の起動から BLE・GPS・IMU を使わない(切り分け用)。
 *   落ちる起動(パニック・ウォッチドッグによるリセット)が 2 回続いたら、表示器が自分でこれを切にして動き続ける(シリアルに理由)。
 *   config.h の RC_ENABLE 0 にすると、この機能をコンパイルから外せる(NimBLE-Arduino が無くてもビルドできる)。
 *
 * 書き込んだ直後の見え方(XIAO をつながなくても確かめられる):
 *   走行画面が出て、輪は全部消え、ギアは「–」、水温と燃料は「– –」(信号がないときの表示)。
 *   左右のスワイプで画面が切り替わる。長押しで設定 →「受信の状態」で、受信・描画・起動のきっかけが見られる。
 *   画面が真っ赤 = 画面用のメモリー(PSRAM)を取れなかった。真っ暗のまま = 画面の初期化まで進んでいない(シリアルを見る)。
 *
 * シリアル(USB。USB CDC なのでモニター側の速度の設定は無視される。何 bps でもよい):
 *   起動時に確かめた結果(画面・メモリー・電源 IC・タッチ・設定・UART・BLE・GPS・IMU、最後に「自己診断: OK / NG」)。
 *   同じ内容を 60 秒ごとに出し直すので、シリアルモニターはいつ開いてもよい。
 *   5 秒ごとに描画・受信・RaceChrono の状態を 1 行。タップのたびに座標と画面の移り変わりを 1 行。
 *   CAN の生データから値を作っているとき(c6_can_link 2.0.0)は、5 秒ごとに「車の値(CAN から)」の行(作った値、燃料の生の値・
 *   なましの値、ギアの推定に使った比)。起動時の結果には、燃料を前回の値から始めたかを 1 行。
 *   回転数の点滅(輪が黄になる所(シフト点 − 400)から、近づくほど速く。消灯は 80 ms で一定、点灯を 150 → 50 ms に縮める。
 *   シフト点 + 200 以上は点滅せず明るさ最大。2026-10-21)があった 5 秒は、点灯・消灯の時間と切り替えの遅れを 1 行
 *   (「点滅: 点灯 50 ms / 消灯 80 ms 切り替え 98 回 遅れ 最大 3 ms OK」。遅れが消灯の時間の 2 割を超えた切り替えがあれば NG)。
 *   BLE の接続(間隔などの条件)・切断(理由)・読み出し・購読、RaceChrono が書いたフィルタ、GPS の受信ピンの判定と設定のやり直し、
 *   IMU の向きを覚えたときも 1 行ずつ出す。
 *
 * ファイル:
 *   config.h       仮の値と基板のピン(仕様書の【仮】【未定】はここ)
 *   meter_link.h   UART のデータ形式(c6_display_debug・c6_can_link と同じ内容)
 *   ui.cpp         画面        gfx.cpp  描画
 *   app.cpp        受信データの解釈・車の CAN から値を作る(veh)・CAN の表・全開と走行の時間・GPS の UBX の解析・RaceChrono 向けの符号化・IMU の向きの補正
 *   src/fonts_data.cpp  書体とアイコン(自動生成)
 *
 * ボード: Waveshare ESP32-S3-Touch-AMOLED-1.75(標準版)、arduino-esp32 3.3.11。
 *   追加ライブラリ: NimBLE-Arduino 1.4.3(h2zero。ライブラリマネージャで版を 1.4.3 に選ぶ。2.x は関数の形が違い、コンパイルで止まる)
 *   Arduino IDE のボードは「ESP32S3 Dev Module」。「ツール」で変えるのは次の 2 つだけ(ほかは既定値のまま):
 *     USB CDC On Boot  → Enabled
 *     PSRAM            → OPI PSRAM
 *   どちらも、違っているとコンパイルがエラーで止まる(下の #error)。「USB Mode」が既定値(Hardware CDC and JTAG)で
 *   ないときも同じ。
 *   スケッチは約 1.2MB で、既定の Partition Scheme(1.2MB APP)の 92 %(2.2.0、2026-10-23)。Flash Size も既定値のままで動く。
 *   コンパイルの最後に「Sketch too big」と出るようになったら、Partition Scheme を「Huge APP (3MB No OTA/1MB SPIFFS)」にする
 *   (設定の保存領域はどちらでも同じ場所にあり、保存した値は消えない)。
 *   「コンパイラの警告」を「すべて」にすると NimBLE-Arduino の nimconfig.h から「redefined」の警告がたくさん出るが、
 *   ライブラリ側のもので害はない。このスケッチ自身のファイルからは警告は出ない。
 *
 * 画面とタッチの初期化の手順とピンは、Waveshare 公式リポジトリ(examples/arduino の 01_HelloWorld と
 * 06_LVGL_Widgets、同梱の GFX Library for Arduino 1.6.4 と SensorLib 0.3.1)を読んで同じにしてある。
 * IMU(QMI8658)のレジスタの手順も SensorLib 0.3.1 と同じ。
 */

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>
#include "driver/gpio.h"
#include "hal/gpio_ll.h"  // GPS の送信ピンをオープンドレインにする(gpio_set_direction だと UART の信号の接続が外れるため LL を使う)
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "config.h"
#if RC_ENABLE
#include "esp32-hal-alloc-ble-mem.h"  // arduino-esp32 3.x: 外部の BLE ライブラリを使うときは、これで BLE 用のメモリーを残す
#if __has_include(<NimBLELocalValueAttribute.h>)
#error "NimBLE-Arduino は 1.4.3 を入れてください(ライブラリマネージャで版を 1.4.3 に選ぶ)。入っているのは 2.x で、関数の形が違います。"
#endif
#include <NimBLEDevice.h>
#endif
#include "app.h"
#include "gfx.h"
#include "ui.h"
#include "sim.h"

#if !ARDUINO_USB_CDC_ON_BOOT
#error "「ツール」→「USB CDC On Boot」を Enabled にしてください。Disabled だとログが USB に出ません。"
#endif
#if !ARDUINO_USB_MODE
#error "「ツール」→「USB Mode」を「Hardware CDC and JTAG」(既定値)にしてください。"
#endif
#if !defined(BOARD_HAS_PSRAM) || !defined(CONFIG_SPIRAM_MODE_OCT)
#error "「ツール」→「PSRAM」を「OPI PSRAM」にしてください。この基板の PSRAM(8MB)は OPI 接続で、画面用のメモリーに使います。"
#endif

static SysInfo g_sys = { false, 0, false, false };  // 起動時に確かめた結果(「受信の状態」とシリアルに出す)

// ================================================================ 画面(CO5300、QSPI)
// コマンド: 0x02 + 24 ビット(コマンド << 8)+ データ。全部 1 本の線で送る。
// 画素:     0x32 + 0x003C00 のあと、データだけ 4 本の線で送る(続きは CS を下げたまま、データだけ)。
static const size_t LCD_CHUNK_BYTES = 16384;  // 1 回に送る画素データ(DMA 用の内部メモリー)
static spi_device_handle_t g_spi = nullptr;
static spi_transaction_ext_t g_tr;
static uint8_t *g_dma = nullptr;
static bool g_lcdOk = false;
static esp_err_t g_spiErr = ESP_OK;  // SPI の送信で最初に出たエラー(画面からは何も読み返せないので、分かるのはこれだけ)

static inline void lcdTransmit() {
  esp_err_t e = spi_device_polling_transmit(g_spi, &g_tr.base);
  if (e != ESP_OK && g_spiErr == ESP_OK) g_spiErr = e;
}

static inline void lcdCs(int level) {
  gpio_set_level((gpio_num_t)PIN_LCD_CS, level);
}

static void lcdCmd(uint8_t cmd, const uint8_t *data = nullptr, int len = 0) {
  memset(&g_tr, 0, sizeof(g_tr));
  g_tr.base.flags = SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR | (len ? SPI_TRANS_USE_TXDATA : 0);
  g_tr.base.cmd = 0x02;
  g_tr.base.addr = ((uint32_t)cmd) << 8;
  for (int i = 0; i < len && i < 4; i++) g_tr.base.tx_data[i] = data[i];
  g_tr.base.length = (size_t)len * 8;
  lcdCs(0);
  lcdTransmit();
  lcdCs(1);
}
static void lcdCmd1(uint8_t cmd, uint8_t v) {
  lcdCmd(cmd, &v, 1);
}

static void lcdSetWindow(int x0, int y0, int w, int h) {
  int xa = x0 + LCD_COL_OFFSET, xb = x0 + w - 1 + LCD_COL_OFFSET, yb = y0 + h - 1;
  uint8_t cx[4] = { (uint8_t)(xa >> 8), (uint8_t)xa, (uint8_t)(xb >> 8), (uint8_t)xb };
  uint8_t cy[4] = { (uint8_t)(y0 >> 8), (uint8_t)y0, (uint8_t)(yb >> 8), (uint8_t)yb };
  lcdCmd(0x2A, cx, 4);
  lcdCmd(0x2B, cy, 4);
  lcdCmd(0x2C);
}

// g_dma にある n バイトを画素データとして送る(first = その範囲の最初の 1 回)
static void lcdSendChunk(size_t n, bool first) {
  memset(&g_tr, 0, sizeof(g_tr));
  if (first) {
    g_tr.base.flags = SPI_TRANS_MODE_QIO;
    g_tr.base.cmd = 0x32;
    g_tr.base.addr = 0x003C00;
  } else {
    g_tr.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
  }
  g_tr.base.tx_buffer = g_dma;
  g_tr.base.length = n * 8;
  lcdTransmit();
}

static void lcdBrightness(int pct);
static Signals g_sig;              // いまの loop() の値(点滅の時刻の管理に使う)
static bool g_ready = false;       // setup() が通った(画面とメモリーが使える)
static bool g_firstFrame = true;   // 最初の絵をまだ送り終えていない(その間は暗いまま)
static uint8_t g_level = BL_NORMAL;  // 画面全体の明るさの段階(ui::blinkPoll → platBlink で変わる): 設定の値 / 0(点滅の消灯) / 100 %
static bool g_solidShown = false;  // BLINK_BY_BRIGHTNESS 0 のとき: 消灯の一色を画面に送ってある

// 画面メモリーの (x0, y0) から w×h を画面へ送る。x0・y0・w・h は偶数(このパネルの決まり)。
// 画面全体を送るのに 35 ms ほどかかるので、16 KB 送るごとに点滅の切り替えの時刻を見る。切り替えるときは画素の送信を
// いったん閉じ(CS を上げる)、明るさのコマンドを送ってから、続きを 3Ch(前の続きから書く)で再開する
static void lcdPush(const uint16_t *fb, int x0, int y0, int w, int h) {
  lcdSetWindow(x0, y0, w, h);
  lcdCs(0);
  uint16_t *out = (uint16_t *)g_dma;
  size_t fill = 0;
  bool first = true;
  const size_t maxPx = LCD_CHUNK_BYTES / 2;
  for (int y = y0; y < y0 + h; y++) {
    const uint16_t *src = fb + y * gfx::W + x0;
    int left = w;
    while (left > 0) {
      size_t n = maxPx - fill;
      if (n > (size_t)left) n = (size_t)left;
      for (size_t i = 0; i < n; i++) out[fill + i] = __builtin_bswap16(src[i]);  // 上位バイトを先に送る
      fill += n;
      src += n;
      left -= (int)n;
      if (fill == maxPx) {
        lcdSendChunk(fill * 2, first);
        first = false;
        fill = 0;
        if (ui::blinkDue(millis())) {
          lcdCs(1);
          ui::blinkPoll(g_sig, millis());  // 切り替わると platBlink → 明るさのコマンド
          lcdCs(0);
          first = true;
        }
      }
    }
  }
  if (fill) lcdSendChunk(fill * 2, first);
  lcdCs(1);
}

// 画面メモリーなしで全面を 1 色にする(メモリーを取れなかったときの知らせ用)
static void lcdFillSolid(uint16_t color) {
  lcdSetWindow(0, 0, gfx::W, gfx::H);
  lcdCs(0);
  uint16_t *out = (uint16_t *)g_dma;
  const size_t maxPx = LCD_CHUNK_BYTES / 2;
  for (size_t i = 0; i < maxPx; i++) out[i] = __builtin_bswap16(color);
  size_t left = (size_t)gfx::W * gfx::H;
  bool first = true;
  while (left) {
    size_t n = left < maxPx ? left : maxPx;
    lcdSendChunk(n * 2, first);
    first = false;
    left -= n;
  }
  lcdCs(1);
}

static void lcdBrightness(int pct) {
  if (!g_lcdOk) return;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  lcdCmd1(0x51, (uint8_t)(pct * 255 / 100));
}

static bool lcdBegin() {
  pinMode(PIN_LCD_CS, OUTPUT);
  digitalWrite(PIN_LCD_CS, HIGH);

  spi_bus_config_t bus = {};
  bus.data0_io_num = PIN_LCD_D0;
  bus.data1_io_num = PIN_LCD_D1;
  bus.sclk_io_num = PIN_LCD_SCLK;
  bus.data2_io_num = PIN_LCD_D2;
  bus.data3_io_num = PIN_LCD_D3;
  bus.data4_io_num = -1;
  bus.data5_io_num = -1;
  bus.data6_io_num = -1;
  bus.data7_io_num = -1;
  bus.max_transfer_sz = LCD_CHUNK_BYTES + 8;
  bus.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS;
  if (spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) return false;

  spi_device_interface_config_t dev = {};
  dev.command_bits = 8;
  dev.address_bits = 24;
  dev.mode = 0;
  dev.clock_source = SPI_CLK_SRC_DEFAULT;
  dev.clock_speed_hz = LCD_SPI_HZ;
  dev.spics_io_num = -1;  // CS は自分で動かす(続きのデータを CS を下げたまま送るため)
  dev.flags = SPI_DEVICE_HALFDUPLEX;
  dev.queue_size = 1;
  if (spi_bus_add_device(SPI2_HOST, &dev, &g_spi) != ESP_OK) return false;
  spi_device_acquire_bus(g_spi, portMAX_DELAY);

  g_dma = (uint8_t *)heap_caps_aligned_alloc(16, LCD_CHUNK_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (!g_dma) return false;

  // リセット(公式の例と同じ長さ)
  pinMode(PIN_LCD_RST, OUTPUT);
  digitalWrite(PIN_LCD_RST, HIGH);
  delay(10);
  digitalWrite(PIN_LCD_RST, LOW);
  delay(200);
  digitalWrite(PIN_LCD_RST, HIGH);
  delay(200);

  // コマンドの並びは、公式の Arduino の例が使う GFX Library for Arduino の CO5300 と同じ。
  // (公式の ESP-IDF 用 BSP は、このほかに 0xFE=0x20、0x19=0x10、0x1C=0xA0、0x35=0x00 も送っている。
  //   Arduino の例はそれ無しで動いているので、ここでは送っていない)
  lcdCmd(0x11);  // スリープ解除
  delay(120);
  lcdCmd1(0xFE, 0x00);
  lcdCmd1(0xC4, 0x80);  // SPI の動作モード
  lcdCmd1(0x3A, 0x55);  // 1 画素 16 ビット
  lcdCmd1(0x53, 0x20);
  lcdCmd1(0x63, 0xFF);
  lcdCmd1(0x51, 0x00);  // 最初の絵を送り終えるまで暗くしておく
  lcdCmd(0x29);         // 表示 ON
  lcdCmd1(0x51, 0x00);
  lcdCmd1(0x58, 0x00);  // コントラスト強調なし
  delay(10);
  lcdCmd(0x20);         // 反転なし
  lcdCmd1(0x36, 0x00);  // 色の並び RGB、向きはそのまま
  return true;
}

static const char *screenName(int sc) {
  static const char *const NAMES[] = { "サーキット", "タコ", "データ", "CAN確認", "CAN確認の詳細", "設定", "設定の変更", "受信の状態", "RaceChrono の状態", "オープニング",
                                       "CAN確認の一覧", "CAN確認のやり直す", "時間の記録のリセット" };
  return (sc >= 0 && sc < (int)(sizeof(NAMES) / sizeof(NAMES[0]))) ? NAMES[sc] : "?";
}

// ================================================================ タッチ(CST9217、I2C 0x5A)
// 公式の例(10_Touch_CST9217、06_LVGL_Widgets)と同じく、リセットだけで使う(設定の書き込みはしない)。
// 「INT」は知らせのたびに短く Low になる。立ち下がりを割り込みで覚えておき、loop() で読む。
static const uint8_t TP_ADDR = 0x5A;
static const uint32_t TAP_MIN_GAP_MS = 50;  // 離してからこの時間以内にまた触れた知らせは、同じタップの続きとして扱う
// 押している間の知らせがこの時間とぎれたら、離したとみなす(「離した」の知らせを取りこぼしたときの保険)。
// 10/23: 長押し(config.h の HOLD_MS)を使うので 2000 → 250 ms。【未確認】CST9217 が押している間どの間隔で知らせを出し続けるか。
// 離したときにシリアルへ「押している間の知らせの間隔 最大 n ms」を出すので、それがこの値より大きければ長押しが効かない(ここを広げる)
static const uint32_t TP_HOLD_GAP_MS = 250;
static volatile bool g_tpIrq = false;
static bool g_tpDown = false;
static uint32_t g_tpLastMs = 0;
static uint32_t g_tpDownMs = 0, g_tpGapMax = 0, g_tpReports = 0;
static int g_tpX0 = 0, g_tpY0 = 0, g_tpX = 0, g_tpY = 0;  // 触れた所と、最後に知らされた所(シリアルにスワイプの動きを出す)
static bool g_tpI2cOk = false;              // 直近の読み取りで、タッチ IC が I2C に応答した

enum { TP_NONE, TP_DOWN, TP_UP };  // touchRead の戻り値。TP_NONE = 読めなかった、または中身が正しくない

// I2C は loop()(タッチ)と rc タスク(IMU)の両方から使う。Wire は 1 回の転送ごとに守るだけなので、
// 「書いてから読む」のような数回の転送をまとめてこれで守る
static SemaphoreHandle_t g_i2c = nullptr;
static inline void i2cLock() {
  if (g_i2c) xSemaphoreTake(g_i2c, portMAX_DELAY);
}
static inline void i2cUnlock() {
  if (g_i2c) xSemaphoreGive(g_i2c);
}

static void IRAM_ATTR tpIsr() {
  g_tpIrq = true;
}

static bool touchBegin() {
  pinMode(PIN_TP_RST, OUTPUT);
  digitalWrite(PIN_TP_RST, LOW);
  delay(10);
  digitalWrite(PIN_TP_RST, HIGH);
  delay(100);
  pinMode(PIN_TP_INT, INPUT_PULLUP);
  // 応答の確認は、ふだんの読み取りと同じ手順で行う(公式のドライバーが使っている手順だけを使う)
  for (int i = 0; i < 3 && !g_tpI2cOk; i++) {
    if (i) delay(50);
    int x, y;
    touchRead(&x, &y);
  }
  attachInterrupt(digitalPinToInterrupt(PIN_TP_INT), tpIsr, FALLING);
  return g_tpI2cOk;
}

// 知らせを 1 つ読む。触れていれば TP_DOWN と画面の座標、離したなら TP_UP
static int touchReadRaw(int *x, int *y) {
  uint8_t b[15] = { 0 };
  g_tpI2cOk = false;
  Wire.beginTransmission(TP_ADDR);
  Wire.write(0xD0);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return TP_NONE;
  if (Wire.requestFrom(TP_ADDR, (uint8_t)sizeof(b)) != sizeof(b)) return TP_NONE;
  Wire.readBytes(b, sizeof(b));
  g_tpI2cOk = true;
  // 2.0.1: 読んだあとに D0 00 AB(読み取り済みの知らせ)を書くのをやめた。書くと報告の先頭が 0xAB になり、次の知らせの前に
  // もう一度読むと b[0] = 0xAB(状態 0x0B ≠ 0x06)で返って、下の判定で「離した」になっていた(長押しが途中で切れてタップになる)。
  // ESPHome #21010(CST9220)と同じ症状で、PR #21034 も書くのをやめて直した。Waveshare の ESP-IDF ドライバー
  // (esp_lcd_touch_cst9217)も読むだけで書かない。書かなければ、読み直しても同じ知らせ(触れている)が返る
  if (b[6] != 0xAB) return TP_NONE;
  uint8_t n = b[5] & 0x7F;
  if (n > 2) return TP_NONE;
  if (n == 0 || (b[0] & 0x0F) != 0x06) return TP_UP;  // 0x06 = 触れている
  if ((b[0] >> 4) >= 2) return TP_NONE;               // 指の番号は 0 か 1(公式のドライバーと同じ確認)
  int rx = (b[1] << 4) | (b[3] >> 4);
  int ry = (b[2] << 4) | (b[3] & 0x0F);
  int sx = gfx::W - rx, sy = gfx::H - ry;  // 公式の例(setMirrorXY(true, true))と同じく、X・Y とも反転
  *x = sx < 0 ? 0 : (sx >= gfx::W ? gfx::W - 1 : sx);
  *y = sy < 0 ? 0 : (sy >= gfx::H ? gfx::H - 1 : sy);
  return TP_DOWN;
}
static int touchRead(int *x, int *y) {
  i2cLock();
  int r = touchReadRaw(x, y);
  i2cUnlock();
  return r;
}

// 触れた瞬間に ui::touchDown、押している間の知らせごとに ui::touchMove、離したら ui::touchUp
// (サーキット・タコ・データは、左右のスワイプで画面を替え、長押しで設定。ほかの画面は離したときにタップ)
static void touchRelease(uint32_t now, const char *why) {
  int before = (int)ui::screen();
  g_tpDown = false;
  ui::touchUp(now);
  Serial.printf("タッチ: 離した(%s)%lu ms 押していた、動き 横 %+d 縦 %+d、知らせ %lu 回、押している間の知らせの間隔 最大 %lu ms  画面: %s → %s\n", why,
                (unsigned long)(now - g_tpDownMs), g_tpX - g_tpX0, g_tpY - g_tpY0, (unsigned long)g_tpReports, (unsigned long)g_tpGapMax, screenName(before),
                screenName((int)ui::screen()));
}
static void touchPoll(uint32_t now) {
  if (g_tpDown && (uint32_t)(now - g_tpLastMs) >= TP_HOLD_GAP_MS) touchRelease(now, "知らせがとぎれた");
  if (!g_tpIrq && digitalRead(PIN_TP_INT) != LOW) return;
  g_tpIrq = false;
  int x = 0, y = 0;
  int st = touchRead(&x, &y);
  if (g_tpI2cOk) g_sys.touchOk = true;  // 起動時に応答が無くても、あとで読めたら OK に直す
  if (st == TP_UP) {
    if (g_tpDown) touchRelease(now, "離した知らせ");
  } else if (st == TP_DOWN) {
    if (!g_tpDown) {
      if (g_tpLastMs != 0 && (uint32_t)(now - g_tpLastMs) < TAP_MIN_GAP_MS) return;  // 離した直後の続き
      g_tpDown = true;
      g_tpDownMs = now;
      g_tpGapMax = 0;
      g_tpReports = 1;
      g_tpX0 = g_tpX = x;
      g_tpY0 = g_tpY = y;
      ui::touchDown(x, y, now);
      Serial.printf("タッチ: 触れた (%d, %d)  画面: %s\n", x, y, screenName((int)ui::screen()));
    } else {
      uint32_t gap = now - g_tpLastMs;
      if (gap > g_tpGapMax) g_tpGapMax = gap;
      g_tpReports++;
      g_tpX = x;
      g_tpY = y;
      ui::touchMove(x, y, now);  // スワイプの判定と、動いたら長押しを取り消すため
    }
    g_tpLastMs = now ? now : 1;
  }
}

// ================================================================ plat(app.h)
static SemaphoreHandle_t g_mutex = nullptr;
static Preferences g_prefs;
static Settings g_set = { SET_SHIFT_DEFAULT, SET_AUTORET_DEFAULT, SET_BRIGHT_DEFAULT, SET_WARN_DEFAULT, SET_DEMO_DEFAULT, SET_RC_DEFAULT };
static PerfStats g_perf = { 0, 0, 0, 0, 0 };

void platLock() {
  if (g_mutex) xSemaphoreTake(g_mutex, portMAX_DELAY);
}
void platUnlock() {
  if (g_mutex) xSemaphoreGive(g_mutex);
}
void *platAlloc(size_t bytes) {
  void *p = heap_caps_calloc(1, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!p) Serial.printf("メモリーを取れない: %u バイト\n", (unsigned)bytes);
  return p;
}
// 明るさの段階 → パネルに送る明るさ [%]
static int levelPct(uint8_t level, int normalPct) {
  return level == BL_OFF ? 0 : level == BL_MAX ? 100 : normalPct;
}
void platSetBrightness(int pct) {
  lcdBrightness(levelPct(g_level, pct));  // 点滅の消灯の間は 0、最大輝度の間は 100 のまま
}
// ui::blinkPoll から: 画面全体の明るさの段階が変わった(点滅の消灯・点灯、シフト点 + RING_SOLID_OFFSET 以上の最大輝度)
void platBlink(uint8_t level) {
  g_level = level;
  if (g_firstFrame) return;  // 最初の絵を送り終えるまでは暗いまま(送り終えたときに合わせる)
  if (!BLINK_BY_BRIGHTNESS && level == BL_OFF) return;  // 一色を送る方式の消灯は loop() で扱う(明るさはそのまま)
  lcdBrightness(levelPct(level, g_set.brightnessPct));
}
// ui の長い描画の合間から: 点滅の切り替えの時刻を見る(SPI は送信中でないので、そのままコマンドを送れる)
void platBlinkTick() {
  if (g_ready) ui::blinkPoll(g_sig, millis());
}

static int clampInt(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
static uint8_t g_homeLoaded = ui::SC_DRIVE;  // NVS の覚えた画面
static uint32_t g_timesSavedA = 0, g_timesSavedB = 0, g_timesSaveMs = 0;
static int g_fuelSaved = 0;  // NVS の燃料のなましの値(生の値の 10 倍。0 = 無い)
static void loadSettings() {
  g_sys.nvsOk = g_prefs.begin("meter", false);
  if (!g_sys.nvsOk) return;
  g_homeLoaded = (uint8_t)g_prefs.getUChar("home", ui::SC_DRIVE);
  if (g_homeLoaded != ui::SC_DRIVE && g_homeLoaded != ui::SC_BOOST && g_homeLoaded != ui::SC_DATA) g_homeLoaded = ui::SC_DRIVE;
  g_timesSavedA = g_prefs.getULong("tabove", 0);
  g_timesSavedB = g_prefs.getULong("tbelow", 0);
  trip::setTimes(g_timesSavedA, g_timesSavedB);
  g_fuelSaved = g_prefs.getInt("fuel", 0);  // 燃料は前回の値から始める(veh::begin のあとに呼ぶこと)
  if (g_fuelSaved <= 0) g_fuelSaved = 0;
  veh::fuelRestore(g_fuelSaved);
  g_set.shiftRpm = clampInt(g_prefs.getInt("shift", SET_SHIFT_DEFAULT), SET_SHIFT_MIN, SET_SHIFT_MAX);
  g_set.autoReturnMin = clampInt(g_prefs.getInt("autoret", SET_AUTORET_DEFAULT), SET_AUTORET_MIN, SET_AUTORET_MAX);
  g_set.brightnessPct = clampInt(g_prefs.getInt("bright", SET_BRIGHT_DEFAULT), SET_BRIGHT_MIN, SET_BRIGHT_MAX);
  g_set.warnTempC = clampInt(g_prefs.getInt("warn", SET_WARN_DEFAULT), SET_WARN_MIN, SET_WARN_MAX);
  g_set.demoOn = clampInt(g_prefs.getInt("demo", SET_DEMO_DEFAULT), 0, 1);
  g_set.rcOn = clampInt(g_prefs.getInt("rc", SET_RC_DEFAULT), 0, 1);
}
// 保存して読み返し、合っていれば true
bool platSaveSettings(const Settings &s) {
  if (!g_sys.nvsOk) return false;
  // フラッシュへ書いている間は UART の受信が遅れて数バイト落ちることがある。その間の誤りは線の品質の数字に入れない
  mlink::graceUntil(millis() + 5000);
  g_prefs.putInt("shift", s.shiftRpm);
  g_prefs.putInt("autoret", s.autoReturnMin);
  g_prefs.putInt("bright", s.brightnessPct);
  g_prefs.putInt("warn", s.warnTempC);
  g_prefs.putInt("demo", s.demoOn);
  g_prefs.putInt("rc", s.rcOn);
  bool ok = g_prefs.getInt("shift", -1) == s.shiftRpm && g_prefs.getInt("autoret", -1) == s.autoReturnMin && g_prefs.getInt("bright", -1) == s.brightnessPct
            && g_prefs.getInt("warn", -1) == s.warnTempC && g_prefs.getInt("demo", -1) == s.demoOn && g_prefs.getInt("rc", -1) == s.rcOn;
  mlink::graceUntil(millis() + 200);
  Serial.printf("設定の保存: %s(シフト点 %d、自動で戻る %d 分、明るさ %d %%、警告温度 %d ℃、デモ %s、RaceChrono 送信 %s%s)\n", ok ? "OK" : "失敗", s.shiftRpm, s.autoReturnMin,
                s.brightnessPct, s.warnTempC, s.demoOn ? "入" : "切", s.rcOn ? "入" : "切", s.rcOn != g_sys.rc.enabled ? "(次の起動から)" : "");
  return ok;
}

// 3000rpm 超・以下の累計時間、覚えた画面、燃料のなましの値を NVS に書く(変わったものだけ)。force = 時間を待たずに今すぐ
static void saveTimes(uint32_t now, bool force) {
  if (!g_sys.nvsOk) return;
  if (!force && (uint32_t)(now - g_timesSaveMs) < TIME_SAVE_MS) return;
  g_timesSaveMs = now;
  uint32_t a = trip::aboveMs(), b = trip::belowMs();
  uint8_t home = ui::homeScreen();
  int fuel = 0;
  bool wt = a != g_timesSavedA || b != g_timesSavedB, wh = home != g_homeLoaded;
  bool wf = veh::fuelSaveValue(&fuel) && fuel != g_fuelSaved;  // 0.1 L 単位で変わったとき
  if (!wt && !wh && !wf) return;
  mlink::graceUntil(millis() + 5000);  // フラッシュへ書く間の UART の取りこぼしを線の品質に入れない
  if (wt) {
    g_prefs.putULong("tabove", a);
    g_prefs.putULong("tbelow", b);
    g_timesSavedA = a;
    g_timesSavedB = b;
  }
  if (wh) {
    g_prefs.putUChar("home", home);
    g_homeLoaded = home;
    Serial.printf("覚えた画面を保存: %s\n", screenName(home));
  }
  if (wf) {
    g_prefs.putInt("fuel", fuel);
    g_fuelSaved = fuel;
  }
  mlink::graceUntil(millis() + 200);
}
void platSaveTimes() {
  saveTimes(millis(), true);
}

// ================================================================ 受信(UART、別タスク)
static const uart_port_t LINK_UART = UART_NUM_1;

// デモ表示: c6_display_debug と同じ模擬データ(sim.cpp)を表示器の中で作り、UART で届いたのと同じ形で受信処理へ渡す
static uint8_t g_demoSeq = 0;
static void demoFeed(const void *payload, uint8_t type, uint8_t len, uint32_t now) {
  uint8_t f[ML_MAX_PAYLOAD + ML_OVERHEAD];
  size_t n = mlBuild(f, type, g_demoSeq++, payload, len);
  mlink::feed(f, n, now);
}
static void demoStep(uint32_t now) {
  static uint32_t lastSig = 0, lastInfo = 0;
  sim::step(now, false);
  if (!sim::linkOn()) return;  // 場面 9 は送信を止める(信号がないときの表示)
  if (now - lastSig >= 20) {
    lastSig = now;
    MlSignals s;
    sim::signals(&s, now);
    demoFeed(&s, ML_TYPE_SIGNALS, sizeof(s), now);
  }
  MlCanHead head;
  uint8_t data[8];
  for (int i = 0; i < 8 && sim::canPoll(now, &head, data); i++) {
    uint8_t p[sizeof(MlCanHead) + 8];
    uint8_t dlc = head.info & 0x0F;
    memcpy(p, &head, sizeof(head));
    memcpy(p + sizeof(head), data, dlc);
    demoFeed(p, ML_TYPE_CAN, (uint8_t)(sizeof(head) + dlc), now);
  }
  if (now - lastInfo >= 1000) {
    lastInfo = now;
    MlInfo info = {};
    info.proto_ver = METER_LINK_PROTO_VER;
    info.flags = ML_F_SIM;
    uint16_t fps[2], ids[2];
    sim::canRates(fps, ids);
    memcpy(info.can_fps, fps, sizeof(fps));
    memcpy(info.can_ids, ids, sizeof(ids));
    info.uptime_s = now / 1000;
    memcpy(info.name, "demo", 5);
    demoFeed(&info, ML_TYPE_INFO, sizeof(info), now);
  }
}

static void linkTask(void *) {
  static uint8_t buf[1024];
  bool demoPrev = false;
  sim::begin();
  for (;;) {
    bool demo = g_set.demoOn != 0;
    if (demo != demoPrev) {  // 受信元が変わった
      demoPrev = demo;
      mlink::resyncSeq();
      if (demo) sim::begin();
      uart_flush_input(LINK_UART);
      Serial.printf("デモ表示: %s\n", demo ? "入(UART は使わない)" : "切(UART を受信する)");
    }
    if (demo) {
      uart_flush_input(LINK_UART);  // XIAO がつながっていても読まない
      uint32_t now = millis();
      demoStep(now);
      mlink::tick(now);
      cantab::tick(now);
      vTaskDelay(1);
      continue;
    }
    // uart_read_bytes は「頼んだ長さがそろうか、待ち時間のあいだ何も来ない」まで戻らない。
    // 溜まっているぶんだけを待たずに読み、何も無いときだけ 1 バイトを 5 ms まで待つ(値の遅れを作らない)。
    size_t avail = 0;
    uart_get_buffered_data_len(LINK_UART, &avail);
    if (avail > sizeof(buf)) avail = sizeof(buf);
    int n = uart_read_bytes(LINK_UART, buf, avail ? avail : 1, avail ? 0 : pdMS_TO_TICKS(5));
    if (n < 0) {
      vTaskDelay(1);
      n = 0;
    }
    uint32_t now = millis();
    if (n > 0) mlink::feed(buf, (size_t)n, now);
    mlink::tick(now);
    cantab::tick(now);
    uint32_t next = mlink::autoBaud(now, n > 0 ? (size_t)n : 0);  // 速度が合っていないと判断したら、次の候補が返る
    if (next) {
      uart_set_baudrate(LINK_UART, next);
      uart_flush_input(LINK_UART);
    }
  }
}

static bool linkBegin() {
  uart_config_t cfg = {};
  cfg.baud_rate = (int)mlink::baud();
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  if (uart_driver_install(LINK_UART, 16384, 0, 0, nullptr, 0) != ESP_OK) return false;
  if (uart_param_config(LINK_UART, &cfg) != ESP_OK) return false;
  if (uart_set_pin(LINK_UART, UART_PIN_NO_CHANGE, PIN_LINK_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return false;
  gpio_pullup_en((gpio_num_t)PIN_LINK_RX);  // 線がつながっていないときは H(何も来ていない状態)に見せる
  return xTaskCreatePinnedToCore(linkTask, "link", 4096, nullptr, 5, nullptr, 0) == pdPASS;
}

// ================================================================ RaceChrono へ送る(BLE・GPS・IMU)
// config.h の RC_ENABLE 0 でこの部分ごと外せる。設定「RaceChrono 送信」が切のときは、コンパイルはするが動かさない。
// 落ちる起動が RC_CRASH_LIMIT 回続いたら、表示器が自分で設定を切にする(rcCrashCheck)。
static bool g_rcOk = false, g_rcAutoOff = false;
static uint32_t g_crashCount = 0;
static bool g_crashCleared = false;
static esp_reset_reason_t g_resetReason = ESP_RST_UNKNOWN;

static const char *resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "電源が入った";
    case ESP_RST_SW: return "ソフトウェアのリセット(書き込み後など)";
    case ESP_RST_PANIC: return "パニック(プログラムの異常)";
    case ESP_RST_INT_WDT: return "割り込みのウォッチドッグ";
    case ESP_RST_TASK_WDT: return "タスクのウォッチドッグ";
    case ESP_RST_WDT: return "ウォッチドッグ";
    case ESP_RST_BROWNOUT: return "電圧の低下";
    case ESP_RST_EXT: return "外部のリセット";
    default: return "そのほか";
  }
}
static bool resetWasCrash(esp_reset_reason_t r) {
  return r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT;
}

// 起動時: 前回のリセットが「落ちた」ものなら数える。RC_CRASH_LIMIT 回続いたら RaceChrono の送信を切にする
static void rcCrashCheck() {
  g_resetReason = esp_reset_reason();
  if (!g_sys.nvsOk) return;
  g_crashCount = (uint32_t)g_prefs.getInt("rccrash", 0);
  if (!resetWasCrash(g_resetReason) || !g_set.rcOn) return;
  g_crashCount++;
  g_prefs.putInt("rccrash", (int)g_crashCount);
  if (g_crashCount >= RC_CRASH_LIMIT) {
    g_set.rcOn = 0;
    g_prefs.putInt("rc", 0);
    g_rcAutoOff = true;
  }
}
// 60 秒落ちずに動いたら、数えた回数を 0 に戻す(loop() から)
static void rcCrashClear(uint32_t now) {
  if (g_crashCleared || now < 60000) return;
  g_crashCleared = true;
  if (!g_sys.nvsOk || g_crashCount == 0) return;
  mlink::graceUntil(millis() + 5000);
  g_prefs.putInt("rccrash", 0);
  mlink::graceUntil(millis() + 200);
  g_crashCount = 0;
  Serial.println("60 秒落ちずに動いたので、落ちた回数を 0 に戻した");
}

#if RC_ENABLE
// 表示器は RaceChrono DIY の BLE 機器(サービス 0x1FF8)として、次の 4 つを持つ。
//   0x0001 CAN(READ|NOTIFY)   : 4 バイトの ID(下位が先)+ 中身 1〜16 バイト。XIAO から届いた CAN の生データと、IMU の値
//   0x0002 フィルタ(WRITE)    : RaceChrono が「全部拒否」「全部許可」「この PID を n ms 間隔で」を書いてくる
//   0x0003 GPS(READ|NOTIFY)   : 20 バイト(時刻・位置・高さ・速度・方位・精度)。M100 の NAV-PVT が届くたび(25 回/秒)
//   0x0004 GPS の日時(READ|NOTIFY): 3 バイト(日と時)。0x0003 と同じ 3 ビットの印(sync)で対応づける
//   0x0005 Monitor の設定(INDICATE|WRITE)、0x0006 Monitor の値(WRITE_NR): RaceChrono が機器に「Monitor」の機能も期待して
//          列挙するので、特性だけ置いてある(表示器は何も監視しない = INDICATE を送らない)。無いと iPhone の RaceChrono が
//          列挙のあと切断とつなぎ直しを繰り返した(2026-10-15)
// 流れ: 受信タスク(CAN)と rc タスク(GPS・IMU)が通知を g_rcQueue に入れ、ble タスクが順に送る。
//   フィルタ(どの PID を送るか)は app.cpp の rcdiy にあり、3 つのタスクから使うので g_rcMutex で守る。
static const uart_port_t GPS_UART = UART_NUM_2;
static const uint8_t IMU_ADDR = 0x6B;  // QMI8658(HARDWARE_REFERENCE.md の I2C の表)
enum { RC_CAN = 0, RC_GPS = 1, RC_TIME = 2 };
struct RcMsg {
  uint8_t kind, len;
  uint8_t data[20];
};
static QueueHandle_t g_rcQueue = nullptr;
static SemaphoreHandle_t g_rcMutex = nullptr;
static NimBLECharacteristic *g_chCan = nullptr, *g_chFilter = nullptr, *g_chGps = nullptr, *g_chTime = nullptr, *g_chMonCfg = nullptr, *g_chMonVal = nullptr;
static volatile bool g_bleOk = false, g_bleConnected = false;
static uint32_t g_canSent = 0, g_gpsSent = 0, g_dropped = 0;  // 数えるだけ(別のタスクから読む。多少の取りこぼしは構わない)
static uint32_t g_bleConnMs = 0;
static bool g_gpsUartOk = false, g_imuOk = false;
// GPS: 0 = 設定を送って NAV-PVT を待っている、1 = 届いている
static int g_gpsState = 0;
static uint32_t g_gpsCfgMs = 0, g_gpsLastPvtMs = 0, g_gpsAttempts = 0, g_gpsTimeMs = 0;
// GPS の受信ピンの判定: 0 = していない、1 = 候補 A(PIN_GPS_RX)で聞いている、2 = 候補 B(PIN_GPS_TX)で聞いている
static int g_gpsDetect = 0;
static uint32_t g_gpsDetectMs = 0, g_gpsDetectLastMs = 0, g_gpsDetectBytesA = 0;
static int g_gpsRxPin = PIN_GPS_RX, g_gpsTxPin = PIN_GPS_TX;
static bool g_gpsPinsKnown = false;  // 信号のあるピンを見つけた
static uint8_t g_gpsSync = 0;
static uint32_t g_gpsPrevCode = 0xFFFFFFFF;
// IMU
static int g_imuFails = 0;
static uint32_t g_imuRetryMs = 0, g_imuSendMs = 0, g_imuCalSavedVer = 0, g_imuCalSaveMs = 0;
static float g_gyro[3] = { 0, 0, 0 };
static float g_gyroBias[3] = { 0, 0, 0 };  // 静止中に取るジャイロの零点
static bool g_gyroBiasOk = false;

// 通知を 1 つキューへ(あふれたら捨てて数える)。.ino が自動で作る関数の宣言は構造体より前に入るので、引数は組み込みの型にしてある
static void rcPush(uint8_t kind, const uint8_t *data, uint8_t len) {
  if (!g_rcQueue) return;
  RcMsg m;
  m.kind = kind;
  m.len = len;
  memcpy(m.data, data, len);
  if (xQueueSend(g_rcQueue, &m, 0) != pdTRUE) g_dropped++;
}

static bool rcFilterPass(uint32_t pid, uint32_t now, uint16_t minMs = 0) {
  if (!g_rcMutex) return false;
  xSemaphoreTake(g_rcMutex, portMAX_DELAY);
  bool ok = rcdiy::filterPass(pid, now, minMs);
  xSemaphoreGive(g_rcMutex);
  return ok;
}

// 受信タスクから、CAN の生データを 1 フレーム受けるたびに呼ばれる(app.h)
static uint32_t g_imuIdClash = 0, g_imuIdClashMs = 0;
void platCanFrame(uint8_t bus, uint8_t ext, uint32_t id, uint8_t dlc, const uint8_t *data, uint32_t nowMs) {
  (void)ext;
  if (id == rcdiy::PID_IMU_ACCEL || id == rcdiy::PID_IMU_GYRO) {  // 車の ID と IMU 用の ID が重なっている(config.h の RC_PID_IMU_* を変える)
    g_imuIdClash++;
    if (g_imuIdClash == 1 || (uint32_t)(nowMs - g_imuIdClashMs) >= 60000) {
      g_imuIdClashMs = nowMs;
      Serial.printf("警告: 車の CAN に IMU 用の ID %08lX と同じ ID がある(%lu 回)。config.h の RC_PID_IMU_ACCEL / RC_PID_IMU_GYRO を別の値にする\n", (unsigned long)id,
                    (unsigned long)g_imuIdClash);
    }
    return;  // 車の方は送らない(RaceChrono 側で IMU の値と混ざらないように)
  }
  if (!g_bleConnected || dlc == 0) return;
  if (bus == 1 && !RC_SEND_BCAN) return;
  if (!rcFilterPass(id, nowMs, RC_CAN_MIN_MS)) return;  // RaceChrono へは PID ごとに最短 RC_CAN_MIN_MS(1.x の c6_can_link のときと同じ量)
  uint8_t b[20];
  rcPush(RC_CAN, b, (uint8_t)rcdiy::encodeCan(id, data, dlc, b));
}

// ---- BLE
// 切断の理由(HCI のエラー番号。NimBLE は 0x200 を足して返す)
static const char *bleReasonName(int reason) {
  int r = reason >= 0x200 ? reason - 0x200 : reason;
  switch (r) {
    case 0x05: return "認証に失敗した";
    case 0x08: return "電波が途切れた(監視タイムアウト)";
    case 0x13: return "スマホ側が切った";
    case 0x16: return "表示器側が切った";
    case 0x1A: return "相手が対応していない機能";
    case 0x22: return "応答待ちで時間切れ";
    case 0x3B: return "接続の間隔の要求を受け入れられなかった";
    case 0x3D: return "暗号の整合性の誤り";
    case 0x3E: return "接続を確立できなかった";
    default: return "そのほか";
  }
}

class RcServerCb : public NimBLEServerCallbacks {
  using NimBLEServerCallbacks::onConnect;
  using NimBLEServerCallbacks::onDisconnect;
  using NimBLEServerCallbacks::onMTUChange;
  void onConnect(NimBLEServer *srv, ble_gap_conn_desc *desc) override {
    g_bleConnected = true;
    g_bleConnMs = millis();
    Serial.printf("BLE: 接続(%s)。間隔 %.2f ms、遅延 %u、監視 %u ms\n", NimBLEAddress(desc->peer_ota_addr).toString().c_str(), desc->conn_itvl * 1.25, desc->conn_latency,
                  desc->supervision_timeout * 10);
    if (RC_BLE_CONN_PARAMS) srv->updateConnParams(desc->conn_handle, 6, 12, 0, 400);  // 7.5〜15 ms 間隔を頼む(決めるのはスマホ側)
  }
  void onDisconnect(NimBLEServer *) override {
    g_bleConnected = false;
    Serial.printf("BLE: 切断(つながっていた時間 %.1f 秒)。接続待ちに戻る\n", (millis() - g_bleConnMs) / 1000.0);
  }
  void onMTUChange(uint16_t mtu, ble_gap_conn_desc *) override {
    Serial.printf("BLE: 1 回に送れる大きさ(MTU)%u バイト\n", mtu);
  }
};

// すべての GAP の出来事(切断の理由、接続の条件の変更、ペアリングの要求)を 1 行ずつ出す
static int rcGapEvent(ble_gap_event *ev, void *) {
  switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
      if (ev->connect.status != 0) Serial.printf("BLE: 接続に失敗(%d)\n", ev->connect.status);
      break;
    case BLE_GAP_EVENT_DISCONNECT:
      Serial.printf("BLE: 切断の理由: %s(0x%02X)\n", bleReasonName(ev->disconnect.reason), ev->disconnect.reason >= 0x200 ? ev->disconnect.reason - 0x200 : ev->disconnect.reason);
      break;
    case BLE_GAP_EVENT_CONN_UPDATE: {
      ble_gap_conn_desc d;
      if (ev->conn_update.status == 0 && ble_gap_conn_find(ev->conn_update.conn_handle, &d) == 0)
        Serial.printf("BLE: 接続の条件が変わった。間隔 %.2f ms、遅延 %u、監視 %u ms\n", d.conn_itvl * 1.25, d.conn_latency, d.supervision_timeout * 10);
      else Serial.printf("BLE: 接続の条件の変更(結果 %d)\n", ev->conn_update.status);
      break;
    }
    case BLE_GAP_EVENT_CONN_UPDATE_REQ:
      if (ev->conn_update_req.peer_params)
        Serial.printf("BLE: スマホが接続の条件の変更を頼んだ。間隔 %.2f〜%.2f ms、遅延 %u、監視 %u ms\n", ev->conn_update_req.peer_params->itvl_min * 1.25,
                      ev->conn_update_req.peer_params->itvl_max * 1.25, ev->conn_update_req.peer_params->latency, ev->conn_update_req.peer_params->supervision_timeout * 10);
      break;
    case BLE_GAP_EVENT_ENC_CHANGE:
      Serial.printf("BLE: 暗号化(ペアリング)の変更(結果 %d)\n", ev->enc_change.status);
      break;
    case BLE_GAP_EVENT_PASSKEY_ACTION:
      Serial.printf("BLE: ペアリングの操作を求められた(種類 %u)\n", ev->passkey.params.action);
      break;
    case BLE_GAP_EVENT_REPEAT_PAIRING:
      Serial.println("BLE: もう一度ペアリングを求められた");
      break;
    default:
      break;
  }
  return 0;
}

class RcFilterCb : public NimBLECharacteristicCallbacks {
  using NimBLECharacteristicCallbacks::onWrite;
  void onWrite(NimBLECharacteristic *ch) override {
    NimBLEAttValue v = ch->getValue();
    const uint8_t *d = v.data();
    size_t n = v.length();
    int cnt = 0;
    if (g_rcMutex) {
      xSemaphoreTake(g_rcMutex, portMAX_DELAY);
      rcdiy::filterCommand(d, n);
      cnt = rcdiy::filterCount();
      xSemaphoreGive(g_rcMutex);
    }
    if (n >= 1 && d[0] == 0) Serial.println("RaceChrono: CAN のフィルタ = 全部拒否");
    else if (n >= 3 && d[0] == 1) Serial.printf("RaceChrono: CAN のフィルタ = 全部許可(%u ms 間隔)\n", (unsigned)((d[1] << 8) | d[2]));
    else if (n >= 7 && d[0] == 2)
      Serial.printf("RaceChrono: CAN の PID %08lX を %u ms 間隔で許可(許可している PID %d 個)\n",
                    (unsigned long)(((uint32_t)d[3] << 24) | ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 8) | d[6]), (unsigned)((d[1] << 8) | d[2]), cnt);
    else Serial.printf("RaceChrono: フィルタに知らない命令(%u バイト、先頭 0x%02X)\n", (unsigned)n, n ? d[0] : 0);
  }
};

static const char *chName(NimBLECharacteristic *ch) {
  return ch == g_chCan ? "CAN" : ch == g_chGps ? "GPS" : ch == g_chTime ? "GPS の日時" : ch == g_chFilter ? "フィルタ" : ch == g_chMonCfg ? "Monitor の設定" : ch == g_chMonVal ? "Monitor の値" : "?";
}
// Monitor(0x0005/0x0006)への書き込み: 表示器は何も監視しないので来ないはずだが、来たら記録する
class RcMonitorCb : public NimBLECharacteristicCallbacks {
  using NimBLECharacteristicCallbacks::onWrite;
  using NimBLECharacteristicCallbacks::onSubscribe;
  void onWrite(NimBLECharacteristic *ch) override {
    NimBLEAttValue v = ch->getValue();
    Serial.printf("BLE: スマホが %s に %u バイト書いた(先頭 0x%02X)\n", chName(ch), (unsigned)v.length(), v.length() ? v.data()[0] : 0);
  }
  void onSubscribe(NimBLECharacteristic *ch, ble_gap_conn_desc *, uint16_t subValue) override {
    Serial.printf("BLE: スマホが %s の INDICATE を%s\n", chName(ch), subValue ? "登録した" : "やめた");
  }
};
class RcNotifyCb : public NimBLECharacteristicCallbacks {
  using NimBLECharacteristicCallbacks::onSubscribe;
  using NimBLECharacteristicCallbacks::onRead;
  void onSubscribe(NimBLECharacteristic *ch, ble_gap_conn_desc *, uint16_t subValue) override {
    Serial.printf("BLE: スマホが %s の通知を%s\n", chName(ch), subValue ? "購読した" : "やめた");
  }
  void onRead(NimBLECharacteristic *ch) override {
    Serial.printf("BLE: スマホが %s を読んだ\n", chName(ch));
  }
};

static bool bleBegin() {
  static RcServerCb srvCb;
  static RcFilterCb filterCb;
  static RcNotifyCb notifyCb;
  NimBLEDevice::init(RC_BLE_NAME);
  NimBLEServer *srv = NimBLEDevice::createServer();
  if (!srv) return false;
  srv->setCallbacks(&srvCb, false);
  srv->advertiseOnDisconnect(true);
  NimBLEService *svc = srv->createService(NimBLEUUID((uint16_t)0x1FF8));
  if (!svc) return false;
  g_chCan = svc->createCharacteristic(NimBLEUUID((uint16_t)0x0001), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY, 20);
  g_chFilter = svc->createCharacteristic(NimBLEUUID((uint16_t)0x0002), NIMBLE_PROPERTY::WRITE, 20);
  g_chGps = svc->createCharacteristic(NimBLEUUID((uint16_t)0x0003), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY, 20);
  g_chTime = svc->createCharacteristic(NimBLEUUID((uint16_t)0x0004), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY, 3);
  g_chMonCfg = svc->createCharacteristic(NimBLEUUID((uint16_t)0x0005), NIMBLE_PROPERTY::INDICATE | NIMBLE_PROPERTY::WRITE, 20);
  g_chMonVal = svc->createCharacteristic(NimBLEUUID((uint16_t)0x0006), NIMBLE_PROPERTY::WRITE_NR, 20);
  if (!g_chCan || !g_chFilter || !g_chGps || !g_chTime || !g_chMonCfg || !g_chMonVal) return false;
  static RcMonitorCb monitorCb;
  g_chMonCfg->setCallbacks(&monitorCb);
  g_chMonVal->setCallbacks(&monitorCb);
  g_chFilter->setCallbacks(&filterCb);
  g_chCan->setCallbacks(&notifyCb);
  g_chGps->setCallbacks(&notifyCb);
  g_chTime->setCallbacks(&notifyCb);
  static const uint8_t zero[3] = { 0, 0, 0 };
  g_chTime->setValue(zero, 3);
  if (!svc->start()) return false;
  NimBLEDevice::setCustomGapHandler(rcGapEvent);
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(svc->getUUID());
  adv->setScanResponse(true);  // 名前は応答(scan response)側に入れる(接続待ちの電波にサービスの UUID と名前の両方は入りきらない)
  return adv->start();
}

static void bleTask(void *) {
  RcMsg m;
  for (;;) {
    if (xQueueReceive(g_rcQueue, &m, pdMS_TO_TICKS(100)) != pdTRUE) continue;
    NimBLECharacteristic *ch = m.kind == RC_CAN ? g_chCan : m.kind == RC_GPS ? g_chGps : g_chTime;
    ch->setValue(m.data, m.len);  // 読まれたときのために、つながっていなくても値は入れておく
    if (!g_bleConnected || ch->getSubscribedCount() == 0) continue;
    ch->notify();
    if (m.kind == RC_CAN) g_canSent++;
    else if (m.kind == RC_GPS) g_gpsSent++;
  }
}

// ---- GPS(M100、UBX)
static bool gpsUartBegin() {
  uart_config_t cfg = {};
  cfg.baud_rate = RC_GPS_BAUD;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  if (uart_driver_install(GPS_UART, 4096, 512, 0, nullptr, 0) != ESP_OK) return false;
  if (uart_param_config(GPS_UART, &cfg) != ESP_OK) return false;
  gpio_pullup_en((gpio_num_t)PIN_GPS_RX);
  gpio_pullup_en((gpio_num_t)PIN_GPS_TX);
  return gpsApplyPins(PIN_GPS_RX, PIN_GPS_TX);
}

// 受信ピンと送信ピンを割り当てる。送信はオープンドレイン(+ プルアップ)で駆動する: 逆につないでも M100 の TX と出力同士で
// ぶつからない(その場合は設定が届かないだけ)
static bool gpsApplyPins(int rx, int tx) {
  if (uart_set_pin(GPS_UART, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return false;
  gpio_ll_od_enable(&GPIO, (uint32_t)tx);  // 送信はオープンドレイン(休止中は駆動しない)
  gpio_pullup_en((gpio_num_t)tx);
  gpio_pullup_en((gpio_num_t)rx);
  g_gpsRxPin = rx;
  g_gpsTxPin = tx;
  return true;
}

// 受信ピンの判定: 送信ピンの駆動をやめ、候補 A(PIN_GPS_RX)、候補 B(PIN_GPS_TX)の順に 400 ms ずつ受信だけして、バイトが
// 来た方を受信に使う(M100 は電源が入れば UBX か NMEA を出し続けるので、来ていれば分かる。速度が違っても何かは来る)
static void gpsDetectStart(uint32_t now) {
  // 送信ピンはオープンドレインで休止中は駆動していないので、そのまま聞いてよい
  uart_set_pin(GPS_UART, UART_PIN_NO_CHANGE, PIN_GPS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_flush_input(GPS_UART);
  g_gpsDetect = 1;
  g_gpsDetectMs = now;
  g_gpsDetectLastMs = now;
}
static void gpsDetectStep(uint32_t now) {
  if ((uint32_t)(now - g_gpsDetectMs) < 400) return;
  size_t n = 0;
  uart_get_buffered_data_len(GPS_UART, &n);
  if (g_gpsDetect == 1) {
    g_gpsDetectBytesA = (uint32_t)n;
    uart_set_pin(GPS_UART, UART_PIN_NO_CHANGE, PIN_GPS_TX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_flush_input(GPS_UART);
    g_gpsDetect = 2;
    g_gpsDetectMs = now;
    return;
  }
  uint32_t bytesB = (uint32_t)n;
  g_gpsDetect = 0;
  int rx = PIN_GPS_RX, tx = PIN_GPS_TX;
  const char *what;
  if (g_gpsDetectBytesA > 0 && bytesB == 0) {
    what = "配線図どおり";
    g_gpsPinsKnown = true;
  } else if (bytesB > 0 && g_gpsDetectBytesA == 0) {
    rx = PIN_GPS_TX;
    tx = PIN_GPS_RX;
    what = "配線図と逆。入れ替えて使う";
    g_gpsPinsKnown = true;
  } else if (bytesB == 0 && g_gpsDetectBytesA == 0) {
    what = "どちらにも信号が無い(M100 がつながっていないか、電源が無い)。設定どおりのピンで待つ";
  } else {
    what = "どちらにも信号がある(ノイズか配線の誤り)。設定どおりのピンを使う";
  }
  gpsApplyPins(rx, tx);
  uart_flush_input(GPS_UART);
  gpsSendConfig();
  static uint32_t printed = 0;
  if (g_gpsPinsKnown || printed < 2 || (uint32_t)(now - printed) >= 60000) {
    printed = now ? now : 1;
    Serial.printf("GPS: 受信ピンの判定: 「IO%d」に %lu バイト、「IO%d」に %lu バイト → %s(受信「IO%d」、送信「IO%d」)\n", PIN_GPS_RX, (unsigned long)g_gpsDetectBytesA, PIN_GPS_TX,
                  (unsigned long)bytesB, what, rx, tx);
  }
}

static void gpsSend(const uint8_t *d, size_t n) {
  if (n) uart_write_bytes(GPS_UART, (const char *)d, n);
}

static void gpsSendConfig() {
  uint8_t b[128];
  gpsSend(b, gnss::buildConfig(b, sizeof(b), RC_GPS_RATE_HZ));  // 測位の設定(これが通れば NAV-PVT が 25 回/秒で届く)
  gpsSend(b, gnss::buildSignals(b, sizeof(b)));                 // 衛星系(通らなくても測位には困らない)
  g_gpsCfgMs = millis();
}

// 設定を送っても届かないとき: M100 が別の速度(工場出荷の 9600 など)にいるとみなし、その速度で「RC_GPS_BAUD にせよ」を
// 送ってから、RC_GPS_BAUD に戻して設定を送り直す。候補を順に試す
static void gpsRetry() {
  static const uint32_t TRY[] = { 9600, 38400, 57600, 230400 };
  uint32_t baud = TRY[g_gpsAttempts % (sizeof(TRY) / sizeof(TRY[0]))];
  g_gpsAttempts++;
  uint8_t b[32];
  uart_set_baudrate(GPS_UART, baud);
  gpsSend(b, gnss::buildBaud(b, sizeof(b), RC_GPS_BAUD));
  uart_wait_tx_done(GPS_UART, pdMS_TO_TICKS(200));
  vTaskDelay(pdMS_TO_TICKS(100));
  uart_set_baudrate(GPS_UART, RC_GPS_BAUD);
  uart_flush_input(GPS_UART);
  gpsSendConfig();
  if (g_gpsAttempts <= 4 || g_gpsAttempts % 30 == 0)
    Serial.printf("GPS: UBX が届かない(%lu 回目)。%lu bps で速度を %d に書き換えて設定を送り直した(受信「IO%d」、送信「IO%d」)。届かないままなら M100 の電源と配線を見る\n",
                  (unsigned long)g_gpsAttempts, (unsigned long)baud, RC_GPS_BAUD, g_gpsRxPin, g_gpsTxPin);
}

static void gpsOnPvt(uint32_t now) {
  gnss::Fix f;
  if (!gnss::latest(&f)) return;
  if (g_gpsState != 1) {
    g_gpsState = 1;
    g_gpsPinsKnown = true;
    Serial.printf("GPS: 位置(NAV-PVT)が届き始めた(受信「IO%d」、送信「IO%d」。設定の ACK %lu、NAK %lu、衛星 %u)\n", g_gpsRxPin, g_gpsTxPin, (unsigned long)gnss::ackCount(),
                  (unsigned long)gnss::nakCount(), f.numSV);
  }
  g_gpsLastPvtMs = now;
  uint32_t code = rcdiy::dateHourCode(f);
  bool changed = code != g_gpsPrevCode;
  if (changed) {
    g_gpsPrevCode = code;
    g_gpsSync = (uint8_t)((g_gpsSync + 1) & 7);
  }
  uint8_t b[20];
  if (changed || (uint32_t)(now - g_gpsTimeMs) >= 1000) {  // 日時は変わったときと、1 秒に 1 回(つながった直後のため)
    g_gpsTimeMs = now;
    rcdiy::encodeGpsTime(f, g_gpsSync, b);
    rcPush(RC_TIME, b, 3);
  }
  rcdiy::encodeGpsMain(f, g_gpsSync, b);
  rcPush(RC_GPS, b, 20);
}

static void gpsManage(uint32_t now) {
  if (!g_gpsUartOk) return;
  if (g_gpsState == 1) {
    if ((uint32_t)(now - g_gpsLastPvtMs) > RC_GPS_LOST_MS) {
      g_gpsState = 0;
      g_gpsAttempts = 0;
      g_gpsPinsKnown = false;
      Serial.println("GPS: 位置が途切れた。設定を送り直す");
      uart_set_baudrate(GPS_UART, RC_GPS_BAUD);
      gpsSendConfig();
    }
    return;
  }
  if (g_gpsDetect) {
    gpsDetectStep(now);
    return;
  }
  // 位置が届かない間: 信号のあるピンがまだ分かっていなければ 20 秒ごとに判定し直す。そのあいだは速度を変えて設定を送り直す
  if (RC_GPS_PIN_DETECT && !g_gpsPinsKnown && (g_gpsDetectLastMs == 0 || (uint32_t)(now - g_gpsDetectLastMs) >= 20000)) {
    uart_set_baudrate(GPS_UART, RC_GPS_BAUD);
    gpsDetectStart(now);
    return;
  }
  if ((uint32_t)(now - g_gpsCfgMs) >= RC_GPS_RETRY_MS) gpsRetry();
}

// ---- IMU(QMI8658)。レジスタの手順は SensorLib 0.3.1(SensorQMI8658.hpp)と同じ
static bool imuWrite(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}
static bool imuRead(uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(IMU_ADDR, n) != n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

static bool imuBegin() {
  uint8_t v = 0;
  i2cLock();
  bool ok = imuWrite(0x60, 0xB0);  // リセット
  if (ok) {
    ok = false;
    for (int i = 0; i < 50 && !ok; i++) {  // 終わると 0x4D が 0x80 になる(最長 15 ms)
      delay(10);
      ok = imuRead(0x4D, &v, 1) && v == 0x80;
    }
  }
  if (ok) ok = imuRead(0x00, &v, 1) && v == 0x05;  // WHOAMI
  if (ok) {
    ok = imuWrite(0x02, 0x40)              // CTRL1: アドレスの自動増加、下位バイトが先
         && imuWrite(0x03, (1 << 4) | 6)   // CTRL2: 加速度 ±4 g、ODR 符号 6(ジャイロと同時に動かすときは 112 Hz)
         && imuWrite(0x04, (5 << 4) | 6)   // CTRL3: ジャイロ ±512 deg/s、112 Hz
         && imuWrite(0x06, 0x77)           // CTRL5: 両方に LPF(ODR の 13.4 % ≈ 15 Hz)
         && imuWrite(0x08, 0x03);          // CTRL7: 加速度とジャイロを動かす
  }
  i2cUnlock();
  g_imuFails = 0;
  return ok;
}

static int16_t i16c(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)lroundf(v);
}

static void imuCalSave(uint32_t now) {
  uint32_t ver = imu::calVersion();
  if (ver == g_imuCalSavedVer || !g_sys.nvsOk) return;
  if (g_imuCalSaveMs && (uint32_t)(now - g_imuCalSaveMs) < 60000) return;  // 覚え直しが続く間は 1 分に 1 回まで
  float f[9];
  imu::exportCal(f);
  mlink::graceUntil(millis() + 5000);  // フラッシュへ書く間の UART の取りこぼしを線の品質に入れない
  size_t w = g_prefs.putBytes("imucal", f, sizeof(f));
  mlink::graceUntil(millis() + 200);
  g_imuCalSavedVer = ver;
  g_imuCalSaveMs = now;
  Serial.printf("IMU: 向きの補正を保存%s(上 (%.2f, %.2f, %.2f) %s、前 (%.2f, %.2f, %.2f) %s)\n", w == sizeof(f) ? "した" : "できない", f[0], f[1], f[2],
                f[6] > 0.5f ? "済" : "未", f[3], f[4], f[5], f[7] > 0.5f ? "済" : "未");
}

void platResetImuCal() {
  imu::reset();
  if (g_sys.nvsOk) {
    mlink::graceUntil(millis() + 5000);
    g_prefs.remove("imucal");
    mlink::graceUntil(millis() + 200);
  }
  g_imuCalSavedVer = imu::calVersion();
  g_imuCalSaveMs = 0;
  g_sys.rc.imuLevel = g_sys.rc.imuForward = false;  // 画面にすぐ出す
  Serial.println("IMU: 覚えた向きを捨てた(保存した値も消した)。停車 3 秒で「上」、発進で「前」を覚え直す");
}

static void imuCalLoad() {
  float f[9];
  if (!g_sys.nvsOk || g_prefs.getBytes("imucal", f, sizeof(f)) != sizeof(f)) return;
  imu::importCal(f);
  g_imuCalSavedVer = imu::calVersion();
}

static void imuPoll(uint32_t now) {
  if (!g_imuOk) {
    if ((uint32_t)(now - g_imuRetryMs) < 5000) return;
    g_imuRetryMs = now;
    g_imuOk = imuBegin();
    if (g_imuOk) Serial.println("IMU: 応答が戻った");
    return;
  }
  uint8_t b[12];
  i2cLock();
  bool ok = imuRead(0x35, b, sizeof(b));  // AX_L〜GZ_H
  i2cUnlock();
  if (!ok) {
    if (++g_imuFails >= 25) {
      g_imuOk = false;
      g_imuRetryMs = now;
      Serial.println("IMU: 読めなくなった。5 秒ごとに初期化をやり直す");
    }
    return;
  }
  g_imuFails = 0;
  const float AS = 4.0f / 32768.0f, GS = 512.0f / 32768.0f;
  float ax = (int16_t)(b[0] | (b[1] << 8)) * AS, ay = (int16_t)(b[2] | (b[3] << 8)) * AS, az = (int16_t)(b[4] | (b[5] << 8)) * AS;
  g_gyro[0] = (int16_t)(b[6] | (b[7] << 8)) * GS;
  g_gyro[1] = (int16_t)(b[8] | (b[9] << 8)) * GS;
  g_gyro[2] = (int16_t)(b[10] | (b[11] << 8)) * GS;
  // 車速: XIAO から届く値。無ければ GPS の速度(机の上の試験や、XIAO をつながないとき)
  Signals s;
  mlink::getSignals(&s, now);
  bool spOk = s.speedOk && !s.sim;  // 模擬データ(デモ)の車速では向きを覚えない
  float kmh = s.speedKmh;
  if (!spOk && g_gpsState == 1) {
    gnss::Fix f;
    if (gnss::latest(&f) && f.fixOk && (uint32_t)(now - f.rxMs) < 1000) {
      spOk = true;
      kmh = f.gSpeed * 0.0036f;
    }
  }
  bool level0 = imu::levelOk(), fwd0 = imu::forwardOk();
  uint32_t ver0 = imu::calVersion();
  imu::sample(ax, ay, az, spOk, kmh, now);
  if (imu::stillNow()) {  // 静止中はジャイロの零点を取る(時定数 1 秒)
    for (int i = 0; i < 3; i++) g_gyroBias[i] += (g_gyro[i] - g_gyroBias[i]) * 0.02f;
    g_gyroBiasOk = true;
  }
  if (imu::calVersion() != ver0) {
    float f[9];
    imu::exportCal(f);
    bool level1 = imu::levelOk(), fwd1 = imu::forwardOk();
    if (!level0 && level1) Serial.printf("IMU: 「上」を覚えた(停車中の重力の向き %.2f, %.2f, %.2f  大きさ %.3f g)\n", f[0], f[1], f[2], f[8]);
    else if (fwd0 && !fwd1) Serial.printf("IMU: 「上」を覚え直した(%.2f, %.2f, %.2f)。付け直したとみなし、「前」は発進の加速で覚え直す\n", f[0], f[1], f[2]);
    else if (!fwd0 && fwd1) Serial.printf("IMU: 「前」を覚えた(発進の加速の向き %.2f, %.2f, %.2f)\n", f[3], f[4], f[5]);
    else Serial.printf("IMU: 向きの補正を更新した(上 %.2f, %.2f, %.2f  前 %.2f, %.2f, %.2f  重力 %.3f g)\n", f[0], f[1], f[2], f[3], f[4], f[5], f[8]);
  }
  // RC_IMU_HZ 回/秒で RaceChrono へ(CAN の 1 つとして。フィルタで許可されているときだけ)
  if ((int32_t)(now - g_imuSendMs) >= 1000 / RC_IMU_HZ) {
    g_imuSendMs = now;
    float lon, lat, vert;
    imu::car(&lon, &lat, &vert);
    g_sys.rc.gLon = lon;
    g_sys.rc.gLat = lat;
    if (g_bleConnected) {
      uint8_t b[20];
      if (rcFilterPass(rcdiy::PID_IMU_ACCEL, now)) {
        int16_t v[3] = { i16c(lon * 1000.0f), i16c(lat * 1000.0f), i16c(vert * 1000.0f) };
        rcPush(RC_CAN, b, (uint8_t)rcdiy::encodeCan(rcdiy::PID_IMU_ACCEL, (const uint8_t *)v, 6, b));
      }
      if (rcFilterPass(rcdiy::PID_IMU_GYRO, now)) {
        float glon, glat, gvert;
        imu::toCar(g_gyro[0] - g_gyroBias[0], g_gyro[1] - g_gyroBias[1], g_gyro[2] - g_gyroBias[2], &glon, &glat, &gvert);  // 「上」の軸まわりの回転 = ヨーレート(左回りが正)。零点を引く
        int16_t v[3] = { i16c(gvert * 100.0f), 0, 0 };
        rcPush(RC_CAN, b, (uint8_t)rcdiy::encodeCan(rcdiy::PID_IMU_GYRO, (const uint8_t *)v, 6, b));
      }
    }
  }
  imuCalSave(now);
}

// ---- 1 秒ごとの集計(「RaceChrono」の画面用)
static void rcStatsTick(uint32_t now) {
  static uint32_t lastMs = 0, canPrev = 0, gpsPrev = 0, pvtPrev = 0;
  if ((uint32_t)(now - lastMs) < 1000) return;
  lastMs = now;
  RcStats &r = g_sys.rc;
  r.bleOk = g_bleOk;
  r.connected = g_bleConnected;
  r.canPerSec = g_canSent - canPrev;
  canPrev = g_canSent;
  r.gpsPerSec = g_gpsSent - gpsPrev;
  gpsPrev = g_gpsSent;
  r.dropped = g_dropped;
  uint32_t pvt = gnss::pvtCount();
  r.pvtPerSec = pvt - pvtPrev;
  pvtPrev = pvt;
  r.gpsOk = g_gpsState == 1;
  gnss::Fix f;
  if (r.gpsOk && gnss::latest(&f)) {
    r.fixType = f.fixOk ? f.fixType : 0;
    r.numSV = f.numSV;
    r.lat = f.lat;
    r.lon = f.lon;
    r.gpsKmh = f.gSpeed * 0.0036f;
  } else {
    r.fixType = 0;
    r.numSV = 0;
    r.gpsKmh = 0;
  }
  r.imuOk = g_imuOk;
  r.imuLevel = imu::levelOk();
  r.imuForward = imu::forwardOk();
}

static void rcTask(void *) {
  static uint8_t buf[512];
  uint32_t nextImu = millis();
  for (;;) {
    uint32_t now = millis();
    if (g_gpsUartOk) {
      size_t avail = 0;
      uart_get_buffered_data_len(GPS_UART, &avail);
      if (avail > sizeof(buf)) avail = sizeof(buf);
      int n = avail ? uart_read_bytes(GPS_UART, buf, avail, 0) : 0;
      uint32_t pvt0 = gnss::pvtCount();
      if (n > 0) gnss::feed(buf, (size_t)n, now);
      if (gnss::pvtCount() != pvt0) gpsOnPvt(now);
      gpsManage(now);
    }
    if ((int32_t)(now - nextImu) >= 0) {
      nextImu += 1000 / IMU_SAMPLE_HZ;
      if ((int32_t)(now - nextImu) > 200) nextImu = now;  // 大きく遅れたら追いつかせない
      imuPoll(now);
    }
    rcStatsTick(now);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static bool rcBegin() {
  g_rcQueue = xQueueCreate(RC_QUEUE_LEN, sizeof(RcMsg));
  g_rcMutex = xSemaphoreCreateMutex();
  rcdiy::filterReset();
  gnss::begin();
  g_gpsUartOk = gpsUartBegin();
  if (g_gpsUartOk && !RC_GPS_PIN_DETECT) gpsSendConfig();  // 判定が入のときは、判定のあとで送る
  g_imuOk = imuBegin();
  g_bleOk = bleBegin();
  bool t1 = xTaskCreatePinnedToCore(rcTask, "rc", 6144, nullptr, 4, nullptr, 0) == pdPASS;
  bool t2 = xTaskCreatePinnedToCore(bleTask, "ble", 4096, nullptr, 3, nullptr, 0) == pdPASS;
  return g_rcQueue && g_rcMutex && t1 && t2;
}

static void rcPrintSelfCheck() {
  Serial.printf("BLE(RaceChrono DIY「%s」): %s\n", RC_BLE_NAME, !g_bleOk ? "開始できない" : g_bleConnected ? "接続中" : "接続待ち(未接続)");
  Serial.printf("GPS(受信「IO%d」← M100 TX、送信「IO%d」→ M100 RX、%d bps、%d 回/秒): %s(設定の ACK %lu、NAK %lu、UBX の誤り %lu)\n", g_gpsRxPin, g_gpsTxPin, RC_GPS_BAUD,
                RC_GPS_RATE_HZ, !g_gpsUartOk ? "UART を開けない" : g_gpsState == 1 ? "位置(NAV-PVT)が届いている" : "UBX が届いていない(起動直後は数秒かかる。続くなら M100 の電源と配線を見る)",
                (unsigned long)gnss::ackCount(), (unsigned long)gnss::nakCount(), (unsigned long)gnss::badCount());
  Serial.printf("IMU(QMI8658、0x6B): %s。向きの補正: 上 %s、前 %s\n", g_imuOk ? "応答あり" : "応答なし", imu::levelOk() ? "済" : "未", imu::forwardOk() ? "済" : "未");
  Serial.printf("RaceChrono の送信: %s\n", g_rcOk ? "準備できた" : "準備できない(タスクかキューを作れない)");
}
static bool rcSelfCheckOk() {
  return g_rcOk && g_bleOk && g_gpsUartOk && g_imuOk;
}
static void rcPrintStatusLine() {
  const RcStats &rc = g_sys.rc;
  Serial.printf("  RaceChrono: BLE %s 送った CAN %lu/秒 GPS %lu/秒 捨てた %lu | GPS %s 衛星 %u 位置 %lu 回/秒 %.1f km/h(UBX 誤り %lu) | IMU %s 前後 %+.2f 横 %+.2f g 上 %s 前 %s\n",
                !rc.bleOk ? "なし" : rc.connected ? "接続中" : "未接続", (unsigned long)rc.canPerSec, (unsigned long)rc.gpsPerSec, (unsigned long)rc.dropped,
                !rc.gpsOk ? "受信なし" : rc.fixType == 3 || rc.fixType == 4 ? "3D" : rc.fixType == 2 ? "2D" : "測位なし", rc.numSV, (unsigned long)rc.pvtPerSec, rc.gpsKmh,
                (unsigned long)gnss::badCount(), rc.imuOk ? "OK" : "なし", rc.gLon, rc.gLat, rc.imuLevel ? "済" : "未", rc.imuForward ? "済" : "未");
}
#else
void platCanFrame(uint8_t, uint8_t, uint32_t, uint8_t, const uint8_t *, uint32_t) {}
void platResetImuCal() {
  imu::reset();
}
static bool rcBegin() {
  return false;
}
static void rcPrintSelfCheck() {
  Serial.println("RaceChrono の送信: コンパイルから外してある(config.h の RC_ENABLE 0)");
}
static bool rcSelfCheckOk() {
  return true;
}
static void rcPrintStatusLine() {}
#endif

// ================================================================ setup / loop
static uint16_t *g_fb = nullptr;
static bool g_memOk = false, g_linkOk = false;
static uint32_t g_checkPrintMs = 0;

static bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *v) {
  i2cLock();
  bool ok = false;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom(addr, (uint8_t)1) == 1) {
    *v = Wire.read();
    ok = true;
  }
  i2cUnlock();
  return ok;
}

static bool selfCheckOk() {
  return g_lcdOk && g_spiErr == ESP_OK && g_memOk && g_sys.touchOk && g_sys.nvsOk && g_linkOk && (!g_sys.rc.enabled || rcSelfCheckOk());
}

// 起動時に確かめた結果をまとめて出す。setup() の最後と、そのあと 60 秒ごとに呼ぶ
static void printSelfCheck() {
  g_checkPrintMs = millis();
  Serial.println();
  Serial.println("==== meter_display v" METER_FW_VERSION "  595 メーター表示器 ====");
  Serial.printf("PSRAM: %u バイト(空き %u)  内部メモリーの空き: %u バイト  フラッシュ: %u バイト\n", (unsigned)ESP.getPsramSize(), (unsigned)ESP.getFreePsram(),
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFlashChipSize());
  if (!g_lcdOk) Serial.println("画面(CO5300): SPI の準備に失敗");
  else if (g_spiErr != ESP_OK) Serial.printf("画面(CO5300): SPI の送信でエラー 0x%X\n", (unsigned)g_spiErr);
  else Serial.printf("画面(CO5300、QSPI %d MHz): 初期化のコマンドを送った(画面からの返事は無いので、映ったかは目で見る)\n", LCD_SPI_HZ / 1000000);
  Serial.printf("画面用のメモリー: %s\n", g_memOk ? "OK" : "取れない(画面は赤 1 色)");
  if (g_sys.axpOk) {
    // AXP2101 のレジスタ 0x20: bit0 = PWR ボタン、bit2 = VBUS(5V)が入った、bit4 = 電池をつないだ
    Serial.printf("電源 IC(AXP2101): 起動のきっかけ 0x%02X(%s)\n", g_sys.powerOnSrc,
                  (g_sys.powerOnSrc & 0x04) ? "5V が入った" : (g_sys.powerOnSrc & 0x01) ? "PWR ボタン" : (g_sys.powerOnSrc & 0x10) ? "電池をつないだ" : "そのほか");
  } else {
    Serial.println("電源 IC(AXP2101): 応答なし");
  }
  Serial.printf("タッチ(CST9217、0x5A): %s\n", g_sys.touchOk ? "応答あり" : "応答なし(タップが効かない)");
  Serial.printf("設定(%s): シフト点 %d rpm、自動で戻る %d 分、明るさ %d %%、警告温度 %d ℃、デモ %s、RaceChrono 送信 %s\n", g_sys.nvsOk ? "保存領域 OK" : "保存領域を開けない。初期値を使う",
                g_set.shiftRpm, g_set.autoReturnMin, g_set.brightnessPct, g_set.warnTempC, g_set.demoOn ? "入" : "切", g_set.rcOn ? "入" : "切");
  Serial.printf("UART 受信(「IO16」): %s\n", g_linkOk ? "開始。速度は自動で合わせる" : "開始していない");
  {
    veh::FuelInfo fi;
    veh::fuelInfo(&fi, millis());
    if (fi.restored) Serial.printf("燃料: 前回の値 %.1f(生の値。%.0f %%)から始めた。0C014003 の b5 = 0 は使わない\n", fi.restoredTenths / 10.0,
                                   (fi.restoredTenths / 10.0 - VEH_FUEL_EMPTY_RAW) * 100.0 / (VEH_FUEL_FULL_RAW - VEH_FUEL_EMPTY_RAW));
    else Serial.println("燃料: 前回の値なし(最初に受けた正しい値から始める)。0C014003 の b5 = 0 は使わない");
  }
  Serial.printf("前回のリセットの理由: %s(%d)。落ちた回数 %lu\n", resetReasonName(g_resetReason), (int)g_resetReason, (unsigned long)g_crashCount);
  if (g_rcAutoOff) Serial.printf("RaceChrono の送信: 切にした(落ちる起動が %d 回続いたため。設定 → 3 ページ目「RaceChrono 送信」で入に戻せる)\n", RC_CRASH_LIMIT);
  else if (!g_sys.rc.enabled) Serial.println("RaceChrono の送信: 切(設定)");
  else rcPrintSelfCheck();
  Serial.printf("自己診断: %s\n", selfCheckOk() ? "OK" : "NG(上の行を見る)");
  Serial.println();
}

void setup() {
  // シリアルモニターがつながるのは待たない(車では USB がつながっていない)。
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // Mac が読んでいなくても止まらない
  Serial.println();
  Serial.println("meter_display: 起動");

  g_mutex = xSemaphoreCreateMutex();
  g_i2c = xSemaphoreCreateMutex();
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);  // 公式の BSP と同じ 400 kHz

  g_lcdOk = lcdBegin();
  // 電源 IC(AXP2101、0x34)のレジスタ 0x20 = 起動のきっかけ。読むだけで、設定は変えない
  g_sys.axpOk = i2cRead(0x34, 0x20, &g_sys.powerOnSrc);
  g_sys.touchOk = touchBegin();
  veh::begin();  // loadSettings が燃料の前回の値を入れるので、その前に
  loadSettings();

  g_fb = (uint16_t *)platAlloc(sizeof(uint16_t) * gfx::W * gfx::H);
  float *scratch = (float *)platAlloc(sizeof(float) * gfx::W * (gfx::H + 2));
  g_memOk = g_fb && scratch;
  if (!g_lcdOk || !g_memOk) {
    if (g_lcdOk) {
      lcdFillSolid(gfx::rgb(0xA31515));  // 真っ赤 = メモリーを取れなかった
      lcdBrightness(60);
    }
    printSelfCheck();
    Serial.println("画面を出せないので、ここで止まる。");
    return;
  }
  gfx::begin(g_fb, scratch);
  mlink::begin();
  cantab::begin();
  imu::begin();
  rcCrashCheck();
  ui::begin(&g_set, g_homeLoaded);
  g_linkOk = linkBegin();
  g_sys.rc.enabled = g_set.rcOn != 0;
  if (g_sys.rc.enabled) {
#if RC_ENABLE
    imuCalLoad();
#endif
    g_rcOk = rcBegin();
  }
  printSelfCheck();
  g_ready = true;
}

void loop() {
  static uint32_t statMs = 0, logMs = 0, frames = 0, logFrames = 0;
  static uint32_t renderMax = 0, flushMax = 0, loopMax = 0;
  static bool onPagePrev = false, pageNew = true;  // 前の周がサーキット・タコだったか / 次に数える回から測り直す
  uint32_t now = millis();

  // シリアルモニターをいつ開いても見られるよう、起動時の結果を 60 秒ごとに出し直す
  if ((uint32_t)(now - g_checkPrintMs) >= 60000) printSelfCheck();
  rcCrashClear(now);
  if (!g_ready) {
    delay(100);
    return;
  }
  uint32_t t0 = micros();
  touchPoll(now);

  Signals &s = g_sig;
  LinkStats ls;
  mlink::getSignals(&s, now);
  mlink::getStats(&ls);
  trip::update(s, now);
  saveTimes(now, false);
  uint32_t tu = micros();  // サーキット・タコの「1 回描いて送る時間」はここから(タッチ・受信・保存の時間は入れない)
  ui::update(s, ls, g_perf, g_sys, now);  // この中で ui::blinkPoll も呼ばれる(切り替わると platBlink)
  uint32_t t1 = micros();

  int n = gfx::dirtyCount();
  if (BLINK_BY_BRIGHTNESS) {
    // 画面全体の点滅: 明るさだけを切り替える(platBlink)。描いた分はいつもどおり送る(消灯の間は見えない)。
    // 送っている途中の切り替えは lcdPush の中で見る。シフト点 + RING_SOLID_OFFSET 以上は点滅せず明るさ 100 %
    for (int i = 0; i < n; i++) {
      gfx::Rect r = gfx::dirtyAt(i);
      int x0 = r.x0 & ~1, y0 = r.y0 & ~1, x1 = (r.x1 + 1) & ~1, y1 = (r.y1 + 1) & ~1;
      lcdPush(g_fb, x0, y0, x1 - x0, y1 - y0);
    }
  } else if (ui::blankNow()) {
    // 画面全体の点滅の消灯: 一色を 1 回だけ送る。この間に描いた分は、点灯に戻るときにまとめて送る
    n = 0;
    if (!g_solidShown) {
      lcdFillSolid(gfx::rgb(BLINK_OFF_RGB));
      g_solidShown = true;
      n = 1;
    }
  } else if (g_solidShown) {
    // 点灯に戻る: 画面全体を送り直す
    lcdPush(g_fb, 0, 0, gfx::W, gfx::H);
    g_solidShown = false;
    n = 1;
  } else {
    for (int i = 0; i < n; i++) {
      gfx::Rect r = gfx::dirtyAt(i);
      int x0 = r.x0 & ~1, y0 = r.y0 & ~1, x1 = (r.x1 + 1) & ~1, y1 = (r.y1 + 1) & ~1;  // 偶数の位置・偶数の大きさにそろえる
      lcdPush(g_fb, x0, y0, x1 - x0, y1 - y0);
    }
  }
  gfx::dirtyClear();
  uint32_t t2 = micros();
  if (n) {
    frames++;
    if (g_firstFrame) {  // 最初の絵を送り終えてから明るくする
      g_firstFrame = false;
      lcdBrightness(levelPct(g_level, g_set.brightnessPct));
    }
  }
  ui::blinkPoll(s, millis());  // 送り終えた直後にも見る(次の loop() の描画を待たない)

  // 受信の状態の「描画」(2.0.1): サーキット・タコで 1 回描いて送るのにかかった最長。サーキット・タコに入り直したら測り直し、
  // ほかの画面にいる間は前の値のまま。画面を切り替えた回の全体の描き直しと、長押しの表示は数えない(ui::pageFrame)
  const ui::Screen scNow = ui::screen();
  const bool onPage = scNow == ui::SC_DRIVE || scNow == ui::SC_BOOST;
  if (onPage && !onPagePrev) pageNew = true;
  onPagePrev = onPage;
  if (n && ui::pageFrame()) {
    uint32_t us = t2 - tu;
    if (us == 0) us = 1;  // 0 は「まだ測っていない」の印なので使わない
    if (pageNew || us > g_perf.pageMaxUs) g_perf.pageMaxUs = us;
    pageNew = false;
  }

  uint32_t rm = (t1 - t0) / 1000, fm = (t2 - t1) / 1000, lm = (t2 - t0) / 1000;
  if (rm > renderMax) renderMax = rm;
  if (fm > flushMax) flushMax = fm;
  if (lm > loopMax) loopMax = lm;
  if ((uint32_t)(now - statMs) >= 1000) {
    statMs = now;
    g_perf.fps = frames - logFrames;
    g_perf.renderMaxMs = renderMax;
    g_perf.flushMaxMs = flushMax;
    g_perf.loopMaxMs = loopMax;
    logFrames = frames;
    renderMax = flushMax = loopMax = 0;
  }
  if ((uint32_t)(now - logMs) >= 5000) {
    logMs = now;
    char pageTxt[16] = "– –";  // サーキット・タコの 1 回 最長(受信の状態の「描画」と同じ値。切り上げの ms)
    if (g_perf.pageMaxUs) snprintf(pageTxt, sizeof(pageTxt), "%lu ms", (unsigned long)((g_perf.pageMaxUs + 999) / 1000));
    Serial.printf("t=%lus 画面=%s | 描画 %lu 回/秒 最長: 描く %lu ms 送る %lu ms 1周 %lu ms サーキット・タコの 1 回 %s | 受信 %lu bps%s %lu フレーム/秒 %lu バイト/秒 CRC誤り %lu 欠落 %lu 捨てた %lu バイト 送り側で捨てた %lu フレーム%s | 無い文字 %lu | 自己診断 %s\n",
                  (unsigned long)(now / 1000), screenName((int)ui::screen()), (unsigned long)g_perf.fps, (unsigned long)g_perf.renderMaxMs, (unsigned long)g_perf.flushMaxMs,
                  (unsigned long)g_perf.loopMaxMs, pageTxt, (unsigned long)ls.baud, (ls.locked || (s.linkUp && s.sim)) ? "" : "(受信なし)", (unsigned long)ls.framesPerSec, (unsigned long)ls.bytesPerSec, (unsigned long)ls.crcErrors,
                  (unsigned long)ls.lostFrames, (unsigned long)ls.junkBytes, (unsigned long)(ls.infoValid ? ls.info.tx_drop : 0), (s.linkUp && s.sim) ? " 模擬" : "",
                  (unsigned long)gfx::missingGlyphs(), selfCheckOk() ? "OK" : "NG");
    // 回転数の点滅(この 5 秒に切り替えがあったときだけ): 点灯・消灯の時間と、予定の時刻からの遅れ。遅れが消灯の時間の 2 割を超えたら NG
    ui::BlinkStats bs;
    ui::blinkStats(&bs, true);
    if (bs.toggles) {
      Serial.printf("  点滅: 点灯 %lu ms / 消灯 %lu ms  切り替え %lu 回  遅れ 最大 %lu ms  %s", (unsigned long)bs.onMs, (unsigned long)bs.offMs,
                    (unsigned long)bs.toggles, (unsigned long)bs.lateMaxMs, bs.lateOver ? "NG" : "OK");
      if (bs.lateOver) Serial.printf("(消灯の時間の 2 割を超えた遅れ %lu 回)", (unsigned long)bs.lateOver);
      Serial.println();
    }
    // CAN の生データから値を作っているとき(c6_can_link 2.0.0): 作った値と、燃料のなまし・ギアの推定の中身
    if (mlink::usingRaw(now)) {
      veh::FuelInfo fi;
      veh::fuelInfo(&fi, now);
      char gearText[4] = "–";
      if (s.gearOk) {
        gearText[0] = s.gear;
        gearText[1] = 0;
      }
      Serial.printf("  車の値(CAN から): rpm %s%d 車速 %s%.1f アクセル %s%.0f%% 水温 %s%.0f ブースト %s%.2f 燃料 %s%.1f%% ギア %s(比 %.1f) | 燃料 生 %u hp %.1f lp %.1f %s%s%s"
                    " b5=0 を捨てた %lu 回 給油 %lu 回\n",
                    s.rpmOk ? "" : "×", s.rpm, s.speedOk ? "" : "×", (double)s.speedKmh, s.throttleOk ? "" : "×", (double)s.throttlePct, s.waterOk ? "" : "×",
                    (double)s.waterC, s.boostOk ? "" : "×", (double)s.boostBar, s.fuelOk ? "" : "×", (double)s.fuelPct, gearText, (double)veh::lastRatio(), fi.raw,
                    (double)fi.hp, (double)fi.lp, fi.stable ? "安定" : "揺れ", fi.armed ? " 停止後" : "", fi.restored ? " 前回の値から" : "", (unsigned long)fi.zeroFrames,
                    (unsigned long)fi.refuels);
    }
    if (g_sys.rc.enabled) rcPrintStatusLine();
  }
  delay(1);
}

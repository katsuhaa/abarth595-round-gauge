// unit_main.cpp - PC 上の単体試験: UART データの受け取り方、画面の操作(タップ)、設定、自動で戻る、
//                 車の CAN の生データから値を作る(veh。../logs/ の 10/22 の実車ログも流す)、受信の状態の「描画」(2.0.1)
#include "../meter_display/app.h"
#include "../meter_display/gfx.h"
#include "../meter_display/ui.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

extern int g_hostBrightness, g_hostSaves, g_hostImuResets, g_hostBlank, g_hostBlinks, g_hostLevel, g_hostTimeSaves;
extern bool g_hostSaveOk;
static uint16_t fb[gfx::W * gfx::H];
static float scratch[gfx::W * (gfx::H + 2)];
static Settings set = { SET_SHIFT_DEFAULT, SET_AUTORET_DEFAULT, SET_BRIGHT_DEFAULT, SET_WARN_DEFAULT, SET_DEMO_DEFAULT, SET_RC_DEFAULT };
static LinkStats ls;
static PerfStats ps;
static SysInfo si;
static int fails = 0, passes = 0;
#define CHECK(cond, what) do { if (cond) passes++; else { fails++; printf("FAIL: %s  (%s:%d)\n", what, __FILE__, __LINE__); } } while (0)

static std::vector<uint8_t> frame(uint8_t type, uint8_t seq, const void *p, uint8_t len) {
  uint8_t buf[ML_MAX_PAYLOAD + ML_OVERHEAD];
  size_t n = mlBuild(buf, type, seq, p, len);
  return std::vector<uint8_t>(buf, buf + n);
}
static MlSignals sigMsg(uint16_t rpm) {
  MlSignals s;
  memset(&s, 0, sizeof(s));
  s.valid = ML_V_RPM | ML_V_SPEED | ML_V_THROTTLE | ML_V_WATER | ML_V_FUEL | ML_V_GEAR | ML_V_BOOST;
  s.rpm = rpm; s.speed_c = 6000; s.throttle_h = 200; s.fuel_h = 124; s.water_d = 925; s.gear = '3'; s.boost_c = -45;
  return s;
}
static void upd(const Signals &s, uint32_t now) {
  ui::update(s, ls, ps, si, now);
  ui::update(s, ls, ps, si, now + 60);
  gfx::dirtyClear();
}

int main() {
  gfx::begin(fb, scratch);
  mlink::begin();
  cantab::begin();
  ui::begin(&set);
  memset(&ps, 0, sizeof(ps));
  memset(&si, 0, sizeof(si));
  LinkStats st;
  Signals s;

  // ---------------- 1. 受け取り: CRC が合っている値のフレーム
  {
    MlSignals m = sigMsg(4200);
    auto f = frame(ML_TYPE_SIGNALS, 0, &m, sizeof(m));
    mlink::feed(f.data(), f.size(), 1000);
    mlink::getSignals(&s, 1000);
    CHECK(s.linkUp && s.rpmOk && s.rpm == 4200, "値のフレームを受ける");
    CHECK(s.waterOk && s.waterC > 92.4f && s.waterC < 92.6f && s.fuelPct == 62.0f && s.gear == '3', "水温 0.1℃・燃料 0.5% の換算");
    CHECK(s.speedKmh == 60.0f && s.throttlePct == 100.0f, "車速・アクセルの換算");
    CHECK(s.boostOk && s.boostBar > -0.451f && s.boostBar < -0.449f, "ブースト圧 0.01 bar の換算(負圧)");
    mlink::getSignals(&s, 1000 + LINK_TIMEOUT_MS);
    CHECK(s.linkUp, "LINK_TIMEOUT_MS ちょうどまでは受信あり");
    mlink::getSignals(&s, 1001 + LINK_TIMEOUT_MS);
    CHECK(!s.linkUp && !s.rpmOk, "LINK_TIMEOUT_MS を過ぎたら信号なし");
  }
  // ---------------- 2. 1 バイトずつ届いても同じ結果
  {
    MlSignals m = sigMsg(5100);
    auto f = frame(ML_TYPE_SIGNALS, 1, &m, sizeof(m));
    for (uint8_t b : f) mlink::feed(&b, 1, 2000);
    mlink::getSignals(&s, 2000);
    CHECK(s.rpm == 5100, "1 バイトずつ届いたフレーム");
  }
  // ---------------- 3. ごみ(XIAO の起動ログ)に続くフレーム、ごみに同期と同じ並びが混じる場合
  {
    mlink::getStats(&st);
    uint32_t crc0 = st.crcErrors, ok0 = st.framesOk;
    std::vector<uint8_t> w;
    const char *boot = "ESP-ROM:esp32c6-20220919\r\nBuild:Sep 19 2022\r\nrst:0x1 (POWERON),boot:0xc (SPI_FAST_FLASH_BOOT)\r\n";
    w.insert(w.end(), boot, boot + strlen(boot));
    const uint8_t fake[] = { 0xA5, 0x5A, 0x01, 0x07, (uint8_t)sizeof(MlSignals), 1, 2, 3 };  // 途中で切れた偽のフレーム(長さは値のフレームと同じ)
    w.insert(w.end(), fake, fake + sizeof(fake));
    MlSignals m = sigMsg(3300);
    auto f = frame(ML_TYPE_SIGNALS, 2, &m, sizeof(m));
    w.insert(w.end(), f.begin(), f.end());
    auto f2 = frame(ML_TYPE_SIGNALS, 3, &m, sizeof(m));
    w.insert(w.end(), f2.begin(), f2.end());
    mlink::feed(w.data(), w.size(), 3000);
    mlink::getSignals(&s, 3000);
    mlink::getStats(&st);
    CHECK(s.rpm == 3300 && st.framesOk == ok0 + 2, "ごみの後ろのフレームを 2 つとも拾う");
    CHECK(st.crcErrors == crc0 + 1, "途中で切れた偽のフレームは CRC 誤り 1 回");
    CHECK(st.junkBytes > 50, "ごみは捨てたバイトとして数える");
  }
  // ---------------- 4. 壊れたフレームと連番の飛び
  {
    mlink::getStats(&st);
    uint32_t crc0 = st.crcErrors, lost0 = st.lostFrames, ok0 = st.framesOk;
    MlSignals m = sigMsg(2500);
    auto bad = frame(ML_TYPE_SIGNALS, 4, &m, sizeof(m));
    bad[10] ^= 0x04;  // 中身を 1 ビット壊す
    mlink::feed(bad.data(), bad.size(), 3100);
    auto good = frame(ML_TYPE_SIGNALS, 8, &m, sizeof(m));  // 4〜7 が届かなかった
    mlink::feed(good.data(), good.size(), 3120);
    mlink::getStats(&st);
    mlink::getSignals(&s, 3120);
    CHECK(st.crcErrors == crc0 + 1, "1 ビット壊れたフレームは CRC 誤り");
    CHECK(st.framesOk == ok0 + 1 && s.rpm == 2500, "壊れたフレームは使わず、次のフレームを使う");
    CHECK(st.lostFrames == lost0 + 4, "連番の飛び 4 つぶんを欠落として数える");
    // 長い切れ目(受信なし)の後は、連番の飛びを数えない
    auto later = frame(ML_TYPE_SIGNALS, 200, &m, sizeof(m));
    mlink::feed(later.data(), later.size(), 3120 + LINK_TIMEOUT_MS + 500);
    mlink::getStats(&st);
    CHECK(st.lostFrames == lost0 + 4, "受信が途切れた後の最初のフレームでは欠落を数えない");
  }
  // ---------------- 5. CAN の生データ
  {
    uint8_t p[sizeof(MlCanHead) + 8];
    MlCanHead h;
    h.info = 0x40 | 8; h.id = 0x0618A001;
    uint8_t d[8] = { 0, 0x19, 0x10, 0x68, 7, 0x18, 0x46, 0x00 };
    memcpy(p, &h, sizeof(h)); memcpy(p + sizeof(h), d, 8);
    auto f = frame(ML_TYPE_CAN, 201, p, sizeof(p));
    mlink::feed(f.data(), f.size(), 5000);
    h.info = 0x80 | 0x40 | 4; h.id = 0x04214001;
    memcpy(p, &h, sizeof(h));
    auto f2 = frame(ML_TYPE_CAN, 202, p, sizeof(MlCanHead) + 4);
    mlink::feed(f2.data(), f2.size(), 5000);
    cantab::View v;
    CHECK(cantab::count() == 2, "CAN の ID を 2 つ覚える");
    CHECK(cantab::getView(0, &v) && v.id == 0x0618A001 && v.bus == 0 && v.ext == 1 && v.dlc == 8 && v.data[3] == 0x68, "C-CAN のフレームの中身");
    CHECK(cantab::getView(1, &v) && v.id == 0x04214001 && v.bus == 1 && v.dlc == 4, "B-CAN のフレーム(長さ 4)");
    // 「変化を記録」(10/23): 1 回目 = 全部が候補、2 回目から = 1〜CAN_REC_MAX_CHANGES 回だけ変わったバイトの ID に絞る
    uint16_t ce[8];
    uint8_t cm[8];
    CHECK(cantab::recordCount() == 0 && cantab::candidates(ce, cm, 8) == 0, "記録 0 回: 候補なし");
    CHECK(cantab::record() == 2 && cantab::recordCount() == 1, "1 回目の記録: 2 つとも候補");
    d[7] = 0x4C;
    h.info = 0x40 | 8; h.id = 0x0618A001;
    memcpy(p, &h, sizeof(h)); memcpy(p + sizeof(h), d, 8);
    auto f3 = frame(ML_TYPE_CAN, 203, p, sizeof(p));
    mlink::feed(f3.data(), f3.size(), 5100);
    CHECK(cantab::record() == 1, "2 回目: 1 回だけ変わった 0618A001 だけが残る");
    int n = cantab::candidates(ce, cm, 8);
    CHECK(n == 1 && ce[0] == 0 && cm[0] == 0x80, "候補は 0618A001 の b7");
    for (int i = 0; i < CAN_REC_MAX_CHANGES + 2; i++) {  // 動き続ける(回数が多い)と外れる
      d[7] = (uint8_t)(0x10 + i);
      cantab::onFrame(0, 1, 0x0618A001, 8, d, 5200 + i);
    }
    CHECK(cantab::record() == 0, "3 回目: CAN_REC_MAX_CHANGES 回より多く変わったバイトは外れる");
    int hist[CAN_REC_HIST];
    int nh = cantab::recordHistory(hist);
    CHECK(nh == 3 && hist[0] == 2 && hist[1] == 1 && hist[2] == 0, "件数の移り変わり 2 → 1 → 0");
    cantab::clearRecord();
    CHECK(cantab::recordCount() == 0 && cantab::candidates(ce, cm, 8) == 0 && cantab::recordHistory(hist) == 0, "やり直すと記録 0 回");
    // 長さが合わない CAN フレームは受け付けない
    mlink::getStats(&st);
    uint32_t ok0 = st.framesOk;
    h.info = 0x40 | 8;
    memcpy(p, &h, sizeof(h));
    auto f4 = frame(ML_TYPE_CAN, 204, p, sizeof(MlCanHead) + 3);
    mlink::feed(f4.data(), f4.size(), 5200);
    mlink::getStats(&st);
    CHECK(st.framesOk == ok0, "長さの合わない CAN フレームは使わない");
  }

  // ---------------- 6. 画面の操作(2.1.0): サーキット・タコ・データは左右のスワイプで画面を替え、長押しで設定。短いタップは何もしない。
  //                  設定の中は上・下の帯と左右の半分
  Signals live;
  memset(&live, 0, sizeof(live));
  live.linkUp = live.rpmOk = live.waterOk = live.fuelOk = live.gearOk = live.speedOk = live.throttleOk = true;
  live.rpm = 3000; live.waterC = 90; live.fuelPct = 50; live.gear = '2';
  const int TOPX = 233, TOPY = 60, BOTX = 233, BOTY = 400, LX = 100, RX = 366, MY = 233;
  uint32_t t = 10000;
  // 長押し(HOLD_MS)で設定に入って離す
  auto longPress = [&](const Signals &sg, uint32_t &tt) {
    ui::touchDown(233, 233, tt += 100);
    ui::update(sg, ls, ps, si, tt + HOLD_SHOW_MS + 10);
    ui::update(sg, ls, ps, si, tt + HOLD_MS);
    ui::touchUp(tt += HOLD_MS + 50);
    upd(sg, tt);
  };
  // 真ん中から (dx, dy) だけ指を動かして離す(途中の知らせ 2 回、全部で 100 ms)
  auto swipe = [&](const Signals &sg, uint32_t &tt, int dx, int dy) {
    ui::touchDown(233, 233, tt += 100);
    ui::update(sg, ls, ps, si, tt + 20);
    ui::touchMove(233 + dx / 2, 233 + dy / 2, tt + 40);
    ui::touchMove(233 + dx, 233 + dy, tt + 80);
    ui::update(sg, ls, ps, si, tt + 80);
    ui::touchUp(tt += 100);
    upd(sg, tt);
  };
  ui::testShow(ui::SC_DRIVE);
  upd(live, t);
  CHECK(ui::screen() == ui::SC_DRIVE, "サーキットから");
  // スワイプ: 左へ = 次、右へ = 前(サーキット → タコ → データ → サーキット)
  swipe(live, t, -120, 10);
  CHECK(ui::screen() == ui::SC_BOOST, "スワイプ: 左へでタコ(次)");
  swipe(live, t, -120, -20);
  CHECK(ui::screen() == ui::SC_DATA, "スワイプ: 左へでデータ(次)");
  swipe(live, t, -120, 0);
  CHECK(ui::screen() == ui::SC_DRIVE, "スワイプ: データの次はサーキット");
  swipe(live, t, 120, 0);
  CHECK(ui::screen() == ui::SC_DATA, "スワイプ: 右へで前(サーキットの前はデータ)");
  swipe(live, t, 120, 0);
  CHECK(ui::screen() == ui::SC_BOOST && !ui::testHolding(), "スワイプ: 右へで前(データの前はタコ)。長押しの表示は出ない");
  swipe(live, t, -(SWIPE_MIN_PX - 10), 0);
  CHECK(ui::screen() == ui::SC_BOOST, "スワイプ: 横の動きが SWIPE_MIN_PX 未満なら替えない");
  swipe(live, t, -100, 60);
  CHECK(ui::screen() == ui::SC_BOOST, "スワイプ: 斜め(横が縦の SWIPE_DIR_RATIO 倍未満)なら替えない");
  swipe(live, t, 0, -150);
  CHECK(ui::screen() == ui::SC_BOOST, "スワイプ: 縦は何もしない");
  // 短いタッチ(タップ)は何もしない(すぐ設定に入ると、元に戻るのが面倒なため)
  ui::touchDown(233, 233, t += 100);
  ui::update(live, ls, ps, si, t + 50);
  ui::touchUp(t += 150);
  upd(live, t);
  CHECK(ui::screen() == ui::SC_BOOST && !ui::testHolding(), "タコ: 短いタッチは何もしない");
  ui::tap(233, 233, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_BOOST, "タコ: タップ(ui::tap)も何もしない");
  // 長押し: HOLD_SHOW_MS から暗くして「設定」、HOLD_MS で設定へ。押したままでも画面は回らず、離してもタップにしない
  ui::touchDown(233, 233, t += 100);
  ui::update(live, ls, ps, si, t + HOLD_SHOW_MS - 10);
  CHECK(!ui::testHolding(), "長押し: HOLD_SHOW_MS より前は表示を出さない");
  ui::touchMove(236, 231, t + HOLD_SHOW_MS);  // HOLD_CANCEL_PX 未満のずれは取り消さない
  ui::update(live, ls, ps, si, t + HOLD_SHOW_MS + 10);
  CHECK(ui::testHolding() && ui::screen() == ui::SC_BOOST, "長押し: 押している間は表示を出す(まだ設定に入らない)");
  ui::update(live, ls, ps, si, t + HOLD_MS);
  CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 0, "長押し HOLD_MS で設定の 1 項目目");
  ui::update(live, ls, ps, si, t + 3 * HOLD_MS);
  CHECK(ui::screen() == ui::SC_SET_LIST, "押したままでも設定のまま(画面は回らない)");
  ui::touchUp(t += 3 * HOLD_MS + 50);
  upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 0 && !ui::testHolding(), "離してもタップにしない(設定の項目はそのまま)");
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_BOOST, "設定: 上の帯「設定」で来た画面(タコ)へ");
  // 長押しの取り消し: 表示を出したあと HOLD_CANCEL_PX 以上動いたら、表示を消して設定に入らない
  ui::touchDown(233, 233, t += 100);
  ui::update(live, ls, ps, si, t + HOLD_SHOW_MS + 10);
  CHECK(ui::testHolding(), "取り消し: まず長押しの表示が出る");
  ui::touchMove(233, 233 + HOLD_CANCEL_PX, t + HOLD_SHOW_MS + 20);
  ui::update(live, ls, ps, si, t + HOLD_SHOW_MS + 30);
  CHECK(!ui::testHolding() && ui::screen() == ui::SC_BOOST, "取り消し: HOLD_CANCEL_PX 動いたら表示を消す");
  ui::update(live, ls, ps, si, t + 2 * HOLD_MS);
  CHECK(ui::screen() == ui::SC_BOOST && !ui::testHolding(), "取り消し: HOLD_MS を過ぎても設定に入らない");
  ui::touchUp(t += 2 * HOLD_MS + 10);
  upd(live, t);
  CHECK(ui::screen() == ui::SC_BOOST, "取り消し: 離しても何もしない(縦の動きはスワイプでない)");
  gfx::dirtyClear();
  // 設定(長押しで入る。前回見ていた項目から)
  longPress(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 0, "長押しで設定へ");
  ui::tap(RX, MY, t += 100); upd(live, t);
  ui::tap(RX, MY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 2, "設定: 右半分で次の項目");
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  longPress(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 2, "設定は前回見ていた項目から(電源を切るまで覚える)");
  ui::tap(LX, MY, t += 100); upd(live, t);
  ui::tap(LX, MY, t += 100); upd(live, t);
  ui::tap(LX, MY, t += 100); upd(live, t);
  CHECK(ui::testSetItem() == 9, "左半分で前へ(最初から最後の CAN確認に回る)");
  // CAN確認(10 項目目)
  {
    uint8_t d[8] = { 0, 0x19, 0x10, 0x68, 7, 0x18, 0x46, 0x77 };
    cantab::onFrame(0, 1, 0x0618A001, 8, d, t);
    ui::tap(BOTX, BOTY, t += 100); upd(live, t);
    CHECK(ui::screen() == ui::SC_CAN, "CAN確認: 下「開く」で CAN の画面");
    ui::tap(RX, MY, t += 100); upd(live, t);
    CHECK(ui::screen() == ui::SC_CAN, "記録 0 回では一覧へ行かない");
    ui::tap(BOTX, BOTY, t += 100); upd(live, t);
    CHECK(cantab::recordCount() == 1, "下「変化を記録」で 1 回目の記録");
    d[7] = 0x78;
    cantab::onFrame(0, 1, 0x0618A001, 8, d, t);
    ui::tap(BOTX, BOTY, t += 100); upd(live, t);
    CHECK(cantab::recordCount() == 2, "2 回目の記録");
    ui::tap(RX, MY, t += 100); upd(live, t);
    CHECK(ui::screen() == ui::SC_CAN_ID, "右半分「一覧」で 1 件ずつの画面");
    ui::tap(BOTX, BOTY, t += 100); upd(live, t);
    CHECK(ui::screen() == ui::SC_CAN_DETAIL, "下「詳細」で詳細");
    ui::tap(RX, MY, t += 100); upd(live, t);
    upd(live, t += 4 * 60000);
    CHECK(ui::screen() == ui::SC_CAN_DETAIL, "詳細: 左右はバイトを選ぶだけ。4 分たっても自動では戻らない");
    ui::tap(TOPX, TOPY, t += 100); upd(live, t);
    CHECK(ui::screen() == ui::SC_CAN_ID, "詳細: 上「‹ 戻る」で一覧");
    ui::tap(TOPX, TOPY, t += 100); upd(live, t);
    CHECK(ui::screen() == ui::SC_CAN, "一覧: 上「‹ 戻る」で CAN の画面");
    ui::tap(LX, MY, t += 100); upd(live, t);
    CHECK(ui::screen() == ui::SC_CAN_REDO, "左半分「やり直す」で確認");
    ui::tap(TOPX, TOPY, t += 100); upd(live, t);
    CHECK(ui::screen() == ui::SC_CAN && cantab::recordCount() == 2, "確認: 上「‹ 戻る」は消さない");
    ui::tap(LX, MY, t += 100); upd(live, t);
    ui::tap(BOTX, BOTY, t += 100); upd(live, t);
    CHECK(ui::screen() == ui::SC_CAN && cantab::recordCount() == 0, "確認: 下「やり直す」で記録を消す");
    ui::tap(TOPX, TOPY, t += 100); upd(live, t);
    CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 9, "CAN の画面: 上「‹ 戻る」で設定の CAN確認");
  }

  // ---------------- 7. 設定の変更(下「変更」→ 左右で − +、下「決定」で保存、上「‹ 戻る」で変えずに戻る)
  ui::tap(RX, MY, t += 100); upd(live, t);
  CHECK(ui::testSetItem() == 0, "最後の次は最初(シフトポイント)");
  ui::tap(BOTX, BOTY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_EDIT, "設定: 下「変更」で変更の画面へ");
  ui::tap(RX, MY, t += 100); upd(live, t);
  CHECK(set.shiftRpm == SET_SHIFT_DEFAULT + 100, "右半分で 100 上がる");
  ui::tap(LX, MY, t += 100); ui::tap(LX, MY, t += 100); upd(live, t);
  CHECK(set.shiftRpm == SET_SHIFT_DEFAULT - 100, "左半分 2 回で 200 下がる");
  int saves0 = g_hostSaves;
  ui::tap(BOTX, BOTY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && g_hostSaves == saves0 + 1, "下「決定」で保存して項目の画面へ");
  ui::tap(BOTX, BOTY, t += 100); upd(live, t);
  ui::tap(BOTX, BOTY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && g_hostSaves == saves0 + 1, "変えずに決定したときは保存しない");
  ui::tap(BOTX, BOTY, t += 100); upd(live, t);
  ui::tap(RX, MY, t += 100); ui::tap(RX, MY, t += 100);
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && set.shiftRpm == SET_SHIFT_DEFAULT - 100 && g_hostSaves == saves0 + 1, "上「‹ 戻る」は変えずに戻る(保存しない)");
  // 上下限
  ui::tap(BOTX, BOTY, t += 100);
  for (int i = 0; i < 60; i++) ui::tap(RX, MY, t += 10);
  CHECK(set.shiftRpm == SET_SHIFT_MAX, "上限で止まる");
  for (int i = 0; i < 80; i++) ui::tap(LX, MY, t += 10);
  CHECK(set.shiftRpm == SET_SHIFT_MIN, "下限で止まる");
  // 保存に失敗: 1 回目の「決定」では留まり、2 回目で抜ける
  g_hostSaveOk = false;
  upd(live, t);
  ui::tap(BOTX, BOTY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_EDIT, "保存に失敗したら変更画面に留まる");
  ui::tap(BOTX, BOTY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST, "もう一度「決定」で抜ける");
  g_hostSaveOk = true;
  set.shiftRpm = SET_SHIFT_DEFAULT;
  // 明るさ(3 項目目): 変えたその場で反映、「‹ 戻る」で元に戻る
  ui::tap(RX, MY, t += 100); ui::tap(RX, MY, t += 100);
  ui::tap(BOTX, BOTY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_EDIT && ui::testSetItem() == 2, "3 項目目(明るさ)の変更");
  ui::tap(LX, MY, t += 100); upd(live, t);
  CHECK(set.brightnessPct == SET_BRIGHT_DEFAULT - SET_BRIGHT_STEP && g_hostBrightness == set.brightnessPct, "明るさは左半分でその場で変わる");
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  CHECK(set.brightnessPct == SET_BRIGHT_DEFAULT && g_hostBrightness == SET_BRIGHT_DEFAULT, "上「‹ 戻る」で明るさも元に戻る");
  // 時間の記録(7 項目目)
  for (int i = 0; i < 4; i++) ui::tap(RX, MY, t += 100);
  upd(live, t);
  CHECK(ui::testSetItem() == 6, "7 項目目 = 時間の記録");
  trip::setTimes(5000, 7000);
  ui::tap(BOTX, BOTY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_RESET, "下「リセット」で確認の画面");
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && trip::aboveMs() == 5000, "確認: 上「‹ 戻る」は消さない");
  int ts0 = g_hostTimeSaves;
  ui::tap(BOTX, BOTY, t += 100); upd(live, t);
  ui::tap(BOTX, BOTY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && trip::aboveMs() == 0 && trip::belowMs() == 0 && g_hostTimeSaves == ts0 + 1, "確認: 下「リセット」で 0 に戻して、すぐ保存");
  // 受信の状態(8 項目目)
  ui::tap(RX, MY, t += 100); upd(live, t);
  ui::tap(BOTX, BOTY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_INFO, "8 項目目の下「開く」で受信の状態");
  ui::tap(RX, MY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_INFO, "受信の状態: 右半分で次の項目(画面はそのまま)");
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 7, "受信の状態: 上「‹ 戻る」で設定の受信の状態へ");
  ui::tap(RX, MY, t += 100); upd(live, t);
  ui::tap(BOTX, BOTY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_RC, "9 項目目の下「開く」で RaceChrono の状態");
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_BOOST, "設定: 上の帯「設定」で来た画面へ");

  // ---------------- 8. 覚える画面と、設定から時間で戻る(2.3)
  CHECK(ui::homeScreen() == ui::SC_DRIVE, "まだ 1 分続けて見ていないので、覚えた画面はサーキットのまま");
  upd(live, t += HOME_CONFIRM_MS + 100);
  CHECK(ui::homeScreen() == ui::SC_BOOST, "タコを 1 分見ていたら覚える");
  upd(live, t += 4 * 60000);
  CHECK(ui::screen() == ui::SC_BOOST, "タコ: 4 分たっても自動では変わらない");
  swipe(live, t, -120, 0);
  CHECK(ui::screen() == ui::SC_DATA, "左へスワイプでデータ");
  longPress(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST, "データ: 長押しで設定");
  upd(live, t + 3 * 60000 - 2000);
  CHECK(ui::screen() == ui::SC_SET_LIST, "3 分たつ前は戻らない");
  upd(live, t + 3 * 60000 + 200);
  CHECK(ui::screen() == ui::SC_BOOST, "3 分で覚えた画面(タコ)に戻る(来た画面のデータではない)");
  t += 3 * 60000 + 1000;
  set.autoReturnMin = 1;
  longPress(live, t);
  ui::testShow(ui::SC_SET_LIST, 0);
  ui::tap(BOTX, BOTY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_EDIT, "設定の変更画面へ");
  ui::tap(RX, MY, t += 100); upd(live, t);
  saves0 = g_hostSaves;
  upd(live, t + 61000);
  CHECK(ui::screen() == ui::SC_BOOST && g_hostSaves == saves0 + 1, "変更画面からも 1 分(設定値)で戻り、変えた値は保存する");
  set.autoReturnMin = SET_AUTORET_DEFAULT;
  set.shiftRpm = SET_SHIFT_DEFAULT;
  // サーキットを覚え直しておく(あとの試験のため)
  ui::testShow(ui::SC_DRIVE);
  upd(live, t += 100000);
  upd(live, t += HOME_CONFIRM_MS + 100);
  CHECK(ui::homeScreen() == ui::SC_DRIVE, "サーキットを 1 分見て覚え直す");

  // ---------------- 9. 全開の時間と走行時間(仕様書 6.3)
  {
    Signals d;
    memset(&d, 0, sizeof(d));
    d.linkUp = d.speedOk = d.throttleOk = true;
    uint32_t tt = t + 100000;
    d.speedKmh = 0; d.throttlePct = 100;
    trip::update(d, tt);
    uint32_t m0 = trip::movingMs(), w0 = trip::wotMs();
    for (int i = 0; i < 100; i++) trip::update(d, tt += 20);          // 停車中に全開 2 秒: どちらも数えない
    CHECK(trip::movingMs() == m0 && trip::wotMs() == w0, "停車中は走行時間も全開の時間も増えない");
    d.speedKmh = 40; d.throttlePct = 50;
    for (int i = 0; i < 300; i++) trip::update(d, tt += 20);          // 走行 6 秒、アクセル 50%
    CHECK(trip::movingMs() == m0 + 6000 && trip::wotMs() == w0, "走行中(アクセル 50%)は走行時間だけ増える");
    d.throttlePct = 96;
    for (int i = 0; i < 100; i++) trip::update(d, tt += 20);          // 全開 2 秒
    CHECK(trip::movingMs() == m0 + 8000 && trip::wotMs() == w0 + 2000, "全開(95% 以上)の間は両方増える");
    d.speedOk = false;
    for (int i = 0; i < 100; i++) trip::update(d, tt += 20);          // 車速の信号なし
    CHECK(trip::movingMs() == m0 + 8000 && trip::wotMs() == w0 + 2000, "車速の信号がない間は数えない");
  }

  // ---------------- 10. 受信の速度の自動合わせ
  {
    const uint32_t cand[] = METER_LINK_BAUD_CANDIDATES;
    MlSignals m = sigMsg(3000);
    uint8_t junk[64];
    for (int i = 0; i < 64; i++) junk[i] = (uint8_t)(i * 37 + 11) | 0x80;  // 同期の並び(A5 5A)を含まないごみ
    uint8_t sq = 0;
    // (a) 何も届かない: 速度を変えない
    mlink::begin();
    uint32_t tt = 1000, sw = 0;
    for (int i = 0; i < 2000; i++) sw += mlink::autoBaud(tt += 5, 0) ? 1 : 0;
    mlink::getStats(&st);
    CHECK(sw == 0 && mlink::baud() == cand[0] && !st.locked && st.baud == cand[0], "何も届かないときは速度を変えない");
    // (b) XIAO の起動ログ(短いごみ)→ 0.4 秒の無音 → 正しいフレーム: 速度を変えない
    mlink::begin();
    tt = 1000; sw = 0;
    for (int i = 0; i < 6; i++) {  // 30 ms の間に 384 バイトのごみ
      mlink::feed(junk, sizeof(junk), tt);
      sw += mlink::autoBaud(tt += 5, sizeof(junk)) ? 1 : 0;
    }
    for (int i = 0; i < 80; i++) sw += mlink::autoBaud(tt += 5, 0) ? 1 : 0;
    for (int i = 0; i < 150; i++) {  // 3 秒ぶんのフレーム
      auto f = frame(ML_TYPE_SIGNALS, sq++, &m, sizeof(m));
      mlink::feed(f.data(), f.size(), tt);
      for (int k = 0; k < 4; k++) sw += mlink::autoBaud(tt += 5, k == 0 ? f.size() : 0) ? 1 : 0;
    }
    mlink::getStats(&st);
    CHECK(sw == 0 && st.locked && st.baud == cand[0], "起動ログのあとに正しいフレームが来れば速度を変えない");
    // (c) 区間の境目をまたぐごみ → 2 秒の無音 → 正しいフレーム: 速度を変えない
    mlink::begin();
    tt = 1000; sw = 0;
    mlink::autoBaud(tt, 0);  // 区間の始まり = 1000
    tt += LINK_BAUD_WINDOW_MS - 15;
    for (int i = 0; i < 6; i++) {
      mlink::feed(junk, sizeof(junk), tt);
      sw += mlink::autoBaud(tt += 5, sizeof(junk)) ? 1 : 0;
    }
    for (int i = 0; i < 400; i++) sw += mlink::autoBaud(tt += 5, 0) ? 1 : 0;
    {
      auto f = frame(ML_TYPE_SIGNALS, sq++, &m, sizeof(m));
      mlink::feed(f.data(), f.size(), tt);
      sw += mlink::autoBaud(tt += 5, f.size()) ? 1 : 0;
    }
    mlink::getStats(&st);
    CHECK(sw == 0 && st.locked && st.baud == cand[0], "区間をまたぐ短いごみでは速度を変えない");
    // (d) ごみが続く(速度が合っていない): 1.5 秒ごとに次の候補へ移り、一巡して最初に戻る
    mlink::begin();
    tt = 1000; sw = 0;
    uint32_t got[8] = { 0 }, at[8] = { 0 };
    for (int i = 0; i < 1300 && sw < 8; i++) {  // 6.5 秒
      mlink::feed(junk, 40, tt);
      uint32_t nb = mlink::autoBaud(tt += 5, 40);
      if (nb) {
        got[sw] = nb;
        at[sw] = tt - 1000;
        sw++;
      }
    }
    CHECK(sw == 4 && got[0] == cand[1] && got[1] == cand[2] && got[2] == cand[3] && got[3] == cand[0], "ごみが続くと候補を順に試し、一巡する");
    CHECK(at[0] >= 1400 && at[0] <= 1600 && at[1] - at[0] >= 1400 && at[1] - at[0] <= 1600, "移るまでの時間は約 1.5 秒");
    mlink::getStats(&st);
    CHECK(!st.locked && st.baud == cand[0], "合うまでは「受信なし」");
    // (e) 合った速度で正しいフレームが来れば、そこで止まる。ごみが混じっても移らない
    sw = 0;
    for (int i = 0; i < 300; i++) {  // 6 秒。フレームごとにごみも 20 バイト混ぜる
      auto f = frame(ML_TYPE_SIGNALS, sq++, &m, sizeof(m));
      mlink::feed(junk, 20, tt);
      mlink::feed(f.data(), f.size(), tt);
      for (int k = 0; k < 4; k++) sw += mlink::autoBaud(tt += 5, k == 0 ? f.size() + 20 : 0) ? 1 : 0;
    }
    mlink::getStats(&st);
    CHECK(sw == 0 && st.locked && st.baud == cand[0], "正しいフレームが来ていれば、ごみが混じっても移らない");
    // (f) 送信が止まる: 「受信なし」になるが、速度は変えない
    for (int i = 0; i < 2000; i++) sw += mlink::autoBaud(tt += 5, 0) ? 1 : 0;
    mlink::getStats(&st);
    CHECK(sw == 0 && !st.locked && st.baud == cand[0], "送信が止まっても速度は変えない");
  }

  // ---------------- 11. 受信タスクと loop() の時刻のずれ、長い途切れ、保存中の誤りの扱い、バイト数
  {
    mlink::begin();
    MlSignals m = sigMsg(4100);
    uint8_t sq = 0;
    // loop() が now を読んだあとに届いたフレーム(受信タスクの時刻のほうが新しい)でも「受信あり」
    auto f = frame(ML_TYPE_SIGNALS, sq++, &m, sizeof(m));
    mlink::feed(f.data(), f.size(), 7003);
    mlink::getSignals(&s, 7000);
    CHECK(s.linkUp && s.rpm == 4100, "now より 3 ms 新しいフレームでも「受信あり」のまま");
    mlink::getSignals(&s, 7003 + LINK_TIMEOUT_MS + 1);
    CHECK(!s.linkUp, "そのあと途切れれば「信号なし」");
    // 長く途切れたあと、時刻が大きく進んでも「受信あり」に戻らない
    for (uint32_t tt = 7003; tt <= 7003 + 4000; tt += 5) mlink::tick(tt);
    mlink::getSignals(&s, 7003 + 0x7FFFFF00u);
    CHECK(!s.linkUp, "途切れたまま 24 日たっても「信号なし」");
    mlink::getSignals(&s, 7003 + 0x80000100u);
    CHECK(!s.linkUp, "途切れたまま 25 日たっても「信号なし」(時刻の差の符号が変わる所)");
    // 設定の保存中(grace)の CRC 誤りと欠落は数えない。終わったあとは数える
    uint32_t tt = 20000;
    auto good = [&](uint8_t seqNo) { auto g = frame(ML_TYPE_SIGNALS, seqNo, &m, sizeof(m)); mlink::feed(g.data(), g.size(), tt); };
    auto bad = [&](uint8_t seqNo) { auto g = frame(ML_TYPE_SIGNALS, seqNo, &m, sizeof(m)); g[9] ^= 0x10; mlink::feed(g.data(), g.size(), tt); };
    good(10);
    mlink::getStats(&st);
    uint32_t c0 = st.crcErrors, l0 = st.lostFrames;
    mlink::graceUntil(tt + 300);
    tt += 100; bad(11);
    tt += 20; good(15);  // 12〜14 が落ちた
    mlink::getStats(&st);
    CHECK(st.crcErrors == c0 && st.lostFrames == l0, "保存中の CRC 誤りと欠落は数えない");
    tt += 400; bad(16);
    tt += 20; good(19);  // 17・18 が落ちた(16 は壊れていたので 3 つぶん)
    mlink::getStats(&st);
    CHECK(st.crcErrors == c0 + 1 && st.lostFrames == l0 + 3, "保存が終わったあとの誤りと欠落は数える");
    // 1 秒あたりのバイト数
    mlink::begin();
    uint8_t junk[50];
    memset(junk, 0x33, sizeof(junk));
    mlink::tick(30000);
    for (int i = 0; i < 10; i++) mlink::feed(junk, sizeof(junk), 30100 + i * 50);
    mlink::tick(31000);
    mlink::getStats(&st);
    CHECK(st.bytesPerSec == 500 && st.framesPerSec == 0, "届いたバイト数(フレームにならなくても数える)");
  }

  // ---------------- 12. 画面全体の点滅(BLINK_WHOLE_SCREEN)。シフト点以上では点灯 BLINK_ON_FAST_MS、消灯 BLINK_OFF_MS
  {
    Signals b;
    memset(&b, 0, sizeof(b));
    b.linkUp = b.rpmOk = true;
    b.rpm = set.shiftRpm + 100;  // シフト点より上、点滅の終わり(+ RING_SOLID_OFFSET)より下
    const uint32_t ON = BLINK_ON_FAST_MS, OFF = BLINK_OFF_MS, P = ON + OFF;
    uint32_t tb = P * 10000;  // 点滅は点灯側から始まり、ON 後に消灯、さらに OFF 後に点灯
    ui::testTouch(tb - 20);
    ui::testShow(ui::SC_DATA);  // 前の試験の点滅の状態を捨てる
    ui::update(b, ls, ps, si, tb - 10);
    ui::testShow(ui::SC_DRIVE);
    ui::update(b, ls, ps, si, tb);
    gfx::dirtyClear();
    bool on = !ui::blankNow();
    int blinks0 = g_hostBlinks;
    ui::update(b, ls, ps, si, tb + ON - 1);
    bool stillOn = !ui::blankNow();
    ui::update(b, ls, ps, si, tb + ON);
    bool off = ui::blankNow();
    bool told = g_hostBlank == 1 && g_hostBlinks == blinks0 + 1;  // 消灯に切り替わったことが platBlink で 1 回だけ知らされる
    gfx::dirtyClear();
    ui::update(b, ls, ps, si, tb + ON + OFF - 1);
    bool stillOff = ui::blankNow();
    ui::update(b, ls, ps, si, tb + P);
    bool on2 = !ui::blankNow() && g_hostBlank == 0;
    gfx::dirtyClear();
    CHECK(BLINK_WHOLE_SCREEN == 0 || (on && stillOn && off && told && stillOff && on2), "シフト点 + 100 rpm: 点灯 BLINK_ON_FAST_MS → 消灯 BLINK_OFF_MS → 点灯(platBlink で知らせる)");
    CHECK(ui::blinkOnMs() == ON && ui::blinkOffMs() == OFF, "シフト点以上の点灯は BLINK_ON_FAST_MS、消灯は BLINK_OFF_MS");
    b.rpm = set.shiftRpm + RING_BLINK_OFFSET - 300;  // 点滅の始まりより下
    ui::update(b, ls, ps, si, tb + 2 * P);
    CHECK(!ui::blankNow() && ui::blinkOnMs() == 0 && ui::blinkOffMs() == 0 && g_hostBlank == 0, "点滅の始まりより下では消灯しない(点灯・消灯の時間 0)");
    b.rpm = set.shiftRpm + 100;
    ui::update(b, ls, ps, si, tb + 3 * P);
    ui::testShow(ui::SC_BOOST);
    ui::update(b, ls, ps, si, tb + 3 * P + 20);  // サーキット → タコ。点滅は続く(tb + 3P の始まりから ON 後に消灯)
    gfx::dirtyClear();
    ui::update(b, ls, ps, si, tb + 3 * P + ON);
    CHECK(ui::screen() == ui::SC_BOOST && (BLINK_WHOLE_SCREEN == 0 || ui::blankNow()), "ブースト計でも消灯する");
    gfx::dirtyClear();
    ui::testShow(ui::SC_DATA);
    ui::update(b, ls, ps, si, tb + 3 * P + ON + 20);
    gfx::dirtyClear();
    ui::update(b, ls, ps, si, tb + 4 * P + ON);
    CHECK(ui::screen() == ui::SC_DATA && !ui::blankNow() && g_hostBlank == 0, "走行画面とブースト計以外では消灯しない");
    gfx::dirtyClear();
  }

  // ---------------- 12a. 点滅の終わり(シフト点 + RING_SOLID_OFFSET)以上: 点滅せず、明るさは最大(BL_MAX)
  {
    Signals b;
    memset(&b, 0, sizeof(b));
    b.linkUp = b.rpmOk = true;
    const uint32_t ON = BLINK_ON_FAST_MS;
    uint32_t ts = 1500000;
    ui::testShow(ui::SC_DRIVE);
    b.rpm = set.shiftRpm + RING_SOLID_OFFSET - 1;
    ui::update(b, ls, ps, si, ts);
    CHECK(ui::blinkOnMs() == ON && ui::blinkLevel() != BL_MAX, "点滅の終わりの 1 rpm 下: まだ点滅(最大輝度ではない)");
    b.rpm = set.shiftRpm + RING_SOLID_OFFSET;
    ui::update(b, ls, ps, si, ts + 1);
    CHECK(ui::blinkOnMs() == 0 && !ui::blankNow() && ui::blinkLevel() == BL_MAX && g_hostLevel == BL_MAX, "点滅の終わり: 点滅せず、platBlink に BL_MAX");
    int blinks0 = g_hostBlinks;
    for (uint32_t dt = 2; dt <= 1000; dt++) ui::update(b, ls, ps, si, ts + dt);
    CHECK(g_hostBlinks == blinks0 && !ui::blankNow() && ui::blinkLevel() == BL_MAX, "最大輝度の間は 1 秒たっても消灯しない(platBlink は呼ばれない)");
    ui::BlinkStats bs;
    ui::blinkStats(&bs, true);
    CHECK(bs.onMs == 0 && bs.offMs == 0, "最大輝度の間は点灯・消灯の時間 0(5 秒ごとの行に点滅は出ない)");
    b.rpm = set.shiftRpm + 100;  // 下回ると点滅に戻る
    ui::update(b, ls, ps, si, ts + 1001);
    CHECK(ui::blinkOnMs() == ON && ui::blinkLevel() != BL_MAX, "点滅の終わりを下回ると点滅に戻る(最大輝度をやめる)");
    gfx::dirtyClear();
    // アナログ計: 回転計の針が動き、針の跡が残らない(針を動かしたあとの絵 = 最初から描いた絵)。最大輝度も効く
    ui::testShow(ui::SC_BOOST);
    b.boostOk = true; b.boostBar = 1.2f;
    static uint16_t fbS[gfx::W * gfx::H];
    b.rpm = 3000;
    ui::update(b, ls, ps, si, ts + 2000);
    // 直前の 6000 rpm のピークホールドの印が下がり切るまで待ってから、基準の絵を取る(印は針の下)
    ui::update(b, ls, ps, si, ts + 2000 + TACH_PEAK_HOLD_MS + 3000u * 1000u / TACH_PEAK_FALL_RPM_PER_S);
    const uint32_t t0 = ts + 2100 + TACH_PEAK_HOLD_MS + 3000u * 1000u / TACH_PEAK_FALL_RPM_PER_S;
    ui::update(b, ls, ps, si, t0);
    memcpy(fbS, fb, sizeof(fbS));
    gfx::dirtyClear();
    b.rpm = 6000;
    ui::update(b, ls, ps, si, t0 + 10);
    CHECK(memcmp(fbS, fb, sizeof(fbS)) != 0 && ui::blinkLevel() == BL_MAX, "アナログ計: 回転数で針が動く。6000 で最大輝度");
    b.rpm = 3000;
    b.boostBar = 1.2f;
    ui::update(b, ls, ps, si, t0 + 20);
    CHECK(memcmp(fbS, fb, sizeof(fbS)) != 0, "アナログ計: 針を戻した直後はピークホールドの印が 6000 に残る(最初の絵と違う)");
    // 印は TACH_PEAK_HOLD_MS 止まってから TACH_PEAK_FALL_RPM_PER_S で下がる。3000 まで下がり切ると最初の絵に戻る
    const uint32_t tFall = t0 + 10 + TACH_PEAK_HOLD_MS + 3000u * 1000u / TACH_PEAK_FALL_RPM_PER_S;
    ui::update(b, ls, ps, si, tFall - 200);
    CHECK(memcmp(fbS, fb, sizeof(fbS)) != 0, "アナログ計: 下がり切る手前では印がまだ針より上にある");
    ui::update(b, ls, ps, si, tFall + 100);
    CHECK(memcmp(fbS, fb, sizeof(fbS)) == 0, "アナログ計: 印が今の回転数まで下がると最初の絵と 1 画素も違わない(針・帯・印の跡が残らない)");
    gfx::dirtyClear();
    ui::testShow(ui::SC_DATA);
    ui::update(b, ls, ps, si, tFall + 1000);
    CHECK(ui::screen() == ui::SC_DATA && ui::blinkLevel() == BL_NORMAL && g_hostLevel == BL_NORMAL, "ほかの画面では設定の明るさに戻る");
    gfx::dirtyClear();
  }

  // ---------------- 12c. アナログ計の水温・燃料の札とブーストの数字(2026-10-22): 札は異常のときだけ(中心の下、赤)。
  //                   燃料は FUEL_ALARM_PCT(30 %)未満で異常。札や数字が変わっても針は壊れず、最初から描いたものと同じになる
  {
    Signals b;
    memset(&b, 0, sizeof(b));
    b.linkUp = b.rpmOk = true;
    b.rpm = 3000;
    b.waterOk = true; b.waterC = 90;
    b.fuelOk = true; b.fuelPct = 50;
    b.boostOk = true; b.boostBar = 0.65f;
    uint32_t tc = 65000000;  // (tc / ALARM_BLINK_HALF_MS) が偶数 = 異常の点滅の点灯側
    ui::testShow(ui::SC_BOOST);
    ui::update(b, ls, ps, si, tc);
    auto px = [&](int x, int y) { return fb[y * gfx::W + x]; };
    auto countIn = [&](int x0, int y0, int x1, int y1, uint16_t c) {
      int n = 0;
      for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++)
          if (px(x, y) == c) n++;
      return n;
    };
    const int bx0 = AN_BADGE_X, bx1 = AN_BADGE_X + AN_BADGE_W, by0 = AN_BADGE_Y, by1 = AN_BADGE_Y + AN_BADGE_PITCH + AN_BADGE_H;
    CHECK(countIn(bx0, by0, bx1, by1, gfx::rgb(0xE3241B)) == 0, "札: 水温 90 ℃・燃料 50 % では出ない");
    CHECK(countIn(BOOST_NUM_X - 160, BOOST_NUM_BASE - 50, BOOST_NUM_X, BOOST_NUM_BASE, gfx::rgb(0xFFFFFF)) > 300, "ブーストの今の値を白い数字で出す(10/23)");
    {
      float a = (TACH_START_DEG + BOOST_RING_DEG_PER_BAR * 0.3f) * 3.14159265f / 180.0f, r = (BOOST_RING_R_IN + BOOST_RING_R_OUT) * 0.5f;
      CHECK(px((int)(233 + r * cosf(a)), (int)(233 + r * sinf(a))) == gfx::rgb(0xA9AFBA), "ブーストの輪: 0.65 bar のとき 0.3 bar の所はトーンを落とした白");
      a = (TACH_START_DEG + BOOST_RING_DEG_PER_BAR * 1.2f) * 3.14159265f / 180.0f;
      CHECK(px((int)(233 + r * cosf(a)), (int)(233 + r * sinf(a))) == gfx::rgb(0x161E2A), "ブーストの輪: 1.2 bar の所は地の色");
    }
    gfx::dirtyClear();
    b.fuelPct = FUEL_ALARM_PCT - 1;  // 29 %: 燃料の札(1 つだけ: 上の段)
    b.rpm = 1000;  // 針も動かす
    ui::update(b, ls, ps, si, tc + 2 * ALARM_BLINK_HALF_MS);
    CHECK(countIn(bx0, AN_BADGE_Y, bx1, AN_BADGE_Y + AN_BADGE_H, gfx::rgb(0xE3241B)) > 2000 &&
          countIn(bx0, AN_BADGE_Y + AN_BADGE_PITCH, bx1, AN_BADGE_Y + AN_BADGE_PITCH + AN_BADGE_H, gfx::rgb(0xE3241B)) == 0, "札: 燃料 29 % で上の段に赤い札が出る(下の段は空)");
    static uint16_t fbC[gfx::W * gfx::H];
    memcpy(fbC, fb, sizeof(fbC));
    ui::testRedraw();
    ui::update(b, ls, ps, si, tc + 2 * ALARM_BLINK_HALF_MS);
    CHECK(memcmp(fbC, fb, sizeof(fbC)) == 0, "札が出て針が動いたあとの絵 = 同じ時刻に最初から描いた絵");
    gfx::dirtyClear();
    b.waterC = 115;  // 警告温度(110)以上: 2 段
    b.boostBar = 1.8f;  // 赤の範囲
    b.rpm = 5000;
    ui::update(b, ls, ps, si, tc + 3 * ALARM_BLINK_HALF_MS);  // 消灯側
    CHECK(countIn(bx0, by0, bx1, by1, gfx::rgb(0x8E130D)) > 4000, "札: 2 つとも出て、点滅の消灯側は暗い赤");
    memcpy(fbC, fb, sizeof(fbC));
    ui::testRedraw();
    ui::update(b, ls, ps, si, tc + 3 * ALARM_BLINK_HALF_MS);
    CHECK(memcmp(fbC, fb, sizeof(fbC)) == 0, "札が 2 つ・ブーストが赤の範囲の絵 = 同じ時刻に最初から描いた絵");
    gfx::dirtyClear();
    b.fuelPct = FUEL_ALARM_PCT + FUEL_ALARM_HYST_PCT;  // 31 %: 燃料の異常が終わる(水温だけ)
    b.waterOk = false;  // 水温の信号もなくなる: 札は出ない
    b.boostOk = false;  // ブーストの信号がない: 帯なし、数字は「– –」
    b.rpm = 2000;
    ui::update(b, ls, ps, si, tc + 4 * ALARM_BLINK_HALF_MS);
    CHECK(countIn(bx0, by0, bx1, by1, gfx::rgb(0xE3241B)) == 0 && countIn(bx0, by0, bx1, by1, gfx::rgb(0x8E130D)) == 0, "札: 異常が終わると消える");
    CHECK(countIn(BOOST_NUM_X - 200, BOOST_NUM_BASE - 56, BOOST_NUM_X, BOOST_NUM_BASE, gfx::rgb(0x29C5FF)) == 0, "ブーストの信号がない: 水色の数字は消える");
    memcpy(fbC, fb, sizeof(fbC));
    ui::testRedraw();
    ui::update(b, ls, ps, si, tc + 4 * ALARM_BLINK_HALF_MS);
    CHECK(memcmp(fbC, fb, sizeof(fbC)) == 0, "札と帯が消えた絵 = 同じ時刻に最初から描いた絵(跡が残らない)");
    gfx::dirtyClear();
  }

  // ---------------- 12d. 走行画面の異常(2026-10-22): 点滅の点灯側は弧の側を赤く塗る、消灯側は背景のまま弧が赤。燃料は 30 % 未満
  {
    Signals b;
    memset(&b, 0, sizeof(b));
    b.linkUp = b.rpmOk = true;
    b.rpm = 2000;  // 背景は黒
    b.gearOk = true; b.gear = '2';
    b.waterOk = true; b.waterC = 90;
    b.fuelOk = true; b.fuelPct = FUEL_ALARM_PCT + 5;
    uint32_t td = 66000000;  // 異常の点滅の点灯側
    ui::testShow(ui::SC_DRIVE);
    ui::update(b, ls, ps, si, td);
    auto px = [&](int x, int y) { return fb[y * gfx::W + x]; };
    // 塗る所のうち、弧にもアイコンにも掛からない点: 燃料は (233, 300)、水温は (233, 165)
    CHECK(px(233, 300) == gfx::rgb(0x000000) && px(233, 165) == gfx::rgb(0x000000), "異常でないときは塗らない");
    {
      float a = (FUEL_ARC_START_DEG - ARC_SWEEP_DEG * 0.1f) * 3.14159265f / 180.0f, r = (ARC_R_IN + ARC_R_OUT) * 0.5f;
      CHECK(px((int)(233 + r * cosf(a)), (int)(233 + r * sinf(a))) == gfx::rgb(0xFFFFFF), "燃料 35 %: 10 % の所は白(伸びた所)");
    }
    gfx::dirtyClear();
    b.fuelPct = FUEL_ALARM_PCT - 1;
    ui::update(b, ls, ps, si, td + 2 * ALARM_BLINK_HALF_MS);
    CHECK(px(233, 300) == gfx::rgb(0xE3241B) && px(233, 165) == gfx::rgb(0x000000), "燃料 29 %: 点灯側は下側を赤く塗る(上は塗らない)");
    static uint16_t fbD[gfx::W * gfx::H];
    memcpy(fbD, fb, sizeof(fbD));
    ui::testRedraw();
    ui::update(b, ls, ps, si, td + 2 * ALARM_BLINK_HALF_MS);
    CHECK(memcmp(fbD, fb, sizeof(fbD)) == 0, "燃料の異常(点灯側)の絵 = 同じ時刻に最初から描いた絵");
    gfx::dirtyClear();
    ui::update(b, ls, ps, si, td + 3 * ALARM_BLINK_HALF_MS);
    CHECK(px(233, 300) == gfx::rgb(0x000000), "燃料 29 %: 消灯側は塗らない");
    memcpy(fbD, fb, sizeof(fbD));
    ui::testRedraw();
    ui::update(b, ls, ps, si, td + 3 * ALARM_BLINK_HALF_MS);
    CHECK(memcmp(fbD, fb, sizeof(fbD)) == 0, "燃料の異常(消灯側)の絵 = 同じ時刻に最初から描いた絵(塗った跡が残らない)");
    gfx::dirtyClear();
    b.waterC = 112;
    ui::update(b, ls, ps, si, td + 4 * ALARM_BLINK_HALF_MS);
    CHECK(px(233, 165) == gfx::rgb(0xE3241B) && px(233, 300) == gfx::rgb(0xE3241B), "水温 112 ℃ と燃料 29 %: 点灯側は上下とも赤く塗る");
    int gearWhite = 0;  // ギアの文字(白)は塗る所に削られない
    for (int y = 170; y < 296; y++)
      for (int x = 160; x < 306; x++)
        if (px(x, y) == gfx::rgb(0xFFFFFF)) gearWhite++;
    memcpy(fbD, fb, sizeof(fbD));
    ui::testRedraw();
    ui::update(b, ls, ps, si, td + 4 * ALARM_BLINK_HALF_MS);
    CHECK(memcmp(fbD, fb, sizeof(fbD)) == 0 && gearWhite > 1000, "上下とも異常の絵 = 最初から描いた絵(ギアの文字は残る)");
    gfx::dirtyClear();
  }

  // ---------------- 12b. 点滅の速さ: 消灯は BLINK_OFF_MS で一定。点灯は始まり(シフト点 + RING_BLINK_OFFSET)で BLINK_ON_SLOW_MS、
  //                   シフト点で BLINK_ON_FAST_MS、間は比例
  {
    Signals b;
    memset(&b, 0, sizeof(b));
    b.linkUp = b.rpmOk = true;
    const int start = set.shiftRpm + RING_BLINK_OFFSET;
    const int span = set.shiftRpm - start;
    uint32_t tr = 70000000;
    ui::testShow(ui::SC_DRIVE);
    b.rpm = start - 1;
    ui::update(b, ls, ps, si, tr);
    CHECK(ui::blinkOnMs() == 0 && ui::blinkOffMs() == 0, "始まりの 1 rpm 下: 点滅しない");
    b.rpm = start;
    ui::update(b, ls, ps, si, tr += 1);
    CHECK(ui::blinkOnMs() == BLINK_ON_SLOW_MS && ui::blinkOffMs() == BLINK_OFF_MS, "始まり: 点灯は BLINK_ON_SLOW_MS、消灯は BLINK_OFF_MS");
    b.rpm = start + span / 2;
    ui::update(b, ls, ps, si, tr += 1);
    CHECK(ui::blinkOnMs() == (BLINK_ON_SLOW_MS + BLINK_ON_FAST_MS) / 2 && ui::blinkOffMs() == BLINK_OFF_MS, "始まりとシフト点の中間: 点灯はその中間、消灯は同じ");
    b.rpm = set.shiftRpm;
    ui::update(b, ls, ps, si, tr += 1);
    CHECK(ui::blinkOnMs() == BLINK_ON_FAST_MS, "シフト点: 点灯は BLINK_ON_FAST_MS");
    b.rpm = set.shiftRpm + RING_SOLID_OFFSET - 1;
    ui::update(b, ls, ps, si, tr += 1);
    CHECK(ui::blinkOnMs() == BLINK_ON_FAST_MS, "シフト点より上(点滅の終わりの手前まで): BLINK_ON_FAST_MS のまま");
    gfx::dirtyClear();
    // 回転数を 1 秒で始まりからシフト点まで上げる(1 ms ごとに update)。消灯の時間はいつも BLINK_OFF_MS、点灯の時間は
    // 遅い側と速い側の範囲に収まり、だんだん短くなる(位相が飛んで極端に短い・長い間隔が出ない)
    ui::testShow(ui::SC_DATA);  // いったん点滅を止める
    ui::update(b, ls, ps, si, tr += 1);
    ui::testShow(ui::SC_DRIVE);
    uint32_t t0 = tr + 1, lastEdge = 0, prevOn = 0, gaps = 0, badGap = 0, growGap = 0, badOff = 0;
    bool prevBlank = false;
    ui::BlinkStats bs0;
    ui::blinkStats(&bs0, true);
    for (uint32_t dt = 0; dt <= 1000; dt++) {
      b.rpm = start + (int)(span * dt / 1000);
      ui::update(b, ls, ps, si, t0 + dt);
      bool bl = ui::blankNow();
      if (bl != prevBlank) {
        if (lastEdge) {
          uint32_t gap = t0 + dt - lastEdge;
          gaps++;
          if (bl) {  // 点灯 → 消灯: 終わった区間は点灯
            if (gap < BLINK_ON_FAST_MS || gap > BLINK_ON_SLOW_MS) badGap++;
            if (prevOn && gap > prevOn) growGap++;
            prevOn = gap;
          } else if (gap != BLINK_OFF_MS) {  // 消灯 → 点灯: 終わった区間は消灯
            badOff++;
          }
        }
        lastEdge = t0 + dt;
        prevBlank = bl;
      }
    }
    ui::BlinkStats bs;
    ui::blinkStats(&bs, false);
    CHECK(gaps >= 8 && badGap == 0, "速さを上げながらの点灯の時間は、速い側と遅い側の範囲に収まる");
    CHECK(badOff == 0, "消灯の時間はいつも BLINK_OFF_MS");
    CHECK(growGap == 0, "点灯の時間はだんだん短くなる(位相が飛ばない)");
    CHECK(bs.toggles >= 8 && bs.lateMaxMs == 0 && bs.lateOver == 0, "1 ms ごとに見ていれば切り替えの遅れは 0");
    b.rpm = 850;
    ui::update(b, ls, ps, si, t0 + 1001);
    CHECK(!ui::blankNow() && ui::blinkOnMs() == 0, "点滅の範囲を下回ると点灯に戻る");
    gfx::dirtyClear();
  }

  // ---------------- 13. 設定の「デモ」(6 項目目)
  {
    Signals d;
    memset(&d, 0, sizeof(d));
    uint32_t td = 50000000;
    set.demoOn = 0;
    ui::testShow(ui::SC_SET_LIST, 4);
    ui::tap(233, 400, td += 100);  // 先に update すると、前の操作から時間がたっていて自動で戻る
    ui::update(d, ls, ps, si, td);
    CHECK(ui::screen() == ui::SC_SET_EDIT, "設定 5 項目目で「デモ」の変更画面へ");
    ui::tap(366, 233, td += 100);
    CHECK(set.demoOn == 1, "右半分(入)でデモが入");
    ui::tap(366, 233, td += 100);
    CHECK(set.demoOn == 1, "もう一度押しても入のまま");
    int sv = g_hostSaves;
    ui::tap(233, 400, td += 100);
    ui::update(d, ls, ps, si, td);
    CHECK(ui::screen() == ui::SC_SET_LIST && g_hostSaves == sv + 1, "「決定」で保存");
    set.demoOn = 0;
    gfx::dirtyClear();
  }

  // ---------------- 14. 異常の表示の点滅(水温が警告温度以上、燃料が少ない): ALARM_BLINK_HALF_MS ごとに描き直す
  {
    static uint16_t fbA[gfx::W * gfx::H], fbB[gfx::W * gfx::H];
    Signals a;
    memset(&a, 0, sizeof(a));
    a.linkUp = a.rpmOk = a.waterOk = a.fuelOk = a.gearOk = true;
    a.rpm = 3000; a.waterC = 118; a.fuelPct = 8; a.gear = '3';
    uint32_t ta = (uint32_t)ALARM_BLINK_HALF_MS * 40000;  // (ta / ALARM_BLINK_HALF_MS) が偶数 = 点灯
    ui::testShow(ui::SC_DRIVE);
    ui::update(a, ls, ps, si, ta);
    memcpy(fbA, fb, sizeof(fbA));
    gfx::dirtyClear();
    ui::update(a, ls, ps, si, ta + ALARM_BLINK_HALF_MS);
    bool redrawn = gfx::dirtyCount() > 0;
    memcpy(fbB, fb, sizeof(fbB));
    gfx::dirtyClear();
    ui::update(a, ls, ps, si, ta + 2 * ALARM_BLINK_HALF_MS);
    CHECK(redrawn && memcmp(fbA, fbB, sizeof(fbA)) != 0, "走行画面: 異常の表示が消灯側で描き変わる");
    CHECK(memcmp(fbA, fb, sizeof(fbA)) == 0, "走行画面: 1 周期後は点灯側と同じ絵");
    gfx::dirtyClear();
    a.waterC = 90; a.fuelPct = 50;  // 正常に戻す(戻る側の幅 1 ℃ / 1 % より大きく下げる)
    ui::update(a, ls, ps, si, ta + 3 * ALARM_BLINK_HALF_MS);
    memcpy(fbA, fb, sizeof(fbA));
    gfx::dirtyClear();
    ui::update(a, ls, ps, si, ta + 4 * ALARM_BLINK_HALF_MS);
    CHECK(gfx::dirtyCount() == 0 && memcmp(fbA, fb, sizeof(fbA)) == 0, "走行画面: 正常なら点滅しない(描き直しなし)");
    gfx::dirtyClear();
    // データ画面でも同じ
    a.waterC = 118; a.fuelPct = 8;
    ui::testShow(ui::SC_DATA);
    ui::update(a, ls, ps, si, ta + 8 * ALARM_BLINK_HALF_MS);
    ui::update(a, ls, ps, si, ta + 8 * ALARM_BLINK_HALF_MS + 60);
    memcpy(fbA, fb, sizeof(fbA));
    gfx::dirtyClear();
    ui::update(a, ls, ps, si, ta + 9 * ALARM_BLINK_HALF_MS);
    ui::update(a, ls, ps, si, ta + 9 * ALARM_BLINK_HALF_MS + 60);
    CHECK(memcmp(fbA, fb, sizeof(fbA)) != 0, "データ画面: 異常の表示が消灯側で描き変わる");
    gfx::dirtyClear();
  }

  // ---------------- 15. 設定の「RaceChrono」(7 項目目)→ RaceChrono の状態 → 戻る。下の点の判定
  {
    Signals d;
    memset(&d, 0, sizeof(d));
    uint32_t tr = 60000000;
    ui::testShow(ui::SC_SET_LIST, 8);
    ui::tap(233, 400, tr += 100);
    ui::update(d, ls, ps, si, tr);
    CHECK(ui::screen() == ui::SC_SET_RC, "設定 9 項目目で「RaceChrono の状態」へ");
    si.rc.enabled = true; si.rc.bleOk = true; si.rc.connected = true; si.rc.gpsOk = true; si.rc.fixType = 3; si.rc.numSV = 9; si.rc.lat = 355812345; si.rc.lon = 1396821234;
    si.rc.imuOk = true; si.rc.imuLevel = true; si.rc.imuForward = true;
    ui::update(d, ls, ps, si, tr += 600);
    ui::update(d, ls, ps, si, tr += 600);
    CHECK(ui::screen() == ui::SC_SET_RC, "値が変わっても画面はそのまま");
    {
      // 下の点(7 個、y 330、間隔 26): 全部正常なら全部緑
      auto dotCol = [&](int i) { return fb[330 * gfx::W + (int)(233 - 26 * 3 + 26 * i)]; };
      int green = 0;
      for (int i = 0; i < 7; i++) if (dotCol(i) == gfx::rgb(0x22E36A)) green++;
      CHECK(green == 7, "RaceChrono: 全項目が正常なら下の点は 7 個とも緑");
      si.rc.imuOk = false;  // IMU が応答しない: 6 個目(IMU)が赤
      ui::update(d, ls, ps, si, tr += 600);
      CHECK(dotCol(5) == gfx::rgb(0xE3241B), "RaceChrono: IMU が応答しないと 6 個目の点が赤");
      si.rc.imuOk = true;
    }
    ui::update(d, ls, ps, si, tr + 4 * 60000);
    CHECK(ui::screen() == ui::SC_DRIVE, "RaceChrono の状態: 自動で覚えた画面(サーキット)に戻る");
    ui::testShow(ui::SC_SET_RC);
    ui::update(d, ls, ps, si, tr += 100);
    ui::tap(233, 60, tr += 100);
    ui::update(d, ls, ps, si, tr);
    CHECK(ui::screen() == ui::SC_SET_LIST, "RaceChrono の状態: 上の「戻る」で設定の項目の画面へ");
    ui::testShow(ui::SC_SET_RC, 5);
    ui::update(d, ls, ps, si, tr += 100);
    int resets0 = g_hostImuResets;
    ui::tap(233, 400, tr += 100);  // 「IMU」の項目には下の操作が無い(何も起きない)
    ui::tap(366, 233, tr += 100);  // 右半分 → 「向きの補正」
    ui::update(d, ls, ps, si, tr += 600);  // 描くと下に「向きを取り直す」が出る
    ui::tap(233, 400, tr += 100);  // 下 → 覚えた向きを捨てる
    ui::update(d, ls, ps, si, tr);
    CHECK(ui::screen() == ui::SC_SET_RC && g_hostImuResets == resets0 + 1, "「向きの補正」の下「向きを取り直す」で覚えた向きを捨てる(1 回だけ)");
    // 6 項目目「RaceChrono 送信」(入・切)
    set.rcOn = 1;
    ui::testShow(ui::SC_SET_LIST, 5);
    ui::tap(233, 400, tr += 100);
    ui::update(d, ls, ps, si, tr);
    CHECK(ui::screen() == ui::SC_SET_EDIT, "設定 6 項目目で「RaceChrono 送信」の変更画面へ");
    ui::tap(100, 233, tr += 100);
    CHECK(set.rcOn == 0, "左半分(切)で切");
    ui::tap(100, 233, tr += 100);
    CHECK(set.rcOn == 0, "もう一度押しても切のまま");
    int sv = g_hostSaves;
    ui::tap(233, 400, tr += 100);
    ui::update(d, ls, ps, si, tr);
    CHECK(ui::screen() == ui::SC_SET_LIST && g_hostSaves == sv + 1, "「決定」で保存");
    set.rcOn = SET_RC_DEFAULT;
    memset(&si.rc, 0, sizeof(si.rc));
    gfx::dirtyClear();
  }

  // ---------------- 16. GPS(UBX の解析): NAV-PVT、ACK、チェックサム誤り、長いメッセージの読み飛ばし、ごみ
  {
    gnss::begin();
    // NAV-PVT(92 バイト)を組み立てる。2026-10-14 03:15:30.250、3D、衛星 11、東京駅あたり、48.3 km/h、方位 123.45°
    uint8_t pay[92];
    memset(pay, 0, sizeof(pay));
    auto put16 = [&](int off, uint16_t v) { pay[off] = (uint8_t)v; pay[off + 1] = (uint8_t)(v >> 8); };
    auto put32 = [&](int off, uint32_t v) { for (int i = 0; i < 4; i++) pay[off + i] = (uint8_t)(v >> (8 * i)); };
    put32(0, 123456789);          // iTOW
    put16(4, 2026); pay[6] = 10; pay[7] = 14; pay[8] = 3; pay[9] = 15; pay[10] = 30;
    pay[11] = 0x07;               // validDate | validTime | fullyResolved
    put32(16, (uint32_t)250000000);  // nano
    pay[20] = 3;                  // fixType 3D
    pay[21] = 0x01;               // gnssFixOK
    pay[23] = 11;                 // numSV
    put32(24, (uint32_t)1396821234);  // lon
    put32(28, (uint32_t)355812345);   // lat
    put32(36, (uint32_t)45000);       // hMSL [mm]
    put32(60, (uint32_t)13417);       // gSpeed [mm/s] = 48.30 km/h
    put32(64, (uint32_t)12345000);    // headMot [1e-5 deg] = 123.45°
    put16(76, 150);                   // pDOP 1.50
    std::vector<uint8_t> msg = { 0xB5, 0x62, 0x01, 0x07, 92, 0 };
    msg.insert(msg.end(), pay, pay + 92);
    uint8_t ckA = 0, ckB = 0;
    for (size_t i = 2; i < msg.size(); i++) { ckA = (uint8_t)(ckA + msg[i]); ckB = (uint8_t)(ckB + ckA); }
    msg.push_back(ckA);
    msg.push_back(ckB);
    int got = 0;
    for (size_t i = 0; i < msg.size(); i++) got += gnss::feed(&msg[i], 1, 5000);  // 1 バイトずつ
    gnss::Fix f;
    CHECK(got == 1 && gnss::pvtCount() == 1 && gnss::latest(&f), "NAV-PVT を 1 バイトずつ受けて 1 つ完成");
    CHECK(f.fixOk && f.fixType == 3 && f.numSV == 11 && !f.diffSoln, "fix の種類と衛星数");
    CHECK(f.timeOk && f.year == 2026 && f.month == 10 && f.day == 14 && f.hour == 3 && f.min == 15 && f.sec == 30 && f.nano == 250000000, "日時");
    CHECK(f.lat == 355812345 && f.lon == 1396821234 && f.hMSL == 45000 && f.gSpeed == 13417 && f.headMot == 12345000 && f.pDOP == 150 && f.rxMs == 5000, "位置・高さ・速度・方位・精度");
    // まとめて 2 つ + ACK-ACK + NAK
    std::vector<uint8_t> two(msg);
    two.insert(two.end(), msg.begin(), msg.end());
    uint8_t ack[] = { 0xB5, 0x62, 0x05, 0x01, 2, 0, 0x06, 0x8A, 0x98, 0xC1 };  // ACK-ACK for CFG-VALSET(チェックサムは下で計算)
    { uint8_t a = 0, b = 0; for (int i = 2; i < 8; i++) { a = (uint8_t)(a + ack[i]); b = (uint8_t)(b + a); } ack[8] = a; ack[9] = b; }
    uint8_t nak[10]; memcpy(nak, ack, 10); nak[3] = 0x00;
    { uint8_t a = 0, b = 0; for (int i = 2; i < 8; i++) { a = (uint8_t)(a + nak[i]); b = (uint8_t)(b + a); } nak[8] = a; nak[9] = b; }
    two.insert(two.end(), ack, ack + 10);
    two.insert(two.end(), nak, nak + 10);
    got = gnss::feed(two.data(), two.size(), 6000);
    CHECK(got == 2 && gnss::pvtCount() == 3 && gnss::ackCount() == 1 && gnss::nakCount() == 1 && gnss::badCount() == 0, "続けて届いた NAV-PVT 2 つと ACK・NAK");
    // チェックサムが合わない(1 ビット壊す)
    std::vector<uint8_t> bad(msg);
    bad[30] ^= 0x01;
    got = gnss::feed(bad.data(), bad.size(), 7000);
    CHECK(got == 0 && gnss::badCount() == 1 && gnss::pvtCount() == 3, "チェックサムの合わない NAV-PVT は使わない");
    // 長い別のメッセージ(600 バイト)を読み飛ばし、その後の NAV-PVT を拾う
    std::vector<uint8_t> lng = { 0xB5, 0x62, 0x02, 0x15, (uint8_t)(600 & 0xFF), (uint8_t)(600 >> 8) };
    for (int i = 0; i < 600; i++) lng.push_back((uint8_t)i);
    { uint8_t a = 0, b = 0; for (size_t i = 2; i < lng.size(); i++) { a = (uint8_t)(a + lng[i]); b = (uint8_t)(b + a); } lng.push_back(a); lng.push_back(b); }
    lng.insert(lng.end(), msg.begin(), msg.end());
    got = gnss::feed(lng.data(), lng.size(), 8000);
    CHECK(got == 1 && gnss::pvtCount() == 4 && gnss::badCount() == 1, "長いメッセージを読み飛ばして次の NAV-PVT を拾う");
    // ごみ(NMEA の文)の中の NAV-PVT
    std::vector<uint8_t> junk;
    const char *nmea = "$GNGGA,031530.00,3534.87,N,13940.93,E,1,11,1.5,45.0,M,39.0,M,,*5C\r\n";
    junk.insert(junk.end(), nmea, nmea + strlen(nmea));
    junk.push_back(0xB5);  // 偽の同期
    junk.push_back(0x11);
    junk.insert(junk.end(), msg.begin(), msg.end());
    junk.insert(junk.end(), nmea, nmea + strlen(nmea));
    got = gnss::feed(junk.data(), junk.size(), 9000);
    CHECK(got == 1 && gnss::pvtCount() == 5, "NMEA のごみと偽の同期の間の NAV-PVT を拾う");
    // 設定メッセージの形: CFG-VALSET、層、鍵の数
    uint8_t cfg[128];
    size_t n = gnss::buildConfig(cfg, sizeof(cfg), 25);
    CHECK(n > 8 && cfg[0] == 0xB5 && cfg[1] == 0x62 && cfg[2] == 0x06 && cfg[3] == 0x8A && n == (size_t)(cfg[4] | (cfg[5] << 8)) + 8, "CFG-VALSET の枠");
    CHECK(cfg[6] == 0 && cfg[7] == RC_GPS_CFG_LAYERS, "版 0、層は config.h の値");
    {
      uint8_t a = 0, b = 0;
      for (size_t i = 2; i < n - 2; i++) { a = (uint8_t)(a + cfg[i]); b = (uint8_t)(b + a); }
      CHECK(cfg[n - 2] == a && cfg[n - 1] == b, "CFG-VALSET のチェックサム");
      // 鍵を数える(4 + 値の大きさ)。RATE_MEAS = 40 ms が入っている
      size_t pos = 10;
      int keys = 0;
      bool meas40 = false;
      while (pos + 4 <= n - 2) {
        uint32_t key = (uint32_t)cfg[pos] | ((uint32_t)cfg[pos + 1] << 8) | ((uint32_t)cfg[pos + 2] << 16) | ((uint32_t)cfg[pos + 3] << 24);
        int sz = (int)((key >> 28) & 7);
        int vn = sz == 3 ? 2 : sz == 4 ? 4 : 1;
        if (key == 0x30210001 && vn == 2 && (cfg[pos + 4] | (cfg[pos + 5] << 8)) == 40) meas40 = true;
        pos += 4 + (size_t)vn;
        keys++;
      }
      CHECK(keys == 7 && pos == n - 2 && meas40, "測位の設定は鍵 7 個、測位の間隔 40 ms(25 回/秒)");
    }
    n = gnss::buildSignals(cfg, sizeof(cfg));
    CHECK(n > 8 && cfg[2] == 0x06 && cfg[3] == 0x8A && cfg[7] == RC_GPS_CFG_LAYERS && n == (size_t)(cfg[4] | (cfg[5] << 8)) + 8, "衛星系の設定も CFG-VALSET");
    {
      uint8_t a = 0, b = 0;
      for (size_t i = 2; i < n - 2; i++) { a = (uint8_t)(a + cfg[i]); b = (uint8_t)(b + a); }
      size_t pos = 10;
      int keys = 0, on = 0;
      while (pos + 4 <= n - 2) {
        uint32_t key = (uint32_t)cfg[pos] | ((uint32_t)cfg[pos + 1] << 8) | ((uint32_t)cfg[pos + 2] << 16) | ((uint32_t)cfg[pos + 3] << 24);
        int sz = (int)((key >> 28) & 7);
        int vn = sz == 3 ? 2 : sz == 4 ? 4 : 1;
        if (vn == 1 && cfg[pos + 4]) on++;
        pos += 4 + (size_t)vn;
        keys++;
      }
      CHECK(cfg[n - 2] == a && cfg[n - 1] == b && keys == 9 && on == 6 && pos == n - 2, "衛星系の設定: 鍵 9 個(入 6、切 3)、チェックサム");
    }
    n = gnss::buildBaud(cfg, sizeof(cfg), 115200);
    CHECK(n == 8 + 4 + 8 && cfg[10] == 0x01 && cfg[11] == 0x00 && cfg[12] == 0x52 && cfg[13] == 0x40 && cfg[14] == 0x00 && cfg[15] == 0xC2 && cfg[16] == 0x01 && cfg[17] == 0x00,
          "速度の設定: 鍵 0x40520001 と 115200(4 バイト)");
    CHECK(gnss::buildConfig(cfg, 16, 25) == 0, "出力が小さすぎるときは 0");
  }

  // ---------------- 17. RaceChrono DIY の符号化(GPS 20 バイト、日時 3 バイト、CAN)
  {
    gnss::Fix f;
    memset(&f, 0, sizeof(f));
    f.fixOk = true; f.fixType = 3; f.numSV = 11; f.timeOk = true;
    f.year = 2026; f.month = 10; f.day = 14; f.hour = 3; f.min = 15; f.sec = 30; f.nano = 250000000;
    f.lat = 355812345; f.lon = 1396821234; f.hMSL = 45000; f.gSpeed = 13417; f.headMot = 12345000; f.pDOP = 150;
    uint8_t g[20];
    rcdiy::encodeGpsMain(f, 5, g);
    // 参照: 仕様書の式を直接書いたもの
    uint32_t tt = 15 * 30000 + 30 * 500 + 250 / 2;  // 465125
    CHECK(g[0] == ((5 << 5) | ((tt >> 16) & 0x1F)) && g[1] == ((tt >> 8) & 0xFF) && g[2] == (tt & 0xFF), "GPS: sync 5 と正時からの時刻");
    CHECK(g[3] == ((1 << 6) | 11), "GPS: fix 1(GPS)と衛星 11");
    CHECK(g[4] == 0x15 && g[5] == 0x35 && g[6] == 0x43 && g[7] == 0xF9, "GPS: 緯度 355812345 = 0x153543F9(上位が先)");
    CHECK(g[8] == 0x53 && g[9] == 0x41 && g[10] == 0xCC && g[11] == 0xF2, "GPS: 経度 1396821234 = 0x5341CCF2");
    CHECK(((g[12] << 8) | g[13]) == (45 + 500) * 10, "GPS: 高さ (m + 500) × 10");
    CHECK(((g[14] << 8) | g[15]) == 4830, "GPS: 速度 48.30 km/h × 100");
    CHECK(((g[16] << 8) | g[17]) == 12345, "GPS: 方位 123.45° × 100");
    CHECK(g[18] == 15 && g[19] == 0xFF, "GPS: 精度 1.5 × 10、VDOP なし");
    f.diffSoln = true;
    rcdiy::encodeGpsMain(f, 0, g);
    CHECK(g[3] == ((2 << 6) | 11), "GPS: 補正ありは fix 2");
    f.diffSoln = false;
    f.hMSL = 3000000; f.gSpeed = 100000;  // 3000 m、360 km/h
    rcdiy::encodeGpsMain(f, 0, g);
    CHECK(((g[12] << 8) | g[13]) == ((3000 + 500) | 0x8000) && ((g[14] << 8) | g[15]) == (3600 | 0x8000), "GPS: 高さ 3000 m と 360 km/h は粗い刻み(上位ビット)");
    f.hMSL = 45000; f.gSpeed = 13417; f.headMot = -1000000;  // −10°
    rcdiy::encodeGpsMain(f, 0, g);
    CHECK(((g[16] << 8) | g[17]) == 35000, "GPS: 負の方位は 0〜360 に直す");
    f.fixOk = false;
    rcdiy::encodeGpsMain(f, 7, g);
    CHECK(g[0] >> 5 == 7 && g[3] == 11 && g[4] == 0x7F && g[5] == 0xFF && g[6] == 0xFF && g[7] == 0xFF && g[8] == 0x7F && g[9] == 0xFF && g[12] == 0xFF && g[13] == 0xFF && g[14] == 0xFF && g[16] == 0xFF && g[18] == 0xFF,
          "GPS: 測位なしのときは位置・高さ・速度・方位・精度が「無効」の値");
    f.fixOk = true;
    uint8_t tm[3];
    rcdiy::encodeGpsTime(f, 5, tm);
    uint32_t code = (2026 - 2000) * 8928 + (10 - 1) * 744 + (14 - 1) * 24 + 3;  // 239139
    CHECK(rcdiy::dateHourCode(f) == code && tm[0] == ((5 << 5) | ((code >> 16) & 0x1F)) && tm[1] == ((code >> 8) & 0xFF) && tm[2] == (code & 0xFF), "日時: sync 5 と日・時の符号");
    f.hour = 4;
    CHECK(rcdiy::dateHourCode(f) == code + 1, "日時: 時が進むと符号が 1 増える");
    uint8_t c[20];
    uint8_t d[8] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };
    size_t cn = rcdiy::encodeCan(0x0618A001, d, 8, c);
    CHECK(cn == 12 && c[0] == 0x01 && c[1] == 0xA0 && c[2] == 0x18 && c[3] == 0x06 && memcmp(c + 4, d, 8) == 0, "CAN: ID は下位が先、中身はそのまま");
    CHECK(rcdiy::encodeCan(0x1FFFFF01, d, 3, c) == 7 && c[3] == 0x1F && c[4] == 0x11 && c[6] == 0x33, "CAN: 長さ 3");
  }

  // ---------------- 18. RaceChrono の PID フィルタ(0x0002 への命令)
  {
    rcdiy::filterReset();
    CHECK(!rcdiy::filterPass(0x0618A001, 1000) && rcdiy::filterCount() == 0, "最初は全部拒否");
    uint8_t allow[] = { 2, 0x00, 0x64, 0x06, 0x18, 0xA0, 0x01 };  // PID 0618A001 を 100 ms 間隔で
    rcdiy::filterCommand(allow, sizeof(allow));
    CHECK(rcdiy::filterCount() == 1 && rcdiy::filterPass(0x0618A001, 1000), "許可した PID は通る");
    CHECK(!rcdiy::filterPass(0x0618A001, 1050), "間隔 100 ms の途中は通らない");
    CHECK(rcdiy::filterPass(0x0618A001, 1100), "100 ms たてば通る");
    CHECK(!rcdiy::filterPass(0x0210A006, 1100), "許可していない PID は通らない");
    uint8_t allow2[] = { 2, 0x00, 0x00, 0x02, 0x10, 0xA0, 0x06 };  // 間隔 0 = 毎回
    rcdiy::filterCommand(allow2, sizeof(allow2));
    CHECK(rcdiy::filterCount() == 2 && rcdiy::filterPass(0x0210A006, 1100) && rcdiy::filterPass(0x0210A006, 1101), "間隔 0 の PID は毎回通る");
    uint8_t allow3[] = { 2, 0x01, 0xF4, 0x06, 0x18, 0xA0, 0x01 };  // 同じ PID を 500 ms に変える
    rcdiy::filterCommand(allow3, sizeof(allow3));
    CHECK(rcdiy::filterCount() == 2 && !rcdiy::filterPass(0x0618A001, 1500) && rcdiy::filterPass(0x0618A001, 1600), "同じ PID の命令は間隔を変えるだけ");
    uint8_t deny[] = { 0 };
    rcdiy::filterCommand(deny, 1);
    CHECK(rcdiy::filterCount() == 0 && !rcdiy::filterPass(0x0618A001, 2000) && !rcdiy::filterPass(0x0210A006, 2000), "全部拒否で許可が消える");
    uint8_t all[] = { 1, 0x00, 0xC8 };  // 全部許可、200 ms
    rcdiy::filterCommand(all, 3);
    CHECK(rcdiy::filterCount() == -1 && rcdiy::filterPass(0x12345678, 3000) && !rcdiy::filterPass(0x12345678, 3100) && rcdiy::filterPass(0x12345679, 3100) && rcdiy::filterPass(0x12345678, 3200),
          "全部許可は PID ごとに 200 ms 間隔");
    uint8_t all0[] = { 1, 0x00, 0x00 };
    rcdiy::filterCommand(all0, 3);
    CHECK(rcdiy::filterPass(0x1, 1) && rcdiy::filterPass(0x1, 1), "全部許可(間隔 0)は毎回通る");
    uint8_t shortCmd[] = { 2, 0x00 };
    rcdiy::filterCommand(shortCmd, 2);  // 短すぎる命令は無視
    rcdiy::filterCommand(deny, 1);
    for (uint32_t i = 0; i < 60; i++) {  // 表のあふれ(48 個)
      uint8_t a[] = { 2, 0, 0, (uint8_t)(i >> 24), (uint8_t)(i >> 16), (uint8_t)(i >> 8), (uint8_t)i };
      rcdiy::filterCommand(a, 7);
    }
    CHECK(rcdiy::filterCount() == 48 && rcdiy::filterPass(47, 5000) && !rcdiy::filterPass(59, 5000), "許可の表は 48 個まで(それ以上は無視)");
    // 車の CAN は RC_CAN_MIN_MS より詰めない(2.0.0: c6_can_link が 20 ms 間隔で送るようになったので、RaceChrono へ送る量を 1.x と同じに保つ)
    rcdiy::filterReset();
    uint8_t allow20[] = { 2, 0x00, 0x14, 0x06, 0x18, 0xA0, 0x01 };  // RaceChrono が 20 ms 間隔を頼む
    rcdiy::filterCommand(allow20, sizeof(allow20));
    CHECK(rcdiy::filterPass(0x0618A001, 6000, RC_CAN_MIN_MS) && !rcdiy::filterPass(0x0618A001, 6020, RC_CAN_MIN_MS) && !rcdiy::filterPass(0x0618A001, 6049, RC_CAN_MIN_MS)
              && rcdiy::filterPass(0x0618A001, 6050, RC_CAN_MIN_MS),
          "車の CAN: 頼まれた間隔(20 ms)より RC_CAN_MIN_MS(50 ms)が長ければ 50 ms");
    CHECK(rcdiy::filterPass(0x0618A001, 6070), "最短の間隔を渡さないとき(IMU)は頼まれた間隔のまま");
    uint8_t allow200[] = { 2, 0x00, 0xC8, 0x02, 0x10, 0xA0, 0x06 };  // 200 ms
    rcdiy::filterCommand(allow200, sizeof(allow200));
    CHECK(rcdiy::filterPass(0x0210A006, 7000, RC_CAN_MIN_MS) && !rcdiy::filterPass(0x0210A006, 7100, RC_CAN_MIN_MS) && rcdiy::filterPass(0x0210A006, 7200, RC_CAN_MIN_MS),
          "車の CAN: 頼まれた間隔の方が長ければそちら(200 ms)");
    rcdiy::filterCommand(all0, 3);
    CHECK(rcdiy::filterPass(0x77, 8000, RC_CAN_MIN_MS) && !rcdiy::filterPass(0x77, 8030, RC_CAN_MIN_MS) && rcdiy::filterPass(0x77, 8050, RC_CAN_MIN_MS) && rcdiy::filterPass(0x78, 8050, RC_CAN_MIN_MS),
          "全部許可(間隔 0)でも、車の CAN は PID ごとに RC_CAN_MIN_MS");
    rcdiy::filterReset();
  }

  // ---------------- 19. IMU の向きの補正(停車中の重力で「上」、発進の加速で「前」、走行中の長期平均、付け直しの検出、保存と復元)
  {
    imu::begin();
    // センサーは前に 30° 傾けて付いているとする: 車の上 = センサーの (0, −sin30, cos30)、車の前 = (0, cos30, sin30)、右 = (1, 0, 0)
    const float s30 = 0.5f, c30 = 0.8660254f;
    auto sensor = [&](float lon, float lat, float vert, float *x, float *y, float *z) {  // 車の座標 [g](vert は重力込み)→ センサー
      *x = lat;
      *y = lon * c30 - vert * s30;
      *z = lon * s30 + vert * c30;
    };
    float x, y, z, lon, lat, vert;
    uint32_t t = 100000;
    auto still = [&](float tiltLon, int samples) {  // 停車(坂なら重力の前後成分 tiltLon)
      for (int i = 0; i < samples; i++) {
        sensor(tiltLon, 0, sqrtf(1.0f - tiltLon * tiltLon), &x, &y, &z);
        imu::sample(x, y, z, true, 0.0f, t);
        t += 20;
      }
    };
    auto launch = [&](float aLon, float aLat, float kmhEnd) {  // 発進: 3 km/h から kmhEnd まで、0.2 秒ごとに車速が増える
      float kmh = 3.0f;
      for (int i = 0; kmh <= kmhEnd; i++) {
        sensor(aLon, aLat, 1.0f, &x, &y, &z);
        imu::sample(x, y, z, true, kmh, t);
        t += 20;
        if (i % 10 == 9) kmh += 2.1f;  // 0.2 秒で 2.1 km/h ≈ 0.3 g
      }
    };
    still(0, 175);  // 3.5 秒
    CHECK(imu::levelOk() && !imu::forwardOk() && imu::calVersion() == 1, "停車 3 秒で「上」を覚える(「前」はまだ)");
    sensor(0, 0, 1.0f, &x, &y, &z);
    imu::sample(x, y, z, true, 0.0f, t);
    imu::car(&lon, &lat, &vert);
    CHECK(fabsf(vert) < 0.01f, "「上」を覚えると、停車中の上下の加速度は 0(重力を引いた値)");
    launch(0.3f, 0, 10.0f);  // 10 km/h までの発進(後退の想定)では覚えない
    CHECK(!imu::forwardOk(), "IMU_FWD_DONE_KMH に届かない発進では「前」を覚えない");
    still(0, 20);
    launch(0.3f, 0, 25.0f);
    CHECK(imu::forwardOk() && imu::calVersion() == 2, "15 km/h を超える発進の加速で「前」を覚える");
    sensor(0.3f, 0, 1.0f, &x, &y, &z);
    imu::sample(x, y, z, true, 30.0f, t);
    imu::car(&lon, &lat, &vert);
    CHECK(fabsf(lon - 0.3f) < 0.02f && fabsf(lat) < 0.02f && fabsf(vert) < 0.02f, "加速 0.3 g を車の座標で前後 +0.30 に直す");
    sensor(0, 0.4f, 1.0f, &x, &y, &z);
    imu::sample(x, y, z, true, 30.0f, t += 20);
    imu::car(&lon, &lat, &vert);
    CHECK(fabsf(lon) < 0.02f && fabsf(lat - 0.4f * (float)IMU_LAT_SIGN) < 0.02f, "右 0.4 g を横 +0.40(IMU_LAT_SIGN)に直す");
    float gl, gt, gv;
    imu::toCar(0, -s30 * 10.0f, c30 * 10.0f, &gl, &gt, &gv);  // 車の上の軸まわりに 10 deg/s
    CHECK(fabsf(gv - 10.0f) < 0.05f && fabsf(gl) < 0.05f, "ジャイロ: 上の軸まわりの回転をそのまま取り出す");
    // 「前」の 2 回目以降は少しずつ寄せる。左に 0.1 g 曲がりながらの発進(約 18° ずれ)で、向きは 18° ほどは動かない
    still(0, 20);
    launch(0.3f, -0.1f, 25.0f);
    float cal[9], cal2[9];
    imu::exportCal(cal);
    {
      float fdot = cal[4] * c30 + cal[5] * s30;  // 覚えた「前」と本当の前の内積
      CHECK(imu::forwardOk() && fdot > 0.995f, "曲がりながらの発進では「前」が 5° 以上は動かない(少しずつ寄せる)");
    }
    // 保存と復元
    imu::exportCal(cal);
    CHECK(cal[6] == 1.0f && cal[7] == 1.0f && fabsf(cal[1] + s30) < 0.01f && fabsf(cal[2] - c30) < 0.01f, "保存する値: 上 (0, −0.5, 0.87)、済の印");
    imu::reset();
    CHECK(!imu::levelOk() && !imu::forwardOk(), "試験用の reset で忘れる");
    imu::importCal(cal);
    imu::exportCal(cal2);
    CHECK(imu::levelOk() && imu::forwardOk() && memcmp(cal, cal2, sizeof(float) * 9) == 0, "復元すると同じ向きに戻る(重力の大きさも)");
    float broken[9] = { 0, 0, 0, 0, 0, 0, 1, 1, 0 };
    imu::reset();
    imu::importCal(broken);
    CHECK(!imu::levelOk() && !imu::forwardOk(), "壊れた保存値(長さ 0 のベクトル)は使わない");
    imu::importCal(cal);
    // 坂での停車(上が 15° ずれる)では「上」を変えない
    uint32_t ver = imu::calVersion();
    still(-0.2588f, 175 * 3);  // 上り坂で 10 秒
    imu::exportCal(cal2);
    CHECK(imu::calVersion() == ver && imu::forwardOk() && fabsf(cal2[2] - c30) < 0.005f, "坂での停車では「上」を変えない");
    // 走行中の長期平均: 坂で覚えてしまった「上」(15° ずれ)が、平らな道を 20 分走ると直る
    imu::reset();
    still(-0.2588f, 175);
    imu::exportCal(cal2);
    CHECK(imu::levelOk() && fabsf(cal2[2] - c30) > 0.03f, "坂で最初に覚えた「上」はずれている");
    for (int i = 0; i < 50 * 60 * 30; i++) {  // 30 分(時定数 10 分の 3 倍)、30 km/h、加速なし
      sensor(0, 0, 1.0f, &x, &y, &z);
      imu::sample(x, y, z, true, 30.0f, t);
      t += 20;
    }
    imu::exportCal(cal2);
    CHECK(fabsf(cal2[1] + s30) < 0.02f && fabsf(cal2[2] - c30) < 0.02f && imu::calVersion() > 1, "30 分の走行で「上」が正しい向きに寄る(版も進む)");
    // 付け直し(60° ずれ)が 5 回続いたら覚え直し、「前」を忘れる
    imu::reset();
    imu::importCal(cal);
    ver = imu::calVersion();
    for (int k = 0; k < IMU_RELEARN_COUNT - 1; k++) {
      for (int i = 0; i < 175; i++) {
        imu::sample(0.866f, 0, 0.5f, true, 0.0f, t);  // 60° 違う向きで停車
        t += 20;
      }
      imu::sample(0, 0, 1.0f, true, 10.0f, t += 20);  // 間に少し動く(停車の区間を区切る)
      imu::sample(0, 0, 1.0f, true, 10.0f, t += 20);
    }
    CHECK(imu::calVersion() == ver && imu::forwardOk(), "大きく違う「上」が 4 回では変えない");
    for (int i = 0; i < 175; i++) {
      imu::sample(0.866f, 0, 0.5f, true, 0.0f, t);
      t += 20;
    }
    imu::exportCal(cal2);
    CHECK(imu::calVersion() == ver + 1 && imu::levelOk() && !imu::forwardOk() && fabsf(cal2[0] - 0.866f) < 0.02f, "5 回続いたら「上」を覚え直し、「前」は忘れる");
    // 車速の信号が無いとき(机の上)。覚える前の向きは「画面を立てて(+X が上)、背面(+Z)が前」
    imu::reset();
    imu::sample(1.0f, 0, 0, false, 0.0f, t += 20);
    imu::car(&lon, &lat, &vert);
    CHECK(fabsf(lon) < 0.001f && fabsf(lat) < 0.001f && fabsf(vert) < 0.001f, "覚える前: 画面を立てた向き(+X が上)で 0/0/0");
    imu::sample(0, 0, 1.0f, false, 0.0f, t += 20);  // 伏せる(背面が上)
    imu::car(&lon, &lat, &vert);
    CHECK(fabsf(lon - 1.0f) < 0.001f && fabsf(lat) < 0.001f && fabsf(vert + 1.0f) < 0.001f, "覚える前: 伏せると前後 +1.0、上下 −1.0(車が真上を向いた形)");
    imu::sample(0, 0, -1.0f, false, 0.0f, t += 20);  // 仰向け
    imu::car(&lon, &lat, &vert);
    CHECK(fabsf(lon + 1.0f) < 0.001f && fabsf(lat) < 0.001f && fabsf(vert + 1.0f) < 0.001f, "覚える前: 仰向けで前後 −1.0、上下 −1.0");
    imu::sample(0, -1.0f, 0, false, 0.0f, t += 20);
    imu::car(&lon, &lat, &vert);
    CHECK(fabsf(lat + 1.0f * (float)IMU_LAT_SIGN) < 0.001f && fabsf(lon) < 0.001f && fabsf(vert + 1.0f) < 0.001f, "覚える前: −Y が上(右端を下げた形)で横 −1.0(IMU_LAT_SIGN)、上下 −1.0");
    // 値が揺れていれば覚えない(走行中かもしれない)
    for (int i = 0; i < 300; i++) {
      float j = (i & 1) ? 0.04f : -0.04f;  // 直前との差 0.08 g の揺れ
      imu::sample(1.0f, j, 0, false, 0.0f, t);
      t += 20;
    }
    CHECK(!imu::levelOk() && !imu::stillNow(), "車速が無く値が揺れている間は「上」を覚えない");
    // 立てて 20° 後ろへ傾けて置く(倍率が 1.1 のセンサー)。3 秒動かなければ、その向きが「上」、大きさ 1.1 が重力
    const float c20 = 0.9396926f, s20 = 0.3420201f;
    for (int i = 0; i < 175; i++) {
      imu::sample(1.1f * c20, 0, 1.1f * s20, false, 0.0f, t);
      t += 20;
    }
    imu::exportCal(cal2);
    imu::car(&lon, &lat, &vert);
    CHECK(imu::levelOk() && !imu::forwardOk() && imu::stillNow() && fabsf(cal2[0] - c20) < 0.01f && fabsf(cal2[2] - s20) < 0.01f, "車速が無くても、立てた向きで 3 秒動かなければ「上」を覚える(「前」は覚えない)");
    CHECK(fabsf(imu::gravity() - 1.1f) < 0.005f && fabsf(vert) < 0.005f && fabsf(lon) < 0.005f && fabsf(cal2[8] - 1.1f) < 0.005f, "重力の大きさ 1.1 g を覚え(保存値にも入る)、停車中の値は 0/0/0 になる(倍率のずれを直す)");
    for (int i = 0; i < 175; i++) {  // さらに 3.5 秒: 静止は続いている
      imu::sample(1.1f * c20, 0, 1.1f * s20, false, 0.0f, t);
      t += 20;
    }
    CHECK(imu::stillNow(), "静止の判定は 3 秒の区間をまたいで続く");
    imu::sample(1.1f * c20, 0.11f, 1.1f * s20, false, 0.0f, t += 20);  // 右 0.1 g(センサーの値では 0.11)
    imu::car(&lon, &lat, &vert);
    CHECK(fabsf(lat - 0.1f * (float)IMU_LAT_SIGN) < 0.005f && fabsf(vert) < 0.005f && fabsf(lon) < 0.005f, "横 0.1 g も倍率を直して出る");
    // 立て直す(20° 戻す): 車速が無いので置いた向きに合わせ直す(車速があるときは 25° 未満のずれでは動かさない)
    for (int i = 0; i < 180; i++) {
      imu::sample(1.1f, 0, 0, false, 0.0f, t);
      t += 20;
    }
    imu::exportCal(cal2);
    imu::car(&lon, &lat, &vert);
    CHECK(fabsf(cal2[0] - 1.0f) < 0.01f && fabsf(vert) < 0.005f && fabsf(lon) < 0.005f, "車速が無いときは置き直した向きに「上」を合わせ直す");
    // 伏せたまま(初期の上から 90°)では、車速が無くても「上」を変えない(試験で伏せたり仰向けにしたりしても値が読める)
    uint32_t v0 = imu::calVersion();
    for (int i = 0; i < 300; i++) {
      imu::sample(0, 0, 1.1f, false, 0.0f, t);
      t += 20;
    }
    imu::exportCal(cal2);
    imu::car(&lon, &lat, &vert);
    CHECK(imu::calVersion() == v0 && fabsf(cal2[0] - 1.0f) < 0.01f && !imu::stillNow() && fabsf(lon - 1.0f) < 0.01f && fabsf(vert + 1.0f) < 0.01f, "伏せて置いても「上」は変えない(前後 +1.0、上下 −1.0 のまま)");
    for (int i = 0; i < 60; i++) {  // 手で動かすと静止でなくなる
      imu::sample(1.1f + 0.3f * ((i & 1) ? 1 : -1), 0, 0, false, 0.0f, t);
      t += 20;
    }
    CHECK(!imu::stillNow() && imu::levelOk(), "動かすと静止ではなくなるが、覚えた「上」は残る");
    imu::reset();
  }

  // ---------------- 20. オープニング(2026-10-21): 時間で進み、OPENING_MS で走行画面へ。タップで飛ばせる。NG なら延びる。明るさの段階は戻る
  {
    Signals o;
    memset(&o, 0, sizeof(o));
    LinkStats lo;
    memset(&lo, 0, sizeof(lo));
    SysInfo so;
    memset(&so, 0, sizeof(so));
    so.touchOk = true; so.nvsOk = true; so.rc.enabled = true; so.rc.bleOk = true; so.rc.imuOk = true;
    uint32_t to = 90000000;
    static uint16_t fbO[gfx::W * gfx::H];
    ui::testShow(ui::SC_OPENING);
    ui::update(o, lo, ps, so, to);
    CHECK(ui::screen() == ui::SC_OPENING && gfx::dirtyCount() > 0, "オープニング: 最初の update で描き始める");
    gfx::dirtyClear();
    bool sawMax = false;
    int lastLevel = -1;
    uint16_t pxArc = 0;
    for (uint32_t dt = 1; dt < OPENING_MS; dt += 5) {
      if (dt == 701) lo.locked = true;  // 途中で受信が始まる
      ui::update(o, lo, ps, so, to + dt);
      if (ui::blinkLevel() == BL_MAX) sawMax = true;
      lastLevel = ui::blinkLevel();
      if (dt == 241) pxArc = fb[130 * gfx::W + 130];  // 輪が点いていく途中(半分): 水温の弧の始まりの近く(225°、半径 146)
      if (dt == 1201) memcpy(fbO, fb, sizeof(fbO));
      gfx::dirtyClear();
    }
    CHECK(ui::screen() == ui::SC_OPENING, "OPENING_MS の手前ではまだオープニング");
    CHECK(sawMax, "輪が全部点いたときに最大輝度になる");
    CHECK(lastLevel == BL_NORMAL, "終わりの手前では設定の明るさに戻っている");
    CHECK(pxArc == gfx::rgb(0x2F8BFF), "輪が点いていく間、水温の弧も伸びる(始まりの近くが青)");
    // 受信が始まったので「CAN」の行は OK(緑)になっている: 1.2 s の絵に緑の点がある(x 112, y 164 の円)。そのとき弧は消えている
    uint16_t px = fbO[164 * gfx::W + 112];
    CHECK(px == gfx::rgb(0x22E36A), "受信が始まると CAN の行が OK(緑)に変わる");
    CHECK(fbO[130 * gfx::W + 130] == gfx::rgb(0x000000), "自己診断が出るときには弧は消えている");
    ui::update(o, lo, ps, so, to + OPENING_MS);
    CHECK(ui::screen() == ui::SC_DRIVE && ui::blinkLevel() == BL_NORMAL, "OPENING_MS で覚えた画面(サーキット)へ(明るさは設定の値)");
    gfx::dirtyClear();
    // タップで飛ばす
    ui::testShow(ui::SC_OPENING);
    ui::update(o, lo, ps, so, to + 10000);
    ui::tap(233, 233, to + 10200);
    ui::update(o, lo, ps, so, to + 10210);
    CHECK(ui::screen() == ui::SC_DRIVE, "オープニングはタップで飛ばせる");
    gfx::dirtyClear();
    // NG(タッチが応答しない)なら OPENING_NG_EXTRA_MS だけ長く見せる
    so.touchOk = false;
    ui::testShow(ui::SC_OPENING);
    ui::update(o, lo, ps, so, to + 20000);
    for (uint32_t dt = 5; dt <= OPENING_MS + 100; dt += 5) ui::update(o, lo, ps, so, to + 20000 + dt);
    CHECK(ui::screen() == ui::SC_OPENING, "NG があると OPENING_MS を過ぎてもオープニングのまま");
    ui::update(o, lo, ps, so, to + 20000 + OPENING_MS + OPENING_NG_EXTRA_MS);
    CHECK(ui::screen() == ui::SC_DRIVE, "NG があっても OPENING_MS + OPENING_NG_EXTRA_MS で覚えた画面へ");
    gfx::dirtyClear();
    so.touchOk = true;
  }

  // ---------------- 21. サーキットの 3000rpm 以下の経過時間(10/23): N が IDLE_SHOW_AFTER_N_MS 続いたらギアの所に出す
  {
    Signals n;
    memset(&n, 0, sizeof(n));
    n.linkUp = n.rpmOk = n.gearOk = true;
    n.rpm = 850; n.gear = 'N';
    uint32_t ti = 97000000;
    ui::testShow(ui::SC_DRIVE);
    ui::update(n, ls, ps, si, ti);
    static uint16_t fbI[gfx::W * gfx::H];
    ui::update(n, ls, ps, si, ti + IDLE_SHOW_AFTER_N_MS - 1000);
    memcpy(fbI, fb, sizeof(fbI));
    ui::update(n, ls, ps, si, ti + IDLE_SHOW_AFTER_N_MS + 1000);
    CHECK(memcmp(fbI, fb, sizeof(fbI)) != 0, "N が 1 分続いたら、ギアの所が経過時間に変わる");
    memcpy(fbI, fb, sizeof(fbI));
    ui::testRedraw();
    ui::update(n, ls, ps, si, ti + IDLE_SHOW_AFTER_N_MS + 1000);
    CHECK(memcmp(fbI, fb, sizeof(fbI)) == 0, "経過時間の絵 = 同じ時刻に最初から描いた絵");
    n.rpm = 3500;  // 3000 を超えると 0 に戻る(N のままなので、ギアの所は 0:00 から)
    ui::update(n, ls, ps, si, ti + IDLE_SHOW_AFTER_N_MS + 2000);
    n.rpm = 900;
    ui::update(n, ls, ps, si, ti + IDLE_SHOW_AFTER_N_MS + 3000);
    memcpy(fbI, fb, sizeof(fbI));
    ui::testRedraw();
    ui::update(n, ls, ps, si, ti + IDLE_SHOW_AFTER_N_MS + 3000);
    CHECK(memcmp(fbI, fb, sizeof(fbI)) == 0, "数え直したあとも最初から描いた絵と同じ");
    n.gear = '1';
    ui::update(n, ls, ps, si, ti + IDLE_SHOW_AFTER_N_MS + 4000);
    gfx::dirtyClear();
    // 3000rpm 超・以下の累計(trip)
    trip::resetTimes();
    Signals r;
    memset(&r, 0, sizeof(r));
    trip::update(r, 1000);  // 信号なし(前の試験からの間は数えない)
    r.rpmOk = true;
    r.rpm = 3001;
    trip::update(r, 1400);
    r.rpm = 3000;
    trip::update(r, 1800);
    r.rpmOk = false;
    trip::update(r, 2200);
    CHECK(trip::aboveMs() == 400 && trip::belowMs() == 400, "3000rpm 超・以下の時間を数える(信号が無い間は数えない)");
  }

  // ---------------- 22. 車の CAN の生データから値を作る(2.0.0。c6_can_link 2.0.0 は値のフレームを送らず、生データだけを送る)
  {
    veh::begin();
    mlink::begin();
    cantab::begin();
    uint8_t seq = 0;
    auto canFrame = [&](uint8_t bus, uint32_t id, const uint8_t *d, uint8_t dlc, uint32_t t) {
      uint8_t p[sizeof(MlCanHead) + 8];
      MlCanHead h;
      h.info = (uint8_t)((bus ? 0x80 : 0) | 0x40 | (dlc & 0x0F));  // 29 ビット ID
      h.id = id;
      memcpy(p, &h, sizeof(h));
      memcpy(p + sizeof(h), d, dlc);
      auto f = frame(ML_TYPE_CAN, seq++, p, (uint8_t)(sizeof(h) + dlc));
      mlink::feed(f.data(), f.size(), t);
    };
    uint32_t t = 500000;
    const uint8_t rpm3000[8] = { 0, 0, 0x0B, 0xB8, 0, 0, 0, 255 };  // 3000 rpm、アクセル 255(100 %)
    const uint8_t kmh60[8] = { 0, 0, 0, 0, 0x1E, 0x00, 0, 0 };     // 7680 ÷ 128 = 60 km/h
    const uint8_t eng[8] = { 0, 0, 0, 130, 65, 0, 0, 0 };          // 水温 130 − 40 = 90 ℃、ブースト 65 × 0.01 = 0.65 bar
    const uint8_t clutch[8] = { 0, 0, 0, 0, 0, 0x20, 0, 0 };
    canFrame(0, 0x0618A001, rpm3000, 8, t);
    canFrame(0, 0x0210A006, kmh60, 8, t);
    canFrame(0, 0x0A18A001, eng, 8, t);
    canFrame(0, 0x0628A001, clutch, 8, t);
    mlink::getSignals(&s, t);
    CHECK(s.linkUp && mlink::usingRaw(t) && s.rpmOk && s.rpm == 3000 && s.throttleOk && s.throttlePct == 100.0f, "生データ: 回転数・アクセル(0618A001 b2-3、b7)");
    CHECK(s.speedOk && s.speedKmh == 60.0f, "生データ: 車速(0210A006 b4-5 ÷ 128)");
    CHECK(s.waterOk && s.waterC == 90.0f && s.boostOk && fabsf(s.boostBar - 0.65f) < 0.0001f, "生データ: 水温 b3 − 40、ブースト b4 × 0.01 bar(0A18A001)");
    CHECK(s.gearOk && s.gear == '3', "生データ: 3000 rpm ÷ 60 km/h = 50 → 3 速(仮の変速比、クラッチを踏んでいても止めない)");
    CHECK(!s.fuelOk && !s.sim && s.scene == 0, "生データ: 燃料はまだ無い。模擬ではない");
    const uint8_t engR[8] = { 0, 0, 0, 130, 0, 0, 0, 0x04 };
    canFrame(0, 0x0A18A001, engR, 8, t + 20);
    mlink::getSignals(&s, t + 20);
    CHECK(s.gearOk && s.gear == 'R', "生データ: 0A18A001 b7 bit2 = R(推定より優先)");
    const uint8_t kmh1[8] = { 0, 0, 0, 0, 0x00, 0x80, 0, 0 };  // 1 km/h
    canFrame(0, 0x0A18A001, eng, 8, t + 40);
    canFrame(0, 0x0210A006, kmh1, 8, t + 40);
    mlink::getSignals(&s, t + 40);
    CHECK(s.gearOk && s.gear == 'N', "生データ: 2 km/h 未満は N");
    const uint8_t rpm5400[8] = { 0, 0, 0x15, 0x18, 0, 0, 0, 0 };
    canFrame(0, 0x0618A001, rpm5400, 8, t + 60);
    canFrame(0, 0x0210A006, kmh60, 8, t + 60);
    mlink::getSignals(&s, t + 60);
    CHECK(!s.gearOk && s.throttlePct == 0.0f, "生データ: 5400 rpm ÷ 60 km/h = 90 → 1 速(116)と 2 速(67)の ±12 % の間(変速の途中扱い)= 分からない");
    // 「信号なし」: 同じ ID が VEH_TIMEOUT_MS 来なければ外す(線のほかのフレームは来ていて、受信はある)
    const uint8_t fuelZero[8] = { 0, 0, 0x59, 0x95, 0x88, 0x00, 0, 0 };
    canFrame(1, 0x0C014003, fuelZero, 8, t + 60 + VEH_TIMEOUT_MS);
    mlink::getSignals(&s, t + 60 + VEH_TIMEOUT_MS);
    CHECK(s.linkUp && s.rpmOk && s.speedOk, "生データ: VEH_TIMEOUT_MS ちょうどまでは信号あり");
    mlink::getSignals(&s, t + 61 + VEH_TIMEOUT_MS);
    CHECK(s.linkUp && !s.rpmOk && !s.speedOk && !s.throttleOk && !s.gearOk && s.waterOk, "生データ: VEH_TIMEOUT_MS を過ぎた値は信号なし(水温は VEH_WATER_TIMEOUT_MS)");
    CHECK(!s.fuelOk, "燃料: b5 = 0(IG ON 直後)だけでは値なし(前回の値も無いとき)");
    veh::FuelInfo fi;
    veh::fuelInfo(&fi, t + 61 + VEH_TIMEOUT_MS);
    CHECK(fi.zeroFrames == 1 && !fi.have && fi.seen, "燃料: b5 = 0 は数えて捨てる");
    const uint8_t fuel155[8] = { 0, 0, 0x59, 0x95, 0x80, 155, 0, 0 };
    canFrame(1, 0x0C014003, fuel155, 8, t + 1060 + VEH_TIMEOUT_MS);
    mlink::getSignals(&s, t + 1060 + VEH_TIMEOUT_MS);
    CHECK(s.fuelOk && s.fuelPct == 71.5f, "燃料: 最初の正しい値から始める(155 → (155 − 120) ÷ 49 = 71.4 % → 0.5 % 刻みで 71.5)");
    int tenths = 0;
    CHECK(veh::fuelSaveValue(&tenths) && tenths == 1550, "燃料: 保存する値は生の値の 10 倍");

    // 前回の値から始める(電源を入れ直した)
    veh::begin();
    CHECK(!veh::fuelSaveValue(&tenths), "燃料: 値が無いときは保存しない");
    veh::fuelRestore(1600);
    uint32_t t2 = t + 100000;
    canFrame(1, 0x0C014003, fuelZero, 8, t2);
    mlink::getSignals(&s, t2);
    CHECK(s.fuelOk && s.fuelPct == 81.5f, "燃料: IG ON 直後(b5 = 0 の間)から前回の値を出す(160 → 81.6 % → 81.5)");
    const uint8_t fuel162[8] = { 0, 0, 0x59, 0x95, 0x80, 162, 0, 0 };
    canFrame(1, 0x0C014003, fuel162, 8, t2 + 1000);
    mlink::getSignals(&s, t2 + 1000);
    veh::fuelInfo(&fi, t2 + 1000);
    CHECK(s.fuelPct == 81.5f && fi.refuels == 0 && fi.restored && fi.lp == 160.0f, "燃料: 最初の値が前回 + 5 L 未満なら前回の値を続ける(停めた場所の傾き)");
    veh::begin();
    veh::fuelRestore(1400);
    const uint8_t fuel169[8] = { 0, 0, 0x59, 0x95, 0x81, 169, 0, 0 };
    canFrame(1, 0x0C014003, fuelZero, 8, t2 + 2000);
    canFrame(1, 0x0C014003, fuel169, 8, t2 + 3000);
    mlink::getSignals(&s, t2 + 3000);
    veh::fuelInfo(&fi, t2 + 3000);
    CHECK(s.fuelOk && s.fuelPct == 100.0f && fi.refuels == 1, "燃料: 最初の値が前回 + 5 L 以上なら給油とみなし、その値から(140 → 169)");
    CHECK(veh::fuelSaveValue(&tenths) && tenths == 1690, "燃料: 給油のあとの値を保存する");
    // 途切れずに受けている間は、増える方向へは跳ばない(なまし)
    canFrame(1, 0x0C014003, fuel155, 8, t2 + 4000);
    mlink::getSignals(&s, t2 + 4000);
    CHECK(s.fuelPct > 99.0f && s.fuelPct < 100.5f, "燃料: 1 回の低い値ではほとんど下がらない(600 秒のなまし)");

    // 値のフレームが届いている間はそちらを使う(c6_display_debug・デモ表示・1.x の c6_can_link)
    MlSignals m = sigMsg(4200);
    auto f = frame(ML_TYPE_SIGNALS, seq++, &m, sizeof(m));
    mlink::feed(f.data(), f.size(), t2 + 5000);
    canFrame(0, 0x0618A001, rpm3000, 8, t2 + 5000);
    mlink::getSignals(&s, t2 + 5000);
    CHECK(s.linkUp && !mlink::usingRaw(t2 + 5000) && s.rpm == 4200 && s.gear == '3' && s.fuelPct == 62.0f, "値のフレームが届いている間はそちらを使う");
    canFrame(0, 0x0618A001, rpm3000, 8, t2 + 5000 + LINK_TIMEOUT_MS + 1);
    mlink::getSignals(&s, t2 + 5000 + LINK_TIMEOUT_MS + 1);
    CHECK(s.linkUp && mlink::usingRaw(t2 + 5000 + LINK_TIMEOUT_MS + 1) && s.rpm == 3000, "値のフレームが LINK_TIMEOUT_MS 来なければ生データから作る");
    mlink::getSignals(&s, t2 + 5002 + 2 * LINK_TIMEOUT_MS);
    CHECK(!s.linkUp && !s.rpmOk, "どのフレームも LINK_TIMEOUT_MS 来なければ受信なし");

    // 壊れた・短いフレームで落ちない(値は範囲の端)
    veh::begin();
    uint8_t rnd[8];
    uint32_t r = 12345;
    for (int i = 0; i < 20000; i++) {
      for (int b = 0; b < 8; b++) rnd[b] = (uint8_t)((r = r * 1103515245u + 12345u) >> 16);
      static const uint32_t IDS[] = { 0x0618A001, 0x0210A006, 0x0628A001, 0x0A18A001, 0x0C014003, 0x12345678 };
      uint32_t id = IDS[(r >> 8) % 6];
      veh::onFrame((uint8_t)((r >> 4) & 1), (uint8_t)((r >> 5) & 1), id, (uint8_t)((r >> 12) % 9), rnd, t2 + 10000 + (uint32_t)i * 7);
      if (i % 50 == 0) {
        Signals v;
        memset(&v, 0, sizeof(v));
        veh::getSignals(&v, t2 + 10000 + (uint32_t)i * 7);
        CHECK(v.fuelPct >= 0.0f && v.fuelPct <= 100.0f && v.speedKmh >= 0.0f && v.speedKmh < 512.0f && v.throttlePct <= 100.0f && v.waterC >= -40.0f && v.waterC <= 215.0f,
              "でたらめな生データでも値は範囲の中");
      }
    }
  }

  // ---------------- 22a. 10/22 の実車ログ(logs/ の L 行)を流す: IG ON 直後の b5 = 0 で燃料が 0 にならない、R とブーストが出る
  {
    struct LogRun {
      const char *path;
      int lines;
      uint32_t zero;
      float fuelMin, fuelAfterRefuel, boostMax;
      bool sawR, sawGear;
    };
    auto run = [&](const char *path, uint32_t refuelFrom, LogRun *out) -> bool {
      FILE *fp = fopen(path, "r");
      if (!fp) return false;
      veh::begin();
      memset(out, 0, sizeof(*out));
      out->path = path;
      out->fuelMin = 1000;
      out->fuelAfterRefuel = 1000;
      char line[256];
      while (fgets(line, sizeof(line), fp)) {
        char bus;
        unsigned long tm;
        unsigned id, len, b[8];
        if (sscanf(line, "L %c %lu %x %u %x %x %x %x %x %x %x %x", &bus, &tm, &id, &len, &b[0], &b[1], &b[2], &b[3], &b[4], &b[5], &b[6], &b[7]) < 4 || len > 8) continue;
        uint8_t d[8] = { 0 };
        for (unsigned k = 0; k < len; k++) d[k] = (uint8_t)b[k];
        veh::onFrame(bus == 'B' ? 1 : 0, 1, id, (uint8_t)len, d, (uint32_t)tm);
        out->lines++;
        Signals v;
        memset(&v, 0, sizeof(v));
        veh::getSignals(&v, (uint32_t)tm);
        if (v.fuelOk && v.fuelPct < out->fuelMin) out->fuelMin = v.fuelPct;
        if (refuelFrom && tm >= refuelFrom && v.fuelOk && v.fuelPct < out->fuelAfterRefuel) out->fuelAfterRefuel = v.fuelPct;
        if (v.boostOk && v.boostBar > out->boostMax) out->boostMax = v.boostBar;
        if (v.gearOk && v.gear == 'R') out->sawR = true;
        if (v.gearOk && v.gear >= '1' && v.gear <= '5') out->sawGear = true;
      }
      fclose(fp);
      veh::FuelInfo fi;
      veh::fuelInfo(&fi, 0);
      out->zero = fi.zeroFrames;
      return true;
    };
    LogRun lr;
    if (run("../logs/can_reverse.txt", 0, &lr)) {
      printf("  実車ログ %s: %d 行、b5 = 0 を捨てた %u 回、燃料の最小 %.1f %%、R %s\n", lr.path, lr.lines, (unsigned)lr.zero, (double)lr.fuelMin, lr.sawR ? "あり" : "なし");
      CHECK(lr.zero >= 1 && lr.fuelMin > 50.0f, "実車ログ(リバース): IG ON 直後の b5 = 0 で燃料が 0 にならない(1.1.0 は約 15 分 0 % だった)");
      CHECK(lr.sawR, "実車ログ(リバース): R が出る");
    } else {
      printf("  (../logs/can_reverse.txt が無いので、実車ログの試験は飛ばす)\n");
    }
    if (run("../logs/can_boost.txt", 0, &lr)) {
      printf("  実車ログ %s: %d 行、ブーストの最大 %.2f bar、ギア %s\n", lr.path, lr.lines, (double)lr.boostMax, lr.sawGear ? "あり" : "なし");
      CHECK(fabsf(lr.boostMax - 0.67f) < 0.001f && lr.sawGear, "実車ログ(ブースト): ログの b4 の最大 0x43 = 67 → 0.67 bar、走行中にギアが出る");
    }
    if (run("../logs/can_fuel.txt", 850932, &lr)) {
      printf("  実車ログ %s: %d 行、b5 = 0 を捨てた %u 回、給油(10/22 846.9 秒に IG ON)のあとの燃料の最小 %.1f %%\n", lr.path, lr.lines, (unsigned)lr.zero, (double)lr.fuelAfterRefuel);
      CHECK(lr.zero >= 1 && lr.fuelAfterRefuel >= 95.0f, "実車ログ(給油): 給油後の IG ON から満タン近くを出し続ける(0 % にならない)");
    }
  }

  // ---------------- 23. 受信の状態の「描画」(2.0.1): サーキット・タコで 1 回描いて送るのにかかった最長(PerfStats::pageMaxUs)。
  //                    数える回(ui::pageFrame)は、サーキット・タコをふだんどおり描いた回だけ(画面の切り替えと長押しの表示は除く)
  {
    Signals d;
    memset(&d, 0, sizeof(d));
    d.linkUp = d.rpmOk = d.waterOk = d.fuelOk = d.gearOk = d.speedOk = d.throttleOk = true;
    d.rpm = 2500; d.waterC = 85; d.fuelPct = 60; d.gear = '3';
    uint32_t tp = 90000000;
    // 値の文字と判定(0 = 待ち、1 = 正常、2 = 異常)
    char v[48];
    PerfStats p;
    memset(&p, 0, sizeof(p));
    p.fps = 2; p.loopMaxMs = 77;  // 2.0.0 まではこれで「2 fps 77 ms」の赤だった(受信の状態の画面を見ている間は必ず 2)
    int vd = ui::testInfoItem(5, ls, p, si, tp, v, sizeof(v));
    CHECK(vd == 0 && strcmp(v, "– –") == 0, "描画: まだ測っていなければ「– –」の灰(送った回数は使わない)");
    p.pageMaxUs = 21400;
    vd = ui::testInfoItem(5, ls, p, si, tp, v, sizeof(v));
    CHECK(vd == 1 && strcmp(v, "max 22 ms (45 fps)") == 0, "描画: 21.4 ms → 切り上げて「max 22 ms (45 fps)」の正常");
    p.pageMaxUs = 50000;
    vd = ui::testInfoItem(5, ls, p, si, tp, v, sizeof(v));
    CHECK(vd == 1 && strcmp(v, "max 50 ms (20 fps)") == 0, "描画: 50 ms ちょうどは正常(INFO_PAGE_MAX_MS を超えたら異常)");
    p.pageMaxUs = 50001;
    vd = ui::testInfoItem(5, ls, p, si, tp, v, sizeof(v));
    CHECK(vd == 2 && strcmp(v, "max 51 ms (19 fps)") == 0, "描画: 50 ms を超えたら異常");
    p.pageMaxUs = 1;
    vd = ui::testInfoItem(5, ls, p, si, tp, v, sizeof(v));
    CHECK(vd == 1 && strcmp(v, "max 1 ms (1000 fps)") == 0, "描画: 1 ms 未満は 1 ms(0 で割らない)");
    // 画面に描いて、字が全部あること(値の書体に無い字を使っていない)と、下の 6 個目の点の色
    const uint32_t miss0 = gfx::missingGlyphs();
    auto dot5 = [&]() { return fb[330 * gfx::W + (int)(233 - 26 * 3 + 26 * 5)]; };
    const uint32_t us[4] = { 0, 21400, 50001, 120000 };
    const uint16_t col[4] = { gfx::rgb(0x5A606A), gfx::rgb(0x22E36A), gfx::rgb(0xE3241B), gfx::rgb(0xE3241B) };
    bool colOk = true;
    for (int k = 0; k < 4; k++) {
      p.pageMaxUs = us[k];
      ui::testShow(ui::SC_SET_INFO, 0);  // 0 項目目を見ているので、6 個目の点はその項目の判定の色(小さい点)
      ui::testTouch(tp += 600);          // 設定から時間で戻らないように
      ui::update(d, ls, p, si, tp);
      if (ui::screen() != ui::SC_SET_INFO || dot5() != col[k]) colOk = false;
      ui::testShow(ui::SC_SET_INFO, 5);  // 「描画」の画面そのもの
      ui::testTouch(tp += 600);
      ui::update(d, ls, p, si, tp);
      if (ui::screen() != ui::SC_SET_INFO) colOk = false;
    }
    CHECK(colOk, "描画: 下の 6 個目の点は 測っていない = 灰、正常 = 緑、異常 = 赤");
    CHECK(gfx::missingGlyphs() == miss0, "描画: 値の字はすべて書体にある(「最長」は書体に無いので max)");
    gfx::dirtyClear();

    // 数える回: サーキットに入った回(全体の描き直し)は数えない。そのあとのふだんの回は数える
    ui::testShow(ui::SC_DRIVE);
    ui::update(d, ls, p, si, tp += 100);
    CHECK(!ui::pageFrame(), "数える回: サーキットに切り替えた回の全体の描き直しは数えない");
    d.rpm = 4200;
    ui::update(d, ls, p, si, tp += 20);
    CHECK(ui::pageFrame() && gfx::dirtyCount() > 0, "数える回: サーキットのふだんの回(回転数が変わって描き直した所がある)");
    gfx::dirtyClear();
    // 長押しの表示: 出し始めた回(画面全体を暗くする)と、出している間は数えない。動いて取り消し、表示を消した回(全体の描き直し)も数えない
    ui::touchDown(233, 233, tp += 20);
    ui::update(d, ls, p, si, tp + 50);
    CHECK(ui::pageFrame(), "数える回: 押した直後(長押しの表示の前)はふだんの回");
    ui::update(d, ls, p, si, tp + HOLD_SHOW_MS + 10);
    CHECK(ui::testHolding() && !ui::pageFrame(), "数える回: 長押しの表示を出し始めた回は数えない");
    ui::update(d, ls, p, si, tp + HOLD_SHOW_MS + 30);
    CHECK(!ui::pageFrame(), "数える回: 長押しの表示を出している間は数えない");
    ui::touchMove(233 - HOLD_CANCEL_PX - 10, 233, tp + HOLD_SHOW_MS + 40);
    ui::update(d, ls, p, si, tp + HOLD_SHOW_MS + 50);
    CHECK(ui::screen() == ui::SC_DRIVE && !ui::testHolding() && !ui::pageFrame(), "数える回: 動いて長押しを取り消し、表示を消した回(全体の描き直し)は数えない");
    d.rpm = 4600;
    ui::update(d, ls, p, si, tp + HOLD_SHOW_MS + 70);
    CHECK(ui::pageFrame(), "数える回: 取り消したあと、押している間のふだんの回");
    ui::touchMove(233 - 130, 236, tp + HOLD_SHOW_MS + 80);
    ui::touchUp(tp += HOLD_SHOW_MS + 90);  // 左へスワイプ = タコへ
    ui::update(d, ls, p, si, tp);
    CHECK(ui::screen() == ui::SC_BOOST && !ui::pageFrame(), "数える回: スワイプで切り替えた回(タコの全体の描き直し)は数えない");
    d.rpm = 3100;
    ui::update(d, ls, p, si, tp += 20);
    CHECK(ui::pageFrame(), "数える回: タコのふだんの回");
    ui::testShow(ui::SC_DATA);
    ui::update(d, ls, p, si, tp += 100);
    d.rpm = 3300;
    ui::update(d, ls, p, si, tp += 100);
    CHECK(!ui::pageFrame(), "数える回: データは数えない");
    ui::testShow(ui::SC_SET_LIST, 7);
    ui::update(d, ls, p, si, tp += 100);
    ui::update(d, ls, p, si, tp += 100);
    CHECK(!ui::pageFrame(), "数える回: 設定は数えない");
    gfx::dirtyClear();
  }

  printf("単体試験: %d 個 OK、%d 個 NG、無かった文字 %u\n", passes, fails, (unsigned)gfx::missingGlyphs());
  printf("%s\n", (fails == 0 && gfx::missingGlyphs() == 0) ? "UNIT PASS" : "UNIT FAIL");
  return fails ? 1 : 0;
}

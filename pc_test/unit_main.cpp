// unit_main.cpp - PC 上の単体試験: UART データの受け取り方、画面の操作(タップ・スワイプ・長押し)、設定、自動で戻る、
//                 車の CAN の生データから値を作る(veh。../logs/ の 10/22 の実車ログも流す)、本体の状態の「FPS」(2.3.0)、タコの赤(2.4.0)
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
extern size_t g_hostCanPreN;
extern int g_hostCanPreSaves;
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
// 設定の中の下の帯「長押しで開く」を長押しして離す(2.4.4。2.4.3 までは帯のタップで開いていた)
static void holdOpen(const Signals &sg, uint32_t &tt) {
  ui::touchDown(233, 400, tt += 100);
  ui::update(sg, ls, ps, si, tt + TOUCH_CONFIRM_MS + 10);
  ui::update(sg, ls, ps, si, tt + HOLD_SHOW_MS + 10);
  ui::update(sg, ls, ps, si, tt + HOLD_MS);
  ui::touchUp(tt += HOLD_MS + 20);
  ui::update(sg, ls, ps, si, tt);
  ui::update(sg, ls, ps, si, tt + 60);
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

  // ---------------- 6. 画面の操作(2.1.0 / 2.3.0 / 2.4.0): タッチは TOUCH_CONFIRM_MS 押し続けたら決まる(短いタッチは何もしない)。
  //                  2.4.0: どの画面も離したときに決める。左へスワイプ = 次、右へ = 前(2.4.2。サーキット・タコ・データは画面、設定の中は
  //                  項目・ページ)。動きが HOLD_CANCEL_PX 未満ならタップ。サーキット・タコ・データは長押しで設定、タッチが決まったら案内
  Signals live;
  memset(&live, 0, sizeof(live));
  live.linkUp = live.rpmOk = live.waterOk = live.fuelOk = live.gearOk = live.speedOk = live.throttleOk = true;
  live.rpm = 3000; live.waterC = 90; live.fuelPct = 50; live.gear = '2';
  const int TOPX = 233, TOPY = 60, BOTX = 233, BOTY = 400, LX = 100, RX = 366, MY = 233;
  uint32_t t = 10000;
  // 長押し(HOLD_MS)で設定に入って離す
  auto longPress = [&](const Signals &sg, uint32_t &tt) {
    ui::touchDown(233, 233, tt += 100);
    ui::update(sg, ls, ps, si, tt + TOUCH_CONFIRM_MS + 10);
    ui::update(sg, ls, ps, si, tt + HOLD_SHOW_MS + 10);
    ui::update(sg, ls, ps, si, tt + HOLD_MS);
    ui::touchUp(tt += HOLD_MS + 50);
    upd(sg, tt);
  };
  // 真ん中から (dx, dy) だけ指を動かして離す(途中の知らせ 2 回)。触れてから離すまで ms
  auto swipe = [&](const Signals &sg, uint32_t &tt, int dx, int dy, uint32_t ms = TOUCH_CONFIRM_MS + 100) {
    ui::touchDown(233, 233, tt += 100);
    ui::update(sg, ls, ps, si, tt + 20);
    ui::touchMove(233 + dx / 2, 233 + dy / 2, tt + ms / 3);
    ui::touchMove(233 + dx, 233 + dy, tt + ms * 2 / 3);
    ui::update(sg, ls, ps, si, tt + ms * 2 / 3 + 5);
    ui::touchUp(tt += ms);
    upd(sg, tt);
  };
  // 設定の中などの送り(指で左へ = 次、右へ = 前)
  auto next = [&](uint32_t &tt) { swipe(live, tt, -200, 0); };
  auto prev = [&](uint32_t &tt) { swipe(live, tt, 200, 0); };
  ui::testShow(ui::SC_DRIVE);
  upd(live, t);
  CHECK(ui::screen() == ui::SC_DRIVE, "サーキットから");
  // スワイプ(2.4.2: 左へ = 次、右へ = 前。2.4.0〜2.4.1 の右へ = 次を戻した。本人「間違えました 左にスワイプでタコでした」)
  swipe(live, t, -200, 10);
  CHECK(ui::screen() == ui::SC_BOOST, "スワイプ: 左へでタコ(次)");
  swipe(live, t, -(SWIPE_MIN_PX + 10), -20);
  CHECK(ui::screen() == ui::SC_DATA, "スワイプ: 左へでデータ(次)");
  swipe(live, t, -200, 0);
  CHECK(ui::screen() == ui::SC_DRIVE, "スワイプ: データの次はサーキット");
  swipe(live, t, 200, 0);
  CHECK(ui::screen() == ui::SC_DATA, "スワイプ: 右へで前(サーキットの前はデータ)");
  swipe(live, t, 200, 0);
  CHECK(ui::screen() == ui::SC_BOOST && !ui::testHolding() && !ui::testGuide(), "スワイプ: 右へで前(データの前はタコ)。離したら案内・長押しの表示は消える");
  swipe(live, t, -(SWIPE_MIN_PX - 10), 0);
  CHECK(ui::screen() == ui::SC_BOOST, "スワイプ: 横の動きが SWIPE_MIN_PX 未満なら替えない(2.2.0: 80 px では替わっていた幅)");
  swipe(live, t, -80, 0);
  CHECK(ui::screen() == ui::SC_BOOST, "スワイプ: 2.1.0 で替わっていた 80 px では替えない");
  swipe(live, t, -200, 110);
  CHECK(ui::screen() == ui::SC_BOOST, "スワイプ: 斜め(横が縦の SWIPE_DIR_RATIO 倍未満)なら替えない");
  swipe(live, t, 0, -150);
  CHECK(ui::screen() == ui::SC_BOOST, "スワイプ: 縦は何もしない");
  swipe(live, t, -200, 0, TOUCH_CONFIRM_MS - 60);
  CHECK(ui::screen() == ui::SC_BOOST, "スワイプ: TOUCH_CONFIRM_MS より短いスワイプは何もしない(2.3.0)");
  swipe(live, t, -200, 0, TOUCH_CONFIRM_MS + 1);
  CHECK(ui::screen() == ui::SC_DATA, "スワイプ: TOUCH_CONFIRM_MS 押していれば替える(update でまだ決めていなくても、離したときに確かめる)");
  swipe(live, t, 200, 0);
  CHECK(ui::screen() == ui::SC_BOOST, "スワイプ: 右へでタコに戻る");
  // 短いタッチ(タップ)は何もしない。案内も出さない
  ui::touchDown(233, 233, t += 100);
  ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS - 20);
  CHECK(!ui::testGuide(), "案内: タッチが決まる前は出さない");
  ui::touchUp(t += TOUCH_CONFIRM_MS - 10);
  upd(live, t);
  CHECK(ui::screen() == ui::SC_BOOST && !ui::testHolding() && !ui::testGuide(), "タコ: 短いタッチは何もしない");
  ui::tap(233, 233, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_BOOST, "タコ: タップ(ui::tap)も何もしない");
  // 案内(2.4.0: 上に「‹ スワイプ ›」、下に「長押し ▶ 設定」)。出している間は描き直さない。指が HOLD_CANCEL_PX 以上ずれたら、
  // 案内のまま長押しの札を灰にする(元の画面に戻さない。本人「タッチしているのかを認識しているかが分かりづらい」)。離したら消して描き直す
  {
    ui::touchDown(233, 233, t += 100);
    ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 10);
    CHECK(ui::testGuide() && ui::testGuideMode() == 0 && ui::screen() == ui::SC_BOOST && gfx::dirtyCount() > 0, "案内: タッチが決まったら出す(2.3.0)");
    gfx::dirtyClear();
    Signals l2 = live;
    l2.rpm = 4500;
    ui::update(l2, ls, ps, si, t + TOUCH_CONFIRM_MS + 30);
    CHECK(ui::testGuide() && gfx::dirtyCount() == 0, "案内: 出している間は回転数が変わっても描き直さない");
    ui::touchMove(233 + HOLD_CANCEL_PX - 10, 236, t + TOUCH_CONFIRM_MS + 40);
    ui::update(l2, ls, ps, si, t + TOUCH_CONFIRM_MS + 50);
    CHECK(ui::testGuide() && ui::testGuideMode() == 0 && gfx::dirtyCount() == 0, "案内: HOLD_CANCEL_PX 未満のずれでは何も変えない(2.4.0: 30 → 80 px)");
    ui::touchMove(233 + HOLD_CANCEL_PX + 10, 236, t + TOUCH_CONFIRM_MS + 60);
    ui::update(l2, ls, ps, si, t + TOUCH_CONFIRM_MS + 70);
    CHECK(ui::testGuide() && ui::testGuideMode() == 2 && gfx::dirtyCount() > 0 && ui::screen() == ui::SC_BOOST, "案内: HOLD_CANCEL_PX 以上ずれたら、案内のまま長押しの札を灰(2.4.0)");
    gfx::dirtyClear();
    ui::touchMove(233 + SWIPE_MIN_PX + 5, 238, t + TOUCH_CONFIRM_MS + 80);
    ui::update(l2, ls, ps, si, t + TOUCH_CONFIRM_MS + 90);
    CHECK(ui::testGuide() && ui::testGuideMode() == 2 && gfx::dirtyCount() == 0, "案内: 灰にしたあとは、動かしても描き直さない");
    ui::update(l2, ls, ps, si, t + HOLD_MS + 100);
    CHECK(ui::testGuide() && !ui::testHolding() && ui::screen() == ui::SC_BOOST, "案内: ずれたら長押しにならない(HOLD_MS を過ぎても案内のまま)");
    ui::touchMove(233 + 20, 236, t + HOLD_MS + 110);  // 戻してから離す: スワイプにならない
    ui::touchUp(t += HOLD_MS + 120);
    ui::update(live, ls, ps, si, t);
    CHECK(!ui::testGuide() && ui::screen() == ui::SC_BOOST && gfx::dirtyCount() > 0, "案内: 離したら消して描き直す(スワイプの所から戻して離せば替えない)");
    gfx::dirtyClear();
    // タッチが決まる前に動き出していたら(スワイプの途中)案内は出さない
    ui::touchDown(233, 233, t += 100);
    ui::touchMove(233 + HOLD_CANCEL_PX + 10, 234, t + TOUCH_CONFIRM_MS / 2);
    ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 10);
    CHECK(!ui::testGuide(), "案内: 決まる前に動き出していたら出さない(スワイプの途中)");
    ui::touchUp(t += TOUCH_CONFIRM_MS + 20);
    upd(live, t);
    CHECK(ui::screen() == ui::SC_BOOST, "案内: そのまま離してもスワイプにならない所なら替えない");
  }
  // 長押し: タッチが決まったら案内、HOLD_SHOW_MS で長押しの途中(2.4.0: 案内のまま、スワイプの札を灰にして外周)、HOLD_MS で設定へ。
  // 押したままでも画面は回らず、離してもタップにしない
  ui::touchDown(233, 233, t += 100);
  ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 10);
  ui::update(live, ls, ps, si, t + HOLD_SHOW_MS - 10);
  CHECK(ui::testGuide() && !ui::testHolding() && ui::testGuideMode() == 0, "長押し: HOLD_SHOW_MS より前は案内だけ");
  ui::touchMove(236, 231, t + HOLD_SHOW_MS);  // HOLD_CANCEL_PX 未満のずれは取り消さない
  ui::update(live, ls, ps, si, t + HOLD_SHOW_MS + 10);
  CHECK(ui::testHolding() && ui::testGuide() && ui::testGuideMode() == 1 && ui::screen() == ui::SC_BOOST, "長押し: HOLD_SHOW_MS で長押しの途中(案内のまま、スワイプの札を灰。2.4.0)");
  ui::update(live, ls, ps, si, t + HOLD_MS - 10);
  CHECK(ui::testHolding() && ui::screen() == ui::SC_BOOST, "長押し: HOLD_MS の手前ではまだタコ");
  ui::update(live, ls, ps, si, t + HOLD_MS);
  CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 0, "長押し HOLD_MS で設定の 1 項目目(明るさ)");
  ui::update(live, ls, ps, si, t + 3 * HOLD_MS);
  CHECK(ui::screen() == ui::SC_SET_LIST, "押したままでも設定のまま(画面は回らない)");
  ui::touchUp(t += 3 * HOLD_MS + 50);
  upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 0 && !ui::testHolding(), "離してもタップにしない(設定の項目はそのまま)");
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_BOOST, "設定: 上の帯「‹ 設定」で来た画面(タコ)へ");
  // 長押しの取り消し(2.4.0): 長押しの途中で HOLD_CANCEL_PX 以上動いたら、外周を消し(描き直す)、案内のまま長押しの札を灰。設定に入らない
  ui::touchDown(233, 233, t += 100);
  ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 10);
  ui::update(live, ls, ps, si, t + HOLD_SHOW_MS + 10);
  CHECK(ui::testHolding(), "取り消し: まず長押しの途中の表示が出る");
  ui::touchMove(233, 233 + HOLD_CANCEL_PX, t + HOLD_SHOW_MS + 20);
  ui::update(live, ls, ps, si, t + HOLD_SHOW_MS + 30);
  CHECK(!ui::testHolding() && ui::testGuide() && ui::testGuideMode() == 2 && ui::screen() == ui::SC_BOOST, "取り消し: HOLD_CANCEL_PX 動いたら外周を消し、案内のまま長押しの札を灰(2.4.0: 元の画面に戻さない)");
  ui::update(live, ls, ps, si, t + 2 * HOLD_MS);
  CHECK(ui::screen() == ui::SC_BOOST && !ui::testHolding() && ui::testGuide(), "取り消し: HOLD_MS を過ぎても設定に入らない(離すまで案内)");
  ui::touchUp(t += 2 * HOLD_MS + 10);
  upd(live, t);
  CHECK(ui::screen() == ui::SC_BOOST && !ui::testGuide(), "取り消し: 離しても何もしない(縦の動きはスワイプでない)");
  gfx::dirtyClear();
  // 少しずれても長押しになる(2.4.0: HOLD_CANCEL_PX 30 → 80。本人「少しだけ指がずれたときに長押しになりません」)
  ui::touchDown(233, 233, t += 100);
  ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 10);
  ui::touchMove(233 + 50, 233 + 30, t + TOUCH_CONFIRM_MS + 50);
  ui::update(live, ls, ps, si, t + HOLD_SHOW_MS + 10);
  CHECK(ui::testHolding() && ui::testGuideMode() == 1, "長押し: 58 px のずれ(2.3.1 の 30 px では取り消していた)でも長押しの途中");
  ui::update(live, ls, ps, si, t + HOLD_MS);
  CHECK(ui::screen() == ui::SC_SET_LIST, "長押し: 少しずれても HOLD_MS で設定");
  ui::touchUp(t += HOLD_MS + 50);
  upd(live, t);
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_BOOST, "設定: 上の帯「‹ 設定」でタコへ");
  // 設定(長押しで入る。前回見ていた項目から)。並び(2.4.0): 明るさ・受信の状態・本体の状態・シフトポイント・水温の警告・時間の記録・
  // 自動で戻る・デモ
  longPress(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 0, "長押しで設定へ");
  // 項目の送り: スワイプ(2.4.0。本人「設定の項目移動もスワイプにしないとオペレーションがあわない」。左へ = 次、右へ = 前)と、
  // 左右の「‹」「›」(左半分・右半分)のタップ(2.4.1: 2.4.0 でやめたのを戻した。本人「ボタンの方がわかりやすい」)
  next(t); next(t);
  CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 2, "設定: 左へスワイプで次の項目(サーキット → タコと同じ向き)");
  ui::tap(RX, MY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 3, "設定: 右の「›」(右半分)のタップで次(2.4.1)");
  ui::tap(LX, MY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 2, "設定: 左の「‹」(左半分)のタップで前");
  ui::touchDown(RX, MY, t += 100);
  ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 10);
  CHECK(ui::testSetItem() == 2, "設定: 右半分を押している間はまだ送らない(離したときに決める)");
  ui::touchUp(t += TOUCH_CONFIRM_MS + 30);
  upd(live, t);
  CHECK(ui::testSetItem() == 3, "設定: 右半分を押して離したら次");
  ui::touchDown(LX, MY, t += 100);
  ui::touchMove(LX + 30, MY, t + 50);
  ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 10);
  ui::touchUp(t += TOUCH_CONFIRM_MS + 30);
  upd(live, t);
  CHECK(ui::testSetItem() == 2, "設定: 動きが HOLD_CANCEL_PX 未満なら、触れた側(左半分)のタップ = 前");
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  longPress(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 2, "設定は前回見ていた項目から(電源を切るまで覚える)");
  prev(t); prev(t); prev(t);
  CHECK(ui::testSetItem() == 7, "右へスワイプで前へ(最初から最後のデモに回る)");
  next(t);
  CHECK(ui::testSetItem() == 0, "左へスワイプ(最後の次は最初の明るさ)");
  // 指で押す(2.4.0: 離したときにタップ)。短いタッチと、HOLD_CANCEL_PX 以上動いたタッチは何もしない
  ui::touchDown(BOTX, BOTY, t += 100);
  ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS - 10);
  ui::touchUp(t += TOUCH_CONFIRM_MS - 5);
  upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST, "設定: 短いタッチは何もしない(2.3.0)");
  ui::touchDown(BOTX, BOTY, t += 100);
  ui::touchMove(BOTX + HOLD_CANCEL_PX + 20, BOTY, t + 50);
  ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 10);
  ui::touchUp(t += TOUCH_CONFIRM_MS + 30);
  upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 0, "設定: HOLD_CANCEL_PX 以上動いて離したタッチ(スワイプ未満)は何もしない");
  // 2.4.4: 下の帯「長押しで開く」は、タップでは開かず、長押し(HOLD_MS)で開く。2.5.0: HOLD_SHOW_MS で帯が水色になり外周が回る
  // (決まっただけでは帯を変えない)。動いたら取り消し(帯の字は灰、外周は消す)
  {
    auto countIn = [&](uint16_t c, int x0, int y0, int x1, int y1) {
      int n = 0;
      for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++) n += fb[y * gfx::W + x] == c;
      return n;
    };
    const uint16_t ACC = gfx::rgb(0x5FD0FF), DIM = gfx::rgb(0x5A606A), TRACK = gfx::rgb(0x1A2433);
    CHECK(countIn(ACC, 100, 380, 366, 466) > 1500 && countIn(ACC, 100, 380, 366, 466) < 6000, "項目の画面: 下の帯は「長押しで開く」(暗い帯に水色の字)");
    ui::touchDown(BOTX, BOTY, t += 100);
    ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 10);
    CHECK(!ui::testGuide() && !ui::testHolding() && countIn(ACC, 100, 380, 366, 466) < 6000, "押して決まっただけでは帯を変えない(2.5.0)");
    ui::update(live, ls, ps, si, t + HOLD_SHOW_MS + 10);
    CHECK(ui::testHolding() && ui::testGuide() && countIn(ACC, 100, 380, 366, 466) > 10000 && countIn(TRACK, 0, 0, 466, 30) > 100 && ui::screen() == ui::SC_SET_LIST,
          "HOLD_SHOW_MS で帯が水色になり、外周が回り始める(まだ開かない。2.5.0)");
    ui::touchUp(t += HOLD_SHOW_MS + 30);
    upd(live, t);
    CHECK(ui::screen() == ui::SC_SET_LIST && !ui::testGuide() && countIn(TRACK, 0, 0, 466, 30) == 0 && countIn(ACC, 100, 380, 366, 466) < 6000,
          "HOLD_MS の前に離したら開かない(タップでは開かない)。帯と外周は元に戻る");
    ui::touchDown(BOTX, BOTY, t += 100);
    ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 10);
    ui::update(live, ls, ps, si, t + HOLD_SHOW_MS + 10);
    ui::touchMove(BOTX + HOLD_CANCEL_PX + 5, BOTY, t + HOLD_SHOW_MS + 20);
    ui::update(live, ls, ps, si, t + HOLD_SHOW_MS + 30);
    CHECK(!ui::testHolding() && ui::testGuide() && ui::testGuideMode() == 2 && countIn(DIM, 100, 380, 366, 466) > 1000 && countIn(TRACK, 0, 0, 466, 30) == 0,
          "長押しの途中で動いたら取り消し: 外周を消し、帯の字を灰に");
    ui::update(live, ls, ps, si, t + 2 * HOLD_MS);
    CHECK(ui::screen() == ui::SC_SET_LIST, "取り消したら HOLD_MS を過ぎても開かない");
    ui::touchUp(t += 2 * HOLD_MS + 10);
    upd(live, t);
    CHECK(ui::screen() == ui::SC_SET_LIST && !ui::testGuide() && countIn(DIM, 100, 380, 366, 466) == 0, "取り消して離したら帯は元に戻る(何もしない)");
    ui::tap(BOTX, BOTY, t += 100); upd(live, t);
    CHECK(ui::screen() == ui::SC_SET_LIST, "下の帯のタップでは開かない(2.4.4)");
    // 2.4.5: 長押しは画面のどこでも(本人「長押しで開くはその場所ではなく画面全体です」)。短いタップはその所のタップ。
    // 2.5.0: 矢印(左右の半分)のタップ・スワイプでは帯を変えない(本人「設定で左右の矢印をタップした時に長押しで開くボタンが反転して
    // しまうので混乱します」)
    ui::touchDown(RX, MY, t += 100);
    ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 10);
    CHECK(!ui::testGuide() && countIn(ACC, 100, 380, 366, 466) > 1500 && countIn(ACC, 100, 380, 366, 466) < 6000, "右の矢印を押して決まっても、下の帯は変えない(2.5.0)");
    ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 60);
    CHECK(!ui::testGuide() && countIn(ACC, 100, 380, 366, 466) < 6000, "右の矢印を押している間(HOLD_SHOW_MS の前)も、下の帯は変えない(2.5.0)");
    ui::touchUp(t += TOUCH_CONFIRM_MS + 80);
    upd(live, t);
    CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 1 && !ui::testGuide(), "右の矢印を押してすぐ離したのはタップ(次)");
    {
      bool band = true;
      ui::touchDown(RX, MY, t += 100);
      ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 10);
      band = band && countIn(ACC, 100, 380, 366, 466) < 6000 && countIn(DIM, 100, 380, 366, 466) == 0;
      ui::touchMove(RX - 100, MY, t + TOUCH_CONFIRM_MS + 40);
      ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 50);
      band = band && countIn(ACC, 100, 380, 366, 466) < 6000 && countIn(DIM, 100, 380, 366, 466) == 0;
      ui::touchMove(RX - 200, MY, t + TOUCH_CONFIRM_MS + 80);
      ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 90);
      band = band && countIn(ACC, 100, 380, 366, 466) < 6000 && countIn(DIM, 100, 380, 366, 466) == 0 && !ui::testGuide();
      ui::touchUp(t += TOUCH_CONFIRM_MS + 100);
      upd(live, t);
      CHECK(band && ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 2, "スワイプ(次)の途中も、下の帯は水色にも灰にもしない(2.5.0)");
    }
    prev(t);
    prev(t);
    ui::touchDown(RX, MY, t += 100);
    ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 10);
    ui::update(live, ls, ps, si, t + HOLD_SHOW_MS + 10);
    CHECK(ui::testHolding() && countIn(ACC, 100, 380, 366, 466) > 10000, "右の矢印でも、HOLD_SHOW_MS 押し続けたら帯が水色になり外周が回る(長押し)");
    ui::touchMove(RX, MY + HOLD_CANCEL_PX + 10, t + HOLD_SHOW_MS + 20);  // 縦に動かして取り消し、離す(タップにもスワイプにもならない)
    ui::update(live, ls, ps, si, t + HOLD_SHOW_MS + 30);
    ui::touchUp(t += HOLD_SHOW_MS + 40);
    upd(live, t);
    CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 0 && !ui::testGuide() && countIn(ACC, 100, 380, 366, 466) < 6000 && countIn(TRACK, 0, 0, 466, 30) == 0,
          "取り消して離したら、帯と外周は元に戻る");
    // 2.7.0: 外周が出たあと(HOLD_SHOW_MS から)HOLD_MS の前に、動かさずに離したら何もしない(本人「長押しを認識して、アニメーションが
    // 出た後に指を離した場合は何も動作しないほうがいい」。2.6.0 までは、その所のタップになって次・前・戻るに移っていた)
    ui::touchDown(RX, MY, t += 100);
    ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 10);
    ui::update(live, ls, ps, si, t + HOLD_SHOW_MS + 10);
    ui::touchUp(t += HOLD_SHOW_MS + 200);
    upd(live, t);
    CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 0 && !ui::testGuide() && countIn(ACC, 100, 380, 366, 466) < 6000 && countIn(TRACK, 0, 0, 466, 30) == 0,
          "外周が出てから HOLD_MS の前に離したら何もしない(次へ動かない。帯と外周は元に戻る。2.7.0)");
    ui::touchDown(TOPX, TOPY, t += 100);
    ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 10);
    ui::update(live, ls, ps, si, t + HOLD_SHOW_MS + 10);
    ui::touchUp(t += HOLD_MS - 100);
    upd(live, t);
    CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 0, "上の帯でも、外周が出てから離したら戻らない(2.7.0)");
    ui::touchDown(RX, MY, t += 100);
    ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 10);
    ui::touchUp(t += HOLD_SHOW_MS - 50);
    upd(live, t);
    CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 1, "外周が出る前(HOLD_SHOW_MS より前)に離したのはタップ(次)");
    prev(t);
    for (int k = 0; k < 3; k++) {  // 上の帯・真ん中・左半分の長押しでも開く
      static const int px[3] = { TOPX, MY, LX }, py[3] = { TOPY, MY, MY };
      ui::touchDown(px[k], py[k], t += 100);
      ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 10);
      ui::update(live, ls, ps, si, t + HOLD_SHOW_MS + 10);
      ui::update(live, ls, ps, si, t + HOLD_MS);
      CHECK(ui::screen() == ui::SC_SET_EDIT && ui::testSetItem() == 0, k == 0 ? "上の帯の長押しで開く(2.4.5)" : k == 1 ? "真ん中の長押しで開く(2.4.5)" : "左半分の長押しで開く(2.4.5)");
      ui::touchUp(t += HOLD_MS + 20);
      upd(live, t);
      CHECK(ui::screen() == ui::SC_SET_EDIT, "開いたあとに離しても、タップにはならない");
      ui::tap(TOPX, TOPY, t += 100); upd(live, t);
    }
    CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 0, "変更の画面の上「‹ 戻る」で明るさの項目へ");
  }
  ui::touchDown(BOTX, BOTY, t += 100);
  ui::update(live, ls, ps, si, t + TOUCH_CONFIRM_MS + 10);
  ui::update(live, ls, ps, si, t + HOLD_MS);
  CHECK(ui::screen() == ui::SC_SET_EDIT && ui::testSetItem() == 0, "設定: 下の帯を HOLD_MS 押し続けても開く(明るさの変更へ)");
  ui::update(live, ls, ps, si, t + 3 * HOLD_MS);
  ui::touchUp(t += 3 * HOLD_MS + 10);
  upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_EDIT, "開いたあとに離しても、変更の画面の「決定」にはならない");
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 0, "変更: 上「‹ 戻る」で明るさの項目へ");
  // 受信の状態(2 項目目。2.4.0): CAN・GPS・IMU・RaceChrono の 4 グループ(まとめ。下「開く」で中へ)。中は 1 ページ 1 項目
  next(t);
  holdOpen(live, t);
  CHECK(ui::screen() == ui::SC_SET_INFO && ui::testPage() == 0, "2 項目目の下「長押しで開く」で受信の状態(1 ページ目 = CAN)");
  next(t);
  CHECK(ui::screen() == ui::SC_SET_INFO && ui::testPage() == 1, "受信の状態: 左へスワイプで次のグループ(GPS)");
  ui::tap(RX, MY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_INFO && ui::testPage() == 2, "受信の状態: 右の「›」のタップで次(IMU)");
  ui::tap(LX, MY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_INFO && ui::testPage() == 1, "受信の状態: 左の「‹」のタップで前(GPS)");
  prev(t); prev(t);
  CHECK(ui::screen() == ui::SC_SET_INFO && ui::testPage() == 3, "受信の状態: 最初の前は最後の RaceChrono");
  holdOpen(live, t);
  CHECK(ui::screen() == ui::SC_SET_GRP && ui::testGroup() == 3 && ui::testPage() == 0, "RaceChrono の下「長押しで開く」で中(1 ページ目 = 送信)");
  next(t); next(t); next(t);
  CHECK(ui::screen() == ui::SC_SET_GRP && ui::testPage() == 0, "RaceChrono の中: 3 ページで回る(送信・接続・送った数)");
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_INFO && ui::testPage() == 3, "RaceChrono の中: 上「‹ RaceChrono」で受信の状態の RaceChrono のページ");
  next(t);
  ui::tap(BOTX, BOTY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_INFO, "受信の状態: 下の帯のタップでは開かない(2.4.4)");
  holdOpen(live, t);
  CHECK(ui::screen() == ui::SC_SET_GRP && ui::testGroup() == 0 && ui::testPage() == 0, "CAN の下「長押しで開く」で中(1 ページ目 = 通信速度)");
  prev(t);
  CHECK(ui::testPage() == 7, "CAN の中: 最初の前は最後の「前後で比べる」(2.6.0)");
  prev(t);
  CHECK(ui::testPage() == 6, "CAN の中: その前が CAN確認(2.4.3)");
  // CAN確認(2.4.0: CAN の「C-CAN・B-CAN」の下「CAN確認」。CAN のメニューは 1 つ)
  {
    uint8_t d[8] = { 0, 0x19, 0x10, 0x68, 7, 0x18, 0x46, 0x77 };
    cantab::onFrame(0, 1, 0x0618A001, 8, d, t);
    ui::tap(BOTX, BOTY, t += 100); upd(live, t);
    CHECK(ui::screen() == ui::SC_SET_GRP, "CAN確認のページ: 下の帯のタップでは開かない(2.4.4)");
    holdOpen(live, t);
    CHECK(ui::screen() == ui::SC_CAN, "CAN確認: CAN確認のページの下「長押しで開く」で CAN の画面(2.4.4)");
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
    CHECK(ui::screen() == ui::SC_CAN_ID, "一覧: 下の帯のタップでは開かない(2.4.4)");
    holdOpen(live, t);
    CHECK(ui::screen() == ui::SC_CAN_DETAIL, "一覧: 下「長押しで開く」で詳細(2.4.4)");
    next(t);
    upd(live, t += 4 * 60000);
    CHECK(ui::screen() == ui::SC_CAN_DETAIL, "詳細: スワイプはバイトを選ぶだけ。4 分たっても自動では戻らない");
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
    CHECK(ui::screen() == ui::SC_SET_GRP && ui::testGroup() == 0 && ui::testPage() == 6, "CAN の画面: 上「‹ 戻る」で CAN の「CAN確認」のページ(2.4.3)");
  }
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_INFO && ui::testPage() == 0, "CAN の中: 上「‹ CAN」で受信の状態の CAN のページ");
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 1, "受信の状態: 上「‹ 受信の状態」で設定の受信の状態");
  // 本体の状態(3 項目目。2.4.0: 受信と関係ない FPS・起動を受信の状態から出した): 下「開く」で中。上の帯で設定へ
  next(t);
  holdOpen(live, t);
  CHECK(ui::screen() == ui::SC_SET_GRP && ui::testGroup() == 4 && ui::testPage() == 0, "3 項目目 = 本体の状態。下「長押しで開く」で中(1 ページ目 = FPS)");
  next(t);
  CHECK(ui::testPage() == 1, "本体の状態: 2 ページ目 = CPU(2.4.3)");
  prev(t); prev(t);
  CHECK(ui::testPage() == 7, "本体の状態: 最初の前は最後の 起動(8 ページ。2.4.3)");
  next(t);
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 2, "本体の状態: 上「‹ 本体の状態」で設定の本体の状態");

  // ---------------- 7. 設定の変更(下「変更」→ 左右の半分で − +、下「決定」で保存して戻り、値が緑で点滅する(2.3.1)。上「‹ 戻る」で変えずに戻る)
  //                  2.4.0: 変更の画面は値を枠で囲み、下の「決定」を水色の帯にした(項目の画面と見た目が同じで、移ったのが分からなかった)
  next(t);
  CHECK(ui::testSetItem() == 3, "4 項目目 = シフトポイント");
  auto countIn = [&](uint16_t c, int x0, int y0, int x1, int y1) {
    int n = 0;
    for (int y = y0; y < y1; y++)
      for (int x = x0; x < x1; x++) n += fb[y * gfx::W + x] == c;
    return n;
  };
  const uint16_t ACCENT = gfx::rgb(0x5FD0FF);
  CHECK(countIn(ACCENT, 80, 196, 386, 304) == 0 && countIn(ACCENT, 100, 380, 366, 466) < 6000, "項目の画面: 値は水色でなく、下の帯も水色でない");
  holdOpen(live, t);
  CHECK(ui::screen() == ui::SC_SET_EDIT, "設定: 下「長押しで開く」で変更の画面へ");
  CHECK(countIn(ACCENT, 80, 196, 386, 304) > 1000 && countIn(ACCENT, 100, 380, 366, 466) > 10000, "変更の画面: 値は水色、下の「決定」は水色の帯(2.4.1: 2.4.0 の枠はやめた)");
  ui::tap(RX, MY, t += 100); upd(live, t);
  CHECK(set.shiftRpm == SET_SHIFT_DEFAULT + 100, "右半分で 100 上がる");
  CHECK(countIn(ACCENT, 80, 196, 386, 304) > 1000, "値を変えても水色のまま");
  {
    static uint16_t fbE[gfx::W * gfx::H];
    memcpy(fbE, fb, sizeof(fbE));
    ui::testRedraw();
    upd(live, t);
    CHECK(memcmp(fbE, fb, sizeof(fbE)) == 0, "変更の画面: 値を変えて描き直した絵 = 最初から描いた絵");
  }
  ui::tap(LX, MY, t += 100); ui::tap(LX, MY, t += 100); upd(live, t);
  CHECK(set.shiftRpm == SET_SHIFT_DEFAULT - 100, "左半分 2 回で 200 下がる");
  int saves0 = g_hostSaves;
  {
    // 値の所(名前の下〜下の点の上)の画素を数える
    auto areaCount = [&](uint16_t c) {
      int n = 0;
      for (int y = 196; y < 304; y++)
        for (int x = 80; x < 386; x++) n += fb[y * gfx::W + x] == c;
      return n;
    };
    auto areaNonBg = [&]() {
      int n = 0;
      for (int y = 196; y < 304; y++)
        for (int x = 80; x < 386; x++) n += fb[y * gfx::W + x] != 0;
      return n;
    };
    const uint16_t GREEN = gfx::rgb(0x22E36A), WHITE = gfx::rgb(0xFFFFFF);
    const int accent0 = countIn(ACCENT, 80, 196, 386, 304);  // ふだんの太さの値
    (void)areaNonBg;
    const uint32_t t0 = t += 100;
    ui::tap(BOTX, BOTY, t0);
    ui::update(live, ls, ps, si, t0);
    // 2.4.3: 押した所(下の「決定」の帯)が点滅し、その間は値が太い。変更の画面のまま。点滅が終わってから戻る
    CHECK(ui::screen() == ui::SC_SET_EDIT && g_hostSaves == saves0 + 1 && ui::testFlashing(), "下「決定」で保存し、押した所が点滅する(その間は変更の画面のまま。2.4.3)");
    CHECK(countIn(ACCENT, 100, 380, 366, 466) < 3000, "点滅: 最初は帯を消す(押した所がすぐ変わる)");
    CHECK(countIn(ACCENT, 80, 196, 386, 304) > accent0 + 200, "点滅の間は値が太い");
    ui::update(live, ls, ps, si, t0 + SAVED_FLASH_MS + 10);
    CHECK(countIn(ACCENT, 100, 380, 366, 466) > 10000, "点滅: 次は水色の帯");
    ui::tap(RX, MY, t0 + SAVED_FLASH_MS + 20);
    CHECK(set.shiftRpm == SET_SHIFT_DEFAULT - 100 && g_hostSaves == saves0 + 1, "点滅の間は − + を受けない");
    ui::update(live, ls, ps, si, t0 + 2 * SAVED_FLASH_MS + 10);
    CHECK(countIn(ACCENT, 100, 380, 366, 466) < 3000 && ui::screen() == ui::SC_SET_EDIT, "点滅: また消す");
    {
      static uint16_t fbS[gfx::W * gfx::H];
      memcpy(fbS, fb, sizeof(fbS));
      ui::testRedraw();
      ui::update(live, ls, ps, si, t0 + 2 * SAVED_FLASH_MS + 10);
      CHECK(memcmp(fbS, fb, sizeof(fbS)) == 0, "点滅: 少しずつ描いた絵 = 同じ時刻に最初から描いた絵");
    }
    ui::update(live, ls, ps, si, t0 + 2 * SAVED_FLASH_COUNT * SAVED_FLASH_MS - 10);
    CHECK(ui::screen() == ui::SC_SET_EDIT, "点滅の終わる手前では、まだ変更の画面");
    ui::update(live, ls, ps, si, t0 + 2 * SAVED_FLASH_COUNT * SAVED_FLASH_MS + 10);
    ui::update(live, ls, ps, si, t0 + 2 * SAVED_FLASH_COUNT * SAVED_FLASH_MS + 70);
    ui::update(live, ls, ps, si, t0 + 2 * SAVED_FLASH_COUNT * SAVED_FLASH_MS + 130);
    CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 3 && !ui::testFlashing(), "点滅が終わったら項目の画面へ");
    CHECK(areaCount(GREEN) == 0 && areaCount(WHITE) > 100 && countIn(ACCENT, 80, 196, 386, 304) == 0 && countIn(ACCENT, 100, 380, 366, 466) < 6000,
          "戻った画面: 値はふだんの白(緑の点滅はやめた)、水色の帯も無い");
    t = t0 + 2 * SAVED_FLASH_COUNT * SAVED_FLASH_MS + 100;
    upd(live, t);
    CHECK(g_hostSaves == saves0 + 1, "点滅のあとも、もう一度は保存しない");
  }
  holdOpen(live, t);
  ui::tap(BOTX, BOTY, t += 100);
  ui::update(live, ls, ps, si, t);
  CHECK(ui::screen() == ui::SC_SET_EDIT && ui::testFlashing() && g_hostSaves == saves0 + 1, "変えずに決定しても、押した帯が点滅する(保存はしない。2.4.5)");
  upd(live, t += 2 * SAVED_FLASH_COUNT * SAVED_FLASH_MS + 10);
  CHECK(ui::screen() == ui::SC_SET_LIST && !ui::testFlashing() && g_hostSaves == saves0 + 1, "点滅が終わったら項目の画面へ(保存はしない)");
  holdOpen(live, t);
  ui::tap(RX, MY, t += 100); ui::tap(RX, MY, t += 100);
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && set.shiftRpm == SET_SHIFT_DEFAULT - 100 && g_hostSaves == saves0 + 1, "上「‹ 戻る」は変えずに戻る(保存しない)");
  // もう一度変えて決定: 点滅が終わると項目の画面。送るのは戻ってから
  holdOpen(live, t);
  ui::tap(RX, MY, t += 100);
  ui::tap(BOTX, BOTY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_EDIT && g_hostSaves == saves0 + 2 && ui::testFlashing(), "もう一度変えて決定(点滅の間は変更の画面)");
  upd(live, t += 2 * SAVED_FLASH_COUNT * SAVED_FLASH_MS);
  CHECK(ui::screen() == ui::SC_SET_LIST && !ui::testFlashing() && set.shiftRpm == SET_SHIFT_DEFAULT, "点滅が終わって項目の画面");
  next(t);
  CHECK(ui::testSetItem() == 4, "左へスワイプで次へ");
  prev(t);
  CHECK(ui::testSetItem() == 3 && !ui::testFlashing(), "戻る");
  // 上下限
  holdOpen(live, t);
  for (int i = 0; i < 60; i++) ui::tap(RX, MY, t += 10);
  CHECK(set.shiftRpm == SET_SHIFT_MAX, "上限で止まる");
  for (int i = 0; i < 80; i++) ui::tap(LX, MY, t += 10);
  CHECK(set.shiftRpm == SET_SHIFT_MIN, "下限で止まる");
  // 保存に失敗: 1 回目の「決定」では留まり、2 回目で抜ける(保存していないので点滅しない)
  g_hostSaveOk = false;
  upd(live, t);
  ui::tap(BOTX, BOTY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_EDIT, "保存に失敗したら変更画面に留まる");
  ui::tap(BOTX, BOTY, t += 100);
  ui::update(live, ls, ps, si, t);
  CHECK(ui::screen() == ui::SC_SET_EDIT && ui::testFlashing(), "もう一度「決定」で、押した帯が点滅してから抜ける(保存はしない。2.4.5)");
  upd(live, t += 2 * SAVED_FLASH_COUNT * SAVED_FLASH_MS + 10);
  CHECK(ui::screen() == ui::SC_SET_LIST && !ui::testFlashing(), "点滅が終わったら項目の画面へ");
  g_hostSaveOk = true;
  set.shiftRpm = SET_SHIFT_DEFAULT;
  // 明るさ(1 項目目): 変えたその場で反映、「‹ 戻る」で元に戻る
  prev(t); prev(t); prev(t);
  holdOpen(live, t);
  CHECK(ui::screen() == ui::SC_SET_EDIT && ui::testSetItem() == 0, "1 項目目(明るさ)の変更");
  ui::tap(LX, MY, t += 100); upd(live, t);
  CHECK(set.brightnessPct == SET_BRIGHT_DEFAULT - SET_BRIGHT_STEP && g_hostBrightness == set.brightnessPct, "明るさは左半分でその場で変わる");
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  CHECK(set.brightnessPct == SET_BRIGHT_DEFAULT && g_hostBrightness == SET_BRIGHT_DEFAULT, "上「‹ 戻る」で明るさも元に戻る");
  // 時間の記録(6 項目目)
  for (int i = 0; i < 5; i++) next(t);
  CHECK(ui::testSetItem() == 5, "6 項目目 = 時間の記録");
  trip::setTimes(5000, 7000);
  holdOpen(live, t);
  CHECK(ui::screen() == ui::SC_SET_RESET, "下「長押しで開く」で確認の画面(2.4.4)");
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && trip::aboveMs() == 5000, "確認: 上「‹ 戻る」は消さない");
  int ts0 = g_hostTimeSaves;
  holdOpen(live, t);
  ui::tap(BOTX, BOTY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_SET_LIST && trip::aboveMs() == 0 && trip::belowMs() == 0 && g_hostTimeSaves == ts0 + 1, "確認: 下「リセット」で 0 に戻して、すぐ保存");
  ui::tap(TOPX, TOPY, t += 100); upd(live, t);
  CHECK(ui::screen() == ui::SC_BOOST, "設定: 上の帯「‹ 設定」で来た画面へ");

  // ---------------- 8. 覚える画面と、設定から時間で戻る(2.3)
  CHECK(ui::homeScreen() == ui::SC_DRIVE, "まだ 1 分続けて見ていないので、覚えた画面はサーキットのまま");
  upd(live, t += HOME_CONFIRM_MS + 100);
  CHECK(ui::homeScreen() == ui::SC_BOOST, "タコを 1 分見ていたら覚える");
  upd(live, t += 4 * 60000);
  CHECK(ui::screen() == ui::SC_BOOST, "タコ: 4 分たっても自動では変わらない");
  swipe(live, t, -200, 0);
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
  ui::testShow(ui::SC_SET_LIST, 3);
  holdOpen(live, t);
  CHECK(ui::screen() == ui::SC_SET_EDIT, "設定の変更画面へ(シフトポイント)");
  ui::tap(RX, MY, t += 100); upd(live, t);
  saves0 = g_hostSaves;
  upd(live, t + 61000);
  CHECK(ui::screen() == ui::SC_BOOST && g_hostSaves == saves0 + 1, "変更画面からも 1 分(設定値)で戻り、変えた値は保存する");
  set.autoReturnMin = SET_AUTORET_DEFAULT;
  set.shiftRpm = SET_SHIFT_DEFAULT;
  // グループの中(2.4.0)からも時間で戻る
  set.autoReturnMin = 1;
  ui::testShow(ui::SC_SET_GRP, 1, 0);
  ui::testTouch(t += 100);
  upd(live, t);
  upd(live, t + 61000);
  CHECK(ui::screen() == ui::SC_BOOST, "グループの中からも 1 分(設定値)で覚えた画面に戻る");
  t += 62000;
  set.autoReturnMin = SET_AUTORET_DEFAULT;
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
    // (2.4.0: ピークが PEAK_MIN_RPM 以上なら印は針の下に出したままなので、基準の絵は PEAK_MIN_RPM 未満の 2000 rpm で取る)
    ui::testShow(ui::SC_BOOST);
    b.boostOk = true; b.boostBar = 1.2f;
    static uint16_t fbS[gfx::W * gfx::H];
    b.rpm = 2000;
    // 直前の 6000 rpm のピークホールドを消してから、基準の絵を取る
    ui::testPeakReset();
    ui::update(b, ls, ps, si, ts + 2000);
    const uint32_t t0 = ts + 2100 + TACH_PEAK_HOLD_MS + 4000u * 1000u / TACH_PEAK_FALL_RPM_PER_S;
    ui::update(b, ls, ps, si, t0);
    memcpy(fbS, fb, sizeof(fbS));
    gfx::dirtyClear();
    b.rpm = 6000;
    ui::update(b, ls, ps, si, t0 + 10);
    CHECK(memcmp(fbS, fb, sizeof(fbS)) != 0 && ui::blinkLevel() == BL_MAX, "アナログ計: 回転数で針が動く。6000 で最大輝度");
    b.rpm = 2000;
    b.boostBar = 1.2f;
    ui::update(b, ls, ps, si, t0 + 20);
    CHECK(memcmp(fbS, fb, sizeof(fbS)) != 0 && ui::testPeakDrawn(), "アナログ計: 針を戻した直後はピークホールドの印が 6000 に残る(最初の絵と違う)");
    // 2.4.4: 印は TACH_PEAK_HOLD_MS(1.5 秒)止まってから TACH_PEAK_FALL_RPM_PER_S で今の回転数まで下がり、ピークを付けてから
    // TACH_PEAK_SHOW_MS(5 秒)で消える。消えたら最初の絵に戻る
    ui::update(b, ls, ps, si, t0 + 10 + TACH_PEAK_HOLD_MS - 100);
    CHECK(ui::testPeakRpm() == 6000.0f && ui::testPeakDrawn(), "アナログ計: TACH_PEAK_HOLD_MS(1.5 秒)の間は 6000 で止まる(2.4.4)");
    ui::update(b, ls, ps, si, t0 + 10 + TACH_PEAK_HOLD_MS + 1000);
    CHECK(fabsf(ui::testPeakRpm() - (6000.0f - TACH_PEAK_FALL_RPM_PER_S)) < 30.0f && ui::testPeakDrawn(), "アナログ計: そのあと 2000 rpm/秒で下がる(1 秒で 4000)");
    // 2.4.5: 印が PEAK_MIN_RPM(3000)を切ったら消す(6000 → 3000 は止まる 1.5 秒 + 下がる 1.5 秒)
    const uint32_t tFall = t0 + 10 + TACH_PEAK_HOLD_MS + 3000u * 1000u / TACH_PEAK_FALL_RPM_PER_S;
    ui::update(b, ls, ps, si, tFall - 100);
    CHECK(ui::testPeakDrawn() && ui::testPeakRpm() > 3000.0f, "アナログ計: 3000 を切る手前では印がまだ出ている");
    ui::update(b, ls, ps, si, tFall + 100);
    CHECK(memcmp(fbS, fb, sizeof(fbS)) == 0 && !ui::testPeakDrawn(), "アナログ計: 印が 3000 を切ると消え、最初の絵と 1 画素も違わない(針・帯・印の跡が残らない。2.4.5)");
    // 2.4.0: 回転数がピークを超えても印は出したまま(針の下で一緒に動く)。下がるとその新しいピークに印
    b.rpm = 6000;
    ui::update(b, ls, ps, si, tFall + 110);
    b.rpm = 4000;
    ui::update(b, ls, ps, si, tFall + 120);
    CHECK(ui::testPeakDrawn(), "アナログ計: 6000 から下がると印を出す");
    b.rpm = 6200;
    ui::update(b, ls, ps, si, tFall + 130);
    CHECK(ui::testPeakDrawn(), "アナログ計: 回転数がピーク(6000)を超えても印は出したまま(針の下。2.4.0)");
    b.rpm = 2000;
    ui::update(b, ls, ps, si, tFall + 140);
    CHECK(ui::testPeakDrawn(), "アナログ計: 下がると新しいピーク(6200)に印を出す");
    ui::update(b, ls, ps, si, tFall + 130 + TACH_PEAK_HOLD_MS + 3200u * 1000u / TACH_PEAK_FALL_RPM_PER_S + 100);
    CHECK(memcmp(fbS, fb, sizeof(fbS)) == 0, "アナログ計: 新しいピーク(6200)も 3000 を切ると消え、最初の絵に戻る");
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
    // ブーストの輪の角度(2.8.0 の決まり): 0 bar = 左下 120°、1.4 bar = 真上 270°、2.0 bar = 端 320°。その間はそれぞれ比例
    auto boostDeg = [](float bar) { return bar <= 1.4f ? 120.0f + 150.0f * bar / 1.4f : 270.0f + 50.0f * (bar - 1.4f) / 0.6f; };
    auto ringPx = [&](float deg) {  // 輪の太さの真ん中の画素
      const float r = (BOOST_RING_R_IN + BOOST_RING_R_OUT) * 0.5f, a = deg * 3.14159265f / 180.0f;
      return px((int)(233 + r * cosf(a)), (int)(233 + r * sinf(a)));
    };
    CHECK(ringPx(boostDeg(0.3f)) == gfx::rgb(0xA9AFBA), "ブーストの輪: 0.65 bar のとき 0.3 bar の所はトーンを落とした白");
    CHECK(ringPx(boostDeg(1.2f)) == gfx::rgb(0x161E2A), "ブーストの輪: 1.2 bar の所は地の色");
    b.boostBar = 1.4f;  // 2.8.0: 1.4 bar で真上(0 時)
    ui::update(b, ls, ps, si, tc);
    {
      const float rm = (BOOST_MARK_R_IN + BOOST_MARK_R_OUT) * 0.5f;
      CHECK(px(233, (int)(233 - rm)) == gfx::rgb(0xFFFFFF), "ブーストの輪: 1.4 bar のとき先端の白い目印が真上(0 時。2.8.0)");
    }
    CHECK(ringPx(265.0f) == gfx::rgb(0xA9AFBA) && ringPx(275.0f) == gfx::rgb(0x2E1A1C), "ブーストの輪: 1.4 bar のとき真上の手前まで白、真上から先は赤の範囲の地");
    b.boostBar = 0.65f;
    ui::update(b, ls, ps, si, tc);
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
    b.boostBar = 1.8f;  // 赤の範囲(2.8.0: 1.4 bar = 真上から)
    b.rpm = 5000;
    ui::update(b, ls, ps, si, tc + 3 * ALARM_BLINK_HALF_MS);  // 消灯側
    CHECK(countIn(bx0, by0, bx1, by1, gfx::rgb(0x8E130D)) > 4000, "札: 2 つとも出て、点滅の消灯側は暗い赤");
    // 2.8.0: 1.8 bar は 270 + 50 × 0.4 / 0.6 = 303.3°
    CHECK(ringPx(260.0f) == gfx::rgb(0xA9AFBA) && ringPx(290.0f) == gfx::rgb(0x9E3A33), "ブーストの輪: 1.8 bar のとき真上の手前は白、真上から先は赤(2.8.0)");
    CHECK(ringPx(312.0f) == gfx::rgb(0x2E1A1C), "ブーストの輪: 1.8 bar の先(312°)は赤の範囲の地(端は 320°)");
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

  // ---------------- 13. 設定の「デモ」(2.4.0: 最後の 8 項目目)
  {
    Signals d;
    memset(&d, 0, sizeof(d));
    uint32_t td = 50000000;
    set.demoOn = 0;
    ui::testShow(ui::SC_SET_LIST, 7);
    holdOpen(d, td);  // 触れた時刻を操作した時刻にする(先に update すると、前の操作から時間がたっていて自動で戻る)
    CHECK(ui::screen() == ui::SC_SET_EDIT, "設定 8 項目目で「デモ」の変更画面へ");
    ui::tap(366, 233, td += 100);
    CHECK(set.demoOn == 1, "右半分(入)でデモが入");
    ui::tap(366, 233, td += 100);
    CHECK(set.demoOn == 1, "もう一度押しても入のまま");
    int sv = g_hostSaves;
    ui::tap(233, 400, td += 100);
    ui::update(d, ls, ps, si, td);
    CHECK(ui::screen() == ui::SC_SET_EDIT && g_hostSaves == sv + 1 && ui::testFlashing(), "「決定」で保存し、下の帯が点滅(2.4.3)");
    ui::update(d, ls, ps, si, td + 2 * SAVED_FLASH_COUNT * SAVED_FLASH_MS + 10);
    CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 7 && !ui::testFlashing(), "点滅が終わったらデモの項目へ");
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

  // ---------------- 15. 受信の状態の RaceChrono・IMU(2.4.0: グループ)。下の点の判定、向きの補正、送信の入・切、上の帯で戻る
  {
    Signals d;
    memset(&d, 0, sizeof(d));
    uint32_t tr = 60000000;
    const uint16_t GREEN = gfx::rgb(0x22E36A), RED = gfx::rgb(0xE3241B);
    // 下の点(n 個、y 330、間隔 26)の i 個目の色(今のページの点も、真ん中はその判定の色)
    auto dotCol = [&](int n, int i) { return fb[330 * gfx::W + (int)(233 - 26 * (n - 1) * 0.5f + 26 * i)]; };
    ui::testShow(ui::SC_SET_INFO, 3);
    holdOpen(d, tr);
    CHECK(ui::screen() == ui::SC_SET_GRP && ui::testGroup() == 3 && ui::testPage() == 0, "受信の状態 4 ページ目(RaceChrono)の「長押しで開く」で中へ");
    si.rc.enabled = true; si.rc.bleOk = true; si.rc.connected = true; si.rc.gpsOk = true; si.rc.fixType = 3; si.rc.numSV = 9; si.rc.lat = 355812345; si.rc.lon = 1396821234;
    si.rc.imuOk = true; si.rc.imuLevel = true; si.rc.imuForward = true;
    ui::update(d, ls, ps, si, tr += 600);
    ui::update(d, ls, ps, si, tr += 600);
    CHECK(ui::screen() == ui::SC_SET_GRP, "値が変わっても画面はそのまま");
    // RaceChrono の点(3 個): 1 個目は送信の入・切(判定なし)、2 個目 = 接続、3 個目 = 送った数
    CHECK(dotCol(3, 1) == GREEN && dotCol(3, 2) == GREEN, "RaceChrono: 接続中なら 接続・送った数 の点が緑");
    si.rc.bleOk = false;  // BLE を開始できない: 2 個目(接続)が赤
    ui::update(d, ls, ps, si, tr += 600);
    CHECK(dotCol(3, 1) == RED, "RaceChrono: BLE を開始できないと 2 個目(接続)の点が赤");
    si.rc.bleOk = true;
    // IMU の点(2 個): 加速度・向きの補正。IMU が応答しないと 1 個目が赤
    ui::testShow(ui::SC_SET_GRP, 0, 2);
    ui::testTouch(tr);
    ui::update(d, ls, ps, si, tr += 600);
    CHECK(dotCol(2, 0) == GREEN && dotCol(2, 1) == GREEN, "IMU: 正常なら 2 個とも緑");
    si.rc.imuOk = false;
    ui::update(d, ls, ps, si, tr += 600);
    CHECK(dotCol(2, 0) == RED, "IMU: 応答しないと 1 個目(加速度)の点が赤");
    si.rc.imuOk = true;
    ui::update(d, ls, ps, si, tr + 4 * 60000);
    CHECK(ui::screen() == ui::SC_DRIVE, "グループの中: 自動で覚えた画面(サーキット)に戻る");
    ui::testShow(ui::SC_SET_GRP, 0, 2);
    ui::update(d, ls, ps, si, tr += 100);
    ui::tap(233, 60, tr += 100);
    ui::update(d, ls, ps, si, tr);
    CHECK(ui::screen() == ui::SC_SET_INFO && ui::testPage() == 2, "IMU の中: 上の「‹ IMU」で受信の状態の IMU のページへ");
    ui::testShow(ui::SC_SET_GRP, 0, 2);
    ui::update(d, ls, ps, si, tr += 100);
    int resets0 = g_hostImuResets;
    holdOpen(d, tr);  // 「加速度」のページには下の操作が無い(何も起きない)
    CHECK(ui::screen() == ui::SC_SET_GRP && ui::testPage() == 0, "「加速度」のページは下の帯の長押しでも何もしない");
    ui::swipe(1, tr += 100);       // 次 → 「向きの補正」
    ui::update(d, ls, ps, si, tr += 600);  // 描くと下に「長押しで開く」が出る
    holdOpen(d, tr);  // 確認の画面(2.4.3。前は押すとすぐ捨てていた)
    CHECK(ui::screen() == ui::SC_SET_RESET && g_hostImuResets == resets0, "IMU の「向きの補正」の下「長押しで開く」で確認の画面(まだ捨てない)");
    ui::tap(233, 60, tr += 100);   // 上「‹ 戻る」= 捨てずに戻る
    ui::update(d, ls, ps, si, tr);
    CHECK(ui::screen() == ui::SC_SET_GRP && ui::testGroup() == 2 && ui::testPage() == 1 && g_hostImuResets == resets0, "確認: 上「‹ 戻る」は捨てずに向きの補正のページへ");
    ui::update(d, ls, ps, si, tr += 600);
    holdOpen(d, tr);
    ui::tap(233, 400, tr += 100);  // 赤い「取り直す」(タップ)
    ui::update(d, ls, ps, si, tr);
    CHECK(ui::screen() == ui::SC_SET_GRP && ui::testGroup() == 2 && ui::testPage() == 1 && g_hostImuResets == resets0 + 1, "確認: 下「取り直す」で覚えた向きを捨て(1 回だけ)、向きの補正のページへ");
    // RaceChrono の 1 ページ目「送信」(入・切)
    set.rcOn = 1;
    ui::testShow(ui::SC_SET_GRP, 0, 3);
    ui::update(d, ls, ps, si, tr += 100);  // 描くと下に「長押しで開く」が出る
    holdOpen(d, tr);
    CHECK(ui::screen() == ui::SC_SET_EDIT, "RaceChrono の 1 ページ目「送信」の「長押しで開く」で変更画面へ");
    ui::tap(100, 233, tr += 100);
    CHECK(set.rcOn == 0, "左半分(切)で切");
    ui::tap(100, 233, tr += 100);
    CHECK(set.rcOn == 0, "もう一度押しても切のまま");
    int sv = g_hostSaves;
    ui::tap(233, 400, tr += 100);
    ui::update(d, ls, ps, si, tr);
    CHECK(ui::screen() == ui::SC_SET_EDIT && set.rcOn == 0 && g_hostSaves == sv + 1 && ui::testFlashing(), "「決定」で保存し、下の帯が点滅(2.4.3)");
    ui::update(d, ls, ps, si, tr + 2 * SAVED_FLASH_COUNT * SAVED_FLASH_MS + 10);
    CHECK(ui::screen() == ui::SC_SET_GRP && ui::testGroup() == 3 && ui::testPage() == 0 && !ui::testFlashing(), "点滅が終わったら RaceChrono の 1 ページ目へ");
    tr += 1900;
    holdOpen(d, tr);
    ui::tap(233, 60, tr += 100);  // 変えずに「‹ 戻る」
    ui::update(d, ls, ps, si, tr);
    CHECK(ui::screen() == ui::SC_SET_GRP && ui::testGroup() == 3 && g_hostSaves == sv + 1, "変更画面の「‹ 戻る」で RaceChrono の 1 ページ目へ(保存しない)");
    ui::tap(233, 60, tr += 100);
    ui::update(d, ls, ps, si, tr);
    CHECK(ui::screen() == ui::SC_SET_INFO && ui::testPage() == 3, "RaceChrono の中: 上の「‹ RaceChrono」で受信の状態の RaceChrono のページへ");
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

  // ---------------- 20. オープニング(2.5.0: 覚えた画面ごとに 3 種類。サーキット・タコ・データ)。時間で進み、終わったら覚えた画面へ。
  //                  タップで飛ばせる。明るさの段階は戻る。データだけ自己診断を出し、NG なら延びる
  {
    Signals o;
    memset(&o, 0, sizeof(o));
    LinkStats lo;
    memset(&lo, 0, sizeof(lo));
    SysInfo so;
    memset(&so, 0, sizeof(so));
    so.touchOk = true; so.nvsOk = true; so.rc.enabled = true; so.rc.bleOk = true; so.rc.imuOk = true;
    const uint16_t WHITE = gfx::rgb(0xFFFFFF), GREEN = gfx::rgb(0x22E36A), RED = gfx::rgb(0xE3241B);
    static uint16_t fbO[gfx::W * gfx::H];
    auto countIn = [&](uint16_t c, int x0, int y0, int x1, int y1) {
      int n = 0;
      for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++) n += fb[y * gfx::W + x] == c;
      return n;
    };
    auto nonBlack = [&](int x0, int y0, int x1, int y1) {
      int n = 0;
      for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++) n += fb[y * gfx::W + x] != 0;
      return n;
    };
    // ---- サーキット: 白い光が走る → 真上で止まる → 点いていく(最大輝度まで) → 背景も赤 → 消えていく → ギアが浮かぶ → サーキット
    {
      uint32_t to = 90000000;
      ui::testShow(ui::SC_OPENING, 0);
      ui::update(o, lo, ps, so, to);
      CHECK(ui::screen() == ui::SC_OPENING && gfx::dirtyCount() > 0 && nonBlack(0, 0, gfx::W, 400) == 0 && countIn(WHITE, 180, 400, 286, gfx::H) > 100,
            "オープニング(サーキット): 最初の update で黒く塗り、真下のマスから光が走り始める");
      gfx::dirtyClear();
      ui::update(o, lo, ps, so, to + OPC_SCAN_MS / 2);
      const int w1 = countIn(WHITE, 0, 0, gfx::W, gfx::H);
      CHECK(w1 > 400 && countIn(WHITE, 0, 0, 233, gfx::H) > 200 && countIn(WHITE, 233, 0, gfx::W, gfx::H) > 200, "サーキット: 白い光が左右とも下から上へ走る");
      ui::update(o, lo, ps, so, to + OPC_SCAN_MS + OPC_FLASH_MS / 2);
      CHECK(fb[31 * gfx::W + 233] == WHITE && ui::blinkLevel() == BL_NORMAL, "サーキット: 光は真上で止まる(まだ設定の明るさ)");
      bool sawMax = false, sawBlinkOff = false;
      uint32_t tMax = 0;
      for (uint32_t dt = OPC_SCAN_MS + OPC_FLASH_MS; dt < OPC_SCAN_MS + OPC_FLASH_MS + OPC_RISE_MS; dt += 5) {
        ui::update(o, lo, ps, so, to + dt);
        if (ui::blinkLevel() == BL_OFF) sawBlinkOff = true;
        if (ui::blinkLevel() == BL_MAX && !sawMax) {
          sawMax = true;
          tMax = dt;
        }
        gfx::dirtyClear();
      }
      CHECK(sawMax && tMax > OPC_SCAN_MS + OPC_FLASH_MS + OPC_RISE_MS / 2, "サーキット: 点いていく途中から最大輝度(本物と同じ決まり)");
      (void)sawBlinkOff;
      ui::update(o, lo, ps, so, to + OPC_SCAN_MS + OPC_FLASH_MS + OPC_RISE_MS + 50);
      CHECK(fb[233 * gfx::W + 233] == gfx::rgb(0xA31515) && ui::blinkLevel() == BL_MAX, "サーキット: 全部点いたら背景も赤、最大輝度");
      ui::update(o, lo, ps, so, to + OPC_SCAN_MS + OPC_FLASH_MS + OPC_RISE_MS + OPC_LIMIT_MS + 50);
      CHECK(fb[233 * gfx::W + 233] == 0, "サーキット: 消えていくときは背景が黒に戻る");
      o.gearOk = true; o.gear = 'N';
      ui::update(o, lo, ps, so, to + OPC_TOTAL_MS - 20);
      CHECK(ui::screen() == ui::SC_OPENING && nonBlack(120, 170, 346, 293) > 300 && ui::blinkLevel() == BL_NORMAL, "サーキット: 終わりの手前でギア(N)が浮かぶ(明るさは設定の値)");
      ui::update(o, lo, ps, so, to + OPC_TOTAL_MS);
      CHECK(ui::screen() == ui::SC_DRIVE && ui::blinkLevel() == BL_NORMAL, "サーキット: OPC_TOTAL_MS でサーキットの画面へ");
      gfx::dirtyClear();
      ui::testShow(ui::SC_OPENING, 0);
      ui::update(o, lo, ps, so, to + 10000);
      ui::update(o, lo, ps, so, to + 10000 + OPC_SCAN_MS + OPC_FLASH_MS + OPC_RISE_MS + 100);
      CHECK(ui::blinkLevel() == BL_MAX, "サーキット: リミッターの間は最大輝度");
      ui::tap(233, 233, to + 10000 + OPC_SCAN_MS + OPC_FLASH_MS + OPC_RISE_MS + 200);
      ui::update(o, lo, ps, so, to + 10000 + OPC_SCAN_MS + OPC_FLASH_MS + OPC_RISE_MS + 210);
      CHECK(ui::screen() == ui::SC_DRIVE && ui::blinkLevel() == BL_NORMAL, "サーキット: タップで飛ばせる(明るさも戻る)");
      gfx::dirtyClear();
    }
    // ---- タコ: 針が 0 から振り切りまで上がり、通った所から文字盤が出る → 止まる → 0 まで戻る → 今の値に落ち着き、そのままタコの画面
    {
      uint32_t to = 91000000;
      Signals r;
      memset(&r, 0, sizeof(r));
      r.linkUp = r.rpmOk = r.boostOk = r.waterOk = r.fuelOk = r.gearOk = true;
      r.rpm = 850; r.boostBar = -0.52f; r.waterC = 70; r.fuelPct = 60; r.gear = 'N';
      // 「4」の数字の所(4000 rpm、半径 166)
      const float a4 = (TACH_START_DEG + TACH_DEG_PER_1000 * 4) * 3.14159265f / 180.0f;
      const int x4 = (int)(233 + 166 * cosf(a4)), y4 = (int)(233 + 166 * sinf(a4));
      ui::testShow(ui::SC_OPENING, 1);
      ui::update(r, lo, ps, so, to);
      CHECK(ui::screen() == ui::SC_OPENING && nonBlack(x4 - 20, y4 - 30, x4 + 20, y4 + 30) == 0, "オープニング(タコ): 最初は文字盤を隠している(「4」の数字が無い)");
      CHECK(nonBlack(300, 220, 430, 284) > 300, "タコ: ブーストの数字は最初から出ている");
      bool redSeen = false, maxSeen = false;
      for (uint32_t dt = 10; dt <= OPT_UP_MS; dt += 10) {
        ui::update(r, lo, ps, so, to + dt);
        redSeen = redSeen || ui::testTachRed();
        maxSeen = maxSeen || ui::blinkLevel() == BL_MAX;
        gfx::dirtyClear();
      }
      CHECK(nonBlack(x4 - 20, y4 - 30, x4 + 20, y4 + 30) > 100, "タコ: 針が通った所から文字盤が出る(「4」の数字が出た)");
      CHECK(redSeen && maxSeen, "タコ: シフト点を超えると目盛と針が赤、最大輝度も出る(本物と同じ)");
      ui::update(r, lo, ps, so, to + OPT_UP_MS + OPT_TOP_MS / 2);
      CHECK(ui::testTachRed() && ui::blinkLevel() == BL_MAX && ui::screen() == ui::SC_OPENING, "タコ: 振り切りで止まる(赤、最大輝度)");
      for (uint32_t dt = OPT_UP_MS + OPT_TOP_MS; dt < OPT_TOTAL_MS; dt += 10) {
        ui::update(r, lo, ps, so, to + dt);
        gfx::dirtyClear();
      }
      CHECK(!ui::testTachRed() && ui::blinkLevel() == BL_NORMAL && ui::screen() == ui::SC_OPENING, "タコ: 0 まで戻ると赤をやめ、設定の明るさ");
      ui::update(r, lo, ps, so, to + OPT_TOTAL_MS);
      CHECK(ui::screen() == ui::SC_BOOST, "タコ: OPT_TOTAL_MS でタコの画面へ");
      ui::update(r, lo, ps, so, to + OPT_TOTAL_MS + 30);
      memcpy(fbO, fb, sizeof(fbO));
      ui::testRedraw();
      ui::update(r, lo, ps, so, to + OPT_TOTAL_MS + 30);
      CHECK(memcmp(fbO, fb, sizeof(fbO)) == 0, "タコ: 描き直さずに移ったタコの画面 = 最初から描いたタコの画面(針・帯・数字が今の値)");
      gfx::dirtyClear();
      // 文字盤を隠している途中で飛ばす: 隠すのをやめ、タコの画面を全部描く(そのあとも跡が残らない)
      ui::testShow(ui::SC_OPENING, 1);
      ui::update(r, lo, ps, so, to + 20000);
      ui::update(r, lo, ps, so, to + 20000 + OPT_UP_MS / 2);
      ui::swipe(1, to + 20000 + OPT_UP_MS / 2 + 10);
      ui::update(r, lo, ps, so, to + 20000 + OPT_UP_MS / 2 + 20);
      CHECK(ui::screen() == ui::SC_BOOST && nonBlack(x4 - 20, y4 - 30, x4 + 20, y4 + 30) > 100, "タコ: 途中で飛ばしたら文字盤を全部出す");
      r.rpm = 4500; r.boostBar = 0.8f;
      ui::update(r, lo, ps, so, to + 20000 + OPT_UP_MS / 2 + 60);
      memcpy(fbO, fb, sizeof(fbO));
      ui::testRedraw();
      ui::update(r, lo, ps, so, to + 20000 + OPT_UP_MS / 2 + 60);
      CHECK(memcmp(fbO, fb, sizeof(fbO)) == 0, "タコ: 飛ばしたあとに針を動かした絵 = 最初から描いた絵(隠すのをやめている)");
      gfx::dirtyClear();
    }
    // ---- データ: 外周の輪 → 自己診断を 1 行ずつ(確かめている間は点が点滅、輪が埋まる) → 輪を結果の色に、版 → データの画面
    {
      uint32_t to = 92000000;
      ui::testShow(ui::SC_OPENING, 2);
      ui::update(o, lo, ps, so, to);
      CHECK(ui::screen() == ui::SC_OPENING, "オープニング(データ): 始まる");
      ui::update(o, lo, ps, so, to + OPD_RING_MS + OPD_CHECK_MS / 2);
      CHECK(nonBlack(112, 110, 250, 165) > 100 && nonBlack(250, 110, 386, 165) == 0, "データ: 確かめている間は名前だけ(結果はまだ)");
      for (uint32_t dt = OPD_RING_MS + OPD_CHECK_MS / 2; dt < OPD_TOTAL_MS; dt += 10) {
        if (dt >= 700) lo.locked = true;  // 途中で受信が始まる
        ui::update(o, lo, ps, so, to + dt);
        gfx::dirtyClear();
      }
      CHECK(fb[143 * gfx::W + 90] == GREEN && nonBlack(250, 110, 386, 165) > 100, "データ: 受信が始まると CAN の行が OK(緑)");
      CHECK(fb[6 * gfx::W + 233] == GREEN && ui::screen() == ui::SC_OPENING, "データ: 全部出たら外周の輪が緑(NG なし。輪はデータ画面の回転数の輪と同じ所)");
      ui::update(o, lo, ps, so, to + OPD_TOTAL_MS);
      CHECK(ui::screen() == ui::SC_DATA, "データ: OPD_TOTAL_MS でデータの画面へ");
      gfx::dirtyClear();
      // NG(タッチが応答しない)なら輪が赤、OPENING_NG_EXTRA_MS だけ長く見せる
      so.touchOk = false;
      ui::testShow(ui::SC_OPENING, 2);
      ui::update(o, lo, ps, so, to + 20000);
      for (uint32_t dt = 10; dt <= OPD_TOTAL_MS + 100; dt += 10) ui::update(o, lo, ps, so, to + 20000 + dt);
      CHECK(ui::screen() == ui::SC_OPENING && fb[6 * gfx::W + 233] == RED, "データ: NG があると輪が赤で、OPD_TOTAL_MS を過ぎてもオープニングのまま");
      ui::update(o, lo, ps, so, to + 20000 + OPD_TOTAL_MS + OPENING_NG_EXTRA_MS);
      CHECK(ui::screen() == ui::SC_DATA, "データ: NG があっても OPD_TOTAL_MS + OPENING_NG_EXTRA_MS でデータの画面へ");
      gfx::dirtyClear();
      so.touchOk = true;
    }
    // 覚えた画面をサーキットに戻しておく(あとの試験のため)
    ui::testShow(ui::SC_OPENING, 0);
    ui::tap(233, 233, 93000000);
    ui::update(o, lo, ps, so, 93000010);
    CHECK(ui::screen() == ui::SC_DRIVE && ui::homeScreen() == ui::SC_DRIVE, "オープニングのあと: 覚えた画面はサーキット");
    gfx::dirtyClear();
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
    const uint8_t eng[8] = { 0, 0, 0, 130, 65, 0, 0, 0 };          // 水温 130 − 40 = 90 ℃(b4-b5 は燃料の量。ブーストには使わない: 2.8.0)
    const uint8_t trb[8] = { 0, 1, 68, 0x48, 0x31, 0x10, 1, 0x80 };  // ブースト 68 × 0.02 = 1.36 bar(0A1CA001 b2。10/22 のログの全開の最大)
    const uint8_t clutch[8] = { 0, 0, 0, 0, 0, 0x20, 0, 0 };
    canFrame(0, 0x0618A001, rpm3000, 8, t);
    canFrame(0, 0x0210A006, kmh60, 8, t);
    canFrame(0, 0x0A18A001, eng, 8, t);
    canFrame(0, 0x0628A001, clutch, 8, t);
    mlink::getSignals(&s, t);
    CHECK(s.linkUp && mlink::usingRaw(t) && s.rpmOk && s.rpm == 3000 && s.throttleOk && s.throttlePct == 100.0f, "生データ: 回転数・アクセル(0618A001 b2-3、b7)");
    CHECK(s.speedOk && s.speedKmh == 60.0f, "生データ: 車速(0210A006 b4-5 ÷ 128)");
    CHECK(s.waterOk && s.waterC == 90.0f && !s.boostOk, "生データ: 水温 b3 − 40(0A18A001)。b4 はブーストに使わない(0A1CA001 が来るまでブーストは信号なし。2.8.0)");
    canFrame(0, 0x0A1CA001, trb, 8, t);
    mlink::getSignals(&s, t);
    CHECK(s.boostOk && fabsf(s.boostBar - 1.36f) < 0.0001f, "生データ: ブースト 0A1CA001 b2 × 0.02 bar(68 → 1.36 bar。2.8.0)");
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
    // 燃料(2.7.0): C-CAN 0A18A000 b5 = %。b5 = 0 はエンジンが止まっている間などの測れていない値(10/22 のログの形のまま)
    const uint8_t fuelZero[8] = { 0x00, 0x09, 0x18, 0x7B, 0x02, 0x00, 0x00, 0x3D };
    canFrame(0, 0x0A18A000, fuelZero, 8, t + 60 + VEH_TIMEOUT_MS);
    mlink::getSignals(&s, t + 60 + VEH_TIMEOUT_MS);
    CHECK(s.linkUp && s.rpmOk && s.speedOk, "生データ: VEH_TIMEOUT_MS ちょうどまでは信号あり");
    mlink::getSignals(&s, t + 61 + VEH_TIMEOUT_MS);
    CHECK(s.linkUp && !s.rpmOk && !s.speedOk && !s.throttleOk && !s.gearOk && !s.boostOk && s.waterOk, "生データ: VEH_TIMEOUT_MS を過ぎた値は信号なし(水温は VEH_WATER_TIMEOUT_MS)");
    CHECK(!s.fuelOk, "燃料: b5 = 0(エンジンをかける前)だけでは値なし");
    veh::FuelInfo fi;
    veh::fuelInfo(&fi, t + 61 + VEH_TIMEOUT_MS);
    CHECK(fi.ignored == 1 && !fi.have && fi.seen, "燃料: b5 = 0 は数えて捨てる");
    uint8_t fuelV[8] = { 0x20, 0x0D, 0x40, 0x7B, 0x00, 31, 0x20, 0x3D };
    canFrame(0, 0x0A18A000, fuelV, 8, t + 1060 + VEH_TIMEOUT_MS);
    mlink::getSignals(&s, t + 1060 + VEH_TIMEOUT_MS);
    CHECK(s.fuelOk && s.fuelPct == 31.0f, "燃料: b5 をそのまま %(31 → 31 %。なましはしない)");
    uint32_t t2 = t + 100000;
    canFrame(0, 0x0A18A000, fuelZero, 8, t2);
    mlink::getSignals(&s, t2);
    CHECK(s.fuelOk && s.fuelPct == 31.0f, "燃料: エンジンを止めて b5 = 0 になっても、前の値のまま");
    fuelV[5] = 100;
    canFrame(0, 0x0A18A000, fuelV, 8, t2 + 1000);
    mlink::getSignals(&s, t2 + 1000);
    CHECK(s.fuelOk && s.fuelPct == 100.0f, "燃料: 給油してエンジンをかけたら、すぐその値(31 → 100)");
    fuelV[5] = 0xFF;
    canFrame(0, 0x0A18A000, fuelV, 8, t2 + 2000);
    canFrame(1, 0x0A18A000, fuelZero, 8, t2 + 2000);  // B-CAN の同じ ID は使わない(c6_can_link は C-CAN を bus 0 で送る)
    fuelV[5] = 50;
    canFrame(1, 0x0A18A000, fuelV, 8, t2 + 2000);
    mlink::getSignals(&s, t2 + 2000);
    veh::fuelInfo(&fi, t2 + 2000);
    CHECK(s.fuelPct == 100.0f && fi.ignored == 3 && fi.lastB5 == 0xFF, "燃料: 100 を超える b5 は数えて捨てる。B-CAN の同じ ID は使わない");
    mlink::getSignals(&s, t2 + 2001 + VEH_FUEL_TIMEOUT_MS);
    CHECK(!s.fuelOk, "燃料: VEH_FUEL_TIMEOUT_MS 来なければ値なし");
    veh::begin();
    canFrame(0, 0x0A18A000, fuelZero, 8, t2 + 3000);
    mlink::getSignals(&s, t2 + 3000);
    CHECK(!s.fuelOk, "燃料: 電源を入れ直したら、エンジンがかかる(b5 が 0 でなくなる)まで値なし(前の値は保存しない。2.7.0)");

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
      static const uint32_t IDS[] = { 0x0618A001, 0x0210A006, 0x0628A001, 0x0A18A001, 0x0A18A000, 0x0A1CA001, 0x12345678 };
      uint32_t id = IDS[(r >> 8) % 7];
      veh::onFrame((uint8_t)((r >> 4) & 1), (uint8_t)((r >> 5) & 1), id, (uint8_t)((r >> 12) % 9), rnd, t2 + 10000 + (uint32_t)i * 7);
      if (i % 50 == 0) {
        Signals v;
        memset(&v, 0, sizeof(v));
        veh::getSignals(&v, t2 + 10000 + (uint32_t)i * 7);
        CHECK(v.fuelPct >= 0.0f && v.fuelPct <= 100.0f && v.speedKmh >= 0.0f && v.speedKmh < 512.0f && v.throttlePct <= 100.0f && v.waterC >= -40.0f && v.waterC <= 215.0f &&
                  v.boostBar >= 0.0f && v.boostBar <= 255 * VEH_BOOST_BAR_PER_LSB + 0.01f,
              "でたらめな生データでも値は範囲の中");
      }
    }
  }

  // ---------------- 22a. 10/22 の実車ログ(logs/ の L 行)を流す: 燃料(2.7.0: 0A18A000 b5 = %)は落ち着いた値で、エンジンが止まって
  //                     b5 = 0 になっても前の値のまま。給油で 31 → 100。R とブーストが出る
  {
    struct LogRun {
      const char *path;
      int lines;
      uint32_t ignored;
      float fuelMin[2], fuelMax[2];  // [0] = split より前、[1] = split から(値なしの回は数えない)
      float boostMax;
      bool sawR, sawGear;
    };
    auto run = [&](const char *path, uint32_t split, LogRun *out) -> bool {
      FILE *fp = fopen(path, "r");
      if (!fp) return false;
      veh::begin();
      memset(out, 0, sizeof(*out));
      out->path = path;
      out->fuelMin[0] = out->fuelMin[1] = 1000;
      out->fuelMax[0] = out->fuelMax[1] = -1;
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
        const int k = tm >= split ? 1 : 0;
        if (v.fuelOk && v.fuelPct < out->fuelMin[k]) out->fuelMin[k] = v.fuelPct;
        if (v.fuelOk && v.fuelPct > out->fuelMax[k]) out->fuelMax[k] = v.fuelPct;
        if (v.boostOk && v.boostBar > out->boostMax) out->boostMax = v.boostBar;
        if (v.gearOk && v.gear == 'R') out->sawR = true;
        if (v.gearOk && v.gear >= '1' && v.gear <= '5') out->sawGear = true;
      }
      fclose(fp);
      veh::FuelInfo fi;
      veh::fuelInfo(&fi, 0);
      out->ignored = fi.ignored;
      return true;
    };
    LogRun lr;
    if (run("../logs/can_reverse.txt", UINT32_MAX, &lr)) {
      printf("  実車ログ %s: %d 行、捨てた b5 %u 回、燃料 %.0f〜%.0f %%、R %s\n", lr.path, lr.lines, (unsigned)lr.ignored, (double)lr.fuelMin[0], (double)lr.fuelMax[0], lr.sawR ? "あり" : "なし");
      CHECK(lr.ignored >= 1 && lr.fuelMin[0] == 34.0f && lr.fuelMax[0] == 35.0f, "実車ログ(リバース): 燃料は 35 → 34 %(エンジンをかける前の b5 = 0 は捨てる)");
      CHECK(lr.sawR, "実車ログ(リバース): R が出る");
    } else {
      printf("  (../logs/can_reverse.txt が無いので、実車ログの試験は飛ばす)\n");
    }
    if (run("../logs/can_boost.txt", UINT32_MAX, &lr)) {
      printf("  実車ログ %s: %d 行、ブーストの最大 %.2f bar、ギア %s、燃料 %.0f〜%.0f %%\n", lr.path, lr.lines, (double)lr.boostMax, lr.sawGear ? "あり" : "なし", (double)lr.fuelMin[0],
             (double)lr.fuelMax[0]);
      CHECK(fabsf(lr.boostMax - 1.36f) < 0.001f && lr.sawGear, "実車ログ(ブースト): 0A1CA001 b2 の最大 68 → 1.36 bar(2.8.0。2.7.0 は 0A18A001 b4 の 67 → 0.67)、走行中にギアが出る");
      CHECK(lr.fuelMin[0] == 33.0f && lr.fuelMax[0] == 34.0f, "実車ログ(ブースト): 燃料は 33〜34 %");
    }
    if (run("../logs/can_fuel.txt", 848686, &lr)) {
      printf("  実車ログ %s: %d 行、捨てた b5 %u 回、燃料 給油の前 %.0f〜%.0f %%(712 秒にエンジンを止めてからも前の値のまま)、給油の後 %.0f〜%.0f %%\n", lr.path, lr.lines,
             (unsigned)lr.ignored, (double)lr.fuelMin[0], (double)lr.fuelMax[0], (double)lr.fuelMin[1], (double)lr.fuelMax[1]);
      CHECK(lr.ignored >= 1 && lr.fuelMin[0] == 31.0f && lr.fuelMax[0] == 33.0f, "実車ログ(給油): 給油の前は 33 → 31 %(残り 30 % ほど)。エンジンを止めた間の b5 = 0 は捨てる");
      CHECK(lr.fuelMin[1] == 100.0f && lr.fuelMax[1] == 100.0f, "実車ログ(給油): 給油してエンジンをかけたら 100 % を出し続ける(満タン)");
      CHECK(fabsf(lr.boostMax - 0.52f) < 0.001f, "実車ログ(給油): ブーストの最大 26 → 0.52 bar(アクセル 49 % が最大の走り。2.8.0)");
    }
  }

  // ---------------- 23. 本体の状態の「FPS」(2.3.0。2.0.1〜2.2.0 は「描画」= 最長。2.4.0 で受信の状態から移した。2.4.3: 1 秒ごと):
  //                    サーキット・タコで 1 回描いて送る時間を 1 秒ごとにまとめ、回を数えた秒だけを新しい方から PERF_FPS_SECS 秒ぶん覚える。
  //                    平均 = 全部の回の平均、最低 = 1 秒ごとの値で一番低い秒、グラフ = 1 秒ごとの回/秒。赤は平均が INFO_FPS_MIN 未満。
  //                    数える回(ui::pageFrame)は、サーキット・タコをふだんどおり描いた回だけ(画面の切り替え、案内・長押しの表示は除く)
  {
    Signals d;
    memset(&d, 0, sizeof(d));
    d.linkUp = d.rpmOk = d.waterOk = d.fuelOk = d.gearOk = d.speedOk = d.throttleOk = true;
    d.rpm = 2500; d.waterC = 85; d.fuelPct = 60; d.gear = '3';
    uint32_t tp = 90000000;
    // perf: 1 秒ごとのまとめ
    PerfStats q;
    memset(&q, 0, sizeof(q));
    perf::pageClear();
    perf::pageStats(&q);
    CHECK(q.pageFrames == 0 && q.pageAvgUs == 0 && q.pageSlowUs == 0 && q.histN == 0, "perf: 1 回も無ければ 0");
    uint32_t tm = 1000000;  // ちょうど秒の頭
    for (int i = 0; i < 40; i++) perf::pageAdd(20000, tm + (uint32_t)i * 20);   // 1 秒目: 20 ms × 40 回(50 回/秒)
    for (int i = 0; i < 10; i++) perf::pageAdd(110000, tm + 1000 + (uint32_t)i * 50);  // 2 秒目: 110 ms × 10 回(9 回/秒。背景の塗り直しが続いた秒)
    perf::pageStats(&q);
    CHECK(q.pageFrames == 50 && q.histN == 2 && q.fpsHist[0] == 50 && q.fpsHist[1] == 9, "perf: 1 秒ごとの回/秒(50 と 9。今数えている秒も入る)");
    CHECK(q.pageAvgUs == (40u * 20000u + 10u * 110000u) / 50u && q.pageSlowUs == 110000, "perf: 平均は全部の回の平均、最低は一番遅い秒");
    perf::pageAdd(20000, tm + 600000);  // 10 分あと(間に設定などを出していた): 回を数えた秒だけを並べる
    perf::pageStats(&q);
    CHECK(q.histN == 3 && q.fpsHist[2] == 50, "perf: 回を数えていない秒は飛ばす(設定を出していた間)");
    perf::pageClear();
    for (int sec = 0; sec < PERF_FPS_SECS + 20; sec++) perf::pageAdd(sec < 20 ? 100000 : 25000, tm + (uint32_t)sec * 1000);
    perf::pageStats(&q);
    CHECK(q.histN == PERF_FPS_SECS && q.pageFrames == PERF_FPS_SECS && q.pageSlowUs == 25000 && q.fpsHist[0] == 40 && q.fpsHist[PERF_FPS_SECS - 1] == 40,
          "perf: 覚えるのは新しい方から PERF_FPS_SECS 秒(古い 10 回/秒の秒は落ちる)");
    perf::pageClear();
    // 値の文字と判定(0 = 待ち、1 = 正常、2 = 異常)
    char v[48];
    PerfStats p;
    memset(&p, 0, sizeof(p));
    p.fps = 2; p.loopMaxMs = 77; p.pageMaxUs = 110000;  // 送った回数・最長は使わない(2.2.0 は最長 110 ms で「9 fps」の赤だった)
    int vd = ui::testInfoItem(4, 0, ls, p, si, tp, v, sizeof(v));
    CHECK(vd == 0 && strcmp(v, "– –") == 0, "FPS: まだ測っていなければ「– –」の灰");
    p.pageFrames = FPS_MIN_FRAMES - 1; p.pageAvgUs = 21400; p.pageSlowUs = 31800;
    vd = ui::testInfoItem(4, 0, ls, p, si, tp, v, sizeof(v));
    CHECK(vd == 0 && strcmp(v, "– –") == 0, "FPS: 数えた回が FPS_MIN_FRAMES 未満なら「– –」の灰(受信が無く、描き直す所がほとんど無いとき)");
    p.pageFrames = FPS_MIN_FRAMES;
    vd = ui::testInfoItem(4, 0, ls, p, si, tp, v, sizeof(v));
    CHECK(vd == 1 && strcmp(v, "平均 47 最低 31") == 0, "FPS: 21.4 ms と 31.8 ms →「平均 47 最低 31」の正常");
    p.pageSlowUs = 111111;
    vd = ui::testInfoItem(4, 0, ls, p, si, tp, v, sizeof(v));
    CHECK(vd == 1 && strcmp(v, "平均 47 最低 9") == 0, "FPS: 最低が 9 でも、平均が INFO_FPS_MIN 以上なら正常(2.4.3。2.3.0〜2.4.2 は毎回赤だった)");
    p.pageAvgUs = 50000;
    vd = ui::testInfoItem(4, 0, ls, p, si, tp, v, sizeof(v));
    CHECK(vd == 1 && strcmp(v, "平均 20 最低 9") == 0, "FPS: 平均 20 は正常");
    p.pageAvgUs = 52632;
    vd = ui::testInfoItem(4, 0, ls, p, si, tp, v, sizeof(v));
    CHECK(vd == 2 && strcmp(v, "平均 19 最低 9") == 0, "FPS: 平均が INFO_FPS_MIN より少なければ異常");
    p.pageAvgUs = 0;
    vd = ui::testInfoItem(4, 0, ls, p, si, tp, v, sizeof(v));
    CHECK(vd == 0 && strcmp(v, "– –") == 0, "FPS: 0 で割らない");
    // 画面に描いて、字が全部あること(値の書体に無い字を使っていない)、下の 1 個目の点の色(本体の状態は 8 ページ)、グラフ
    const uint32_t miss0 = gfx::missingGlyphs();
    auto dot0 = [&]() { return fb[330 * gfx::W + (int)(233 - 26 * 3.5f)]; };  // 2.4.3: 点は 8 個(間隔 26)
    const uint32_t avg[4] = { 0, 21400, 52632, 120000 };
    const uint16_t col[4] = { gfx::rgb(0x5A606A), gfx::rgb(0x22E36A), gfx::rgb(0xE3241B), gfx::rgb(0xE3241B) };
    bool colOk = true;
    p.histN = PERF_FPS_SECS;
    for (int i = 0; i < PERF_FPS_SECS; i++) p.fpsHist[i] = (uint8_t)(i % 60 < 50 ? 45 : 9);
    for (int k = 0; k < 4; k++) {
      p.pageFrames = 100; p.pageAvgUs = avg[k]; p.pageSlowUs = 111111;
      ui::testShow(ui::SC_SET_GRP, 7, 4);  // 8 ページ目(起動)を見ているので、1 個目の点は FPS の判定の色(小さい点)
      ui::testTouch(tp += 600);            // 設定から時間で戻らないように
      ui::update(d, ls, p, si, tp);
      if (ui::screen() != ui::SC_SET_GRP || dot0() != col[k]) colOk = false;
      ui::testShow(ui::SC_SET_GRP, 0, 4);  // 「FPS」の画面そのもの
      ui::testTouch(tp += 600);
      ui::update(d, ls, p, si, tp);
      if (ui::screen() != ui::SC_SET_GRP) colOk = false;
    }
    CHECK(colOk, "FPS: 本体の状態の 1 個目の点は 測っていない = 灰、正常 = 緑、異常 = 赤");
    {
      int w = 0;
      for (int y = 250; y <= 307; y++)
        for (int x = 93; x <= 373; x++) w += fb[y * gfx::W + x] == gfx::rgb(0xFFFFFF);
      CHECK(w > 200, "FPS: 直近 3 分の 1 秒ごとの値のグラフ(白い線)");
    }
    CHECK(gfx::missingGlyphs() == miss0, "FPS: 名前と値の字はすべて書体にある");
    gfx::dirtyClear();

    // 数える回: サーキットに入った回(全体の描き直し)は数えない。そのあとのふだんの回は数える
    ui::testShow(ui::SC_DRIVE);
    ui::update(d, ls, p, si, tp += 100);
    CHECK(!ui::pageFrame(), "数える回: サーキットに切り替えた回の全体の描き直しは数えない");
    d.rpm = 4200;
    ui::update(d, ls, p, si, tp += 20);
    CHECK(ui::pageFrame() && gfx::dirtyCount() > 0, "数える回: サーキットのふだんの回(回転数が変わって描き直した所がある)");
    gfx::dirtyClear();
    // 案内と長押しの表示: 出し始めた回と、出している間は数えない。動いて取り消し、表示を消した回(全体の描き直し)も数えない
    ui::touchDown(233, 233, tp += 20);
    d.rpm = 4300;
    ui::update(d, ls, p, si, tp + 50);
    CHECK(ui::pageFrame(), "数える回: 押した直後(タッチが決まる前)はふだんの回");
    ui::update(d, ls, p, si, tp + TOUCH_CONFIRM_MS + 10);
    CHECK(ui::testGuide() && !ui::pageFrame(), "数える回: 案内を出し始めた回は数えない");
    d.rpm = 4400;
    ui::update(d, ls, p, si, tp + TOUCH_CONFIRM_MS + 30);
    CHECK(!ui::pageFrame(), "数える回: 案内を出している間は数えない");
    ui::update(d, ls, p, si, tp + HOLD_SHOW_MS + 10);
    CHECK(ui::testHolding() && !ui::pageFrame(), "数える回: 長押しの表示を出し始めた回は数えない");
    ui::update(d, ls, p, si, tp + HOLD_SHOW_MS + 30);
    CHECK(!ui::pageFrame(), "数える回: 長押しの表示を出している間は数えない");
    ui::touchMove(233 + HOLD_CANCEL_PX + 10, 233, tp + HOLD_SHOW_MS + 40);
    ui::update(d, ls, p, si, tp + HOLD_SHOW_MS + 50);
    CHECK(ui::screen() == ui::SC_DRIVE && !ui::testHolding() && ui::testGuide() && !ui::pageFrame(), "数える回: 動いて長押しを取り消し、外周を消した回(全体の描き直し。案内は出し直す)は数えない");
    ui::touchMove(233 - 200, 236, tp + HOLD_SHOW_MS + 80);
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
    ui::testShow(ui::SC_SET_LIST, 1);
    ui::update(d, ls, p, si, tp += 100);
    ui::update(d, ls, p, si, tp += 100);
    CHECK(!ui::pageFrame(), "数える回: 設定は数えない");
    gfx::dirtyClear();
  }

  // ---------------- 24. 2.2.0 / 2.4.0: 受信の状態のまとめ(通信が切れていたら「正常」にしない)、グループのまとめ、CAN の「燃料」、
  //                    ピークホールドの出し方
  {
    char v[48];
    const uint32_t far = 2000000000u;  // 前の試験の「直近に増えた誤り」から離れた時刻
    LinkStats l0;
    memset(&l0, 0, sizeof(l0));  // 何も受けていない
    PerfStats p0;
    memset(&p0, 0, sizeof(p0));
    p0.pageFrames = 100; p0.pageAvgUs = p0.pageSlowUs = 20000;  // FPS は正常
    SysInfo s0;
    memset(&s0, 0, sizeof(s0));
    s0.axpOk = true; s0.powerOnSrc = 0x04;  // 起動は正常(5V 投入)
    // CAN の「通信速度」: 何も届かないときは、起動から INFO_LINK_GRACE_MS までは灰、そのあとは赤(デモ表示の間は灰)
    CHECK(ui::testInfoItem(0, 0, l0, p0, s0, INFO_LINK_GRACE_MS - 1, v, sizeof(v)) == 0, "通信速度: 起動直後で受けていなければ待ち(灰)");
    CHECK(ui::testInfoItem(0, 0, l0, p0, s0, INFO_LINK_GRACE_MS + 1, v, sizeof(v)) == 2, "通信速度: 起動から INFO_LINK_GRACE_MS たっても受けていなければ異常(赤)");
    set.demoOn = 1;
    CHECK(ui::testInfoItem(0, 0, l0, p0, s0, far, v, sizeof(v)) == 0, "通信速度: デモ表示の間は受けていなくても待ち(灰)");
    set.demoOn = 0;
    int r = ui::testInfoSummary(-1, l0, p0, s0, far, v, sizeof(v));
    CHECK(r == 2 && strcmp(v, "異常 1") == 0, "まとめ: 受けていないときは「異常 1」(2.1.0 は「正常」だった)");
    LinkStats l1 = l0;
    l1.locked = true; l1.baud = 921600; l1.framesPerSec = 200;
    r = ui::testInfoSummary(-1, l1, p0, s0, far, v, sizeof(v));
    CHECK(r == 0 && strcmp(v, "待ち") == 0, "まとめ: 受けているが CANリーダーの状態がまだ届かないときは「待ち」");
    l1.infoValid = true; l1.info.proto_ver = METER_LINK_PROTO_VER; l1.info.can_fps[0] = 900; l1.info.can_fps[1] = 140;
    r = ui::testInfoSummary(-1, l1, p0, s0, far, v, sizeof(v));
    CHECK(r == 1 && strcmp(v, "正常") == 0, "まとめ: CAN の項目が全部正常なら「正常」");
    CHECK(ui::testInfoSummary(0, l1, p0, s0, far, v, sizeof(v)) == 1 && strcmp(v, "正常") == 0, "CAN のまとめも「正常」");
    PerfStats p1 = p0;
    p1.pageFrames = 0;  // FPS をまだ測っていない(灰)
    r = ui::testInfoSummary(-1, l1, p1, s0, far, v, sizeof(v));
    CHECK(r == 1, "受信の状態のまとめ: FPS をまだ測っていなくても「正常」(FPS は本体の状態。2.4.0)");
    p1.pageFrames = 100; p1.pageAvgUs = 60000; p1.pageSlowUs = 90000;  // FPS が遅い(平均 17。赤)
    r = ui::testInfoSummary(-1, l1, p1, s0, far, v, sizeof(v));
    CHECK(r == 1, "受信の状態のまとめ: FPS が赤でも数えない(2.4.0: 受信の状態から出した)");
    r = ui::testInfoSummary(4, l1, p1, s0, far, v, sizeof(v));
    CHECK(r == 2 && strcmp(v, "異常 1") == 0, "本体の状態のまとめ: FPS が赤なら「異常 1」");
    p1.pageFrames = 0;
    r = ui::testInfoSummary(4, l1, p1, s0, far, v, sizeof(v));
    CHECK(r == 1 && strcmp(v, "正常") == 0, "本体の状態のまとめ: FPS をまだ測っていなければ「正常」(赤が無い)");
    l1.info.can_fps[0] = 0;  // 車の CAN が止まった
    r = ui::testInfoSummary(-1, l1, p0, s0, far, v, sizeof(v));
    CHECK(r == 2, "まとめ: C-CAN が 0 なら「異常」");
    CHECK(ui::testInfoItem(0, 4, l1, p0, s0, far, v, sizeof(v)) == 2 && strcmp(v, "C 0 B 140") == 0, "CAN の 5 ページ目 = C-CAN・B-CAN(C-CAN が 0 なら赤)");
    l1.info.can_fps[0] = 900;
    CHECK(ui::testInfoItem(0, 3, l1, p0, s0, far, v, sizeof(v)) == 1, "CAN の 4 ページ目 = CANリーダー");
    CHECK(ui::testInfoItem(0, 6, l1, p0, s0, far, v, sizeof(v)) == 0 && strcmp(v, "記録 0 回") == 0, "CAN の 7 ページ目 = CAN確認(判定なし。2.4.3)");
    CHECK(ui::testInfoItem(0, 7, l1, p0, s0, far, v, sizeof(v)) == 0, "CAN の 8 ページ目 = 前後で比べる(判定なし。2.6.0)");
    CHECK(ui::testInfoItem(0, 8, l1, p0, s0, far, v, sizeof(v)) == -1 && ui::testInfoItem(5, 0, l1, p0, s0, far, v, sizeof(v)) == -1, "無いページ・グループは -1");
    // 本体の状態の CPU など(2.4.3): 基板が入れる前は「– –」、入れたら型式・クロック・温度・空き
    CHECK(ui::testInfoItem(4, 1, l1, p0, s0, far, v, sizeof(v)) == 0 && strcmp(v, "– –") == 0, "CPU: まだ入っていなければ「– –」");
    {
      SysInfo sc = s0;
      sc.cpu.ok = true;
      snprintf(sc.cpu.model, sizeof(sc.cpu.model), "ESP32-S3");
      sc.cpu.rev = 2; sc.cpu.cores = 2; sc.cpu.cpuMHz = 240; sc.cpu.flashBytes = 16u << 20; sc.cpu.flashHz = 80000000;
      sc.cpu.heapBytes = 327680; sc.cpu.heapFree = 184320; sc.cpu.psramBytes = 8u << 20; sc.cpu.psramFree = 7340032; sc.cpu.tempOk = true; sc.cpu.tempC = 41.25f;
      CHECK(ui::testInfoItem(4, 1, l1, p0, sc, far, v, sizeof(v)) == 1 && strcmp(v, "ESP32-S3 v0.2 2 コア") == 0, "CPU: 型式・版・コアの数");
      CHECK(ui::testInfoItem(4, 2, l1, p0, sc, far, v, sizeof(v)) == 1 && strcmp(v, "240 MHz") == 0, "クロック");
      CHECK(ui::testInfoItem(4, 3, l1, p0, sc, far, v, sizeof(v)) == 1 && strcmp(v, "41.2 °C") == 0, "温度(チップの中のセンサー)");
      CHECK(ui::testInfoItem(4, 4, l1, p0, sc, far, v, sizeof(v)) == 1 && strcmp(v, "空き 180 / 320 KB") == 0, "RAM の空き");
      CHECK(ui::testInfoItem(4, 5, l1, p0, sc, far, v, sizeof(v)) == 1 && strcmp(v, "空き 7.0 / 8.0 MB") == 0, "PSRAM の空き");
      CHECK(ui::testInfoItem(4, 6, l1, p0, sc, far, v, sizeof(v)) == 1 && strcmp(v, "16 MB 80 MHz") == 0, "フラッシュの大きさと速さ");
      CHECK(ui::testInfoItem(4, 7, l1, p0, sc, far, v, sizeof(v)) == 1 && strcmp(v, "5V 投入") == 0, "起動(8 ページ目)");
      sc.cpu.tempOk = false;
      CHECK(ui::testInfoItem(4, 3, l1, p0, sc, far, v, sizeof(v)) == 0 && strcmp(v, "– –") == 0, "温度: 読めなければ「– –」");
    }
    // 2.4.0: 受信の状態のまとめは 4 グループの赤を数える(同じ値は 1 回だけ)。待ち(スマホが未接続、GPS が測位前)は数えない
    {
      SysInfo s2 = s0;
      s2.rc.enabled = true; s2.rc.bleOk = true; s2.rc.connected = false; s2.rc.gpsOk = true; s2.rc.fixType = 0; s2.rc.imuOk = true;
      r = ui::testInfoSummary(-1, l1, p0, s2, far, v, sizeof(v));
      CHECK(r == 1 && strcmp(v, "正常") == 0, "受信の状態のまとめ: RaceChrono のスマホが未接続・GPS が測位前(待ち)でも「正常」");
      CHECK(ui::testInfoSummary(1, l1, p0, s2, far, v, sizeof(v)) == 0 && strcmp(v, "待ち") == 0, "GPS のまとめ: 測位前は「待ち」");
      CHECK(ui::testInfoSummary(3, l1, p0, s2, far, v, sizeof(v)) == 0 && strcmp(v, "待ち") == 0, "RaceChrono のまとめ: スマホが未接続なら「待ち」");
      s2.rc.bleOk = false;
      r = ui::testInfoSummary(-1, l1, p0, s2, far, v, sizeof(v));
      CHECK(r == 2 && strcmp(v, "異常 1") == 0, "受信の状態のまとめ: RaceChrono の BLE が開始できなければ「異常 1」");
      s2.rc.imuOk = false;
      r = ui::testInfoSummary(-1, l1, p0, s2, far, v, sizeof(v));
      CHECK(r == 2 && strcmp(v, "異常 2") == 0, "受信の状態のまとめ: IMU も応答しなければ「異常 2」(IMU は 1 か所だけ)");
      CHECK(ui::testInfoItem(2, 0, l1, p0, s2, far, v, sizeof(v)) == 2 && strcmp(v, "応答なし") == 0, "IMU の 1 ページ目 = 加速度(応答なしは赤)");
      CHECK(ui::testInfoSummary(2, l1, p0, s2, far, v, sizeof(v)) == 2 && strcmp(v, "異常 1") == 0, "IMU のまとめ: 応答しなければ「異常 1」");
      CHECK(ui::testInfoSummary(3, l1, p0, s2, far, v, sizeof(v)) == 2 && strcmp(v, "異常 1") == 0, "RaceChrono のまとめ: BLE だけを数える(IMU は IMU のグループ。2.4.0)");
      s2.rc.enabled = false;
      CHECK(ui::testInfoItem(1, 0, l1, p0, s2, far, v, sizeof(v)) == 0 && strcmp(v, "切(RaceChrono 送信)") == 0, "GPS の測位: 送信が切なら「切(RaceChrono 送信)」の灰");
      CHECK(ui::testInfoItem(2, 0, l1, p0, s2, far, v, sizeof(v)) == 0 && strcmp(v, "切(RaceChrono 送信)") == 0, "IMU の加速度: 送信が切なら「切(RaceChrono 送信)」の灰");
      CHECK(ui::testInfoSummary(3, l1, p0, s2, far, v, sizeof(v)) == 0 && strcmp(v, "切") == 0, "RaceChrono のまとめ: 送信が切なら「切」の灰");
      CHECK(ui::testInfoSummary(1, l1, p0, s2, far, v, sizeof(v)) == 0 && strcmp(v, "切") == 0, "GPS のまとめ: 送信が切なら「切」の灰");
      CHECK(ui::testInfoSummary(-1, l1, p0, s2, far, v, sizeof(v)) == 1, "受信の状態のまとめ: 送信が切なら GPS・IMU・RaceChrono は数えない");
    }
    // RaceChrono: スマホが未接続なら「正常」にしない。GPS は GPS のグループ(RaceChrono のまとめに入れない)
    SysInfo s1 = s0;
    s1.rc.enabled = true; s1.rc.bleOk = true; s1.rc.connected = false; s1.rc.gpsOk = true; s1.rc.fixType = 3; s1.rc.numSV = 9;
    s1.rc.imuOk = true; s1.rc.imuLevel = true; s1.rc.imuForward = true;
    r = ui::testInfoSummary(3, l1, p0, s1, far, v, sizeof(v));
    CHECK(r == 0 && strcmp(v, "待ち") == 0, "RaceChrono のまとめ: スマホが未接続なら「待ち」");
    s1.rc.connected = true;
    r = ui::testInfoSummary(3, l1, p0, s1, far, v, sizeof(v));
    CHECK(r == 1, "RaceChrono のまとめ: 接続中なら「正常」");
    CHECK(ui::testInfoSummary(1, l1, p0, s1, far, v, sizeof(v)) == 1, "GPS のまとめ: 3D なら「正常」");
    s1.rc.gpsOk = false;
    CHECK(ui::testInfoSummary(1, l1, p0, s1, far, v, sizeof(v)) == 2, "GPS のまとめ: 届かなければ「異常」");
    CHECK(ui::testInfoSummary(3, l1, p0, s1, far, v, sizeof(v)) == 1, "RaceChrono のまとめ: GPS が届かなくても数えない(GPS は 1 か所だけ。2.4.0)");

    // CAN の「燃料」(2.7.0): 表示に使っている %(C-CAN 0A18A000 b5)
    veh::begin();
    uint32_t tf = far;
    CHECK(ui::testInfoItem(0, 5, l1, p0, s0, tf, v, sizeof(v)) == 0 && strcmp(v, "– –") == 0, "燃料: 受けていなければ「– –」");
    uint8_t fd[8] = { 0x00, 0x09, 0x18, 0x7B, 0x02, 0, 0, 0x3D };
    veh::onFrame(0, 1, VEH_ID_FUEL, 8, fd, tf += 1000);
    CHECK(ui::testInfoItem(0, 5, l1, p0, s0, tf, v, sizeof(v)) == 0 && strcmp(v, "– –") == 0, "燃料: b5 = 0(エンジンをかける前)だけなら「– –」");
    fd[2] = 0x40; fd[4] = 0x00; fd[5] = 31;
    veh::onFrame(0, 1, VEH_ID_FUEL, 8, fd, tf += 1000);
    CHECK(ui::testInfoItem(0, 5, l1, p0, s0, tf, v, sizeof(v)) == 1 && strcmp(v, "31 %") == 0, "燃料: 使える値で「31 %」(緑)");
    CHECK(ui::testInfoItem(0, 5, l1, p0, s0, tf + VEH_FUEL_TIMEOUT_MS + 1, v, sizeof(v)) == 0 && strcmp(v, "– –") == 0, "燃料: 来なくなったら「– –」");
    {
      // 画面に描いて、字が全部あること(受信の状態・全グループの全ページと、本体の状態)
      Signals dz;
      memset(&dz, 0, sizeof(dz));
      const uint32_t miss0 = gfx::missingGlyphs();
      SysInfo sOn = s1, sOff = s0;
      sOn.cpu.ok = true;
      snprintf(sOn.cpu.model, sizeof(sOn.cpu.model), "ESP32-S3");
      sOn.cpu.rev = 2; sOn.cpu.cores = 2; sOn.cpu.cpuMHz = 240; sOn.cpu.flashBytes = 16u << 20; sOn.cpu.flashHz = 80000000;
      sOn.cpu.heapBytes = 327680; sOn.cpu.heapFree = 184320; sOn.cpu.psramBytes = 8u << 20; sOn.cpu.psramFree = 7340032; sOn.cpu.tempOk = true; sOn.cpu.tempC = 41.25f;
      bool shown = true;
      for (int k = 0; k < 2; k++) {
        const SysInfo &sx = k ? sOff : sOn;
        for (int g = 0; g < 4; g++) {
          ui::testShow(ui::SC_SET_INFO, g);
          ui::testTouch(tf);
          ui::update(dz, l1, p0, sx, tf);
          shown = shown && ui::screen() == ui::SC_SET_INFO;
        }
        const int pages[5] = { 7, 3, 2, 3, 8 };
        for (int g = 0; g < 5; g++)
          for (int i = 0; i < pages[g]; i++) {
            ui::testShow(ui::SC_SET_GRP, i, g);
            ui::testTouch(tf);
            ui::update(dz, l1, p0, sx, tf);
            shown = shown && ui::screen() == ui::SC_SET_GRP && ui::testPage() == i && ui::testGroup() == g;
          }
      }
      CHECK(shown && gfx::missingGlyphs() == miss0, "受信の状態・本体の状態: 全ページの名前と値の字はすべて書体にある");
      gfx::dirtyClear();
    }
    veh::begin();

    // ピークホールド(2.4.0。本人の指示 10/24「回転が下がっても 5 秒程度は残し、回転数が高い状態では上がっても下がっても表示し続けて」):
    // ピークが PEAK_MIN_RPM 以上なら、上がっていても下がっていても出す。
    // 2.4.4: 最後に更新してから TACH_PEAK_HOLD_MS(1.5 秒)止まり、そのあと下がる。ピークを付けてから TACH_PEAK_SHOW_MS(5 秒)で消える
    Signals d;
    memset(&d, 0, sizeof(d));
    d.linkUp = d.rpmOk = d.waterOk = d.fuelOk = d.gearOk = d.speedOk = d.throttleOk = true;
    d.waterC = 85; d.fuelPct = 60; d.gear = '3';
    for (int scr = 0; scr < 2; scr++) {
      const ui::Screen sc = scr ? ui::SC_BOOST : ui::SC_DRIVE;
      const char *nm = scr ? "タコ" : "サーキット";
      char what[160];
      uint32_t tq = far + 100000u * (uint32_t)(scr + 1);
      d.rpm = 2000;
      ui::testShow(sc);
      ui::testPeakReset();
      ui::update(d, ls, ps, si, tq);
      ui::update(d, ls, ps, si, tq += 5000);
      bool low = false, rising = true;
      for (int rpm = 2200; rpm <= 2800; rpm += 200) {
        d.rpm = rpm;
        ui::update(d, ls, ps, si, tq += 50);
        low = low || ui::testPeakDrawn();
      }
      snprintf(what, sizeof(what), "ピークホールド(%s): ピークが PEAK_MIN_RPM 未満の間は出さない", nm);
      CHECK(!low, what);
      for (int rpm = 3000; rpm <= 5000; rpm += 200) {
        d.rpm = rpm;
        ui::update(d, ls, ps, si, tq += 50);
        rising = rising && ui::testPeakDrawn();
      }
      snprintf(what, sizeof(what), "ピークホールド(%s): 回転数が上がっている間も出す(2.4.0)", nm);
      CHECK(rising, what);
      const uint32_t tPeak = tq;  // 5000 のピークを付けた時刻
      d.rpm = 3000;
      ui::update(d, ls, ps, si, tq += 50);
      snprintf(what, sizeof(what), "ピークホールド(%s): 5000 から 3000 に下がっても出す", nm);
      CHECK(ui::testPeakDrawn(), what);
      d.rpm = 1500;
      ui::update(d, ls, ps, si, tq += 50);
      ui::update(d, ls, ps, si, tq = tPeak + TACH_PEAK_HOLD_MS - 100);
      snprintf(what, sizeof(what), "ピークホールド(%s): TACH_PEAK_HOLD_MS(1.5 秒)の間は 5000 で止まる(2.4.4)", nm);
      CHECK(ui::testPeakDrawn() && ui::testPeakRpm() == 5000.0f, what);
      ui::update(d, ls, ps, si, tq = tPeak + TACH_PEAK_HOLD_MS + 500);
      snprintf(what, sizeof(what), "ピークホールド(%s): そのあと 2000 rpm/秒で下がる(0.5 秒で 4000)", nm);
      CHECK(ui::testPeakDrawn() && ui::testPeakRpm() < 4100.0f && ui::testPeakRpm() > 3900.0f, what);
      ui::update(d, ls, ps, si, tq = tPeak + TACH_PEAK_HOLD_MS + 1000 + 100);
      snprintf(what, sizeof(what), "ピークホールド(%s): 3000 を切ったら消す(2.4.5)", nm);
      CHECK(!ui::testPeakDrawn() && ui::testPeakRpm() < 3000.0f, what);
      // 2.4.5: 回転数が一定(4000 前後のゆらぎ)なら、はっきり上がってから TACH_PEAK_SHOW_MS で消える(2.4.4 はゆらぎで消えなかった)
      const uint32_t tRise = tq + 50;  // 1500 → 4000 にはっきり上がった時刻
      for (int i = 0; i < 160; i++) {
        d.rpm = 4000 + (i % 3) * 10;
        ui::update(d, ls, ps, si, tq += 50);
        if (tq - tRise == TACH_PEAK_SHOW_MS - 100) {
          snprintf(what, sizeof(what), "ピークホールド(%s): 回転数が一定でも、はっきり上がってから TACH_PEAK_SHOW_MS の手前までは出す", nm);
          CHECK(ui::testPeakDrawn(), what);
        }
      }
      snprintf(what, sizeof(what), "ピークホールド(%s): 回転数が一定(4000 前後でゆらぐ)なら TACH_PEAK_SHOW_MS で消える(2.4.5)", nm);
      CHECK(!ui::testPeakDrawn(), what);
      d.rpm = 4000 + PEAK_RISE_RPM;
      ui::update(d, ls, ps, si, tq += 50);
      snprintf(what, sizeof(what), "ピークホールド(%s): PEAK_RISE_RPM 上がったらまた出す", nm);
      CHECK(ui::testPeakDrawn(), what);
      d.rpm = 5000;
      ui::update(d, ls, ps, si, tq += 50);
      d.rpm = 3000;
      ui::update(d, ls, ps, si, tq += 50);
      d.rpm = 5200;
      ui::update(d, ls, ps, si, tq += 50);
      snprintf(what, sizeof(what), "ピークホールド(%s): ピーク(5000)を超えても出し続ける(2.2.0〜2.3.x は消していた)", nm);
      CHECK(ui::testPeakDrawn(), what);
      d.rpm = 3000;
      ui::update(d, ls, ps, si, tq += 50);
      snprintf(what, sizeof(what), "ピークホールド(%s): 下がると新しいピーク(5200)で出す", nm);
      CHECK(ui::testPeakDrawn(), what);
      d.rpmOk = false;  // 回転数の信号が切れたらピークは消える(そこから取り直す)
      ui::update(d, ls, ps, si, tq += 50);
      d.rpmOk = true;
      d.rpm = 2900;
      ui::update(d, ls, ps, si, tq += 50);
      d.rpm = 1200;
      ui::update(d, ls, ps, si, tq += 50);
      snprintf(what, sizeof(what), "ピークホールド(%s): PEAK_MIN_RPM 未満のピーク(2900)は、下がっても出さない", nm);
      CHECK(!ui::testPeakDrawn(), what);
      gfx::dirtyClear();
    }
    // サーキット: 三角形の所のマスが点いていても出す(2.4.0。ピーク 5000 = 10 マス目、今 5000 で 10 マス目まで点いている)
    {
      uint32_t tq = far + 500000u;
      ui::testShow(ui::SC_DRIVE);
      ui::testPeakReset();
      d.rpm = 2000;
      ui::update(d, ls, ps, si, tq);
      ui::update(d, ls, ps, si, tq += 5000);
      d.rpm = 5000;
      ui::update(d, ls, ps, si, tq += 50);
      CHECK(ui::testPeakDrawn(), "ピークホールド(サーキット): 三角形の所のマスが点いていても出す(2.4.0)");
      const uint16_t PEAKC = gfx::rgb(0xFFD000);
      CHECK(fb[15 * gfx::W + 233] != PEAKC, "ピークホールド(サーキット): シフト点の手前では真上に三角形は無い");
      d.rpm = set.shiftRpm;
      ui::update(d, ls, ps, si, tq += 50);
      CHECK(fb[15 * gfx::W + 233] == PEAKC, "ピークホールド(サーキット): シフト点で三角形が輪の真上(天井)まで行く(2.4.3)");
      d.rpm = 3000;
      ui::update(d, ls, ps, si, tq += 50);
      CHECK(fb[15 * gfx::W + 233] == PEAKC, "ピークホールド(サーキット): 下がっても真上に残る");
      {
        static uint16_t fbS[gfx::W * gfx::H];
        memcpy(fbS, fb, sizeof(fbS));
        ui::testRedraw();
        ui::update(d, ls, ps, si, tq);
        CHECK(memcmp(fbS, fb, sizeof(fbS)) == 0, "ピークホールド(サーキット): 点いたマスの上の三角形も、最初から描いた絵と同じ");
      }
      gfx::dirtyClear();
    }
  }

  // ---------------- 25. 2.4.0: タコは回転数がシフト点以上の間、目盛線・数字・針を全部赤(シフト点 − TACH_ALL_RED_HYST_RPM 未満で戻す)
  {
    Signals d;
    memset(&d, 0, sizeof(d));
    d.linkUp = d.rpmOk = d.waterOk = d.fuelOk = d.gearOk = d.speedOk = d.throttleOk = d.boostOk = true;
    d.waterC = 85; d.fuelPct = 60; d.gear = '3'; d.boostBar = 1.2f;
    const uint16_t TICK_R = gfx::rgb(0xE5281E), DIAL_R = gfx::rgb(0xFF3B30), NEEDLE_R = gfx::rgb(0xFF2A2A);
    auto countAll = [&](uint16_t c) {
      int n = 0;
      for (int i = 0; i < gfx::W * gfx::H; i++) n += fb[i] == c;
      return n;
    };
    const int sp = set.shiftRpm;
    uint32_t tq = 2100000000u;
    d.rpm = sp - 300;
    ui::testShow(ui::SC_BOOST);
    ui::testPeakReset();
    ui::update(d, ls, ps, si, tq);
    ui::update(d, ls, ps, si, tq += 20);
    CHECK(!ui::testTachRed() && countAll(TICK_R) == 0 && countAll(DIAL_R) == 0 && countAll(NEEDLE_R) == 0, "タコ: シフト点の手前はふだんの色");
    gfx::dirtyClear();
    d.rpm = sp;
    ui::update(d, ls, ps, si, tq += 20);
    CHECK(ui::testTachRed() && countAll(TICK_R) > 300 && countAll(DIAL_R) > 300 && countAll(NEEDLE_R) > 300, "タコ: シフト点で目盛線・数字・針が全部赤");
    CHECK(!ui::pageFrame(), "タコ: 赤に替えた回(全体の描き直し)は FPS に数えない");
    {
      static uint16_t fbS[gfx::W * gfx::H];
      d.rpm = sp + 500;
      ui::update(d, ls, ps, si, tq += 20);
      memcpy(fbS, fb, sizeof(fbS));
      ui::testRedraw();
      ui::update(d, ls, ps, si, tq);
      CHECK(memcmp(fbS, fb, sizeof(fbS)) == 0, "タコ: 赤の間に少しずつ描いた絵 = 最初から描いた絵");
    }
    d.rpm = sp - TACH_ALL_RED_HYST_RPM;
    gfx::dirtyClear();
    ui::update(d, ls, ps, si, tq += 20);
    CHECK(ui::testTachRed() && countAll(TICK_R) > 300 && ui::pageFrame(), "タコ: シフト点 − TACH_ALL_RED_HYST_RPM ではまだ赤(境目で描き直しを繰り返さない)");
    d.rpm = sp - TACH_ALL_RED_HYST_RPM - 1;
    ui::update(d, ls, ps, si, tq += 20);
    CHECK(!ui::testTachRed() && countAll(TICK_R) == 0 && countAll(NEEDLE_R) == 0, "タコ: さらに下がったら元の色に戻す");
    // サーキットを見ている間も追っている: サーキットでシフト点を超えてからタコへ移ると、最初から赤
    ui::testShow(ui::SC_DRIVE);
    d.rpm = sp + 100;
    ui::update(d, ls, ps, si, tq += 20);
    CHECK(ui::testTachRed(), "サーキットの間も、シフト点以上なら赤の状態");
    ui::testShow(ui::SC_BOOST);
    ui::update(d, ls, ps, si, tq += 20);
    CHECK(countAll(TICK_R) > 300 && countAll(NEEDLE_R) > 300, "サーキットからタコへ移ると、最初から赤で描く");
    d.rpmOk = false;
    ui::update(d, ls, ps, si, tq += 20);
    CHECK(!ui::testTachRed() && countAll(TICK_R) == 0, "回転数の信号が切れたら元の色");
    gfx::dirtyClear();
  }

  // ---------------- 26. 2.5.0: データ画面の外周の回転数の輪(本人「実際の画面にも回転数のグラフを追加したい」「アニメーションと同じ幅で」)。
  //                  真上から時計回りに 0〜DATA_RING_MAX_RPM、色はサーキットの輪と同じ決まり。少しずつ描いた絵 = 最初から描いた絵
  {
    Signals d;
    memset(&d, 0, sizeof(d));
    d.linkUp = d.rpmOk = d.waterOk = d.fuelOk = d.gearOk = d.speedOk = d.throttleOk = true;
    d.waterC = 85; d.fuelPct = 60; d.gear = '3';
    const uint16_t TRACK = gfx::rgb(0x1A2433), GREEN = gfx::rgb(0x22E36A), RED = gfx::rgb(0xFF2A2A), GREY = gfx::rgb(0x8D97AD);
    const int sp = set.shiftRpm;
    auto ringPx = [&](float deg) {  // 輪の真ん中(半径 227)の、真上から時計回り deg 度の画素
      const float a = (deg - 90.0f) * 3.14159265f / 180.0f;
      const int x = (int)floorf(233.0f + 227.0f * cosf(a)), y = (int)floorf(233.0f + 227.0f * sinf(a));
      return fb[y * gfx::W + x];
    };
    auto degOf = [&](int rpm) { return 360.0f * (float)rpm / (float)DATA_RING_MAX_RPM; };
    uint32_t tr = 2150000000u;
    ui::testShow(ui::SC_DATA);
    const int rg = sp + RING_GREEN_OFFSET + 200;  // 緑(混ぜない所)
    d.rpm = rg;
    ui::update(d, ls, ps, si, tr);
    CHECK(ringPx(3) == GREEN && ringPx(degOf(rg) - 3) == GREEN && ringPx(degOf(rg) + 3) == TRACK && ringPx(357) == TRACK,
          "データの輪: 真上から時計回りに回転数のぶんだけ点き(緑の範囲の回転数は緑)、残りは輪の地");
    CHECK(ringPx(180) == (degOf(rg) > 180 ? GREEN : TRACK), "データの輪: 一周 = DATA_RING_MAX_RPM");
    gfx::dirtyClear();
    d.rpm = rg + 40;
    ui::update(d, ls, ps, si, tr += 20);
    CHECK(gfx::dirtyCount() > 0 && gfx::dirtyPixels() < 6000, "データの輪: 回転数が少し動いたら、先の所だけ描き直す");
    gfx::dirtyClear();
    d.rpm = sp + RING_RED_OFFSET + 100;
    ui::update(d, ls, ps, si, tr += 20);
    CHECK(ringPx(3) == RED && ringPx(degOf(d.rpm) - 3) == RED, "データの輪: 赤の範囲では点いている所が全部赤(色が変わったら全部塗り直す)");
    d.rpm = sp + RING_BLUE_OFFSET - RING_BLEND_RPM - 100;
    ui::update(d, ls, ps, si, tr += 20);
    CHECK(ringPx(3) == GREY && ringPx(degOf(d.rpm) + 3) == TRACK && ringPx(200) == TRACK, "データの輪: 下がったら灰に戻り、先から先は地に戻る");
    d.rpmOk = false;
    ui::update(d, ls, ps, si, tr += 20);
    CHECK(ringPx(3) == TRACK && ringPx(90) == TRACK, "データの輪: 回転数の信号がなければ全部が地");
    // 回転数が動いても(色が混ざる所・振り切り・信号なしも)、少しずつ描いた絵 = 同じ時刻に最初から描いた絵
    static uint16_t fbR[gfx::W * gfx::H];
    const int seq[] = { 900, 1000, 2650, 2700, 2900, 3100, 3700, 3900, 5000, 5300, 5500, 6100, 8000, 9000, 7999, 3000, 0, -1, 4000, 4011, 3990, 6300, 1 };
    bool same = true;
    for (int i = 0; i < (int)(sizeof(seq) / sizeof(seq[0])); i++) {
      d.rpmOk = seq[i] >= 0;
      d.rpm = seq[i] < 0 ? 0 : seq[i];
      ui::update(d, ls, ps, si, tr += 20);
      memcpy(fbR, fb, sizeof(fbR));
      ui::testRedraw();
      ui::update(d, ls, ps, si, tr);
      if (memcmp(fbR, fb, sizeof(fbR)) != 0) {
        same = false;
        printf("  違い: %d rpm\n", seq[i]);
      }
      gfx::dirtyClear();
    }
    CHECK(same, "データの輪: 少しずつ描いた絵 = 同じ時刻に最初から描いた絵(色が変わる所・振り切り・信号なしも)");
    // 振り切りで全周が赤。中の絵(字がいちばん長くなる値・SIM の印)は輪に掛からない
    LinkStats ls2 = ls;
    SysInfo si2 = si;
    ls2.infoValid = true; ls2.info.proto_ver = METER_LINK_PROTO_VER; ls2.info.can_fps[0] = 9999; ls2.info.can_fps[1] = 9999;
    si2.rc.enabled = true; si2.rc.gpsOk = true; si2.rc.fixType = 3; si2.rc.numSV = 99; si2.rc.pvtPerSec = 25; si2.rc.imuOk = true;
    si2.rc.gLon = -1.99f; si2.rc.gLat = -1.99f;
    d.rpmOk = true; d.rpm = DATA_RING_MAX_RPM; d.sim = true; d.scene = 7; d.waterC = 118; d.fuelPct = 100;
    trip::setTimes(359999000u, 359999000u);
    ui::testRedraw();
    ui::update(d, ls2, ps, si2, tr += 20);
    int bad = 0, inner = 0;
    for (int y = 0; y < gfx::H; y++) {
      for (int x = 0; x < gfx::W; x++) {
        const float r = hypotf(x + 0.5f - 233.0f, y + 0.5f - 233.0f);
        if (r >= DATA_RING_R_IN + 1.0f && r <= DATA_RING_R_OUT - 1.0f && fb[y * gfx::W + x] != RED) bad++;
        if (r > 217.5f && r < DATA_RING_R_IN - 0.5f && fb[y * gfx::W + x] != 0) inner++;
      }
    }
    CHECK(bad == 0, "データの輪: 振り切りで全周が赤。字がいちばん長くなる値と SIM の印でも、中の絵は輪に掛からない");
    CHECK(inner == 0, "データの輪: 中の絵は半径 217 まで(輪の内側の縁との間はあいている)");
    trip::setTimes(0, 0);
    gfx::dirtyClear();
  }

  // ---------------- 27. 2.6.0: CAN確認の「前後で比べる」(本人 10/24「長く変化する燃料のような CAN ID を絞り込めないか」)。
  //                  前を記録 → 電源を切って給油 → 入れ直して後を記録 で、ゆっくり変わる値のバイトを見つける(config.h の CAN_PP_*)。
  //                  2.7.0: 燃料は C-CAN 0A18A000 b5(%)。前はエンジンを切る前に記録する
  {
    Signals d;
    memset(&d, 0, sizeof(d));
    d.linkUp = d.rpmOk = d.waterOk = d.fuelOk = d.gearOk = d.speedOk = d.throttleOk = true;
    d.rpm = 800; d.waterC = 85; d.fuelPct = 60; d.gear = 'N';
    // 3 つの ID を 10 ms ごとに流す。燃料 = 0A18A000 b5(静か)、カウンター = 0A18A001 b0・b1(いつも変わる)、
    // 0x123: b0 = 静かで、あとで少しだけ変わる値(差 3)、b3 = 電源を入れ直すと 0 に戻る値
    auto feed = [&](uint32_t from, uint32_t to, uint8_t fuel, uint8_t small, uint8_t reset) {
      for (uint32_t tt = from; tt < to; tt += 10) {
        const uint8_t a[8] = { 0x20, 0x0D, 0x40, 0x7B, 0x00, fuel, 0x20, 0x3D };
        const uint8_t b[8] = { (uint8_t)(tt / 10), (uint8_t)(tt / 7), 1, 2, 3, 4, 5, 6 };
        const uint8_t c[4] = { small, 7, 8, reset };
        cantab::onFrame(0, 1, 0x0A18A000, 8, a, tt);
        cantab::onFrame(0, 1, 0x0A18A001, 8, b, tt);
        cantab::onFrame(0, 0, 0x123, 4, c, tt);
        cantab::tick(tt);
      }
    };
    auto countIn = [&](uint16_t col, int x0, int y0, int x1, int y1) {
      int n = 0;
      for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++) n += fb[y * gfx::W + x] == col;
      return n;
    };
    const uint16_t ACC = gfx::rgb(0x5FD0FF), WAITC = gfx::rgb(0x5A606A);
    char val[48];
    uint32_t tp = 2200000000u;
    g_hostCanPreN = 0;
    cantab::begin();  // 電源を入れたところ(表も前の記録も空)
    CHECK(cantab::ppWaitMs(tp) == -1 && !cantab::ppHasPre() && !cantab::ppHasResult(), "前後で比べる: 何も受けていない間は -1(帯に「受信なし」)");
    ui::testShow(ui::SC_SET_GRP, 7, 0);
    ui::testTouch(tp);
    ui::update(d, ls, ps, si, tp);
    holdOpen(d, tp);
    CHECK(ui::screen() == ui::SC_CAN_PP, "CAN の最後のページ「前後で比べる」の下「長押しで開く」で前後で比べるの画面");
    CHECK(ui::testInfoItem(0, 7, ls, ps, si, tp, val, sizeof(val)) == 0 && strcmp(val, "記録なし") == 0, "ページの値: 前の記録が無ければ「記録なし」");
    const uint32_t t0 = tp;
    feed(t0, t0 + 5000, 31, 50, 43);
    tp = t0 + 5000;
    upd(d, tp);
    ui::tap(BOTX, BOTY, tp += 10); upd(d, tp);
    CHECK(!cantab::ppHasPre() && countIn(WAITC, 100, 380, 366, 466) > 300 && countIn(ACC, 100, 380, 366, 466) == 0,
          "受信が始まって CAN_PP_SETTLE_MS たつまでは、下の帯に残りの秒を灰で出し、押しても記録しない");
    feed(t0 + 5000, t0 + 16000, 31, 50, 43);
    tp = t0 + 16000;
    upd(d, tp);
    CHECK(cantab::ppWaitMs(tp) == 0 && countIn(ACC, 100, 380, 366, 466) > 1000, "CAN_PP_SETTLE_MS たったら下の帯は「前を記録」(水色)");
    const int saves0 = g_hostCanPreSaves;
    ui::tap(BOTX, BOTY, tp); upd(d, tp);
    CHECK(cantab::ppHasPre() && !cantab::ppHasResult() && g_hostCanPreSaves == saves0 + 1 && g_hostCanPreN > 0, "下「前を記録」で前を記録して保存する");
    CHECK(ui::testInfoItem(0, 7, ls, ps, si, tp, val, sizeof(val)) == 0 && strcmp(val, "前を記録した") == 0, "ページの値: 「前を記録した」");
    // 電源を切って給油(燃料 31 → 100 %)。入れ直すと表は空になり、前の記録は保存から戻る
    g_hostCanPreSaves = 0;
    cantab::begin();
    CHECK(cantab::ppHasPre() && !cantab::ppHasResult(), "電源を入れ直しても前の記録は残る(保存から戻す)");
    const uint32_t t2 = t0 + 300000;
    feed(t2, t2 + 4000, 0, 50, 0);             // 入れた直後(エンジンをかけるまで)は燃料の値が 0
    feed(t2 + 4000, t2 + 16000, 100, 53, 0);   // 0x123 b0 は 50 → 53(差 3。候補にしない)、b3 は 43 → 0
    tp = t2 + 10000;
    ui::testShow(ui::SC_CAN_PP);
    upd(d, tp);
    ui::tap(BOTX, BOTY, tp += 10); upd(d, tp);
    CHECK(!cantab::ppHasResult() && countIn(WAITC, 100, 380, 366, 466) > 300, "後も、受信が始まって CAN_PP_SETTLE_MS たつまでは記録しない");
    tp = t2 + 16000;
    upd(d, tp);
    ui::tap(BOTX, BOTY, tp); upd(d, tp);
    cantab::PpResult r[CAN_PP_MAX];
    int up = -1, down = -1;
    const int m = cantab::ppResults(r, CAN_PP_MAX, &up, &down);
    CHECK(cantab::ppHasResult() && m == 2 && up == 1 && down == 1 && g_hostCanPreSaves == 0,
          "下「後を記録」: 候補 = 前も後も静かで差が CAN_PP_MIN_DIFF 以上(カウンターと差 3 のバイトは外れる。前の記録は書き直さない)");
    cantab::View v0, v1;
    CHECK(m == 2 && cantab::getView(r[0].entry, &v0) && v0.id == 0x0A18A000 && r[0].byte == 5 && r[0].pre == 31 && r[0].post == 100,
          "増えたものが先: 0A18A000 b5 31 → 100(+69)");
    CHECK(m == 2 && cantab::getView(r[1].entry, &v1) && v1.id == 0x123 && r[1].byte == 3 && r[1].pre == 43 && r[1].post == 0, "減ったものがあと: 123 b3 43 → 0");
    CHECK(ui::testInfoItem(0, 7, ls, ps, si, tp, val, sizeof(val)) == 0 && strcmp(val, "候補 2 件") == 0, "ページの値: 「候補 2 件」");
    ui::tap(RX, MY, tp += 100); upd(d, tp);
    CHECK(ui::screen() == ui::SC_CAN_ID && countIn(ACC, 60, 210, 406, 310) > 100, "右半分「一覧」で 1 バイトずつ(候補のバイトは水色)");
    next(tp);
    CHECK(ui::screen() == ui::SC_CAN_ID, "一覧: 送りで次の候補");
    holdOpen(d, tp);
    CHECK(ui::screen() == ui::SC_CAN_DETAIL, "一覧: 下「長押しで開く」で詳細");
    ui::tap(TOPX, TOPY, tp += 100); upd(d, tp);
    ui::tap(TOPX, TOPY, tp += 100); upd(d, tp);
    CHECK(ui::screen() == ui::SC_CAN_PP, "詳細 → 一覧 → 上「‹ 戻る」で前後で比べるの画面");
    ui::tap(LX, MY, tp += 100); upd(d, tp);
    CHECK(ui::screen() == ui::SC_CAN_REDO, "左半分「やり直す」で確認");
    ui::tap(TOPX, TOPY, tp += 100); upd(d, tp);
    CHECK(ui::screen() == ui::SC_CAN_PP && cantab::ppHasPre() && cantab::ppHasResult(), "確認: 上「‹ 戻る」は消さない");
    ui::tap(LX, MY, tp += 100); upd(d, tp);
    ui::tap(BOTX, BOTY, tp += 100); upd(d, tp);
    CHECK(ui::screen() == ui::SC_CAN_PP && !cantab::ppHasPre() && !cantab::ppHasResult() && g_hostCanPreN == 0, "確認: 下「やり直す」で前の記録(保存も)と候補を消す");
    CHECK(cantab::recordCount() == 0, "前後で比べるのやり直しは、変化を記録の記録には触らない");
    ui::tap(TOPX, TOPY, tp += 100); upd(d, tp);
    CHECK(ui::screen() == ui::SC_SET_GRP && ui::testGroup() == 0 && ui::testPage() == 7, "上「‹ 戻る」で CAN の「前後で比べる」のページ");
    // 10/22 の給油の実車ログ(../logs/can_fuel.txt があれば): 給油所で止まって、エンジンを切る前(712.0 秒に切った)の 711 秒に前を記録 →
    // 電源を切って給油 → 846.9 秒に入れ直し(すぐエンジンをかけた)、862 秒(15 秒後)に後を記録。燃料(0A18A000 b5)が増えた側の
    // 1 番目に入ること(B-CAN 06214000 b5 にも同じ値が流れていて、差が同じなので 2 番目までのどちらか)。2.6.0 の試験は 715 秒に前を
    // 記録していて、前の 10 秒にエンジンを切った所(b5 = 0)が入ったので、燃料が候補から外れていた
    if (FILE *fp = fopen("../logs/can_fuel.txt", "r")) {
      const uint32_t PRE = 711000, IG_ON = 846900, POST = 862000;
      g_hostCanPreN = 0;
      cantab::begin();
      bool preOk = false, restarted = false;
      int post = -1;
      uint32_t nextTick = 0;
      bool ticking = false;
      auto tickTo = [&](uint32_t tm) {
        while (nextTick <= tm) {
          cantab::tick(nextTick);
          nextTick += CAN_HIST_INTERVAL_MS;
        }
      };
      char line[256];
      while (post < 0 && fgets(line, sizeof(line), fp)) {
        char bus;
        unsigned long tml;
        unsigned id, len, b[8];
        if (sscanf(line, "L %c %lu %x %u %x %x %x %x %x %x %x %x", &bus, &tml, &id, &len, &b[0], &b[1], &b[2], &b[3], &b[4], &b[5], &b[6], &b[7]) < 4 || len > 8) continue;
        const uint32_t tm = (uint32_t)tml;
        if (!ticking) {
          ticking = true;
          nextTick = tm;
        }
        if (!preOk && tm >= PRE) {
          tickTo(PRE);
          preOk = cantab::ppRecordPre(PRE);
        }
        if (preOk && !restarted) {
          if (tm < IG_ON) continue;  // 電源を切っている間(表示器は消えている)
          cantab::begin();           // 入れ直した(前の記録は保存から戻る)
          restarted = true;
          nextTick = tm;
        }
        if (restarted && tm >= POST) {
          tickTo(POST);
          post = cantab::ppRecordPost(POST);
          break;
        }
        tickTo(tm);
        uint8_t dd[8] = { 0 };
        for (unsigned k = 0; k < len; k++) dd[k] = (uint8_t)b[k];
        cantab::onFrame(bus == 'B' ? 1 : 0, 1, id, (uint8_t)len, dd, tm);
      }
      fclose(fp);
      cantab::PpResult rr[CAN_PP_MAX];
      int lu = 0, ld = 0;
      const int lm = post > 0 ? cantab::ppResults(rr, CAN_PP_MAX, &lu, &ld) : 0;
      int fuelAt = -1;
      for (int i = 0; i < lm; i++) {
        cantab::View vv;
        if (cantab::getView(rr[i].entry, &vv) && vv.id == 0x0A18A000 && vv.bus == 0 && rr[i].byte == 5) fuelAt = i;
      }
      printf("  実車ログ(10/22 の給油): 前 711 秒・後 862 秒で 候補 %d 件(増えた %d、減った %d)。燃料 0A18A000 b5 は %d 番目", post, lu, ld, fuelAt + 1);
      if (fuelAt >= 0) printf("(%d → %d)", rr[fuelAt].pre, rr[fuelAt].post);
      printf("\n");
      CHECK(preOk && post > 0 && fuelAt >= 0 && fuelAt < 2 && fuelAt < lu && rr[fuelAt].pre == 31 && rr[fuelAt].post == 100 && lu <= 3,
            "実車ログ(給油): 燃料 0A18A000 b5 31 → 100 が、増えた側(3 件まで)の 2 番目までに入る");
    }
    g_hostCanPreN = 0;
    cantab::begin();
    gfx::dirtyClear();
  }

  // ---------------- 28. 2.7.0: 押した帯の反転(本人「can確認画面などでボタンを押した時に押したことがわかりにくいので、反転表示を入れてください」)。
  //                  上の帯とタップで動く下の帯は、タッチが決まってから離すまで反転。下の帯の操作は、離してから PRESS_FLASH_MS 反転を見せてから
  {
    Signals d;
    memset(&d, 0, sizeof(d));
    d.linkUp = d.rpmOk = true;
    d.rpm = 800;
    auto countIn = [&](uint16_t col, int x0, int y0, int x1, int y1) {
      int n = 0;
      for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++) n += fb[y * gfx::W + x] == col;
      return n;
    };
    const uint16_t ACC = gfx::rgb(0x5FD0FF), WHITE = gfx::rgb(0xFFFFFF);
    auto bot = [&](uint16_t c) { return countIn(c, 100, 380, 366, 466); };
    auto top = [&](uint16_t c) { return countIn(c, 100, 0, 366, 120); };
    uint32_t tq = 2400000000u;
    g_hostCanPreN = 0;
    cantab::begin();
    ui::testShow(ui::SC_CAN);
    upd(d, tq);
    const int rec0 = cantab::recordCount();
    CHECK(bot(ACC) > 300 && bot(ACC) < 6000 && top(ACC) > 300 && top(ACC) < 6000, "CAN確認: ふだんの帯は暗い帯に水色の字");
    ui::touchDown(BOTX, BOTY, tq += 100);
    ui::update(d, ls, ps, si, tq + TOUCH_CONFIRM_MS - 30);
    CHECK(bot(ACC) < 6000, "決まる前(TOUCH_CONFIRM_MS より前)は反転しない");
    ui::update(d, ls, ps, si, tq + TOUCH_CONFIRM_MS + 10);
    CHECK(bot(ACC) > 10000 && top(ACC) < 6000, "下「変化を記録」: 押して決まったら反転(水色の帯。上の帯は変えない)");
    ui::touchUp(tq += 180);
    ui::update(d, ls, ps, si, tq);
    ui::update(d, ls, ps, si, tq + PRESS_FLASH_MS - 10);
    CHECK(cantab::recordCount() == rec0 && bot(ACC) > 10000, "離してから PRESS_FLASH_MS は反転のまま(記録はまだ)");
    ui::update(d, ls, ps, si, tq + PRESS_FLASH_MS);
    ui::update(d, ls, ps, si, tq + PRESS_FLASH_MS + 60);
    gfx::dirtyClear();
    CHECK(cantab::recordCount() == rec0 + 1 && bot(ACC) < 6000 && ui::screen() == ui::SC_CAN, "PRESS_FLASH_MS で記録して、帯は元に戻る");
    // 反転を見せている間(PRESS_FLASH_MS)のタッチは何もしない(「決定」の点滅の間と同じ)
    ui::touchDown(BOTX, BOTY, tq += 200);
    ui::update(d, ls, ps, si, tq + TOUCH_CONFIRM_MS + 10);
    ui::touchUp(tq += 150);
    ui::touchDown(BOTX, BOTY, tq + 50);
    ui::update(d, ls, ps, si, tq + 50 + TOUCH_CONFIRM_MS + 10);
    ui::touchUp(tq + 200);
    ui::update(d, ls, ps, si, tq + PRESS_FLASH_MS);
    ui::update(d, ls, ps, si, tq + 200 + PRESS_FLASH_MS + 10);
    ui::update(d, ls, ps, si, tq + 200 + PRESS_FLASH_MS + 70);
    gfx::dirtyClear();
    CHECK(cantab::recordCount() == rec0 + 2 && bot(ACC) < 6000, "反転を見せている間に押したのは数えない(記録は 1 回)");
    tq += 200 + PRESS_FLASH_MS + 100;
    // 動かしたら反転をやめ、離しても記録しない
    tq += PRESS_FLASH_MS + 100;
    ui::touchDown(BOTX, BOTY, tq += 100);
    ui::update(d, ls, ps, si, tq + TOUCH_CONFIRM_MS + 10);
    ui::touchMove(BOTX, BOTY - HOLD_CANCEL_PX - 10, tq + TOUCH_CONFIRM_MS + 20);
    ui::update(d, ls, ps, si, tq + TOUCH_CONFIRM_MS + 30);
    CHECK(bot(ACC) < 6000, "指が HOLD_CANCEL_PX 以上動いたら反転をやめる");
    ui::touchUp(tq += 200);
    ui::update(d, ls, ps, si, tq);
    ui::update(d, ls, ps, si, tq + PRESS_FLASH_MS + 10);
    CHECK(cantab::recordCount() == rec0 + 2, "動かして離したら記録しない");
    // 上の帯: 押している間は反転、離したらすぐ 1 つ上へ
    tq += PRESS_FLASH_MS + 100;
    ui::touchDown(TOPX, TOPY, tq += 100);
    ui::update(d, ls, ps, si, tq + TOUCH_CONFIRM_MS + 10);
    CHECK(top(ACC) > 10000 && bot(ACC) < 6000, "上の帯「‹ 戻る」: 押して決まったら反転(下の帯は変えない)");
    ui::touchUp(tq += 150);
    CHECK(ui::screen() == ui::SC_SET_GRP, "上の帯は離したらすぐ 1 つ上へ");
    upd(d, tq);
    CHECK(top(ACC) < 6000, "移った画面の上の帯は反転しない");
    // 設定の中(長押しで開く画面): 上の帯は押している間は反転、長押しの表示が出たらやめる。矢印では反転しない
    ui::testShow(ui::SC_SET_LIST, 0);
    upd(d, tq += 100);
    ui::touchDown(RX, MY, tq += 100);
    ui::update(d, ls, ps, si, tq + TOUCH_CONFIRM_MS + 10);
    CHECK(top(ACC) < 6000 && bot(ACC) < 6000, "矢印(左右の半分)を押しても帯は反転しない");
    ui::touchUp(tq += 150);
    upd(d, tq);
    CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 1, "矢印のタップは今までどおり(次)");
    ui::touchDown(TOPX, TOPY, tq += 100);
    ui::update(d, ls, ps, si, tq + TOUCH_CONFIRM_MS + 10);
    CHECK(top(ACC) > 10000 && top(gfx::rgb(0x000000)) > 100 && bot(ACC) < 6000, "設定の上の帯「‹ 設定」: 押して決まったら反転(版の字も地の色)");
    ui::update(d, ls, ps, si, tq + HOLD_SHOW_MS + 10);
    CHECK(ui::testHolding() && top(ACC) < 6000 && bot(ACC) > 10000, "長押しの表示が出たら上の帯の反転をやめる(下の帯「長押しで開く」が水色になり外周)");
    ui::touchUp(tq += HOLD_SHOW_MS + 100);
    upd(d, tq);
    CHECK(ui::screen() == ui::SC_SET_LIST && ui::testSetItem() == 1 && top(ACC) < 6000 && bot(ACC) < 6000, "そのまま離したら何もしない(戻らない。帯は元に戻る)");
    // 「決定」(変更の画面)は今までどおり: 押しても反転しない(ふだんが水色の帯)。離したらすぐ点滅
    ui::testShow(ui::SC_SET_EDIT, 0);
    upd(d, tq += 100);
    const int accEdit = bot(ACC);
    ui::touchDown(BOTX, BOTY, tq += 100);
    ui::update(d, ls, ps, si, tq + TOUCH_CONFIRM_MS + 10);
    CHECK(accEdit > 10000 && bot(ACC) == accEdit, "「決定」は押しても反転しない(ふだんが水色の帯)");
    ui::touchUp(tq += 150);
    ui::update(d, ls, ps, si, tq);
    CHECK(ui::testFlashing(), "「決定」は離したらすぐ点滅(今までどおり)");
    ui::update(d, ls, ps, si, tq + 2 * SAVED_FLASH_COUNT * SAVED_FLASH_MS + 10);
    upd(d, tq += 2 * SAVED_FLASH_COUNT * SAVED_FLASH_MS + 100);
    CHECK(ui::screen() == ui::SC_SET_LIST, "点滅が終わったら項目の画面へ");
    // 赤い帯(CAN確認の「やり直す」): 白い帯に赤の字。PRESS_FLASH_MS 見せてから消して戻る
    ui::testShow(ui::SC_CAN);
    upd(d, tq += 100);
    ui::tap(LX, MY, tq += 100);
    upd(d, tq);
    CHECK(ui::screen() == ui::SC_CAN_REDO && bot(WHITE) < 6000, "CAN確認の左「やり直す」で確認の画面(赤い帯)");
    ui::touchDown(BOTX, BOTY, tq += 100);
    ui::update(d, ls, ps, si, tq + TOUCH_CONFIRM_MS + 10);
    CHECK(bot(WHITE) > 10000, "赤い帯「やり直す」: 押して決まったら反転(白い帯)");
    ui::touchUp(tq += 150);
    ui::update(d, ls, ps, si, tq + PRESS_FLASH_MS - 10);
    CHECK(ui::screen() == ui::SC_CAN_REDO && cantab::recordCount() == rec0 + 2 && bot(WHITE) > 10000, "離して PRESS_FLASH_MS までは消さない(反転のまま)");
    ui::update(d, ls, ps, si, tq + PRESS_FLASH_MS);
    ui::update(d, ls, ps, si, tq + PRESS_FLASH_MS + 60);
    gfx::dirtyClear();
    CHECK(ui::screen() == ui::SC_CAN && cantab::recordCount() == 0 && bot(ACC) < 6000, "PRESS_FLASH_MS で消して CAN確認へ(移った画面の下の帯は反転しない)");
    // 前後で比べる: 記録できない間(受信なし・待ち)は押しても反転しない。記録できるときは反転を見せてから記録する
    ui::testShow(ui::SC_CAN_PP);
    upd(d, tq += 100);
    ui::touchDown(BOTX, BOTY, tq += 100);
    ui::update(d, ls, ps, si, tq + TOUCH_CONFIRM_MS + 10);
    CHECK(bot(ACC) < 6000 && bot(WHITE) < 6000, "前後で比べる: 受信なしの間は押しても反転しない");
    ui::touchUp(tq += 150);
    upd(d, tq);
    const uint8_t fa[8] = { 0x20, 0x0D, 0x40, 0x7B, 0x00, 31, 0x20, 0x3D };
    for (uint32_t tt = tq; tt < tq + CAN_PP_SETTLE_MS + 1000; tt += 20) {
      cantab::onFrame(0, 1, 0x0A18A000, 8, fa, tt);
      cantab::tick(tt);
    }
    tq += CAN_PP_SETTLE_MS + 1000;
    upd(d, tq);
    ui::touchDown(BOTX, BOTY, tq += 10);
    ui::update(d, ls, ps, si, tq + TOUCH_CONFIRM_MS + 10);
    CHECK(cantab::ppWaitMs(tq + TOUCH_CONFIRM_MS + 10) == 0 && bot(ACC) > 10000, "前後で比べる: 記録できるときは「前を記録」が反転");
    ui::touchUp(tq += 150);
    ui::update(d, ls, ps, si, tq + PRESS_FLASH_MS - 10);
    CHECK(!cantab::ppHasPre() && bot(ACC) > 10000, "離して PRESS_FLASH_MS までは前を記録しない(反転した「前を記録」のまま)");
    ui::update(d, ls, ps, si, tq + PRESS_FLASH_MS);
    ui::update(d, ls, ps, si, tq + PRESS_FLASH_MS + 60);
    gfx::dirtyClear();
    CHECK(cantab::ppHasPre() && bot(ACC) < 6000 && ui::screen() == ui::SC_CAN_PP, "PRESS_FLASH_MS で前を記録し、帯は元に戻る(「後を記録」)");
    g_hostCanPreN = 0;
    cantab::begin();
    ui::testShow(ui::SC_DRIVE);
    gfx::dirtyClear();
  }

  printf("単体試験: %d 個 OK、%d 個 NG、無かった文字 %u\n", passes, fails, (unsigned)gfx::missingGlyphs());
  printf("%s\n", (fails == 0 && gfx::missingGlyphs() == 0) ? "UNIT PASS" : "UNIT FAIL");
  return fails ? 1 : 0;
}

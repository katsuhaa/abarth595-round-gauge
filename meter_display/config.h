// config.h - 595 メーター表示器の設定値
//
// 画面の配置と色は ui_spec.md(キャンバス版 23)のとおり ui.cpp に書いてある。
// ここには、仕様書で【仮】【未定】になっている値と、基板まわりの値をまとめた。
// 【未定→仮】は、仕様書で未定だった所に今回仮に置いた値。
#pragma once

// ---------------------------------------------------------------- 版(2026-10-23 から git で管理。上げるたびにコミットする)
// 形式は「大.中.小」。小 = 直しだけ、中 = 機能・画面の追加や変更、大 = 送り元(c6_can_link)とのデータ形式が変わるとき。
// オープニング(自己診断の下)と設定の上の帯に出す。送り元の版は 受信の状態 の「送り元」に出る
// 2.0.0(2026-10-23): 値への換算・ギアの推定・燃料のなましを c6_can_link から移した(c6_can_link 2.0.0 と組で使う。1.x の c6_can_link でも動く)
// 2.0.1(2026-10-23): タッチの読み取りのあとに 0xAB を書くのをやめた(長押しが「離した」で切れる)。受信の状態の「描画」を、
//   サーキット・タコで 1 回描いて送る時間の最長にした(前は直近 1 秒に送った回数で、受信の状態の画面では必ず 2 になっていた)
// 2.1.0(2026-10-23): サーキット・タコ・データの操作を、左右のスワイプで画面を替え、長押しで設定に入る形にした(短いタップは何もしない)
#define METER_FW_VERSION "2.1.0"

// ---------------------------------------------------------------- 回転数の輪(仕様書 3 章)
// シフト点(設定画面で変える値)を基準に、ほかの回転数はここからの差で決める。
// 【未定→仮】シフト点を変えると、点き始め・色の切り替わり・点滅の始まりが同じ幅だけ動く。
#define RING_STEP_RPM 400           // 1 マスぶんの回転数
#define RING_R_IN 176.0f            // 輪の内側の半径(10/20: 200 → 184、10/21: → 176。輪を太くする)
#define RING_R_OUT 228.0f           // 輪の外側の半径
// 10/21: 真上と真下の切れ目にも 1 マスずつ置いて全周 26 マスにした(シフト点で全周が点く。真下は 1 マス目と、真上は 12 マス目と
// 一緒に点く)。26 マスを等間隔(360/26 = 13.85°)に並べ、マスの間のすき間はこの角度
#define RING_GAP_DEG 2.0f
#define GEAR_FONT FONT_GEAR_160     // ギアの書体(10/21: 270 → 230 → 190、10/22: 160。遠くから見えるよう弧を太くしたぶん小さく)
#define GEAR_BASELINE 289           // ギアの文字のベースライン y(文字の高さの中心が 233 に来る値。文字の大きさ × 0.35 を足す)
#define RING_IDLE_MARK 0            // 1 = 点き始めより下の回転数で一番下のマスを橙に点ける(仕様書 3.2。LED バーで両端を確かめるためのもの)。
                                    // 0 = 点けない(2026-10-18 の決定。10/18 に渡した版は Mac に入っていなかったので 10/20 に入れた)
#define RING_BLEND_RPM 400          // 色の切り替わりの手前、この幅で色を混ぜる
#define RING_BLUE_OFFSET (-2800)    // 青になる回転数 = シフト点 + この値(5800 なら 3000)
#define RING_GREEN_OFFSET (-1800)   // 緑(5800 なら 4000)
#define RING_YELLOW_OFFSET (-400)   // 黄(5800 なら 5400)
#define RING_RED_OFFSET 400         // 赤(5800 なら 6200)
// 点滅の始まり = シフト点 + この値(10/21: 輪が黄になる所と同じ RING_YELLOW_OFFSET。5800 なら 5400。10/20 の 1000 rpm 手前(4800)は
// 最初の点滅が遅すぎた)。点滅はシフト点に近づくほど速くなる。消灯の時間は速さによらず一定(BLINK_OFF_MS)で、点灯の時間だけを
// BLINK_ON_SLOW_MS から BLINK_ON_FAST_MS まで回転数に比例して縮める(2026-10-20: 遅い点滅で画面が一瞬消えて見えなくなる
// のを避けるため、消灯を長くしない)。周期は始まりで 230 ms(1 秒に 4.3 回)、シフト点以上で 130 ms(1 秒に 7.7 回)。
// 切り替えの時刻は前の切り替えから数えるので、速さが変わっても位相は飛ばない。
// 0 以上(シフト点から点滅)にすると、点灯の時間は BLINK_ON_FAST_MS だけになる。
// 10/13〜10/19 の動き(シフト点から 100 ms 点灯 / 100 ms 消灯)に戻すなら 0、100、100、100。10/20 の動きは −1000、50、250、50。
#define RING_BLINK_OFFSET RING_YELLOW_OFFSET
#define BLINK_OFF_MS 80             // 消灯の時間(一定。10/21: 50 → 80、短すぎた)。パネルの描き替えは 60 Hz(1 コマ 16.7 ms。CO5300
                                    //   データシート)なので、50 より大きく縮めると消灯が出ないことがある
#define BLINK_ON_SLOW_MS 150        // 点滅の始まりでの点灯の時間(10/21: 250 → 150。始まりから、ある程度の速さにする)
#define BLINK_ON_FAST_MS 50         // シフト点以上での点灯の時間
// 点滅の終わり = シフト点 + この値(2026-10-20: 5800 なら 6000)。これ以上では点滅せず、輪は全部点いたまま、画面の明るさを
// 設定の値ではなく最大(100 %)にする。下回ると点滅に戻り、走行画面・ブースト計以外の画面では設定の明るさに戻る
#define RING_SOLID_OFFSET 200
// 点滅のしかた(走行画面)。1 = 画面全体: 消灯の間は画面全体を消す(輪は点いたまま)。
//                         0 = 輪だけ消す(仕様書 3.4、10/8 時点)。
#define BLINK_WHOLE_SCREEN 1
// 画面全体の消し方。1 = パネルの明るさを 0 にする(画面に送らないので、境目が出ない。消灯は黒だけ)。
//                   0 = BLINK_OFF_RGB の一色を画面に送る(色を選べるが、送っている途中の境目が見えることがある)。
#define BLINK_BY_BRIGHTNESS 1
#define BLINK_OFF_RGB 0x000000      // BLINK_BY_BRIGHTNESS 0 のときの消灯の色(0xRRGGBB)。例: 赤なら 0xE3241B
#define BG_MIN_INTERVAL_MS 80       // 背景の色を塗り直す最短の間隔(点滅の間隔を守るため)

// ---------------------------------------------------------------- オープニング画面(2026-10-21。仕様書に無い)
// 電源を入れると、走行画面の前に次を見せる(ちゃんと動いているか)。タップで飛ばせる。
//   1. 輪が下から上へ 1 マスずつ点く(色は回転数と同じ決まり: 灰→青→緑→黄→赤。背景は黒のまま)。
//      同時に水温と燃料の弧(アイコンつき)も 0 から一杯まで伸びる(10/21: 弧も動かす)
//   2. 全部点いたら背景も赤にして、最大輝度で一瞬止まる
//   3. 背景が黒に戻り、輪が上から消え、弧が縮んで消えながら、中央に自己診断(CAN・GPS・IMU・タッチ)が大きく 1 行ずつ出る。
//      値は走行画面に移るまで更新し続ける(受信が始まれば「待ち」が OK に変わる)。NG があれば赤で、少し長く見せる
#define OPENING_MS 2000             // 全体の長さ(これが過ぎたら走行画面へ)。0 にするとオープニングなし。10/21: 1800 → 2000(弧が縮んでから自己診断を出すぶん)
#define OPENING_NG_EXTRA_MS 2000    // NG があるときに延ばす時間
#define OPENING_STEP_MS 40          // 輪が 1 マス点く間隔(12 マスで 0.48 秒。弧もこの時間で一杯まで伸びる)
#define OPENING_HOLD_MS 200         // 全部点いて止まる時間(前半は最大輝度)
#define OPENING_OFF_STEP_MS 20      // 輪が 1 マス消える間隔(弧もこの 12 倍の時間で縮む)
#define OPENING_ROW_MS 60           // 自己診断の行が 1 行出る間隔

// ---------------------------------------------------------------- 水温と燃料(仕様書 5 章)
// 走行画面では、回転数の輪の内側の円弧で出す(仕様書 5.4・5.5 の横バーから変えた。2026-10-14)。
// 水温は上の弧(左端から時計回りに伸びる)、燃料は下の弧(左端から反時計回りに伸びる)。
// 2026-10-22: 実走行で「細くて見えない」ので太さ 18 → 40 にした(遠目の案 A)。長さは 140° → 100°(太くしたぶん、端がギアに近づかないように)
#define ARC_R_IN 126.0f             // 弧の内側の半径(輪の内側 RING_R_IN から 10 px 空けて、太さ 40)
#define ARC_R_OUT 166.0f
#define ARC_SWEEP_DEG 100.0f        // 弧の長さ [度]
#define WATER_ARC_START_DEG 220.0f  // 水温の弧の左端(3 時が 0°、時計回り)。ここから 320° まで
#define FUEL_ARC_START_DEG 140.0f   // 燃料の弧の左端。ここから 40° まで(角度が減る向き)
// 異常のとき(10/22): 弧のある側(水温は上、燃料は下)を、輪の内側の円の中で横一直線に切った形で赤く塗る。点滅の点灯側 = 赤く塗る
// (弧の地は暗い赤、伸びた所とアイコンは白)、消灯側 = 背景のまま(弧の伸びた所とアイコンが赤)。ギアの文字には掛からない
#define ALARM_FILL_R 172.0f         // 塗る円の半径(輪の内側 RING_R_IN より内)
#define ALARM_WATER_Y1 170          // 水温: この y より上を塗る
#define ALARM_FUEL_Y0 296           // 燃料: この y から下を塗る
#define WATER_BAR_MIN_C 40.0f       // 弧の始まり【仮】
#define WATER_BAR_MAX_C 120.0f      // 弧の終わり【仮】
#define WATER_BLUE_END_C 70.0f      // ここまで青【仮】
#define WATER_ALARM_HYST_C 1.0f     // 【未定→仮】警告温度を下回ってから、この温度ぶん下がるまで赤のまま(ちらつき防止)
#define FUEL_ALARM_PCT 30.0f        // これ未満で異常(2026-10-22: 15 → 30。本人の指示)。燃料の弧の 0〜30 % の地は暗い赤
#define FUEL_ALARM_HYST_PCT 1.0f    // 【未定→仮】上と同じ目的
// 異常の表示(水温が警告温度以上、燃料が FUEL_ALARM_PCT 未満)は赤で点滅する(2026-10-14)。
// データ画面: 点灯はアイコン・数字・単位が赤、消灯は暗い赤。走行画面は上の ALARM_*、アナログ計は AN_BADGE_*
#define ALARM_BLINK_HALF_MS 250     // 250 ms 点灯、250 ms 消灯

// ---------------------------------------------------------------- ブースト計のページ(仕様書に無い。2026-10-14 に追加)
// 目盛の範囲は【未確認】: 取扱説明書(2016 年)は「車種により 2.0 bar まで」。写真では 0.5 刻みで 2 までの個体と、
// 0.4 刻みで 1.6 までの個体がある。実車の目盛に合わせてここを変える。
#define BOOST_SCALE_MIN_BAR 0.0f    // 帯の始まり(帯はこれより下に行かない。数字は負の値も出す)
#define BOOST_SCALE_MAX_BAR 2.0f    // 帯の終わり【未確認】1.6 の個体は 1.6
#define BOOST_RED_FROM_BAR 1.6f     // ここから帯を赤にする【仮】(写真では「1.5」は白、「2」は赤)
// 2026-10-21: このページは「アナログ計」になった。回転計が主役。10/22: ブーストは外周の右下の帯、水温・燃料は異常のときだけ。
// 角度は 3 時が 0°、時計回り。回転計は 0 rpm を TACH_START_DEG に置き、1000 rpm ごとに TACH_DEG_PER_1000 進める
// (120° + 25° × 6 = 270° → 6000 rpm が真上に来る)。
#define TACH_MAX_RPM 8000           // 目盛の終わり【仮】(595 の赤線は未確認)
#define TACH_START_DEG 120.0f       // 0 rpm の角度(左下)
#define TACH_DEG_PER_1000 25.0f     // 1000 rpm あたりの角度(6000 で 270° = 真上)
#define TACH_LABEL_STEP_RPM 1000    // 数字の間隔(×1000 で 0〜8)
// 2026-10-22(遠目の案): 実走行で「目盛の主張が強く針が見えにくい」ので、目盛線は灰色に落として 1000 と 500 だけにした。
// 数字は大きく(42 → 60 px)。赤の範囲は目盛の外側の細い帯
// 10/22 夜(決定): 10/21 の配置(白い目盛線 1000・500・250)のまま、目盛線の色を抑えた灰色にした(赤の範囲は暗い赤)。
// 数字は 60 px・太さ 600(白、赤の範囲は赤)
#define TACH_TICK_STEP_RPM 250      // 目盛線の間隔(1000 の倍数は長い、500 の倍数は中くらい)
#define TACH_RED_OFFSET 0           // 赤の始まり = シフト点 + この値【仮】(5800 から赤)
#define TACH_TICK_R_IN 198.0f       // 1000 ごとの目盛線の内側の半径
#define TACH_TICK_R_IN_M 208.0f     // 500 ごと
#define TACH_TICK_R_IN_S 216.0f     // 250 ごと
#define TACH_TICK_R_OUT 228.0f
#define TACH_TICK_HALF_DEG 1.1f     // 1000 ごとの目盛線の太さ(片側の角度。両側で約 8 px)
#define TACH_TICK_HALF_DEG_M 0.75f  // 500 ごと(両側で約 5.6 px)
#define TACH_TICK_HALF_DEG_S 0.5f   // 250 ごと(両側で約 3.9 px)
#define TACH_LABEL_R 166.0f         // 数字の中心の半径(書体は FONT_DIAL600_60)
// 針(10/22 夜): 白、根元から先まで同じ太さ、黒い縁と影つき。中央のブーストの輪(半径 124 まで)の外から伸ばす
#define TACH_NEEDLE_R_IN 126.0f     // 針の根元と先の半径、太さ(根元は中央のブーストの輪と目盛線の外)
#define TACH_NEEDLE_R_OUT 230.0f
#define TACH_NEEDLE_W_IN 14.0f
#define TACH_NEEDLE_W_OUT 14.0f
#define TACH_NEEDLE_EDGE 3.0f       // 針のまわりの黒い縁の幅 [px](目盛線や数字の上でも針が見えるように)
#define TACH_SHADOW_DX 5.0f         // 影: 針から右下へずらす量 [px]
#define TACH_SHADOW_DY 6.0f
#define TACH_SHADOW_GROW 4.0f       // 影の濃い所: 針より太くする量 [px]
#define TACH_SHADOW_SOFT 6.0f       // 影のぼけた所: さらに太くする量 [px](薄く重ねて、ぼかしの代わりにする)
#define TACH_SHADOW_ALPHA 215       // 影の濃さ(0〜255。黒をこの濃さで重ねる)。ぼけた所はこの半分
// 回転計のピークホールド(10/21): 最高の回転数の所に黄色の印(半径 TACH_PEAK_R_IN〜TACH_TICK_R_OUT、片側 TACH_PEAK_HALF_DEG)を
// 残す。TACH_PEAK_HOLD_MS 止まったあと、TACH_PEAK_FALL_RPM_PER_S の速さで今の回転数まで下がる。回転数の信号が無いときは消える。
// 回転数の記録はどの画面でも続けている(アナログ計に切り替えたとき、直前の最高が見える)
#define TACH_PEAK_HOLD_MS 1500
#define TACH_PEAK_FALL_RPM_PER_S 2000
#define TACH_PEAK_R_IN 186.0f       // 10/22 夜: 見にくいので長く(210 → 186)、太く(1.6 → 2.0)
#define TACH_PEAK_HALF_DEG 2.0f
// 10/23: 針とブースト以外を少しだけ暗くした(数字 × 0.82、目盛線 × 0.93、ピークの印 × 0.88)。水温・燃料の札は暗くしない。色は ui.cpp の C_TICK など
// ブースト(10/22 夜・決定、10/23 に太く): 中央の輪。回転計と同じ向き(0 bar が TACH_START_DEG = 左下、時計回りに 1 bar あたり
// BOOST_RING_DEG_PER_BAR)。10/23: 内側へ太くした(84〜108 → 62〜108)。伸びた所はトーンを落とした白(#A9AFBA)、BOOST_RED_FROM_BAR から上は
// 暗めの赤。地は暗い紺(赤の範囲は暗い赤)。0.5 bar ごとの目盛線は輪の外、数字 0・1・2 は輪の内側。今の値は輪の右に大きい数字(太さ 500、白)
#define BOOST_RING_R_IN 62.0f
#define BOOST_RING_R_OUT 108.0f
#define BOOST_RING_DEG_PER_BAR 100.0f
#define BOOST_TICK_R_IN 111.0f      // 0.5 bar ごとの目盛線
#define BOOST_TICK_R_OUT 120.0f
#define BOOST_TICK_HALF_DEG 1.3f
#define BOOST_MARK_R_IN 58.0f       // 先端の白い目印の半径と片側の角度
#define BOOST_MARK_R_OUT 112.0f
#define BOOST_MARK_HALF_DEG 1.3f
#define BOOST_LABEL_R 44.0f         // 数字 0・1・2 の中心の半径(書体は FONT_DIAL500_28)。10/23: 輪を内側へ太くしたので 64 → 44
#define BOOST_NUM_X 426             // 今の値の数字の右端とベースライン(書体は FONT_DIAL500_76、白。10/23: 水色 → 白)
#define BOOST_NUM_BASE 278
// 水温・燃料(10/22): 純正のメーターにあるので、ここには異常のときだけ赤い札を出す(ブーストの数字の下、右下の空き。両方なら 2 段、
// 1 つなら上の段)。札 = アイコン(40 px)+ 数字(FONT_DIAL500_40)。点滅の消灯側は暗い赤の札
#define AN_BADGE_X 234
#define AN_BADGE_W 140
#define AN_BADGE_H 46
#define AN_BADGE_Y 304              // 上の段の上端
#define AN_BADGE_PITCH 52           // 段の間隔

// ---------------------------------------------------------------- サーキット画面(走行画面)の追加(10/23)
// 回転数のピークホールド: 輪の両側、画面の端に付く黄色の三角形(内向き)。位置は輪のマスと同じ並び(ピークの回転数のマスの位置)。
// 三角形は外周の所で幅 32 px・高さ 28 px(正三角形に近い)。底辺は画面の外(半径 PEAK_TRI_R_BASE)に置き、画面の円で切れて端に付く
#define PEAK_TRI_R_TIP 205.0f       // 先の半径
#define PEAK_TRI_R_BASE 250.0f      // 底辺の半径(画面の外)
#define PEAK_TRI_HALF_W 26.0f       // 底辺の半幅(半径 233 の所で 16 px になる)
// 3000rpm 以下の経過時間: 回転数が IDLE_RPM 以下になってからの時間を数える(停止・走行に関係なく。IDLE_RPM を超えたら 0 に戻る)。
// ギアが N のまま IDLE_SHOW_AFTER_N_MS 続いたら、ギアの所に「▼ 3000rpm」と経過時間(分:秒)を出す
#define IDLE_RPM 3000
#define IDLE_SHOW_AFTER_N_MS 60000

// ---------------------------------------------------------------- データ画面(仕様書 6 章)
// 10/23: 回転数と全開の割合をやめて、上に GPS・IMU・CAN の値、下に 3000rpm 超と 3000rpm 以下の累計時間(電源を切っても続けて数える。
// TIME_SAVE_MS ごとに NVS へ。設定の「時間の記録」でリセット)
#define TIME_RPM 3000               // ▲ = この回転数を超えている時間、▼ = この回転数以下の時間(回転数の信号があるときだけ数える)
#define TIME_SAVE_MS 60000          // 累計時間と「覚えている画面」を NVS に書く間隔(変わったときだけ書く)
#define WOT_THROTTLE_PCT 95.0f      // 全開とするアクセル開度【仮】
#define MOVING_SPEED_KMH 1.0f       // 【未定→仮】走行時間 = 車速がこの値以上の時間。全開の時間もこの間だけ数える

// ---------------------------------------------------------------- CAN確認(設定の 10 項目目。10/23)
// 「変化を記録」を押すたびに候補を絞り込む。1 回目 = 基準を取る(全部の ID が候補)。2 回目から = 前の候補のうち、前の記録から今までに
// 「値が違っていて、変わった回数が 1〜CAN_REC_MAX_CHANGES 回」のバイトを持つ ID だけを残し、基準を取り直す
// (カウンターや回転数のように動き続けるバイトは回数が多くて外れる)。【仮】回数の上限は実車で試して決める
#define CAN_MAX_IDS 160             // 覚える ID の数(C-CAN と B-CAN の合計)
#define CAN_HIST_SAMPLES 200        // 詳細の折れ線に出す点の数
#define CAN_HIST_INTERVAL_MS 100    // 【未定→仮】点の間隔。200 点 × 100 ms = 20 秒ぶん
#define CAN_REC_MAX_CHANGES 8       // 記録と記録の間にこの回数より多く変わったバイトは候補から外す
#define CAN_REC_HIST 4              // 件数の移り変わりを出す数(312 → 41 → 12 …)

// ---------------------------------------------------------------- 画面の操作(10/23。2.1.0 でスワイプと長押しに変えた)
// サーキット・タコ・データ: 左へスワイプ = 次の画面、右へスワイプ = 前の画面(サーキット → タコ → データ → サーキット)。
//   スワイプ = 離したときに、触れた所から横に SWIPE_MIN_PX 以上動いていて、横の動きが縦の SWIPE_DIR_RATIO 倍以上。
//   長押し HOLD_MS = 設定(前回見ていた項目)。押している間は画面を暗くし、外周が一周していき、上に「設定」を出す
//   (HOLD_SHOW_MS より短いタッチでは出さない)。指が HOLD_CANCEL_PX 以上動いたら長押しを取り消す(スワイプの途中で設定に入らない)。
//   短いタップは何もしない(すぐ設定に入ると、元に戻るのが面倒なため。本人の決定 10/23)。
//   【未確認】しきい値は実車で試して決める。シリアルの「タッチ: 離した(…)」の行に、触れた所からの動き(横・縦)を出す
// 設定の中: 画面を上・下の帯と、真ん中の左半分・右半分に分ける。上の帯 = 設定(サーキットなどの元の画面へ)か「‹ 戻る」(1 つ上へ)、
// 左右 = 前・次(変更の画面では − +)、下の帯 = その画面の操作。タップは離したときに決まる(位置は触れた所)
#define HOLD_MS 400
#define HOLD_SHOW_MS 100
#define HOLD_CANCEL_PX 30           // 長押しを取り消す動き(触れた所からの距離)
#define SWIPE_MIN_PX 80             // スワイプとみなす横の動き
#define SWIPE_DIR_RATIO 2           // 横の動きが縦の何倍以上ならスワイプか
#define ZONE_TOP_Y 120              // これより上 = 上の帯
#define ZONE_BOT_Y 346              // これから下 = 下の帯
// 「覚えている画面」: サーキット・タコ・データのうち、HOME_CONFIRM_MS 続けて見ていた画面。次の起動と、設定から時間で戻るときにこれを出す
#define HOME_CONFIRM_MS 60000

// ---------------------------------------------------------------- 受信の状態・RaceChrono の状態の判定(2026-10-22 夜。1 画面 1 項目にしたので、
// 下の点の色で全項目の判定を一度に見せる。緑 = 正常、赤 = 異常、灰 = 待ち・切)【仮】
//   受信の状態: 通信速度 = 受けていれば緑(バイトは届くのに読めなければ赤)、受信 = フレームが来ていれば緑、
//     誤り・欠落 = 直近 INFO_RECENT_MS に増えたら赤、送り元 = 版が違えば赤、CAN = C-CAN が 0 なら赤、
//     描画 = サーキット・タコで 1 回描いて送る時間の最長が INFO_PAGE_MAX_MS を超えたら赤(まだ測っていなければ灰)
//   RaceChrono: BLE = 開始できなければ赤、送信 = 直近 INFO_RECENT_MS に捨てたら赤、GPS = 入なのに届かなければ赤、IMU = 応答なしは赤
#define INFO_RECENT_MS 60000
// 描画(2.0.1): 前にサーキット・タコを出していた間(今出していればその間)に、1 回描いて送るのにかかった時間の最長。
// 画面を切り替えた回の全体の描き直しと、長押しの表示は数えない。50 ms = 20 回/秒(2.0.0 までの「20 fps 未満で赤」と同じ基準)
#define INFO_PAGE_MAX_MS 50

// ---------------------------------------------------------------- 設定(仕様書 8 章)
// 【未定→仮】刻みと上下限。値は NVS(名前空間 "meter")に保存する。
#define SET_SHIFT_DEFAULT 5800
#define SET_SHIFT_MIN 3000
#define SET_SHIFT_MAX 7500
#define SET_SHIFT_STEP 100
#define SET_AUTORET_DEFAULT 3       // 分【決定】
#define SET_AUTORET_MIN 1
#define SET_AUTORET_MAX 10
#define SET_AUTORET_STEP 1
#define SET_BRIGHT_DEFAULT 80       // %【仮】
#define SET_BRIGHT_MIN 10
#define SET_BRIGHT_MAX 100
#define SET_BRIGHT_STEP 10
#define SET_WARN_DEFAULT 110        // ℃【仮】
#define SET_WARN_MIN 90
#define SET_WARN_MAX 125
#define SET_WARN_STEP 5
#define SET_DEMO_DEFAULT 0          // デモ表示(0 = 切、1 = 入)。入の間は UART を使わず、表示器の中で作った模擬の値を出す
#define SET_RC_DEFAULT 1            // RaceChrono への送信(0 = 切、1 = 入)。変えると次の起動から効く。
                                    //   BLE・GPS・IMU のどれかが原因で落ちる(リセットが続く)と、表示器が自分で 0 にする(下の RC_CRASH_LIMIT)

// ---------------------------------------------------------------- 受信
#define LINK_TIMEOUT_MS 1000        // 【未定→仮】この時間フレームが来なければ「信号がない」(LED バーの 1 秒と同じ)
// 速度の自動合わせ(候補は meter_link.h の METER_LINK_BAUD_CANDIDATES)。
// 「バイトは届くのに正しいフレームが 1 つも無い」区間が続いたら、次の候補へ移る。
// XIAO のリセット直後に出る短い起動ログ(115200 bps、1 秒未満)では移らない長さにしてある。
#define LINK_BAUD_WINDOW_MS 500     // 判定する区間の長さ
#define LINK_BAUD_BAD_WINDOWS 3     // この回数続いたら移る(500 ms × 3 = 1.5 秒)
#define LINK_BAUD_MIN_BYTES 16      // 区間の中のバイト数がこれ未満なら「届いていない」とみなす(移らない)

// ---------------------------------------------------------------- 車の CAN の生データから表示に使う値を作る(app.cpp の veh。2026-10-23 に c6_can_link から移した)
// 1.x の c6_can_link(XIAO)は値に直して「値のフレーム」(MlSignals)で送っていた。2.0.0 からは CAN の生データだけを送り、
// 値への換算・ギアの推定・燃料のなましは全部ここで行う。値のフレームが届いているとき(c6_display_debug・デモ表示・
// 1.x の c6_can_link)はそちらを使い、届いていないときに生データから作る(mlink::getSignals)。
// 値の出どころ(すべて 29 ビット ID。「確認済み」は実車ログで確かめたもの):
//   回転数     C-CAN 0618A001 b2-b3(ビッグエンディアン、そのまま rpm)           確認済み(9/7)
//   アクセル   C-CAN 0618A001 b7(0〜255 = 0〜100 %)                             確認済み(9/7)
//   車速       C-CAN 0210A006 b4-b5(ビッグエンディアン)÷ 128 = km/h             確認済み(9/7)
//   クラッチ   C-CAN 0628A001 b5 の bit5(1 = 踏んでいる)。ギアの推定にだけ使う   確認済み(9/7)
//   水温       C-CAN 0A18A001 b3 − 40 = ℃                                      確認済み(10/22)
//   ブースト   C-CAN 0A18A001 b4 × VEH_BOOST_BAR_PER_LSB = bar(大気圧との差)     確認済み(10/22。アイドル 0、全開で最大 65)。単位は仮
//   リバース   C-CAN 0A18A001 b7 の bit2(1 = R)                                 確認済み(10/22)
//   燃料       B-CAN 0C014003 b5 − VEH_FUEL_EMPTY_RAW = L(1 LSB = 1 L)            確認済み(10/22 給油 25 L で +25)
//   ギア       R は上のビット。1〜5 は回転数 ÷ 車速の比から推定(下の VEH_GEAR_*)。2 km/h 未満は N
#define VEH_ID_RPM_ACCEL 0x0618A001UL
#define VEH_ID_SPEED 0x0210A006UL
#define VEH_ID_CLUTCH 0x0628A001UL
#define VEH_ID_ENGINE 0x0A18A001UL     // 水温・ブースト・リバース
#define VEH_ID_FUEL 0x0C014003UL       // B-CAN。1 回/秒
#define VEH_BOOST_BAR_PER_LSB 0.01f    // ブーストの 1 LSB [bar]【仮】(本人指定 10/22: 65 → 0.65 と表示)。実車の感触で直す
#define VEH_REVERSE_MASK 0x04          // 0A18A001 b7 のリバースのビット
// 「信号なし」にするまでの時間。c6_can_link は中身の変わらない ID を 1 秒ごとに送り直すので、それより長くする
// (1.x は XIAO が CAN を受けた時刻から 1 秒で判定していた)。受けていない ID は c6_can_link が送らなくなる(2.0.0)
#define VEH_TIMEOUT_MS 2000            // 回転数・アクセル・車速・クラッチ・ブースト・リバース
#define VEH_WATER_TIMEOUT_MS 3000      // 水温(1.x と同じ)
#define VEH_FUEL_TIMEOUT_MS 5000       // 燃料(1 回/秒なので長め。1.x と同じ)
// 燃料のなまし(c6_can_link 1.1.0 の Rev.6 を移した。10/23 に直した所は ★)
//   生の値は坂やカーブで液面が片寄り、数分かけて ±10 L 動く(10/22 実測)。hp = VEH_FUEL_HP_S 秒の指数平均、lp = VEH_FUEL_LP_S 秒の
//   指数平均で、表示は lp。|生 − hp| < VEH_FUEL_STABLE_L のとき(安定)は lp を更新、不安定なときは 生 < lp のときだけ更新(減る方向だけ追う)。
//   ★ b5 = 0 は使わない: IG ON 直後の 2〜4 秒は b4 = 0x88、b5 = 0(まだ測れていない値)で来る(10/22 のログ 3 本すべて)。
//     1.1.0 はこれを最初の値にしたので、lp が 0 から 600 秒のなましで上がり、表示が約 15 分 0 % のままだった(10/23 実車)。
//   ★ lp は他のデータと同じく TIME_SAVE_MS ごとに NVS に書き(変わったときだけ)、起動したらその値から始める。
//     起動後(または途切れたあと)の最初の正しい値が、その値 + VEH_FUEL_REFUEL_L 以上なら給油とみなし、その値から始め直す。
//   給油(電源を切らずに): 回転数 0(または回転数が来ない)が VEH_FUEL_ENGINE_OFF_MS 以上続き、回転数が戻ってからの最初の値が
//     lp + VEH_FUEL_REFUEL_L 以上なら、その値から始め直す。走行中・停止中は増やさない(停めた場所の傾きで +10 L 出ることがある)
#define VEH_FUEL_EMPTY_RAW 120         // 0 L のときの値【仮】(10/22 の走行中の最小 127 が負にならない値。警告灯が点いたときの生の値で決める)
#define VEH_FUEL_FULL_RAW 169          // 満タン(35 L)のときの値(10/22 確認)
#define VEH_FUEL_HP_S 30.0f            // 「今動いているか」を見る短い指数平均の時定数 [秒]
#define VEH_FUEL_LP_S 600.0f           // 表示に使う長い指数平均の時定数 [秒]
#define VEH_FUEL_STABLE_L 1.0f         // |生 − hp| がこれ未満なら安定
#define VEH_FUEL_REFUEL_L 5            // 前の値よりこれ以上大きければ給油
#define VEH_FUEL_ENGINE_OFF_MS 20000   // 回転数 0(または回転数が来ない)がこれだけ続いたら「エンジン停止」
// ギアの推定【仮】: 回転数 ÷ 車速 [rpm per km/h] が、各ギアの計算値の ±VEH_GEAR_TOL_PCT % に入ればそのギア。
// 変速比と最終減速比は 595(C510 5 速)の公表値、タイヤ外周は 205/40R17 の計算値。この車で確かめた値ではない。
// 実車で合わなければ、シリアルの 5 秒ごとの「車の値」の行の「比」(クラッチを離して走っているときの rpm/kmh)を見てここを直す
#define VEH_GEAR_RATIOS { 3.909f, 2.238f, 1.520f, 1.156f, 0.872f }
#define VEH_FINAL_DRIVE 3.353f
#define VEH_TIRE_CIRC_M 1.878f
#define VEH_GEAR_TOL_PCT 12.0f
#define VEH_GEAR_MIN_KMH 5.0f          // これより遅いときは推定しない(前の値のまま)
#define VEH_STOP_KMH 2.0f              // これより遅ければ停車とみなして N
#define VEH_GEAR_MIN_RPM 500
// クラッチのビットでギアを止めるか(c6_can_link 1.0.0 で 0 にした)。10/22 のログでは、1 速でつながって走っている間も
// 0628A001 b5 bit5 がほぼ 1 のまま(1 速 901 回中 892 回が 1、2 速でも 4 割)で、1 速がなかなか出ない原因と見ている(有力)。
// 半クラッチの間は比がどのギアにも合わないので、止めなくても誤ったギアは出にくい。元に戻すときは 1
#define VEH_GEAR_HOLD_ON_CLUTCH 0

// ---------------------------------------------------------------- RaceChrono へ送る(BLE、GPS、IMU)。2026-10-14 に追加
// RC_ENABLE 0 にすると、この機能をコンパイルから外す(NimBLE-Arduino が無くてもビルドできる。切り分け用)
#define RC_ENABLE 1
// 落ちる起動(パニックやウォッチドッグによるリセット)がこの回数続いたら、RaceChrono の送信を設定で「切」にして、
// 表示器としては動き続ける。設定 → 3 ページ目「RaceChrono 送信」で入に戻せる。60 秒落ちずに動いたら回数は 0 に戻る
#define RC_CRASH_LIMIT 2
// 表示器が RaceChrono DIY の BLE 機器(仕様: github.com/aollin/racechrono-ble-diy-device)になり、
// GPS(M100)・CAN(XIAO から届く生データ)・IMU(QMI8658)を 1 台で送る。RaceChrono では「設定 → Add other device →
// Add other device → RaceChrono DIY」(接続の種類 Bluetooth LE)で 1 つ登録する(GPS 機器の一覧には出ない。GPS も CAN も
// この 1 つの登録で届く)。
#define RC_BLE_NAME "595 meter"      // RaceChrono の機器一覧に出る名前
#define RC_BLE_CONN_PARAMS 0         // 1 = つながった直後に接続の間隔 7.5〜15 ms を頼む。0 = 頼まない(スマホ任せ。参考にした 2 つの実装と同じ)
#define RC_GPS_PIN_DETECT 1          // 1 = 起動時と、位置が届かない間 20 秒ごとに、「IO18」「IO17」のどちらに M100 の信号が来ているかを調べて
                                     //     受信ピンを自動で決める(配線図と逆でも動く。調べている間は送信ピンを駆動しない)。0 = PIN_GPS_RX/TX のまま
#define RC_GPS_RATE_HZ 25            // GPS の測位の回数/秒【仮】RaceBox 互換機と同じ。不安定なら 10
#define RC_GPS_BAUD 115200           // M100 の UART の速度。届かなければ工場出荷の 9600 で速度を書き換えてから使う
#define RC_GPS_CFG_LAYERS 0x03       // M100 の設定を書く層(bit0 = RAM、bit1 = BBR(電池で保持)、bit2 = フラッシュ)。
                                     //   RAM だけなら毎回の起動で設定し直す(数秒かかる)。BBR にも書くと次からすぐ届く
#define RC_GPS_RETRY_MS 2000         // 設定を送って、この時間 UBX が届かなければ速度を変えて送り直す
#define RC_GPS_LOST_MS 3000          // 位置(NAV-PVT)がこの時間来なければ「GPS が途切れた」として設定を送り直す
#define RC_IMU_HZ 20                 // IMU の値を RaceChrono へ送る回数/秒【仮】
#define IMU_SAMPLE_HZ 50             // IMU を読む回数/秒
#define RC_QUEUE_LEN 48              // 送る前の通知を溜める数(あふれた分は「捨てた」に数える)
#define RC_SEND_BCAN 1               // 1 = B-CAN の生データも RaceChrono へ送る。0 = C-CAN だけ(ID が C-CAN と重なるときは 0)
// 車の CAN を RaceChrono へ送る最短の間隔(PID ごと)。RaceChrono が頼む間隔がこれより短くても、この間隔より詰めない。
// c6_can_link 2.0.0 は表示器に回転数などを 50 回/秒で届けるため、生データを最短 20 ms 間隔で送る(1.x は 50 ms)。
// RaceChrono へ送る量を 1.x のときと同じに保つため、ここで 50 ms にそろえる(BLE の帯域。IMU は RC_IMU_HZ のまま)
#define RC_CAN_MIN_MS 50
// IMU の値を RaceChrono に CAN の 1 つとして送るときの ID。RaceChrono(iPhone)の PID 欄は 0x1FFFFF01 も 0x0FFFFF01 も
// 「有効な PID を入力してください」で弾いた(10/15。上限の規則は未確認)。通っている車の ID(0x0618A001)と同じ形で、
// 車が使っていない値にしてある【未確認: 車のログの ID 一覧と照合する。同じ ID が車から届いたらシリアルに警告が出る】。
// RaceChrono の CAN チャンネルの PID にこの値を入れる(0x 06 18 A0 F1)。中身は int16(下位が先)× 3:
//   RC_PID_IMU_ACCEL: 前後・横・上下の加速度 [mg](前が正、右が正、上が正。重力は引いてある)
//   RC_PID_IMU_GYRO : ヨーレート [0.01 deg/s](左回りが正)、ほか 2 軸は 0
#define RC_PID_IMU_ACCEL 0x0618A0F1
#define RC_PID_IMU_GYRO 0x0618A0F2
// IMU の向きの補正(取り付け角は自動で覚える。停車中の重力で「上」、発進の加速で「前」。覚えた向きは NVS に保存)
// 覚える前の向き(初期値)は「画面を立てて運転者に向け、本体の背面が車の前」。机の上で画面を立てた状態で 0/0/0 になる。
// 実機で確かめた軸(10/15): 画面を立てると +X が上、画面を伏せる(背面が上)と +Z が上 = +Z は背面の向き。
#define IMU_DEF_UP { 1.0f, 0.0f, 0.0f }   // 覚える前の「上」(センサーの軸、長さ 1)
#define IMU_DEF_FWD { 0.0f, 0.0f, 1.0f }  // 覚える前の「前」(= 背面)
#define IMU_LAT_SIGN 1               // 横加速度の符号(1 = 右が正。RaceChrono の向きと合わなければ -1)
#define IMU_STILL_KMH 0.5f           // この車速未満を停車とみなす
#define IMU_LEVEL_STILL_MS 3000      // 停車がこの時間続いたら「上」を覚える
// 車速の信号が無いとき(机の上、XIAO も GPS も無い)は、値が動かず大きさが 1 g に近い状態が 3 秒続いたら静止とみなして
// そのときの重力の向きを「上」にする(置き直すたびに更新)。ただし初期の「上」から IMU_QUIET_TILT 以内のときだけ
// (伏せた・仰向けの試験で「上」が動かないように)。「前」は車速が要る(発進の加速)
#define IMU_QUIET_G 0.03f            // 直前の値からの変化がこれ未満 [g] なら静止(大きさは 0.8〜1.2 g のとき。実機は 1.1 g と出る)
#define IMU_QUIET_TILT_COS 0.707f    // 初期の「上」との内積がこれ以上(45° 以内)
#define IMU_UP_EMA_MIN_KMH 5.0f      // 走行中の長期平均で「上」を直す(この車速以上の間だけ。坂に止めたままでも動かない)
#define IMU_UP_TAU_MS 600000         // その長期平均の時定数(10 分)
#define IMU_FWD_FROM_KMH 3.0f        // 発進の加速で「前」を覚える車速の範囲(集め始め)
#define IMU_FWD_TO_KMH 25.0f         // これより速いと集めない
#define IMU_FWD_DONE_KMH 15.0f       // ここまで加速したら平均を取る(後退ではここまで出さない前提)
#define IMU_FWD_SAMPLES 40           // 「前」を決めるのに最低限必要な加速中の値の数(50 回/秒なら 0.8 秒ぶん)
#define IMU_RELEARN_COUNT 5          // 覚えた向きと大きく違う値がこの回数続いたら、付け直したとみなして覚え直す

// ---------------------------------------------------------------- 数字の形
// 1 = 数字の幅をそろえる(仕様書 9.2)。Saira の等幅数字を使う。
// 0 = キャンバスの絵と同じ(書体に等幅数字が無く、数字ごとに幅が違う)。値が変わると数字が左右に動く。
#define DIGITS_TABULAR 1

// ---------------------------------------------------------------- 基板(Waveshare ESP32-S3-Touch-AMOLED-1.75)
// ピン番号は Waveshare 公式リポジトリの HARDWARE_REFERENCE.md と examples/arduino の pin_config.h による。
#define PIN_LCD_CS 12
#define PIN_LCD_SCLK 38
#define PIN_LCD_D0 4
#define PIN_LCD_D1 5
#define PIN_LCD_D2 6
#define PIN_LCD_D3 7
#define PIN_LCD_RST 39
#define PIN_I2C_SDA 15
#define PIN_I2C_SCL 14
#define PIN_TP_INT 11
#define PIN_TP_RST 40
#define PIN_LINK_RX 16              // 8 ピンヘッダの「IO16」 ← XIAO「D6」
#define PIN_GPS_RX 18               // 8 ピンヘッダの「IO18」 ← M100「TX」【未確認】配線図 Rev.0.7 の TX/RX の向き。RC_GPS_PIN_DETECT 1 なら自動で見分ける
#define PIN_GPS_TX 17               // 8 ピンヘッダの「IO17」 → M100「RX」(M100 の設定を送る線。オープンドレイン + プルアップで駆動し、逆につないでも壊さない)
#define LCD_SPI_HZ 40000000         // 公式の例と同じ 40 MHz
#define LCD_COL_OFFSET 6            // 公式の例と同じ(画面の左端がコントローラーの 6 列目)

#!/bin/sh
# run_tests.sh - 表示器のコードを PC(Mac / Linux)で動かして確かめる
#
# 基板に依存しない部分(gfx.cpp・ui.cpp・app.cpp と、c6_display_debug の sim.cpp)を PC 用にコンパイルし、
# メモリーの範囲外アクセスと未定義の動作を検出する設定(-fsanitize=address,undefined)で次を順に動かす。
#   unit   : UART の受け取り方、タップでの画面の移り変わり、設定の保存、自動で戻る、速度の自動合わせ
#   e2e    : 模擬データの送信側 → 表示器の受信・画面 を 128 秒ぶん通す(欠落・CRC 誤り・描き直しの照合)
#   fuzz   : でたらめな受信データとタップを流す(既定は 2 回 × 100000 歩。増やすときは引数で: run_tests.sh 12 150000)
#   render : 各画面を out/ に BMP で書き出す(目で見る用)
#   blink  : 回転数の点滅(場面 6: 800→6600→800 rpm)の時系列を確かめ、out/blink_timeline.csv に書く。
#            ffmpeg があれば、走行画面の 60 fps の動画 out/blink_scene6_60fps.mp4 も作る(無ければ動画は作らない)
# どれかが失敗したら、そこで止まる。最後に「ALL PASS」が出れば全部通っている。
#
# 使い方: このフォルダで  sh run_tests.sh
# 必要なもの: C++17 のコンパイラー(Mac は Xcode のコマンドラインツールの c++)
# コンパイラーを変えるとき: CXX=g++ sh run_tests.sh
# 検出の設定を外すとき(その設定が使えない環境用): SAN= sh run_tests.sh
set -e
cd "$(dirname "$0")"
CXX=${CXX:-c++}
SAN=${SAN--fsanitize=address,undefined}
RUNS=${1:-2}
STEPS=${2:-100000}
SRC="plat_pc.cpp ../meter_display/gfx.cpp ../meter_display/ui.cpp ../meter_display/app.cpp ../meter_display/src/fonts_data.cpp"
FLAGS="-std=c++17 -O1 -g -Wall -Wextra $SAN -fno-omit-frame-pointer -I../meter_display"
mkdir -p build out
echo "== コンパイル($CXX)"
$CXX $FLAGS unit_main.cpp $SRC -o build/unit
$CXX $FLAGS e2e_main.cpp $SRC ../c6_display_debug/sim.cpp -o build/e2e
$CXX $FLAGS fuzz_main.cpp $SRC -o build/fuzz
$CXX $FLAGS render_main.cpp $SRC -o build/render
$CXX $FLAGS blink_demo_main.cpp $SRC -o build/blink
echo "== unit"
./build/unit
echo "== e2e(照合つき)"
./build/e2e out check
echo "== e2e(時間の見積もり)"
./build/e2e out
echo "== fuzz($RUNS 回 × $STEPS 歩)"
./build/fuzz "$RUNS" "$STEPS" out
echo "== render"
./build/render out
echo "== blink(場面 6 の点滅)"
./build/blink out
if command -v ffmpeg >/dev/null 2>&1; then
  ./build/blink out - 2>/dev/null | ffmpeg -y -loglevel error -f rawvideo -pix_fmt rgb24 -s 466x466 -r 60 -i - -c:v libx264 -crf 18 -pix_fmt yuv420p -movflags +faststart out/blink_scene6_60fps.mp4
  echo "動画: out/blink_scene6_60fps.mp4(60 fps、実時間 18 秒)"
else
  echo "(ffmpeg が無いので動画は作らない。brew install ffmpeg で入る)"
fi
echo "ALL PASS"

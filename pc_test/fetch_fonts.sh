#!/bin/sh
# fetch_fonts.sh - gen_assets.py が使う書体を fonts/ に取ってくる
#
# 取ってくる先は Google Fonts のリポジトリ(github.com/google/fonts)。どれも SIL Open Font License 1.1。
# 2026-10-13 の版(下の REV)に固定してある。この版のファイルから作ったのが meter_display/src/fonts_data.cpp。
#
# 使い方: このフォルダで  sh fetch_fonts.sh
set -e
cd "$(dirname "$0")"
REV=66928797f271fed5ee428544e7229685e2d90127
BASE=https://raw.githubusercontent.com/google/fonts/$REV/ofl
mkdir -p fonts
get() {
  echo "$2"
  curl -fsSL -o "fonts/$2" "$BASE/$1"
}
get "saira/Saira%5Bwdth%2Cwght%5D.ttf" Saira-VF.ttf
get "saira/OFL.txt" Saira-OFL.txt
get "sairasemicondensed/SairaSemiCondensed-Medium.ttf" SairaSemiCondensed-Medium.ttf
get "sairasemicondensed/SairaSemiCondensed-SemiBold.ttf" SairaSemiCondensed-SemiBold.ttf
get "sairasemicondensed/OFL.txt" SairaSemiCondensed-OFL.txt
get "ibmplexsansjp/IBMPlexSansJP-Medium.ttf" IBMPlexSansJP-Medium.ttf
get "ibmplexsansjp/IBMPlexSansJP-SemiBold.ttf" IBMPlexSansJP-SemiBold.ttf
get "ibmplexsansjp/OFL.txt" IBMPlexSansJP-OFL.txt
get "ibmplexmono/IBMPlexMono-Medium.ttf" IBMPlexMono-Medium.ttf
get "ibmplexmono/IBMPlexMono-SemiBold.ttf" IBMPlexMono-SemiBold.ttf
get "ibmplexmono/IBMPlexMono-Bold.ttf" IBMPlexMono-Bold.ttf
get "ibmplexmono/OFL.txt" IBMPlexMono-OFL.txt
echo "fonts/ に置いた"

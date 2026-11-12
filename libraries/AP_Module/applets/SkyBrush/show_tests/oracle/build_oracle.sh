#!/bin/sh
# Builds the C libskybrush oracle and regenerates fixtures/*.oracle.txt.
# usage: build_oracle.sh <libskybrush source dir> <scratch build dir>
set -e
SRC=$1
BUILD=$2
HERE=$(cd "$(dirname "$0")" && pwd)
FIX="$HERE/../fixtures"
cmake -S "$SRC" -B "$BUILD/lib" -DCMAKE_BUILD_TYPE=Release \
    "-DCMAKE_C_FLAGS=-Wno-maybe-uninitialized" "-DCMAKE_CXX_FLAGS=-Wno-maybe-uninitialized" >/dev/null
cmake --build "$BUILD/lib" -j8 --target skybrush >/dev/null
cc -O2 -ffp-contract=off -I"$SRC/include" "$HERE/sb_oracle.c" "$BUILD/lib/src/libskybrush.a" \
    -lstdc++ -lm -o "$BUILD/sb_oracle"
for f in "$FIX"/*.skyb; do
    "$BUILD/sb_oracle" "$f" > "${f%.skyb}.oracle.txt"
done
"$BUILD/sb_oracle" --touches > "$FIX/touches.oracle.txt"

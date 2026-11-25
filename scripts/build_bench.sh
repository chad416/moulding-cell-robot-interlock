#!/usr/bin/env bash
# Compile the test bench (plc/test/st/tb_bench.st, or the file given) together with the block under test into a
# shared library, with matiec and gcc.
#   usage: scripts/build_bench.sh [tb_bench.st] [out_dir]        default out_dir: build/plc_test
#   MATIEC_DIR  matiec checkout with iec2c built (default: ~/matiec)
#   PYTHON      python interpreter (default: python3)
set -euo pipefail
cd "$(dirname "$0")/.."
MATIEC_DIR="${MATIEC_DIR:-$HOME/matiec}"
PYTHON="${PYTHON:-python3}"
SRC="${1:-plc/test/st/tb_bench.st}"
OUT="${2:-build/plc_test}"
rm -rf "$OUT"
mkdir -p "$OUT/iec"
cat plc/st/cell_plc.st "$SRC" > "$OUT/bench_all.st"
"$MATIEC_DIR/iec2c" -I "$MATIEC_DIR/lib" -T "$OUT/iec" "$OUT/bench_all.st" > "$OUT/iec2c.log"
"$PYTHON" plc/gen_shim.py "$OUT/bench_all.st" "$OUT" TB_BENCH
gcc -O1 -fPIC -shared -w -I "$OUT/iec" -I "$MATIEC_DIR/lib/C" -o "$OUT/libplc.so" "$OUT/shim.c" -lm
echo "built $OUT/libplc.so"

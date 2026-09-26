#!/usr/bin/env bash
# 執行全部章節的 Python simulation lab(sim.py);任一章末 assert 失敗即回報非零。
# 無頭執行(MPLBACKEND=Agg),圖存到各章 out/。
set -u
cd "$(dirname "$0")"
ROOT="$PWD"
PYTHON="${PYTHON:-$ROOT/../.venv/bin/python}"
[ -x "$PYTHON" ] || PYTHON="$(command -v python3)"
export MPLBACKEND="${MPLBACKEND:-Agg}"
LOGDIR="${TMPDIR:-/tmp}"

fail=0
for d in [01][0-9]-*/; do
    d="${d%/}"
    [ -f "$d/sim.py" ] || continue
    printf '%-32s ' "$d"
    if "$PYTHON" "$d/sim.py" >"$LOGDIR/makerlab_$d.log" 2>&1; then
        echo "OK"
    else
        echo "FAIL  (log: $LOGDIR/makerlab_$d.log)"
        fail=1
    fi
done
exit $fail

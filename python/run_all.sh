#!/usr/bin/env bash
# 執行全部 19 章純 Python 腳本(main.py);任一章末的 ✅ assertion 失敗即回報非零結束碼。
# 無頭執行:預設 MPLBACKEND=Agg,不開圖形視窗(plt.show() 變為 no-op)。
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
    [ -f "$d/main.py" ] || continue
    printf '%-32s ' "$d"
    # 從章目錄內執行,讓腳本裡的 sys.path.append('..') 指到 python/,找得到 common/
    if ( cd "$d" && "$PYTHON" main.py ) >"$LOGDIR/pyscript_$d.log" 2>&1; then
        echo "OK"
    else
        echo "FAIL  (log: $LOGDIR/pyscript_$d.log)"
        fail=1
    fi
done
exit $fail

#!/usr/bin/env bash
# 執行全部 19 章 lab notebook;任一章的 ✅ 驗證 assertion 失敗即回報非零結束碼。
# 預設不改寫 notebook(輸出到暫存檔);要把執行結果寫回 notebook 請設 INPLACE=1。
set -u
cd "$(dirname "$0")"
JUPYTER="${JUPYTER:-../.venv/bin/jupyter}"
[ -x "$JUPYTER" ] || JUPYTER=jupyter
LOGDIR="${TMPDIR:-/tmp}"

fail=0
for d in [01][0-9]-*/; do
    d="${d%/}"
    [ -f "$d/lab.ipynb" ] || continue
    printf '%-32s ' "$d"
    if [ "${INPLACE:-0}" = 1 ]; then
        out=(--inplace)
    else
        out=(--output-dir "$LOGDIR" --output "lab_$d.ipynb")
    fi
    if "$JUPYTER" nbconvert --to notebook --execute "${out[@]}" "$d/lab.ipynb" >/dev/null 2>"$LOGDIR/lab_$d.log"; then
        echo "OK"
    else
        echo "FAIL  (log: $LOGDIR/lab_$d.log)"
        fail=1
    fi
done
exit $fail

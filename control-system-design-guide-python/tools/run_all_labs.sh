#!/usr/bin/env bash
# 執行全部 19 章 lab notebook;任一章的 ✅ 驗證 assertion 失敗即中止並回報。
set -u
cd "$(dirname "$0")/.."
JUPYTER="${JUPYTER:-.venv/bin/jupyter}"
[ -x "$JUPYTER" ] || JUPYTER=jupyter

fail=0
for d in 0[1-9]-* 1[0-9]-*; do
    [ -f "$d/lab.ipynb" ] || continue
    printf '%-32s ' "$d"
    if "$JUPYTER" nbconvert --to notebook --execute --inplace "$d/lab.ipynb" >/dev/null 2>"/tmp/lab_$d.log"; then
        echo "OK"
    else
        echo "FAIL  (log: /tmp/lab_$d.log)"
        fail=1
    fi
done

echo "---- host firmware PID test ----"
if command -v c++ >/dev/null; then
    ( cd hardware/esp32/test_host \
      && c++ -std=c++11 -O2 -o /tmp/test_pid test_pid.cpp \
      && /tmp/test_pid > /tmp/pid_c_output.csv \
      && "${PYTHON:-../../../.venv/bin/python}" check_pid.py /tmp/pid_c_output.csv ) || fail=1
else
    echo "c++ 不存在,略過韌體 host 測試"
fi

exit $fail

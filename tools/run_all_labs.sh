#!/usr/bin/env bash
# 執行全部驗證:Python 版 19 章 notebook、韌體 host PID 測試、C++ 版 19 章。
# 任一項失敗即回報非零結束碼。
set -u
cd "$(dirname "$0")/.."
PYTHON="${PYTHON:-$PWD/.venv/bin/python}"
[ -x "$PYTHON" ] || PYTHON=python3

fail=0

echo "---- Python 版 19 章(python/)----"
./python/run_all.sh || fail=1

echo "---- host firmware PID test ----"
if command -v c++ >/dev/null; then
    ( cd hardware/esp32/test_host \
      && c++ -std=c++11 -O2 -o /tmp/test_pid test_pid.cpp \
      && /tmp/test_pid > /tmp/pid_c_output.csv \
      && "$PYTHON" check_pid.py /tmp/pid_c_output.csv ) || fail=1
else
    echo "c++ 不存在,略過韌體 host 測試"
fi

echo "---- C++ 版 19 章(cpp/)----"
if command -v c++ >/dev/null; then
    ./cpp/run_all.sh >/tmp/cpp_labs.log 2>&1 && echo "OK" || { echo "FAIL  (log: /tmp/cpp_labs.log)"; fail=1; }
else
    echo "c++ 不存在,略過 C++ 版"
fi

exit $fail

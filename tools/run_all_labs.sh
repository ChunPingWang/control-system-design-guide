#!/usr/bin/env bash
# 執行全部驗證:Jupyter 19 章 notebook、純 Python 腳本 19 章、韌體 host PID 測試、
# C++ 19 章、Maker Labs 19 章(sim + 韌體 host 編譯/單元測試)。
# 任一項失敗即回報非零結束碼。
#
# 環境變數(皆可覆寫):
#   PYTHON  Python 直譯器(預設 .venv/bin/python,無則 python3)
#   CXX     C++ 編譯器(預設 c++;例如系統只有 g++-15 時:CXX=g++-15 ./tools/run_all_labs.sh)
set -u
cd "$(dirname "$0")/.."
PYTHON="${PYTHON:-$PWD/.venv/bin/python}"
[ -x "$PYTHON" ] || PYTHON=python3
CXX="${CXX:-c++}"
export CXX

fail=0

echo "---- Jupyter notebook 19 章(jupyter/)----"
./jupyter/run_all.sh || fail=1

echo "---- 純 Python 腳本 19 章(python/)----"
PYTHON="$PYTHON" ./python/run_all.sh || fail=1

echo "---- host firmware PID test ----"
if command -v "$CXX" >/dev/null; then
    ( cd hardware/esp32/test_host \
      && "$CXX" -std=c++11 -O2 -o /tmp/test_pid test_pid.cpp \
      && /tmp/test_pid > /tmp/pid_c_output.csv \
      && "$PYTHON" check_pid.py /tmp/pid_c_output.csv ) || fail=1
else
    echo "找不到 C++ 編譯器 '$CXX',略過韌體 host 測試(可用 CXX=... 指定)"
fi

echo "---- C++ 版 19 章(cpp/)----"
if command -v "$CXX" >/dev/null; then
    ./cpp/run_all.sh >/tmp/cpp_labs.log 2>&1 && echo "OK" || { echo "FAIL  (log: /tmp/cpp_labs.log)"; fail=1; }
else
    echo "找不到 C++ 編譯器 '$CXX',略過 C++ 版(可用 CXX=... 指定)"
fi

echo "---- Maker Labs sim 19 章(maker-labs/)----"
PYTHON="$PYTHON" ./maker-labs/run_all.sh || fail=1

echo "---- Maker Labs 韌體(lib 單元測試 + 各章 host 編譯)----"
if command -v "$CXX" >/dev/null; then
    ./maker-labs/verify_firmware.sh >/tmp/makerlab_fw.log 2>&1 && echo "OK" || { echo "FAIL  (log: /tmp/makerlab_fw.log)"; fail=1; }
else
    echo "找不到 C++ 編譯器 '$CXX',略過 Maker Labs 韌體(可用 CXX=... 指定)"
fi

exit $fail

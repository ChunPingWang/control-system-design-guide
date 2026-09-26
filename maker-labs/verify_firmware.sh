#!/usr/bin/env bash
# 韌體驗證(非 CI 硬體、非模擬器):
#   1. host 單元測試共用控制邏輯 lib(test/test_lib.cpp)—— 真的執行、比對數值。
#   2. host 編譯每章 firmware/main.cpp(套 arduino_shim.h)—— 證明「編得過、API 正確」。
# 任一步失敗即回報非零結束碼。
set -u
cd "$(dirname "$0")"
ROOT="$PWD"
CXX="${CXX:-c++}"
command -v "$CXX" >/dev/null || { echo "找不到 C++ 編譯器 '$CXX'(可用 CXX=... 指定)"; exit 1; }

FW="$ROOT/firmware"
LOGDIR="${TMPDIR:-/tmp}"
fail=0

echo "---- 1) 共用 lib host 單元測試 ----"
if "$CXX" -std=c++17 -O2 -Wall -Wextra -I"$FW/lib" "$FW/test/test_lib.cpp" -o "$LOGDIR/makerlab_test_lib" 2>"$LOGDIR/makerlab_test_lib.build.log"; then
    if "$LOGDIR/makerlab_test_lib" >"$LOGDIR/makerlab_test_lib.run.log" 2>&1; then
        echo "test_lib                         OK ($(grep -c '  ok:' "$LOGDIR/makerlab_test_lib.run.log") checks)"
    else
        echo "test_lib                         FAIL  (log: $LOGDIR/makerlab_test_lib.run.log)"; fail=1
    fi
else
    echo "test_lib                         BUILD FAIL  (log: $LOGDIR/makerlab_test_lib.build.log)"; fail=1
fi

echo "---- 2) 各章 firmware host 編譯 ----"
for d in [01][0-9]-*/; do
    d="${d%/}"
    [ -f "$d/firmware/main.cpp" ] || continue
    printf '%-32s ' "$d"
    if "$CXX" -std=c++17 -Wall -Wextra -DMAKERLAB_HOST -I"$FW" -I"$FW/lib" \
        -c "$d/firmware/main.cpp" -o "$LOGDIR/makerlab_${d}.o" 2>"$LOGDIR/makerlab_${d}.fw.log"; then
        echo "compile OK"
    else
        echo "COMPILE FAIL  (log: $LOGDIR/makerlab_${d}.fw.log)"; fail=1
    fi
done
exit $fail

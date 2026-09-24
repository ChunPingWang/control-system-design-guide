#!/usr/bin/env bash
# 建置並執行全部 19 章 C++ lab;任一章 ✅ 驗證失敗即回報非零結束碼。
# 有 cmake 用 cmake + ctest;沒有則退回直接呼叫 c++ 編譯。
set -u
cd "$(dirname "$0")"
BUILD="${BUILD:-build}"
export CSD_OUT="${CSD_OUT:-$PWD/$BUILD/out}"

if command -v cmake >/dev/null; then
    cmake -S . -B "$BUILD" -DCMAKE_BUILD_TYPE=Release >/dev/null || exit 1
    cmake --build "$BUILD" -j >/dev/null || exit 1
    ctest --test-dir "$BUILD" --output-on-failure
    exit $?
fi

CXX="${CXX:-c++}"
mkdir -p "$BUILD"
fail=0
for d in [01][0-9]-*/; do
    d="${d%/}"
    exe="$BUILD/ch${d:0:2}"
    printf '%-32s ' "$d"
    if ! "$CXX" -std=c++17 -O2 -o "$exe" "$d/main.cpp" 2>"$BUILD/${d}.build.log"; then
        echo "BUILD FAIL  (log: $BUILD/${d}.build.log)"; fail=1; continue
    fi
    if "$exe" >"$BUILD/${d}.log" 2>&1; then
        echo "OK"
    else
        echo "FAIL  (log: $BUILD/${d}.log)"; fail=1
    fi
done
exit $fail

// encoder.h — Maker Labs 正交編碼器解碼 + 速度估測(純 C++,host 可測)
// 章節用途:Ch2/3 speed loop、Ch5 quantization、Ch14 encoder、Ch17 position。
#pragma once
#include <cstdint>

namespace maker {

// 正交解碼:輸入 A/B 兩相電位,累積 position(以 count 為單位)。
// 使用標準 4x 解碼狀態機。
struct QuadDecoder {
    int8_t  prev = 0;      // 前一次 (A<<1|B)
    long    position = 0;

    void update(int a, int b) {
        int8_t cur = (int8_t)((a << 1) | b);
        // 轉移查表:index = (prev<<2)|cur → +1 / -1 / 0
        static const int8_t LUT[16] = {
            0, +1, -1, 0,
            -1, 0, 0, +1,
            +1, 0, 0, -1,
            0, -1, +1, 0
        };
        position += LUT[(prev << 2) | cur];
        prev = cur;
    }
    void reset() { prev = 0; position = 0; }
};

// 由 count 差分估速度:每個量測窗回傳 counts/sec。
// counts_per_rev 已知時可用 to_rpm() 轉 RPM。
struct VelocityEstimator {
    long  last_pos = 0;
    bool  first = true;

    // dt = 量測窗長度(秒);回傳 counts/sec
    float step(long position, float dt) {
        if (first) { last_pos = position; first = false; return 0; }
        long dpos = position - last_pos;
        last_pos = position;
        return (float)dpos / dt;
    }
    void reset() { first = true; last_pos = 0; }

    static float counts_per_sec_to_rpm(float cps, float counts_per_rev) {
        return cps / counts_per_rev * 60.0f;
    }
};

}  // namespace maker

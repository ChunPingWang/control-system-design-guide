// filter.h — Maker Labs 共用濾波器(純 C++,host 可測)
// 章節用途:Ch9 filters、Ch14 encoder noise、Ch16 resonance、capstone。
#pragma once
#include <cmath>

namespace maker {

// 一階 IIR 低通:y += alpha*(x-y);alpha 由截止頻率與取樣週期決定。
struct OnePoleLP {
    float alpha = 1.0f;
    float y = 0;
    bool  first = true;

    void init(float fc_hz, float dt) {
        // alpha = dt / (dt + RC),RC = 1/(2*pi*fc)
        float rc = 1.0f / (2.0f * 3.14159265358979f * fc_hz);
        alpha = dt / (dt + rc);
    }
    void set_alpha(float a) { alpha = a; }
    float step(float x) {
        if (first) { y = x; first = false; return y; }
        y += alpha * (x - y);
        return y;
    }
    void reset() { y = 0; first = true; }
};

// 移動平均(環形緩衝,N 點)。N<=32。
template <int N>
struct MovingAverage {
    float buf[N] = {0};
    int   idx = 0;
    int   count = 0;
    float sum = 0;

    float step(float x) {
        if (count < N) {
            sum += x; buf[idx] = x; idx = (idx + 1) % N; count++;
            return sum / count;
        }
        sum -= buf[idx];
        sum += x;
        buf[idx] = x;
        idx = (idx + 1) % N;
        return sum / N;
    }
    void reset() { idx = 0; count = 0; sum = 0; for (int i = 0; i < N; i++) buf[i] = 0; }
};

}  // namespace maker

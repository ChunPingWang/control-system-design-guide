// util.h — Maker Labs 共用小工具(純 C++,host 可測)
// 章節用途:Ch12 nonlinear(saturation / deadband / rate limit)、通用 clamp。
#pragma once
#include <cmath>

namespace maker {

inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

// 飽和(對稱)
inline float saturate(float x, float limit) {
    return clampf(x, -limit, limit);
}

// 死區:|x| < dead → 0,否則平移使輸出連續
inline float deadband(float x, float dead) {
    if (std::fabs(x) < dead) return 0.0f;
    return x > 0 ? (x - dead) : (x + dead);
}

// 變化率限制:限制每步變化不超過 max_delta
struct RateLimiter {
    float y = 0;
    bool  first = true;
    float step(float x, float max_delta) {
        if (first) { y = x; first = false; return y; }
        float d = x - y;
        if (d > max_delta) d = max_delta;
        if (d < -max_delta) d = -max_delta;
        y += d;
        return y;
    }
    void reset() { first = true; y = 0; }
};

}  // namespace maker

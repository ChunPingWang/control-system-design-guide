// feedforward.h — Maker Labs 前饋(純 C++,host 可測)
// 章節用途:Ch8 feed-forward。由 open-loop 實驗建 target→baseline_output 模型。
#pragma once

namespace maker {

// 線性前饋:u_ff = gain * target + offset(即 target_rpm → baseline_pwm)。
// gain/offset 由 open-loop step test 線性擬合而來。
struct LinearFeedforward {
    float gain = 0;
    float offset = 0;

    void init(float gain_, float offset_) { gain = gain_; offset = offset_; }
    float compute(float target) const { return gain * target + offset; }

    // 由兩個 (target, measured_output) 校準點擬合線性模型
    void calibrate(float t1, float u1, float t2, float u2) {
        gain = (u2 - u1) / (t2 - t1);
        offset = u1 - gain * t1;
    }
};

}  // namespace maker

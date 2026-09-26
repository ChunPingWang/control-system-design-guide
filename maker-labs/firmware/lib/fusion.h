// fusion.h — Maker Labs IMU 互補濾波器(純 C++,host 可測)
// 章節用途:capstone 自平衡車(MPU6050 傾角估測)。
//
// 互補濾波:angle = a*(angle + gyro*dt) + (1-a)*acc_angle
//   - gyro 積分短期準、長期漂移;acc 長期準、短期有雜訊。
//   - a 接近 1(如 0.98)偏重 gyro。
#pragma once
#include <cmath>

namespace maker {

struct ComplementaryFilter {
    float a = 0.98f;
    float angle = 0;
    bool  first = true;

    void init(float a_) { a = a_; angle = 0; first = true; }

    // acc_angle_rad:由加速度計算的傾角;gyro_rate:角速度(rad/s);dt:秒
    float step(float acc_angle_rad, float gyro_rate, float dt) {
        if (first) { angle = acc_angle_rad; first = false; return angle; }
        angle = a * (angle + gyro_rate * dt) + (1.0f - a) * acc_angle_rad;
        return angle;
    }
    void reset() { angle = 0; first = true; }
};

// 由加速度計 x/z 分量算傾角(rad)。
inline float accel_tilt(float ax, float az) {
    return std::atan2(ax, az);
}

}  // namespace maker

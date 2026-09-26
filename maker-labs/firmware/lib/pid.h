// pid.h — Maker Labs 共用 PID(純 C++,無 Arduino 相依,host 可編譯測試)
//
// 設計對齊 hardware/esp32/src/pid.h(積分器先更新再算輸出),額外提供:
//   - 輸出飽和 clamp(out_min / out_max)
//   - 條件積分 anti-windup(飽和且誤差同向時,不再累積積分)
//   - 導數低通濾波(dfilt_hz > 0 時啟用;0 表示不濾波)
//   - 導數以「量測值」而非「誤差」計算(避免 setpoint 跳變造成 derivative kick)
//
// 章節用途:Ch3 tuning、Ch6 controllers、Ch7 disturbance、Ch8 feed-forward、
//           Ch12 nonlinear、Ch17 position、Ch18 observer、Ch19 RCP。
#pragma once
#include <cmath>

namespace maker {

struct Pid {
    float kp = 0, ki = 0, kd = 0;
    float dt = 0.01f;
    float out_min = -1e30f, out_max = 1e30f;
    float dfilt_hz = 0;          // 導數低通截止頻率;0 = 不濾波
    bool  deriv_on_meas = true;  // true: 對量測微分;false: 對誤差微分

    // 內部狀態
    float integ = 0;
    float prev_meas = 0;
    float prev_e = 0;
    float d_filt = 0;
    bool  first = true;

    void init(float kp_, float ki_, float kd_, float dt_,
              float out_min_, float out_max_) {
        kp = kp_; ki = ki_; kd = kd_; dt = dt_;
        out_min = out_min_; out_max = out_max_;
        reset();
    }

    void reset() {
        integ = 0; prev_meas = 0; prev_e = 0; d_filt = 0; first = true;
    }

    // setpoint r、量測 meas → 控制輸出 u
    float step(float r, float meas) {
        float e = r - meas;

        // 積分(先更新)
        integ += e * dt;

        // 導數項
        float d_raw = 0;
        if (!first) {
            if (deriv_on_meas)
                d_raw = -(meas - prev_meas) / dt;   // -d(meas)/dt
            else
                d_raw = (e - prev_e) / dt;
        }
        first = false;
        prev_meas = meas;
        prev_e = e;

        // 導數低通(一階)
        float d_term;
        if (dfilt_hz > 0) {
            float alpha = dt / (dt + 1.0f / (2.0f * 3.14159265358979f * dfilt_hz));
            d_filt += alpha * (d_raw - d_filt);
            d_term = d_filt;
        } else {
            d_term = d_raw;
        }

        float u_unsat = kp * e + ki * integ + kd * d_term;

        // 飽和
        float u = u_unsat;
        if (u > out_max) u = out_max;
        if (u < out_min) u = out_min;

        // 條件積分 anti-windup:飽和且積分會使輸出更飽和時,回退這一步積分
        if (u != u_unsat && (e * u_unsat) > 0) {
            integ -= e * dt;
        }
        return u;
    }
};

}  // namespace maker

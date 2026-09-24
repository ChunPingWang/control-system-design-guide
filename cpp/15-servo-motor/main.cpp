// Ch15 伺服馬達與電流迴路:極點對消電流 PI + 串級速度 PI(電流極限)
#include "../common/csd.hpp"
using namespace csd;

int main() {
    const double R = 1.0, L_ind = 1e-3, Kt = 0.1, Ke = 0.1, J = 1e-4, b = 1e-5;
    const double dt = 1e-6;  // 模擬步長 1 µs(電流迴路很快)

    // --- 電流迴路:極點對消設計,目標頻寬 800 Hz ---
    const double wc_i = 2 * PI * 800, kp_i = L_ind * wc_i, ki_i = R * wc_i;
    std::printf("電流迴路 PI:kp=%.3f, ki=%.0f(零點 = ki/kp = %.0f rad/s = 電氣極點 R/L)\n", kp_i, ki_i, ki_i / kp_i);

    const int n = int(0.005 / dt);
    DCMotorPlant motor(R, L_ind, Kt, Ke, J, b, dt);
    DiscretePID ci(kp_i, ki_i, 0.0, dt, -24, 24);
    Vec i_log(n), v_log(n);
    for (int k = 0; k < n; ++k) {
        i_log[k] = motor.i;
        v_log[k] = ci.step(1.0 - motor.i);  // 1 A 電流步階命令
        motor.step(v_log[k]);
    }
    Vec t_i = sample_times(n, dt);
    long i10 = first_index(n, [&](size_t k) { return i_log[k] >= 0.1; });
    long i90 = first_index(n, [&](size_t k) { return i_log[k] >= 0.9; });
    const double rise_us = (t_i[i90] - t_i[i10]) * 1e6, theory_us = 0.35 / 800 * 1e6;
    std::printf("10–90%% 上升時間 %.0f µs,理論 0.35/BW = %.0f µs\n", rise_us, theory_us);
    Vec t_dec, i_dec, v_dec;  // 每 10 µs 取一點輸出
    for (int k = 0; k < n; k += 10) { t_dec.push_back(t_i[k]); i_dec.push_back(i_log[k]); v_dec.push_back(v_log[k]); }
    write_csv("ch15_current_step.csv", {"t", "current", "voltage"}, {t_dec, i_dec, v_dec});

    // --- 速度迴路(外環)包住電流迴路(內環) ---
    const double wc_v = 2 * PI * 80, kp_v = J * wc_v / Kt, ki_v = kp_v * 2 * PI * 8;
    std::printf("速度迴路 PI:kp=%.4f, ki=%.3f\n", kp_v, ki_v);
    const int n2 = int(0.2 / dt);
    DCMotorPlant motor2(R, L_ind, Kt, Ke, J, b, dt);
    DiscretePID ci2(kp_i, ki_i, 0.0, dt, -24, 24);
    DiscretePID cv(kp_v, ki_v, 0.0, dt, -5, 5);  // 電流極限 ±5 A
    Vec w_log(n2), ia_log(n2), va_log(n2);
    for (int k = 0; k < n2; ++k) {
        w_log[k] = motor2.w;
        ia_log[k] = motor2.i;
        double i_cmd = cv.step(100.0 - motor2.w);
        va_log[k] = ci2.step(i_cmd - motor2.i);
        motor2.step(va_log[k]);
    }
    Vec t2, w2, ia2, va2;
    for (int k = 0; k < n2; k += 100) { t2.push_back(k * dt); w2.push_back(w_log[k]); ia2.push_back(ia_log[k]); va2.push_back(va_log[k]); }
    write_csv("ch15_speed_loop.csv", {"t", "speed", "current", "voltage"}, {t2, w2, ia2, va2});

    const double v_bemf_expect = Ke * 100 + R * (b * 100 / Kt);
    std::printf("穩態:轉速 %.2f rad/s,電壓 %.2f V(理論 Ke·w + R·i = %.2f V)\n", w_log.back(), va_log.back(), v_bemf_expect);
    std::printf("峰值電流 %.2f A(極限 5 A),overshoot %.2f rad/s\n", vmax(ia_log), vmax(w_log) - 100);
    int limited = 0;
    for (double ia : ia_log) limited += ia > 4.5;

    Checker chk("Ch15");
    CHECK(chk, std::fabs(rise_us - theory_us) / theory_us < 0.4);  // 電流迴路頻寬達標
    CHECK(chk, std::fabs(i_log.back() - 1.0) < 0.05);              // 電流無穩態誤差
    CHECK(chk, vmax(ia_log) <= 5.0 + 1e-6);                        // 電流極限被尊重
    CHECK(chk, std::fabs(w_log.back() - 100) < 0.5);               // 速度到位
    CHECK(chk, std::fabs(va_log.back() - v_bemf_expect) < 0.5);    // 穩態電壓 = 反電動勢 + IR
    CHECK(chk, limited * dt > 0.01);                               // 有明顯的電流受限段
    return chk.finish();
}

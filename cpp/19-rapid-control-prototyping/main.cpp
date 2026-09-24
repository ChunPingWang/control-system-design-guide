// Ch19 RCP:遙測 → 系統辨識 → λ-tuning → 離散閉迴路驗證 → 韌體 PID 等價性
//
// 與 Python 版不同之處:這裡直接 #include 韌體的 hardware/esp32/src/pid.h,
// 讓「會燒進 ESP32 的那份 C 程式碼」本身參與驗證。
#include <fstream>
#include <sstream>

#include "../../hardware/esp32/src/pid.h"
#include "../common/csd.hpp"
using namespace csd;

// 逐行翻譯 pid.h 的 pid_step()(double 版):integ 先更新,導數不濾波
struct CStylePID {
    double kp, ki, kd, dt, out_min, out_max, integ = 0, prev_e = 0;
    bool first = true;
    double step(double e) {
        integ += e * dt;
        double d = first ? 0.0 : (e - prev_e) / dt;
        first = false;
        prev_e = e;
        return saturate(kp * e + ki * integ + kd * d, out_min, out_max);
    }
};

// 兩參數 Levenberg–Marquardt:y ≈ K*pwm*(1 - exp(-t/tau))
std::pair<double, double> fit_first_order(const Vec& t, const Vec& y, double pwm, double K, double tau) {
    auto sse = [&](double k, double ta) {
        double s = 0;
        for (size_t i = 0; i < t.size(); ++i) {
            double r = y[i] - k * pwm * (1 - std::exp(-t[i] / ta));
            s += r * r;
        }
        return s;
    };
    double lam = 1e-3, cost = sse(K, tau);
    for (int it = 0; it < 200; ++it) {
        double a11 = 0, a12 = 0, a22 = 0, g1 = 0, g2 = 0;
        for (size_t i = 0; i < t.size(); ++i) {
            double e = std::exp(-t[i] / tau);
            double r = y[i] - K * pwm * (1 - e);
            double jK = pwm * (1 - e), jT = -K * pwm * e * t[i] / (tau * tau);
            a11 += jK * jK; a12 += jK * jT; a22 += jT * jT;
            g1 += jK * r; g2 += jT * r;
        }
        bool improved = false;
        while (lam < 1e12) {
            double b11 = a11 * (1 + lam), b22 = a22 * (1 + lam), det = b11 * b22 - a12 * a12;
            double dK = (b22 * g1 - a12 * g2) / det, dT = (b11 * g2 - a12 * g1) / det;
            if (tau + dT > 0) {
                double c = sse(K + dK, tau + dT);
                if (c < cost) {
                    bool done = std::fabs(dK) < 1e-12 * std::fabs(K) + 1e-15 && std::fabs(dT) < 1e-12 * tau + 1e-15;
                    K += dK; tau += dT; cost = c; lam *= 0.3; improved = true;
                    if (done) return {K, tau};
                    break;
                }
            }
            lam *= 10;
        }
        if (!improved) break;
    }
    return {K, tau};
}

int main() {
    // --- 步驟 1:模擬一段「ESP32 開迴路步階」遙測(實機上這段來自序列埠)---
    const double K_true = 3.0, tau_true = 0.25;  // 真實馬達:3 rpm / pwm-unit,τ=250 ms
    const double fs_rcp = 100.0, dt_rcp = 1 / fs_rcp;
    const int n = 300;
    const double pwm_step = 100.0;
    Rng rng(1);
    Vec noise = rng.normal(0, 3.0, n);
    const std::string tele_path = out_path("ch19_telemetry.csv");
    {
        std::ofstream f(tele_path);
        f << "millis,target,actual,pwm\n";
        for (int k = 0; k < n; ++k) {
            double tk = k * dt_rcp;
            char line[96];
            std::snprintf(line, sizeof line, "%d,0.0,%.3f,%.1f\n", int(tk * 1000),
                          K_true * pwm_step * (1 - std::exp(-tk / tau_true)) + noise[k], pwm_step);
            f << line;
        }
    }
    std::printf("遙測寫入 %s(格式同韌體序列輸出)\n", tele_path.c_str());

    // --- 步驟 2:解析 + 擬合 ---
    Vec t_log, actual;
    {
        std::ifstream f(tele_path);
        std::string line;
        std::getline(f, line);  // header
        while (std::getline(f, line)) {
            std::stringstream ss(line);
            std::string ms, tgt, act;
            std::getline(ss, ms, ',');
            std::getline(ss, tgt, ',');
            std::getline(ss, act, ',');
            t_log.push_back(std::stod(ms) / 1000.0);
            actual.push_back(std::stod(act));
        }
    }
    auto [K_fit, tau_fit] = fit_first_order(t_log, actual, pwm_step, 1.0, 0.1);
    std::printf("擬合:K=%.3f(真值 %.1f),tau=%.1f ms(真值 %.0f ms)\n", K_fit, K_true, tau_fit * 1000, tau_true * 1000);
    Vec fitted;
    for (double tk : t_log) fitted.push_back(K_fit * pwm_step * (1 - std::exp(-tk / tau_fit)));
    write_csv("ch19_identification.csv", {"t", "telemetry", "fitted"}, {t_log, actual, fitted});

    // --- 步驟 3:λ-tuning ---
    const double lam = 0.10;
    const double kp_rcp = tau_fit / (K_fit * lam), ki_rcp = kp_rcp / tau_fit;
    std::printf("λ-tuning:kp=%.4f pwm/rpm,ki=%.4f\n", kp_rcp, ki_rcp);

    // 用「真實」受控體(擬合時不知道的)做離散閉迴路驗證,100 Hz、PWM 飽和 ±255
    const int n2 = int(2.0 / dt_rcp);
    const double target = 400.0;
    Vec t2 = sample_times(n2, dt_rcp), y_rpm(n2);
    DiscretePID ctrl(kp_rcp, ki_rcp, 0.0, dt_rcp, -255, 255);
    double y_state = 0.0;
    for (int k = 0; k < n2; ++k) {
        y_rpm[k] = y_state;
        double u = ctrl.step(target - y_state);
        y_state += (K_true * u - y_state) / tau_true * dt_rcp;  // 真實一階馬達
    }
    StepMetrics mets = step_metrics(t2, y_rpm, target);
    std::printf("閉迴路指標:rise=%.4f OS=%.4f%% settle=%.4f sse=%.4f\n", mets.rise_time, mets.overshoot_pct,
                mets.settling_time, mets.steady_state_error);
    write_csv("ch19_closed_loop.csv", {"t", "rpm"}, {t2, y_rpm});

    // --- 步驟 4a:韌體演算法 vs 教材 DiscretePID(差異應全在一拍積分量內)---
    Rng rng2(3);
    Vec e_seq = rng2.normal(0, 50, 500);  // 隨機誤差序列 [rpm]
    CStylePID c_fw{kp_rcp, ki_rcp, 0.0, dt_rcp, -255, 255};
    DiscretePID c_py(kp_rcp, ki_rcp, 0.0, dt_rcp, -255, 255, 0.0, false);
    Vec u_fw, diff, bound;
    int over = 0;
    for (double e : e_seq) {
        u_fw.push_back(c_fw.step(e));
        diff.push_back(std::fabs(u_fw.back() - c_py.step(e)));
        bound.push_back(ki_rcp * std::fabs(e) * dt_rcp);  // 理論差異上限:一拍積分量
        over += diff.back() > bound.back() + 1e-9;
    }
    std::printf("最大差異 %.4f pwm-unit;理論上限 max(ki·|e|·Ts) = %.4f\n", vmax(diff), vmax(bound));
    std::printf("超出理論上限的樣本數:%d / %zu\n", over, e_seq.size());

    // --- 步驟 4b:真正的韌體 pid.h(float)vs double 翻譯版 ---
    pid_t_ fw;
    pid_init(&fw, float(kp_rcp), float(ki_rcp), 0.0f, float(dt_rcp), -255.0f, 255.0f);
    double fw_worst = 0.0;
    for (size_t k = 0; k < e_seq.size(); ++k)
        fw_worst = std::max(fw_worst, std::fabs(double(pid_step(&fw, float(e_seq[k]))) - u_fw[k]));
    std::printf("韌體 pid.h(float32)vs double 版最大差異 %.2e pwm-unit\n", fw_worst);

    Checker chk("Ch19");
    CHECK(chk, std::fabs(K_fit - K_true) / K_true < 0.05);         // 辨識 K 誤差 < 5%
    CHECK(chk, std::fabs(tau_fit - tau_true) / tau_true < 0.10);   // 辨識 tau 誤差 < 10%
    CHECK(chk, mets.overshoot_pct < 15);                           // λ-tuning 閉迴路溫和
    CHECK(chk, std::fabs(mets.steady_state_error) < 5);            // 穩態到位(rpm)
    long i63 = first_index(y_rpm.size(), [&](size_t i) { return y_rpm[i] >= target * 0.632; });
    CHECK(chk, std::fabs(t2[i63] - lam) / lam < 0.5);              // 63% 時間 ≈ λ
    CHECK(chk, over == 0);                                         // 韌體/教材差異全在理論上限內
    CHECK(chk, fw_worst < 1e-3);                                   // float32 韌體與 double 版一致
    return chk.finish();
}

// Ch9 迴路內濾波器:LPF / notch 的「衰減換相位」帳本;迴路內濾雜訊
#include "../common/csd.hpp"
using namespace csd;

const double J = 0.002, b = 0.01;

TF notch_tf(double f0_hz, double Q = 5.0) {
    double w0 = 2 * PI * f0_hz;
    return tf({1, 0, w0 * w0}, {1, w0 / Q, w0 * w0});
}

int main() {
    const TF P = inertia(J, b), C = pi_ctrl(0.5, 5.0);

    // --- 回授路徑低通的相位代價 ---
    std::map<std::string, double> pm;
    pm["no filter"] = margins(C * P).pm_deg;
    std::vector<std::pair<std::string, TF>> items = {{"no_filter", C * P}};
    for (int fc : {200, 50}) {
        TF Lf = C * P * butter2_analog(2 * PI * fc);
        pm["LPF " + std::to_string(fc) + " Hz"] = margins(Lf).pm_deg;
        items.push_back({"LPF" + std::to_string(fc), Lf});
    }
    write_bode_csv("ch09_lpf_loops.csv", logspace(0, 3, 500), items);

    // --- notch vs LPF ---
    TF N = notch_tf(200, 5);
    double pm_notch = margins(C * P * N).pm_deg;
    write_bode_csv("ch09_lpf_vs_notch.csv", logspace(0.5, 3, 600), {{"LPF200", butter2_analog(2 * PI * 200)}, {"notch200", N}});
    double depth = vmin(frf_of_system(N, linspace(190, 210, 201)).mag_db);
    std::printf("PM 比較(迴路頻寬 ~40 Hz):\n");
    for (const char* k : {"no filter", "LPF 200 Hz", "LPF 50 Hz"}) std::printf("  %-12s PM = %6.1f°\n", k, pm[k]);
    std::printf("  %-12s PM = %6.1f°\nnotch 深度 @200 Hz ≈ %.1f dB\n", "Notch 200 Hz", pm_notch, depth);

    // --- 迴路內低通壓量測雜訊 ---
    const double fs = 5000.0, dt = 1 / fs;
    const int n = int(1.0 / dt);
    Rng rng(42);
    Vec noise = rng.normal(0, 0.05, n);
    auto run = [&](bool use_filter, Vec& u_log, Vec& y_log) {
        MotorPlant plant(J, b, dt);
        DiscretePID ctrl(0.5, 5.0, 0.0, dt);
        OnePole lpf(200, dt);
        u_log.assign(n, 0);
        y_log.assign(n, 0);
        for (int k = 0; k < n; ++k) {
            double y_meas = plant.w + noise[k];
            if (use_filter) y_meas = lpf.step(y_meas);
            u_log[k] = ctrl.step(1.0 - y_meas);
            y_log[k] = plant.w;
            plant.step(u_log[k]);
        }
    };
    Vec u_raw, y_raw, u_flt, y_flt;
    run(false, u_raw, y_raw);
    run(true, u_flt, y_flt);
    double rms_raw = stdev(slice(u_raw, int(0.5 / dt))), rms_flt = stdev(slice(u_flt, int(0.5 / dt)));
    std::printf("轉矩雜訊 RMS:不濾 %.4f → 濾波 %.4f(降 %.1f 倍)\n", rms_raw, rms_flt, rms_raw / rms_flt);
    write_csv("ch09_noise.csv", {"t", "u_raw", "u_filtered", "w_raw", "w_filtered"}, {sample_times(n, dt), u_raw, u_flt, y_raw, y_flt});

    Checker chk("Ch9");
    CHECK(chk, pm["no filter"] > pm["LPF 200 Hz"] && pm["LPF 200 Hz"] > pm["LPF 50 Hz"]);
    CHECK(chk, pm["LPF 50 Hz"] < 30);           // 濾太低 → 邊限崩潰
    CHECK(chk, pm["LPF 200 Hz"] > 60);          // 5 倍頻寬 → 損失可控
    CHECK(chk, pm_notch > pm["LPF 200 Hz"]);    // notch 在交越處吃相位較少
    CHECK(chk, depth < -15);                    // notch 深度
    CHECK(chk, rms_flt < rms_raw / 2);          // 低通確實壓雜訊
    return chk.finish();
}

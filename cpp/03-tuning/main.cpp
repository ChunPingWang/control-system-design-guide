// Ch3 zone-based 調機:先掃 kp(ki=0)到 overshoot ≥4%,再掃 ki 到 overshoot ≥15%
#include "../common/csd.hpp"
using namespace csd;

const double J = 0.002, b = 0.01, fs = 1000.0, dt = 1 / fs;

struct Run { Vec t, y; };

Run run_step(double kp, double ki, double t_end = 0.4) {
    MotorPlant plant(J, b, dt);
    DiscretePID ctrl(kp, ki, 0.0, dt);
    Delay dly(1);  // 一拍計算延遲(貼近實機)
    int n = int(t_end / dt);
    Run r{sample_times(n, dt), Vec(n)};
    for (int k = 0; k < n; ++k) {
        r.y[k] = plant.w;
        plant.step(dly.step(ctrl.step(1.0 - plant.w)));
    }
    return r;
}

int main() {
    // --- 步驟 1:ki=0,掃 kp ---
    Vec kps;
    for (double v : arange(0.1, 2.01, 0.1)) kps.push_back(round_to(v, 2));
    Vec os_p;
    for (double kp : kps) {
        Run r = run_step(kp, 0.0);
        os_p.push_back(step_metrics(r.t, r.y).overshoot_pct);
    }
    long ip = first_index(os_p.size(), [&](size_t i) { return os_p[i] >= 4.0; });
    double kp_sel = kps[ip];
    std::printf("kp sweep overshoot%%:");
    for (size_t i = 0; i < kps.size(); ++i) std::printf(" %.1f:%.1f", kps[i], os_p[i]);
    std::printf("\n選定 kp = %.1f(overshoot 首次 ≥ 4%%)\n", kp_sel);

    // --- 步驟 2:固定 kp,掃 ki ---
    Vec kis = arange(0, 81, 5), os_i;
    for (double ki : kis) {
        Run r = run_step(kp_sel, ki);
        os_i.push_back(step_metrics(r.t, r.y).overshoot_pct);
    }
    long ii = first_index(os_i.size(), [&](size_t i) { return os_i[i] >= 15.0; });
    double ki_sel = kis[ii];
    std::printf("ki sweep overshoot%%:");
    for (size_t i = 0; i < kis.size(); ++i) std::printf(" %g:%.1f", kis[i], os_i[i]);
    std::printf("\n選定 ki = %g(overshoot 首次 ≥ 15%%)\n", ki_sel);
    write_csv("ch03_sweeps.csv", {"kp", "os_p", "ki", "os_i"}, {kps, os_p, kis, os_i});

    std::vector<std::string> hdr = {"t"};
    std::vector<Vec> cols = {run_step(0.2, 0).t};
    for (double kp : {0.2, kp_sel, 1.4}) { hdr.push_back("kp" + std::to_string(kp)); cols.push_back(run_step(kp, 0).y); }
    for (double ki : {0.0, ki_sel, 80.0}) { hdr.push_back("ki" + std::to_string(ki)); cols.push_back(run_step(kp_sel, ki).y); }
    write_csv("ch03_steps.csv", hdr, cols);

    // --- 類比 PM 與取樣延遲相位損失 ---
    Margins m_analog = margins(pi_ctrl(kp_sel, ki_sel) * inertia(J, b));
    double phase_loss = 360.0 * m_analog.f_pm_hz * 1.5 * dt;
    double pm_digital = m_analog.pm_deg - phase_loss;
    std::printf("類比 PM = %.1f° @ %.1f Hz\n", m_analog.pm_deg, m_analog.f_pm_hz);
    std::printf("取樣延遲相位損失 ≈ %.1f° → 數位 PM ≈ %.1f°\n", phase_loss, pm_digital);

    Run fin = run_step(kp_sel, ki_sel);
    StepMetrics mets = step_metrics(fin.t, fin.y);
    std::printf("最終調機結果: rise=%.4f OS=%.4f%% settle=%.4f sse=%.4f\n", mets.rise_time, mets.overshoot_pct,
                mets.settling_time, mets.steady_state_error);

    Checker chk("Ch3");
    bool mono = true;  // overshoot 應隨 kp 大致遞增
    for (size_t i = 1; i < os_p.size(); ++i) mono = mono && (os_p[i] - os_p[i - 1] > -0.5);
    CHECK(chk, mono);
    CHECK(chk, 4 <= os_p[ip] && os_p[ip] < 30);
    CHECK(chk, 14 <= mets.overshoot_pct && mets.overshoot_pct < 30);  // 步驟 2 目標 ~15%
    CHECK(chk, 30 < pm_digital && pm_digital < 75);                   // 合理的數位 PM
    CHECK(chk, std::fabs(mets.steady_state_error) < 0.01);
    return chk.finish();
}

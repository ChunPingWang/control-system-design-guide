// Ch7 擾動響應:Gd = P/(1+CP) 頻域 + 時域 + 擾動解耦前饋
#include "../common/csd.hpp"
using namespace csd;

const double J = 0.002, b = 0.01;

int main() {
    const TF P = inertia(J, b);

    // --- 頻域:不同 ki 的擾動響應 ---
    std::vector<std::pair<std::string, TF>> items;
    for (int ki : {1, 5, 20}) items.push_back({"ki" + std::to_string(ki), feedback(P, pi_ctrl(0.5, ki))});
    items.push_back({"open_loop_P", P});
    write_bode_csv("ch07_disturbance_bode.csv", logspace(-1, 3, 500), items);

    // --- 時域:-0.1 Nm 步階負載 ---
    Vec t = linspace(0, 2, 4000), d(t.size());
    for (size_t i = 0; i < t.size(); ++i) d[i] = t[i] >= 0.2 ? -0.1 : 0.0;
    std::map<int, double> dips;
    std::vector<Vec> cols = {t};
    for (int ki : {1, 5, 20}) {
        Vec y = forced_response(feedback(P, pi_ctrl(0.5, ki)), t, d);
        dips[ki] = -vmin(y);
        cols.push_back(y);
    }
    std::printf("max dip by ki: 1→%.4f  5→%.4f  20→%.4f\n", dips[1], dips[5], dips[20]);
    write_csv("ch07_disturbance_time.csv", {"t", "ki1", "ki5", "ki20"}, cols);

    // --- 擾動解耦:量測負載轉矩(延遲 1 ms、增益誤差 10%)前饋抵銷 ---
    const double fs = 1000.0, dt = 1 / fs;
    const int n = int(1.0 / dt);
    Vec tt = sample_times(n, dt), d_sig(n);
    for (int k = 0; k < n; ++k) d_sig[k] = tt[k] >= 0.2 ? 0.1 : 0.0;
    auto run = [&](bool decouple, double meas_delay_ms = 1, double meas_gain = 0.9) {
        MotorPlant plant(J, b, dt);
        DiscretePID ctrl(0.5, 5.0, 0.0, dt);
        Delay dly(int(meas_delay_ms * 1e-3 / dt));
        Vec y(n);
        for (int k = 0; k < n; ++k) {
            y[k] = plant.w;
            double u = ctrl.step(1.0 - plant.w);
            double d_meas = dly.step(d_sig[k]) * meas_gain;
            if (decouple) u += d_meas;  // 前饋補償(抵銷負載轉矩)
            plant.step(u, d_sig[k]);
        }
        return y;
    };
    Vec y_fb = run(false), y_dc = run(true);
    double dip_fb = 1 - vmin(slice(y_fb, int(0.2 / dt)));
    double dip_dc = 1 - vmin(slice(y_dc, int(0.2 / dt)));
    std::printf("速度跌落:純回授 %.4f | 加解耦 %.4f(改善 %.1fx)\n", dip_fb, dip_dc, dip_fb / dip_dc);
    write_csv("ch07_decoupling.csv", {"t", "feedback_only", "with_decoupling"}, {tt, y_fb, y_dc});

    // 低頻抑制才是 ki 的主戰場:|Gd(0.1 Hz)| 應相差 20*log10(20) ≈ 26 dB
    double g1 = mag_db_at(feedback(P, pi_ctrl(0.5, 1)), 0.1);
    double g20 = mag_db_at(feedback(P, pi_ctrl(0.5, 20)), 0.1);
    double asym = 20 * std::log10(2 * PI * 0.1 / 20);  // 低頻 |Gd| ≈ s/ki 漸近線
    std::printf("|Gd(0.1 Hz)|:ki=1 %.1f dB | ki=20 %.1f dB(漸近線 %.1f dB)\n", g1, g20, asym);

    Checker chk("Ch7");
    CHECK(chk, dips[1] > dips[5] && dips[5] > dips[20]);  // ki 越大跌落越小
    CHECK(chk, std::fabs((g1 - g20) - 20 * std::log10(20.0)) < 2);
    CHECK(chk, std::fabs(g20 - asym) < 1.5);
    CHECK(chk, dip_dc < dip_fb / 2);                      // 解耦至少改善 2 倍
    return chk.finish();
}

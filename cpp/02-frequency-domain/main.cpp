// Ch2 頻域分析:PI 速度迴路解析 Bode + DSA chirp 量測交叉比對
#include "../common/csd.hpp"
using namespace csd;

int main() {
    const double J = 0.002, b = 0.01;
    const TF P = inertia(J, b);
    const TF C = pi_ctrl(0.5, 5.0);
    const TF L = C * P;             // 開迴路
    const TF T = closed_loop(C, P); // 閉迴路

    Margins m = margins(L);
    double bw = bandwidth_hz(T);
    std::printf("開迴路邊限: GM=%g dB, PM=%.2f° @ %.2f Hz\n", m.gm_db, m.pm_deg, m.f_pm_hz);
    std::printf("閉迴路 -3dB 頻寬: %.1f Hz\n", bw);
    write_bode_csv("ch02_bode_open_closed.csv", logspace(-1, 3, 600), {{"L", L}, {"T", T}});

    // --- DSA:chirp 激發閉迴路,逐樣本模擬後量測 FRF ---
    const double fs = 5000.0, dt = 1 / fs;
    Excitation ex = chirp_excitation(1, 300, 20, fs, 1.0);
    MotorPlant plant(J, b, dt);
    DiscretePID ctrl(0.5, 5.0, 0.0, dt);
    Vec y(ex.u.size());
    for (size_t k = 0; k < ex.u.size(); ++k) {
        y[k] = plant.w;
        plant.step(ctrl.step(ex.u[k] - plant.w));
    }
    FRF meas = measure_frf(ex.u, y, fs);
    FRF good = meas.select([&](size_t i) { return meas.coherence[i] > 0.95; });

    long i3 = first_index(good.f_hz.size(), [&](size_t i) { return good.mag_db[i] < -3; });
    double bw_meas = good.f_hz[i3];
    std::printf("解析頻寬 %.1f Hz | DSA 量測頻寬 %.1f Hz\n", bw, bw_meas);

    FRF ana = frf_of_system(T, good.f_hz);
    write_csv("ch02_dsa_vs_analytic.csv", {"f_hz", "meas_mag_db", "meas_phase_deg", "coherence", "ana_mag_db", "ana_phase_deg"},
              {good.f_hz, good.mag_db, good.phase_deg, good.coherence, ana.mag_db, ana.phase_deg});

    Checker chk("Ch2");
    CHECK(chk, 85 < m.pm_deg && m.pm_deg < 92);        // PI 速度迴路 PM ≈ 89°
    CHECK(chk, std::isinf(m.gm_db));                   // 二階迴路相位不會到 -180°,GM 無限大
    CHECK(chk, 35 < bw && bw < 46);                    // 頻寬 ≈ 40 Hz
    CHECK(chk, std::fabs(bw_meas - bw) / bw < 0.3);    // DSA 量測與解析誤差 < 30%
    bool lowf_ok = true;                               // 低頻 |T| ≈ 0 dB
    for (size_t i = 0; i < good.f_hz.size(); ++i)
        if (good.f_hz[i] < 5 && std::fabs(good.mag_db[i]) >= 1.0) lowf_ok = false;
    CHECK(chk, lowf_ok);
    return chk.finish();
}

// Ch4 取樣與延遲:相位損失公式 360·fc·1.5Ts vs Padé vs 逐樣本模擬
#include "../common/csd.hpp"
using namespace csd;

int main() {
    const double J = 0.002, b = 0.01;
    const TF P = inertia(J, b);
    const TF C = pi_ctrl(0.5, 5.0);
    Margins m0 = margins(C * P);
    const double fc = m0.f_pm_hz;
    std::printf("連續系統:PM=%.1f° @ fc=%.1f Hz\n", m0.pm_deg, fc);

    std::map<int, double> theory;  // fs -> 數位 PM(直線公式)
    std::printf("%8s %12s %12s\n", "fs [Hz]", "相位損失[°]", "數位 PM[°]");
    for (int fs : {500, 1000, 2000, 5000, 20000}) {
        double loss = 360.0 * fc * 1.5 / fs;  // ZOH 0.5T + 計算 1T
        theory[fs] = m0.pm_deg - loss;
        std::printf("%8d %12.1f %12.1f\n", fs, loss, theory[fs]);
    }

    // --- 同一組增益、不同取樣率的逐樣本步階 ---
    std::map<int, double> os_by_fs;
    for (int fs : {500, 1000, 5000}) {
        double dt = 1.0 / fs;
        MotorPlant plant(J, b, dt);
        DiscretePID ctrl(0.5, 5.0, 0.0, dt);
        Delay delay(1);  // 計算延遲:一拍
        int n = int(0.25 / dt);
        Vec y(n);
        for (int k = 0; k < n; ++k) {
            y[k] = plant.w;
            plant.step(delay.step(ctrl.step(1.0 - plant.w)));
        }
        Vec t = sample_times(n, dt);
        os_by_fs[fs] = step_metrics(t, y).overshoot_pct;
        write_csv("ch04_step_fs" + std::to_string(fs) + ".csv", {"t", "y"}, {t, y});
    }
    Vec ti = linspace(0, 0.25, 1000);
    write_csv("ch04_step_continuous.csv", {"t", "y"}, {ti, step_response(closed_loop(C, P), ti)});
    std::printf("overshoot by fs: 500→%.1f%%  1000→%.1f%%  5000→%.1f%%\n", os_by_fs[500], os_by_fs[1000], os_by_fs[5000]);

    // --- Padé(2 階)延遲模型的 PM ---
    std::map<int, double> pm_pade;
    std::vector<std::pair<std::string, TF>> items = {{"no_delay", C * P}};
    for (int fs : {500, 2000}) {
        TF Ld = C * P * pade2(1.5 / fs);
        pm_pade[fs] = margins(Ld).pm_deg;
        items.push_back({"fs" + std::to_string(fs), Ld});
        std::printf("Padé fs=%d Hz:PM=%.1f°(直線公式 %.1f°)\n", fs, pm_pade[fs], theory[fs]);
    }
    write_bode_csv("ch04_bode_delay.csv", logspace(0, 3, 500), items);

    Checker chk("Ch4");
    CHECK(chk, os_by_fs[500] > os_by_fs[1000] && os_by_fs[1000] > os_by_fs[5000]);  // 取樣率越低振鈴越大
    CHECK(chk, std::fabs(pm_pade[500] - theory[500]) < 6);                          // Padé vs 直線公式一致
    CHECK(chk, std::fabs(pm_pade[2000] - theory[2000]) < 6);
    CHECK(chk, os_by_fs[5000] < 8);                                                  // 高取樣率接近連續
    return chk.finish();
}

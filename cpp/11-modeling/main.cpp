// Ch11 建模入門:DC 馬達完整二階模型 vs 降階模型,逐樣本與解析交叉驗證
#include "../common/csd.hpp"
using namespace csd;

int main() {
    const double R = 1.0, L_ind = 1e-3, Kt = 0.1, Ke = 0.1, J = 1e-4, b = 1e-5;

    // 完整二階模型 V -> w:Kt / ((L s + R)(J s + b) + Kt Ke)
    const TF P_full = tf({Kt}, polyadd(polymul({L_ind, R}, {J, b}), {Kt * Ke}));
    // 降階(忽略電感)
    const TF P_red = tf({Kt}, polyadd(polyscale({J, b}, R), {Kt * Ke}));

    const double tau_e = L_ind / R, tau_m = R * J / (R * b + Kt * Ke);
    std::printf("電氣時間常數 τe = %.2f ms(極點 %.0f rad/s ≈ %.0f Hz)\n", tau_e * 1000, 1 / tau_e, 1 / tau_e / 2 / PI);
    std::printf("機械時間常數 τm = %.2f ms(極點 %.0f rad/s ≈ %.1f Hz)\n", tau_m * 1000, 1 / tau_m, 1 / tau_m / 2 / PI);
    std::printf("完整模型極點:");
    for (cplx p : poles(P_full)) std::printf(" %.2f%+.2fj", p.real(), p.imag());
    std::printf("\n");
    write_bode_csv("ch11_bode_full_vs_reduced.csv", logspace(0, 4, 600), {{"full", P_full}, {"reduced", P_red}});

    // --- 兩條獨立路徑:逐樣本 Euler(1 µs)vs 解析步階響應 ---
    const double dt_sim = 1e-6, t_end = 0.05;
    const int n = int(t_end / dt_sim);
    DCMotorPlant plant(R, L_ind, Kt, Ke, J, b, dt_sim);
    Vec w_sim(n), i_sim(n);
    for (int k = 0; k < n; ++k) {
        i_sim[k] = plant.i;
        w_sim[k] = plant.w;
        plant.step(1.0);  // 1 V 步階
    }
    Vec t_sim = sample_times(n, dt_sim), t_tf;
    for (int k = 0; k < n; k += 50) t_tf.push_back(t_sim[k]);
    Vec w_tf = step_response(P_full, t_tf);
    Vec w_tf_i = interp(t_sim, t_tf, w_tf);
    double max_err = 0.0;
    for (int k = 0; k < n; ++k) max_err = std::max(max_err, std::fabs(w_sim[k] - w_tf_i[k]));
    max_err /= w_tf.back();
    const double dc_full = dcgain(P_full), dc_red = dcgain(P_red);
    std::printf("兩實作最大相對誤差:%.3f%%\n", max_err * 100);
    std::printf("穩態轉速:模擬 %.3f | 解析 DC 增益 %.3f rad/s\n", w_sim.back(), dc_full);
    write_csv("ch11_step_crosscheck.csv", {"t_analytic", "w_analytic", "t_sim_decimated", "w_sim_decimated"},
              {t_tf, w_tf, t_tf, interp(t_tf, t_sim, w_sim)});

    const double d1 = mag_db_at(P_full, 1.0) - mag_db_at(P_red, 1.0);
    const double d1k = mag_db_at(P_full, 1000.0) - mag_db_at(P_red, 1000.0);
    std::printf("完整 vs 降階增益差:1 Hz %.3f dB | 1 kHz %.2f dB\n", d1, d1k);

    Checker chk("Ch11");
    CHECK(chk, max_err < 0.01);                                 // 交叉驗證 < 1%
    CHECK(chk, std::fabs(dc_full - dc_red) / dc_full < 1e-6);   // 降階不改 DC
    CHECK(chk, std::fabs(d1) < 0.1);                            // 1 Hz:重合
    CHECK(chk, std::fabs(d1k) > 3);                             // 1 kHz:明顯分歧
    CHECK(chk, std::fabs(w_sim.back() - dc_full) / dc_full < 0.01);
    return chk.finish();
}

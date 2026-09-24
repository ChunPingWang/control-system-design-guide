// Ch10 Luenberger 觀測器:極點配置、從錯誤初值收斂、由量化位置重建速度
#include "../common/csd.hpp"
using namespace csd;

const double J = 0.002, b = 0.01;

int main() {
    Mat A(2, 2), B(2, 1), Cm(1, 2);
    A(0, 1) = 1.0; A(1, 1) = -b / J;
    B(1, 0) = 1.0 / J;
    Cm(0, 0) = 1.0;

    // 可觀測性矩陣 [C; CA] 的行列式 ≠ 0 → 秩 2
    Mat CA = Cm * A;
    double obs_det = Cm(0, 0) * CA(0, 1) - Cm(0, 1) * CA(0, 0);
    std::printf("可觀測性矩陣行列式 = %g(≠0 → 秩 2)\n", obs_det);

    // 觀測器極點:比預期迴路頻寬(~40 Hz ≈ 250 rad/s)稍快;對偶:L = place(Aᵀ, Cᵀ)ᵀ
    const std::vector<cplx> obs_poles = {-200.0, -220.0};
    Mat At(2, 2), Ct(2, 1);
    for (int i = 0; i < 2; ++i) for (int j = 0; j < 2; ++j) At(i, j) = A(j, i);
    Ct(0, 0) = Cm(0, 0); Ct(1, 0) = Cm(0, 1);
    Mat Kt = place_siso(At, Ct, obs_poles);
    const double L0 = Kt(0, 0), L1 = Kt(0, 1);
    Mat Lm(2, 1);
    Lm(0, 0) = L0; Lm(1, 0) = L1;
    auto eig = eig2(A - Lm * Cm);
    std::printf("觀測器增益 L = [%.1f, %.1f]\n驗證極點: %.4f, %.4f\n", L0, L1, eig[0].real(), eig[1].real());

    // 觀測器一步(前向 Euler):x̂ += (A x̂ + B u + L (y - C x̂)) dt
    auto obs_step = [&](double& x0, double& x1, double u, double y, double dt) {
        double innov = y - x0;
        double dx0 = x1 + L0 * innov;
        double dx1 = -b / J * x1 + u / J + L1 * innov;
        x0 += dx0 * dt;
        x1 += dx1 * dt;
    };

    const double fs = 1000.0, dt = 1 / fs;
    const int n = int(1.0 / dt);
    Vec t = sample_times(n, dt), u_torque(n);
    for (int k = 0; k < n; ++k) u_torque[k] = 0.05 * std::sin(2 * PI * 2 * t[k]);

    // --- 實驗 1:從錯誤初值收斂 ---
    MotorPlant plant(J, b, dt);
    double x0 = 0.0, x1 = -2.0;  // 故意給錯的初始估測
    Vec w_true(n), w_hat(n);
    for (int k = 0; k < n; ++k) {
        w_true[k] = plant.w;
        w_hat[k] = x1;
        obs_step(x0, x1, u_torque[k], plant.pos, dt);  // 只量位置
        plant.step(u_torque[k]);
    }
    double err_early = std::fabs(w_true[0] - w_hat[0]), err_late = 0.0;
    for (int k = n / 2; k < n; ++k) err_late = std::max(err_late, std::fabs(w_true[k] - w_hat[k]));
    std::printf("初始速度誤差 %.2f → 後半段最大誤差 %.5f\n", err_early, err_late);
    write_csv("ch10_convergence.csv", {"t", "w_true", "w_hat"}, {t, w_true, w_hat});

    // --- 實驗 2:量化位置 → 差分速度 vs 觀測器速度 ---
    EncoderModel enc(2500);  // 10000 counts/rev
    const double q_rad = 2 * PI / enc.counts_per_rev;
    MotorPlant plant2(J, b, dt);
    x0 = x1 = 0.0;
    double prev_y = 0.0;
    Vec w_true2(n), w_fd(n), w_obs(n);
    for (int k = 0; k < n; ++k) {
        w_true2[k] = plant2.w;
        double y_meas = enc.read(plant2.pos / (2 * PI)) * 2 * PI;  // 量化位置 [rad]
        w_fd[k] = (y_meas - prev_y) / dt;
        prev_y = y_meas;
        obs_step(x0, x1, u_torque[k], y_meas, dt);
        w_obs[k] = x1;
        plant2.step(u_torque[k]);
    }
    Vec e_fd, e_obs;
    for (int k = 200; k < n; ++k) {
        e_fd.push_back(w_fd[k] - w_true2[k]);
        e_obs.push_back(w_obs[k] - w_true2[k]);
    }
    double noise_fd = stdev(e_fd), noise_obs = stdev(e_obs);
    std::printf("速度誤差 RMS:差分 %.4f | 觀測器 %.4f(改善 %.1f 倍)\n", noise_fd, noise_obs, noise_fd / noise_obs);
    std::printf("差分雜訊理論值 ~ q/(sqrt(6)·Ts) = %.4f\n", q_rad / std::sqrt(6.0) / dt);
    write_csv("ch10_quantized.csv", {"t", "w_true", "w_fd", "w_obs"}, {t, w_true2, w_fd, w_obs});

    Checker chk("Ch10");
    std::vector<double> got = {eig[0].real(), eig[1].real()};
    std::sort(got.begin(), got.end());
    CHECK(chk, std::fabs(got[0] + 220) < 220e-6 && std::fabs(got[1] + 200) < 200e-6);
    CHECK(chk, err_early > 1.5 && err_late < 0.05);  // 從大誤差收斂(殘差為 Euler 離散化誤差)
    CHECK(chk, noise_obs < noise_fd / 3);           // 觀測器速度乾淨得多
    CHECK(chk, noise_fd > q_rad / dt / 10);         // 差分雜訊量級與理論一致
    return chk.finish();
}

// Ch18 運動控制觀測器:速度迴路回授用「編碼器差分」vs「Luenberger 觀測器」
#include "../common/csd.hpp"
using namespace csd;

const double J = 0.002, b = 0.01, fs = 1000.0, dt = 1 / fs;
const int n = int(2.0 / dt);

struct LoopLog { Vec u, w, fb; };

int main() {
    // 觀測器增益:與 Ch10 相同的 Ackermann 對偶設計,極點 -200、-220
    Mat At(2, 2), Ct(2, 1);
    At(1, 0) = 1.0; At(1, 1) = -b / J;  // Aᵀ
    Ct(0, 0) = 1.0;
    Mat Kt = place_siso(At, Ct, {-200.0, -220.0});
    const double L0 = Kt(0, 0), L1 = Kt(0, 1);
    EncoderModel enc(2500);
    const double q_rad = 2 * PI / enc.counts_per_rev;

    auto vel_loop = [&](bool use_observer, double kp, double ki = 5.0) {
        MotorPlant plant(J, b, dt);
        DiscretePID c(kp, ki, 0.0, dt);
        double x0 = 0, x1 = 0, prev_y = 0, u_prev = 0;
        LoopLog L{Vec(n), Vec(n), Vec(n)};
        for (int k = 0; k < n; ++k) {
            L.w[k] = plant.w;
            double y = enc.read(plant.pos / (2 * PI)) * 2 * PI;
            double w_fd = (y - prev_y) / dt;
            prev_y = y;
            double innov = y - x0;
            double dx0 = x1 + L0 * innov, dx1 = -b / J * x1 + u_prev / J + L1 * innov;
            x0 += dx0 * dt;
            x1 += dx1 * dt;
            double fb = use_observer ? x1 : w_fd;
            L.fb[k] = fb;
            double u = c.step(1.0 - fb);
            L.u[k] = u_prev = u;
            plant.step(u);
        }
        return L;
    };

    const size_t settle = size_t(1.0 / dt);
    const double kp0 = 0.5;
    LoopLog fd = vel_loop(false, kp0), ob = vel_loop(true, kp0);
    const double rms_fd = stdev(slice(fd.u, settle)), rms_ob = stdev(slice(ob.u, settle));
    std::printf("kp=%.1f:轉矩雜訊 RMS 差分 %.4f → 觀測器 %.4f(改善 %.0f 倍)\n", kp0, rms_fd, rms_ob, rms_fd / rms_ob);
    write_csv("ch18_fd_vs_observer.csv", {"t", "fb_fd", "fb_obs", "u_fd", "u_obs"}, {sample_times(n, dt), fd.fb, ob.fb, fd.u, ob.u});

    Vec kps = {0.25, 0.5, 1.0, 1.5}, noise_fd, noise_ob, theory;
    std::printf("%6s %9s %9s %7s %9s\n", "kp", "FD", "observer", "ratio", "theory");
    for (double kp : kps) {
        noise_fd.push_back(stdev(slice(vel_loop(false, kp).u, settle)));
        noise_ob.push_back(stdev(slice(vel_loop(true, kp).u, settle)));
        theory.push_back(kp * q_rad / std::sqrt(6.0) / dt);
        std::printf("%6.2f %9.4f %9.4f %7.1f %9.4f\n", kp, noise_fd.back(), noise_ob.back(), noise_fd.back() / noise_ob.back(), theory.back());
    }
    write_csv("ch18_noise_vs_kp.csv", {"kp", "noise_fd", "noise_obs", "theory"}, {kps, noise_fd, noise_ob, theory});

    Checker chk("Ch18");
    CHECK(chk, rms_ob < rms_fd / 10);                                   // 至少一個數量級
    CHECK(chk, std::fabs(mean(slice(ob.w, settle)) - 1.0) < 0.01);      // 追蹤沒有犧牲
    CHECK(chk, std::fabs(mean(slice(fd.w, settle)) - 1.0) < 0.02);      // (平均;瞬時值受雜訊擾動)
    for (size_t i = 0; i < kps.size(); ++i) CHECK(chk, noise_ob[i] < noise_fd[i] / 5);
    for (size_t i = 0; i < kps.size(); ++i) CHECK(chk, 0.3 < noise_fd[i] / theory[i] && noise_fd[i] / theory[i] < 3);  // FD 雜訊與理論同量級
    return chk.finish();
}

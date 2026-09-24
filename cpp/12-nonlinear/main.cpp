// Ch12 非線性:積分器 windup、庫倫摩擦、背隙
#include "../common/csd.hpp"
using namespace csd;

const double J = 0.002, b = 0.01, fs = 1000.0, dt = 1 / fs;
const int n = int(1.5 / dt);

int main() {
    Vec t = sample_times(n, dt);

    // --- 實驗 1:積分器 windup ---
    const double u_max = 0.3, target = 5.0;  // 轉矩飽和 ±0.3 Nm;大步階 → 飽和很久
    auto run_windup = [&](bool anti_windup, Vec& y, Vec& u) {
        MotorPlant plant(J, b, dt);
        DiscretePID ctrl(0.5, 20.0, 0.0, dt, -u_max, u_max, 0.0, anti_windup);
        y.assign(n, 0);
        u.assign(n, 0);
        for (int k = 0; k < n; ++k) {
            y[k] = plant.w;
            u[k] = ctrl.step(target - plant.w);
            plant.step(u[k]);
        }
    };
    Vec y_naive, u_naive, y_aw, u_aw;
    run_windup(false, y_naive, u_naive);
    run_windup(true, y_aw, u_aw);
    const double os_naive = vmax(y_naive) - target, os_aw = vmax(y_aw) - target;
    std::printf("overshoot:naive %.3f rad/s | anti-windup %.3f rad/s\n", os_naive, os_aw);
    write_csv("ch12_windup.csv", {"t", "y_naive", "u_naive", "y_antiwindup", "u_antiwindup"}, {t, y_naive, u_naive, y_aw, u_aw});

    // --- 實驗 2:庫倫摩擦 ---
    Vec w_ref(n);
    for (int k = 0; k < n; ++k) w_ref[k] = 0.5 * std::sin(2 * PI * 1.0 * t[k]);
    auto run_friction = [&](double fc) {
        MotorPlant plant(J, b, dt);
        DiscretePID ctrl(0.5, 5.0, 0.0, dt);
        Vec y(n);
        for (int k = 0; k < n; ++k) {
            y[k] = plant.w;
            double u = ctrl.step(w_ref[k] - plant.w);
            plant.step(u, coulomb(plant.w, fc));
        }
        return y;
    };
    Vec y_nofric = run_friction(0.0), y_fric = run_friction(0.05);
    const int k0 = int(0.3 / dt);
    Vec err_fric, err_nof;
    for (int k = k0; k < n; ++k) {
        err_fric.push_back(std::fabs(w_ref[k] - y_fric[k]));
        err_nof.push_back(std::fabs(w_ref[k] - y_nofric[k]));
    }
    std::printf("最大追蹤誤差:無摩擦 %.4f | 有摩擦 %.4f\n", vmax(err_nof), vmax(err_fric));
    write_csv("ch12_friction.csv", {"t", "w_ref", "w_no_friction", "w_friction"}, {t, w_ref, y_nofric, y_fric});

    // --- 實驗 3:背隙 ---
    Backlash bl(0.02);  // 總失動量 0.02 rad
    Vec motor_pos(n), load_pos(n), gap(n);
    for (int k = 0; k < n; ++k) {
        motor_pos[k] = 0.05 * std::sin(2 * PI * 0.5 * t[k]);
        load_pos[k] = bl.step(motor_pos[k]);
        gap[k] = motor_pos[k] - load_pos[k];
    }
    std::printf("失動量範圍:[%.4f, %.4f](理論 ±0.01 rad)\n", vmin(gap), vmax(gap));
    write_csv("ch12_backlash.csv", {"t", "motor_pos", "load_pos"}, {t, motor_pos, load_pos});

    // 摩擦誤差尖峰應靠近速度過零點
    std::vector<int> zc;
    auto sgn = [](double v) { return (v > 0) - (v < 0); };
    for (int k = k0; k + 1 < n; ++k)
        if (sgn(w_ref[k + 1]) != sgn(w_ref[k])) zc.push_back(k - k0);
    const int peak_idx = int(argmax(err_fric));
    int nearest = n;
    for (int z : zc) nearest = std::min(nearest, std::abs(peak_idx - z));

    Checker chk("Ch12");
    CHECK(chk, os_naive > 2 * std::max(os_aw, 0.05));  // windup 惡化 overshoot
    CHECK(chk, os_aw < 0.5);
    CHECK(chk, max_abs(u_naive) <= u_max + 1e-9);      // 飽和確實生效
    CHECK(chk, vmax(err_fric) > 3 * vmax(err_nof));    // 摩擦造成明顯誤差尖峰
    CHECK(chk, nearest < int(0.15 / dt));              // 尖峰靠近速度過零
    CHECK(chk, std::fabs(vmax(gap) - 0.01) < 1e-3 && std::fabs(vmin(gap) + 0.01) < 1e-3);
    return chk.finish();
}

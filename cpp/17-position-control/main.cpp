// Ch17 位置迴路:串級(P 位置 / PI 速度,含速度極限)vs 單迴路 PID
#include "../common/csd.hpp"
using namespace csd;

const double J = 0.002, b = 0.01, fs = 1000.0, dt = 1 / fs;
const int n = int(2.0 / dt);
const double V_MAX = 2.0, U_MAX = 1.0;  // 速度極限 2 rad/s、轉矩極限 1 Nm
const double DIST_T = 1.2;              // t=1.2 s 加入 0.05 Nm 負載

struct Log { Vec pos, w; };

Log run_cascade(const Vec& t, double step = 1.0, double kp_pos = 30.0) {
    MotorPlant plant(J, b, dt);
    DiscretePID vel(0.5, 5.0, 0.0, dt, -U_MAX, U_MAX);
    Log L{Vec(n), Vec(n)};
    for (int k = 0; k < n; ++k) {
        L.pos[k] = plant.pos;
        L.w[k] = plant.w;
        double v_cmd = saturate(kp_pos * (step - plant.pos), -V_MAX, V_MAX);
        plant.step(vel.step(v_cmd - plant.w), t[k] >= DIST_T ? 0.05 : 0.0);
    }
    return L;
}

Log run_pid(const Vec& t, double step = 1.0) {
    MotorPlant plant(J, b, dt);
    DiscretePID c(15.0, 20.0, 0.5, dt, -U_MAX, U_MAX, 100);
    Log L{Vec(n), Vec(n)};
    for (int k = 0; k < n; ++k) {
        L.pos[k] = plant.pos;
        L.w[k] = plant.w;
        plant.step(c.step(step - plant.pos), t[k] >= DIST_T ? 0.05 : 0.0);
    }
    return L;
}

int main() {
    Vec t = sample_times(n, dt);
    Log c = run_cascade(t), p = run_pid(t);
    write_csv("ch17_cascade_vs_pid.csv", {"t", "pos_cascade", "w_cascade", "pos_pid", "w_pid"}, {t, c.pos, c.w, p.pos, p.w});

    double dev_c = 0, dev_p = 0;
    for (int k = int(DIST_T / dt); k < n; ++k) {
        dev_c = std::max(dev_c, std::fabs(1 - c.pos[k]));
        dev_p = std::max(dev_p, std::fabs(1 - p.pos[k]));
    }
    std::printf("串級:max|w|=%.2f(極限 %.1f),擾動偏移 %.2f mrad\n", max_abs(c.w), V_MAX, dev_c * 1000);
    std::printf("PID :max|w|=%.2f(未受控!),擾動偏移 %.2f mrad\n", max_abs(p.w), dev_p * 1000);

    // --- 等速段追隨誤差:斜坡命令 ---
    const double ramp_v = 1.0;
    std::map<double, double> ferr;
    std::vector<Vec> cols = {t};
    for (double kp_pos : {15.0, 30.0, 60.0}) {
        MotorPlant plant(J, b, dt);
        DiscretePID vel(0.5, 5.0, 0.0, dt, -U_MAX, U_MAX);
        Vec err(n);
        for (int k = 0; k < n; ++k) {
            err[k] = ramp_v * t[k] - plant.pos;
            double v_cmd = saturate(kp_pos * err[k], -V_MAX, V_MAX);
            plant.step(vel.step(v_cmd - plant.w));
        }
        ferr[kp_pos] = mean(slice(err, int(1.0 / dt)));
        cols.push_back(err);
        std::printf("kp_pos=%.0f: 量測 %.2f mrad,理論 %.2f mrad\n", kp_pos, ferr[kp_pos] * 1000, ramp_v / kp_pos * 1000);
    }
    write_csv("ch17_ramp_following_error.csv", {"t", "err_kp15", "err_kp30", "err_kp60"}, cols);

    Checker chk("Ch17");
    CHECK(chk, max_abs(c.w) <= V_MAX * 1.05);  // 串級尊重速度極限
    CHECK(chk, max_abs(p.w) > 3 * V_MAX);      // PID 無速度概念
    CHECK(chk, vmax(c.pos) < 1.005);           // 串級無 overshoot(梯形到位)
    CHECK(chk, std::fabs(c.pos.back() - 1) < 1e-3 && std::fabs(p.pos.back() - 1) < 5e-3);
    CHECK(chk, dev_c < dev_p);                 // 串級剛性較好(內環先擋)
    for (auto& [kp_pos, e] : ferr) CHECK(chk, std::fabs(e - ramp_v / kp_pos) / (ramp_v / kp_pos) < 0.15);  // 追隨誤差 = v/kv
    return chk.finish();
}

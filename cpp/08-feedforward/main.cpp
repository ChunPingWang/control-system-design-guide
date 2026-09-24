// Ch8 前饋:梯形速度規劃 + 速度 / 加速度前饋,追隨誤差量化
#include "../common/csd.hpp"
using namespace csd;

const double J = 0.002, b = 0.01, fs = 1000.0, dt = 1 / fs;

int main() {
    // --- 梯形速度規劃:加速 0.1 s、等速 0.3 s、減速 0.1 s ---
    const double acc_max = 20.0, t_acc = 0.1, t_cruise = 0.3;
    const int n = int(0.8 / dt);
    Vec t = sample_times(n, dt), acc_ref(n, 0.0);
    for (int k = 0; k < n; ++k) {
        if (t[k] < t_acc) acc_ref[k] = acc_max;
        else if (t[k] >= t_acc + t_cruise && t[k] < 2 * t_acc + t_cruise) acc_ref[k] = -acc_max;
    }
    Vec vel_ref = cumsum(acc_ref);
    for (double& v : vel_ref) v *= dt;
    Vec pos_ref = cumsum(vel_ref);
    for (double& p : pos_ref) p *= dt;

    auto run = [&](double kvff, double kaff, Vec* pos_out = nullptr, double kp_pos = 30.0) {
        MotorPlant plant(J, b, dt);
        DiscretePID vel_pi(0.5, 5.0, 0.0, dt);
        Vec pos(n), err(n);
        for (int k = 0; k < n; ++k) {
            pos[k] = plant.pos;
            err[k] = pos_ref[k] - plant.pos;
            double v_cmd = kp_pos * err[k] + kvff * vel_ref[k];
            double u = vel_pi.step(v_cmd - plant.w) + kaff * J * acc_ref[k];
            plant.step(u);
        }
        if (pos_out) *pos_out = pos;
        return err;
    };

    struct Case { std::string name; double kv, ka; };
    std::vector<Case> cases = {{"no FF", 0, 0}, {"vel FF", 1, 0}, {"vel + acc FF", 1, 1}};
    std::map<std::string, Vec> errs;
    std::map<std::string, double> peak;
    std::vector<Vec> cols = {t, pos_ref};
    for (auto& c : cases) {
        Vec pos;
        errs[c.name] = run(c.kv, c.ka, &pos);
        peak[c.name] = max_abs(errs[c.name]);
        cols.push_back(pos);
        cols.push_back(errs[c.name]);
    }
    std::printf("峰值追隨誤差 [mrad]: no FF %.2f | vel FF %.2f | vel + acc FF %.2f\n", peak["no FF"] * 1000,
                peak["vel FF"] * 1000, peak["vel + acc FF"] * 1000);
    write_csv("ch08_feedforward.csv", {"t", "pos_ref", "pos_noFF", "err_noFF", "pos_velFF", "err_velFF", "pos_velaccFF", "err_velaccFF"}, cols);

    // --- Kvff 掃描 ---
    Vec kvs = arange(0.0, 1.21, 0.2), peaks_kv, os_kv;
    for (double kv : kvs) {
        Vec pos;
        peaks_kv.push_back(max_abs(run(kv, 0.0, &pos)));
        os_kv.push_back(std::max(0.0, vmax(pos) - vmax(pos_ref)));
    }
    std::printf("Kvff 掃描峰值誤差 [mrad]:");
    for (size_t i = 0; i < kvs.size(); ++i) std::printf(" %.1f:%.2f", kvs[i], peaks_kv[i] * 1000);
    std::printf("\n");
    write_csv("ch08_kvff_sweep.csv", {"kvff", "peak_err", "endpoint_overshoot"}, {kvs, peaks_kv, os_kv});

    // 無前饋時,等速段誤差 ≈ v / kp_pos
    Vec cruise;
    for (int k = 0; k < n; ++k)
        if (t[k] > t_acc + 0.05 && t[k] < t_acc + t_cruise - 0.05) cruise.push_back(errs["no FF"][k]);
    double e_cruise = mean(cruise), e_theory = vmax(vel_ref) / 30.0;
    std::printf("等速段誤差 %.2f mrad(理論 v/kp = %.2f mrad)\n", e_cruise * 1000, e_theory * 1000);

    Checker chk("Ch8");
    CHECK(chk, peak["no FF"] > peak["vel FF"] && peak["vel FF"] > peak["vel + acc FF"]);
    CHECK(chk, peak["vel FF"] < 0.25 * peak["no FF"]);        // 速度前饋至少改善 4 倍
    CHECK(chk, peak["vel + acc FF"] < 0.5 * peak["vel FF"]);  // 加速度前饋再改善
    CHECK(chk, std::fabs(e_cruise - e_theory) / e_theory < 0.15);
    return chk.finish();
}

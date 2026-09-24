// Ch6 四種控制器:P / PI / PID / PID+(2-DOF)同場對決,命令步階 + 負載擾動
#include "../common/csd.hpp"
using namespace csd;

const double J = 0.002, b = 0.01, fs = 1000.0, dt = 1 / fs;
const int n = int(1.0 / dt);

struct Design {
    std::string name;
    double kp, ki, kd, sp_weight;
};

// 2-DOF PID:P/D 作用在 sp_weight*r - y,I 作用在 r - y
Vec run(const Design& d, const Vec& dist, double dfilt_hz = 100.0) {
    MotorPlant plant(J, b, dt);
    DiscretePID pd_part(d.kp, 0.0, d.kd, dt, -INF, INF, dfilt_hz);
    DiscretePID i_part(0.0, d.ki, 0.0, dt);
    Vec y(n);
    for (int k = 0; k < n; ++k) {
        y[k] = plant.w;
        double u = pd_part.step(d.sp_weight * 1.0 - plant.w) + i_part.step(1.0 - plant.w);
        plant.step(u, dist[k]);
    }
    return y;
}

int main() {
    Vec t = sample_times(n, dt), dist(n);
    for (int k = 0; k < n; ++k) dist[k] = t[k] >= 0.5 ? 0.1 : 0.0;  // +0.1 Nm 負載轉矩

    std::vector<Design> designs = {
        {"P    (kp=0.5)", 0.5, 0.0, 0.0, 1.0},
        {"PI   (kp=0.5, ki=15)", 0.5, 15.0, 0.0, 1.0},
        {"PID  (kp=0.8, ki=25, kd=4m)", 0.8, 25.0, 0.004, 1.0},
        {"PID+ (2-DOF b=0.6)", 0.8, 25.0, 0.004, 0.6},
    };
    struct Row { double os, sse_cmd, dip, sse_end; };
    std::vector<Row> mt;
    std::vector<Vec> cols = {t};
    const int half = n / 2;
    std::printf("%-28s %6s %9s %9s %9s\n", "controller", "OS%", "sse(cmd)", "dist dip", "sse(end)");
    for (auto& d : designs) {
        Vec y = run(d, dist);
        cols.push_back(y);
        StepMetrics m1 = step_metrics(slice(t, 0, half), slice(y, 0, half));
        Row r{m1.overshoot_pct, m1.steady_state_error, 1.0 - vmin(slice(y, half)), 1.0 - y.back()};
        mt.push_back(r);
        std::printf("%-28s %6.1f %9.4f %9.4f %9.4f\n", d.name.c_str(), r.os, r.sse_cmd, r.dip, r.sse_end);
    }
    write_csv("ch06_controllers.csv", {"t", "P", "PI", "PID", "PIDplus"}, cols);

    // --- 開迴路形狀與 PM ---
    TF Ps = inertia(J, b);
    std::vector<std::pair<std::string, TF>> items = {
        {"P", pid_ctrl(0.5) * Ps}, {"PI", pid_ctrl(0.5, 15) * Ps}, {"PID", pid_ctrl(0.8, 25, 0.004, 2 * PI * 100) * Ps}};
    for (auto& [name, L] : items) std::printf("%-4s PM = %.0f°\n", name.c_str(), margins(L).pm_deg);
    write_bode_csv("ch06_open_loop.csv", logspace(0, 3, 500), items);

    const Row &mP = mt[0], &mPI = mt[1], &mPID = mt[2], &mPIDp = mt[3];
    Checker chk("Ch6");
    CHECK(chk, mP.dip > 0.05 && std::fabs(mP.sse_end) > 0.05);  // P:擾動留下穩態誤差
    CHECK(chk, std::fabs(mPI.sse_end) < 0.005);                 // PI:擾動誤差歸零
    CHECK(chk, std::fabs(mPID.sse_end) < 0.005);
    CHECK(chk, mPID.dip < mPI.dip);                             // PID 擾動跌落較小(增益較高)
    CHECK(chk, mPIDp.os < mPID.os - 2);                         // 2-DOF 壓 overshoot
    CHECK(chk, std::fabs(mPIDp.dip - mPID.dip) < 0.01);         // 且不犧牲擾動抑制
    return chk.finish();
}

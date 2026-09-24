// Ch1 回授導論:開迴路 vs 閉迴路,面對受控體增益漂移與階躍擾動
#include "../common/csd.hpp"
using namespace csd;

int main() {
    const double tau = 0.1, K_nom = 2.0;
    const TF C = pi_ctrl(2.0, 20.0);
    const Vec t = linspace(0, 1.5, 1500);

    // --- 實驗 1:受控體增益 K 漂移 ---
    std::map<double, std::pair<double, double>> results;  // K -> (開迴路誤差, 閉迴路誤差)
    std::vector<std::string> hdr = {"t"};
    std::vector<Vec> cols = {t};
    for (double K : {1.0, 2.0, 3.0}) {
        TF P = first_order(tau, K);
        Vec y_o = forced_response(P, t, Vec(t.size(), 1.0 / K_nom));  // u = 1/K_nom
        Vec y_c = step_response(closed_loop(C, P), t);
        results[K] = {1 - y_o.back(), 1 - y_c.back()};
        hdr.push_back("open_K" + std::to_string(int(K)));
        cols.push_back(y_o);
        hdr.push_back("closed_K" + std::to_string(int(K)));
        cols.push_back(y_c);
    }
    std::printf("%4s %14s %14s\n", "K", "開迴路穩態誤差", "閉迴路穩態誤差");
    for (auto& [K, e] : results) std::printf("%4.1f %14.4f %14.6f\n", K, e.first, e.second);
    write_csv("ch01_gain_variation.csv", hdr, cols);

    // --- 實驗 2:t=0.75 s 階躍擾動 ---
    TF P = first_order(tau, K_nom);
    Vec d(t.size()), u_open(t.size());
    for (size_t i = 0; i < t.size(); ++i) {
        d[i] = t[i] >= 0.75 ? -0.5 : 0.0;
        u_open[i] = 1.0 / K_nom + d[i];
    }
    Vec y_open = forced_response(P, t, u_open);
    // 閉迴路:命令與擾動分別響應後疊加(線性疊加原理)
    Vec y_r = step_response(closed_loop(C, P), t);
    Vec y_d = forced_response(feedback(P, C), t, d);  // d -> y = P/(1+CP)
    Vec y_closed(t.size());
    for (size_t i = 0; i < t.size(); ++i) y_closed[i] = y_r[i] + y_d[i];
    std::printf("開迴路最終誤差 : %.4f\n閉迴路最終誤差 : %.6f\n", 1 - y_open.back(), 1 - y_closed.back());
    write_csv("ch01_disturbance.csv", {"t", "open", "closed"}, {t, y_open, y_closed});

    Checker chk("Ch1");
    CHECK(chk, std::fabs(results[2.0].first) < 1e-3);           // 標稱下開迴路無誤差
    CHECK(chk, std::fabs(results[1.0].first - 0.5) < 0.01);     // K=1 時開迴路誤差 50%
    for (double K : {1.0, 2.0, 3.0}) CHECK(chk, std::fabs(results[K].second) < 1e-3);  // 閉迴路穩態誤差趨近 0
    CHECK(chk, std::fabs(1 - y_open.back() - 0.5 * K_nom * 0.5) > 0.2);  // 開迴路留下明顯擾動誤差
    CHECK(chk, std::fabs(1 - y_closed.back()) < 1e-3);                  // 閉迴路把擾動誤差收斂掉
    return chk.finish();
}

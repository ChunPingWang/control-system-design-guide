// Ch14 編碼器與量化雜訊:差分速度雜訊 vs 解析度;移動平均的雜訊/延遲取捨
#include "../common/csd.hpp"
using namespace csd;

int main() {
    const double fs = 1000.0, dt = 1 / fs;
    const int n = 2000;
    Vec t = sample_times(n, dt), w_true(n);
    for (int k = 0; k < n; ++k) w_true[k] = 1.0 + 0.5 * std::sin(2 * PI * 3 * t[k]);  // 變速,避免量化誤差週期化
    Vec pos_rev = cumsum(w_true);
    for (double& p : pos_rev) p *= dt / (2 * PI);  // 位置 [rev]

    // 差分速度(np.diff(..., prepend=0) / dt)
    auto fd_velocity = [&](const EncoderModel& enc) {
        Vec v(n);
        double prev = 0.0;
        for (int k = 0; k < n; ++k) {
            double pq = enc.read(pos_rev[k]) * 2 * PI;
            v[k] = (pq - prev) / dt;
            prev = pq;
        }
        return v;
    };
    auto err_rms = [&](const Vec& v) {
        Vec e;
        for (int k = 100; k < n; ++k) e.push_back(v[k] - w_true[k]);
        return stdev(e);
    };

    struct Res { double meas, theory; };
    std::map<int, Res> results;
    std::vector<Vec> cols = {t, w_true};
    for (int lines : {500, 2500, 10000}) {
        EncoderModel enc(lines);
        Vec v_fd = fd_velocity(enc);
        double q = 2 * PI / enc.counts_per_rev;
        results[lines] = {err_rms(v_fd), q / std::sqrt(6.0) / dt};
        cols.push_back(v_fd);
    }
    write_csv("ch14_resolution.csv", {"t", "w_true", "v_fd_500", "v_fd_2500", "v_fd_10000"}, cols);
    std::printf("%7s %10s %14s\n", "lines", "量測 RMS", "理論 q/(√6·Ts)");
    for (auto& [lines, r] : results) std::printf("%7d %10.4f %14.4f\n", lines, r.meas, r.theory);

    // --- 移動平均:雜訊 vs 延遲 ---
    Vec v_fd = fd_velocity(EncoderModel(2500));
    struct MA { double noise, lag_ms; };
    std::map<int, MA> ma;
    std::vector<Vec> cols2 = {t, w_true};
    for (int N : {1, 4, 16}) {
        Vec v_ma(n, 0.0);  // np.convolve(..., mode='full')[:n]
        for (int k = 0; k < n; ++k)
            for (int j = 0; j < N && j <= k; ++j) v_ma[k] += v_fd[k - j] / N;
        ma[N] = {err_rms(v_ma), (N - 1) / 2.0 * dt * 1000};
        cols2.push_back(v_ma);
        std::printf("MA%2d: 雜訊 RMS %.4f,群延遲 %.1f ms\n", N, ma[N].noise, ma[N].lag_ms);
    }
    write_csv("ch14_moving_average.csv", {"t", "w_true", "ma1", "ma4", "ma16"}, cols2);

    Checker chk("Ch14");
    for (auto& [lines, r] : results) CHECK(chk, std::fabs(r.meas - r.theory) / r.theory < 0.35);  // 與理論同量級
    CHECK(chk, results[500].meas > results[2500].meas && results[2500].meas > results[10000].meas);
    const double ratio = results[500].meas / results[10000].meas;
    CHECK(chk, 10 < ratio && ratio < 40);           // 解析度 20 倍 → 雜訊約 20 倍
    CHECK(chk, ma[16].noise < ma[1].noise / 2);     // 平均確實降雜訊
    CHECK(chk, ma[16].lag_ms > 5);                  // 代價:延遲
    return chk.finish();
}

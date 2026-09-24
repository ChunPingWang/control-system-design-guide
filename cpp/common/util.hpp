// util.hpp — 陣列、統計、步階指標、CSV 輸出與 ✅ 驗證檢查器
#pragma once
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "linalg.hpp"

namespace csd {

using Vec = std::vector<double>;
constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
constexpr double INF = std::numeric_limits<double>::infinity();

// ---------------------------------------------------------------- 陣列產生
inline Vec linspace(double a, double b, int n) {
    Vec v(n);
    for (int i = 0; i < n; ++i) v[i] = n == 1 ? a : a + (b - a) * i / (n - 1);
    return v;
}
inline Vec logspace(double a, double b, int n) {
    Vec v = linspace(a, b, n);
    for (double& x : v) x = std::pow(10.0, x);
    return v;
}
// numpy.arange 語意:長度 ceil((stop-start)/step)
inline Vec arange(double start, double stop, double step) {
    int n = std::max(0, int(std::ceil((stop - start) / step)));
    Vec v(n);
    for (int i = 0; i < n; ++i) v[i] = start + i * step;
    return v;
}
inline Vec sample_times(int n, double dt) {
    Vec t(n);
    for (int i = 0; i < n; ++i) t[i] = i * dt;
    return t;
}
inline Vec cumsum(const Vec& x) {
    Vec y(x.size());
    double s = 0.0;
    for (size_t i = 0; i < x.size(); ++i) y[i] = (s += x[i]);
    return y;
}
inline Vec slice(const Vec& x, size_t a, size_t b = size_t(-1)) {
    b = std::min(b, x.size());
    return a >= b ? Vec{} : Vec(x.begin() + a, x.begin() + b);
}
inline double round_to(double x, int digits) {
    double s = std::pow(10.0, digits);
    return std::round(x * s) / s;
}

// ---------------------------------------------------------------- 統計
inline double vmax(const Vec& x) { return *std::max_element(x.begin(), x.end()); }
inline double vmin(const Vec& x) { return *std::min_element(x.begin(), x.end()); }
inline size_t argmax(const Vec& x) { return std::max_element(x.begin(), x.end()) - x.begin(); }
inline size_t argmin(const Vec& x) { return std::min_element(x.begin(), x.end()) - x.begin(); }
inline double mean(const Vec& x) { return std::accumulate(x.begin(), x.end(), 0.0) / x.size(); }
inline double stdev(const Vec& x) {  // 母體標準差(numpy.std 預設 ddof=0)
    double m = mean(x), s = 0.0;
    for (double v : x) s += (v - m) * (v - m);
    return std::sqrt(s / x.size());
}
inline double max_abs(const Vec& x) {
    double m = 0.0;
    for (double v : x) m = std::max(m, std::fabs(v));
    return m;
}
inline double corrcoef(const Vec& x, const Vec& y) {
    double mx = mean(x), my = mean(y), sxy = 0, sxx = 0, syy = 0;
    for (size_t i = 0; i < x.size(); ++i) {
        sxy += (x[i] - mx) * (y[i] - my);
        sxx += (x[i] - mx) * (x[i] - mx);
        syy += (y[i] - my) * (y[i] - my);
    }
    return sxy / std::sqrt(sxx * syy);
}
// numpy.interp(xq, xp, fp),xp 遞增
inline Vec interp(const Vec& xq, const Vec& xp, const Vec& fp) {
    Vec out(xq.size());
    size_t j = 0;
    for (size_t i = 0; i < xq.size(); ++i) {
        double x = xq[i];
        if (x <= xp.front()) { out[i] = fp.front(); continue; }
        if (x >= xp.back()) { out[i] = fp.back(); continue; }
        while (j + 1 < xp.size() && xp[j + 1] < x) ++j;
        while (j > 0 && xp[j] > x) --j;
        double r = (x - xp[j]) / (xp[j + 1] - xp[j]);
        out[i] = fp[j] + r * (fp[j + 1] - fp[j]);
    }
    return out;
}
inline Vec unwrap_deg(Vec p) {
    for (size_t i = 1; i < p.size(); ++i) {
        while (p[i] - p[i - 1] > 180) p[i] -= 360;
        while (p[i] - p[i - 1] < -180) p[i] += 360;
    }
    return p;
}
// 第一個滿足條件的索引;找不到回傳 -1
template <class F>
inline long first_index(size_t n, F pred) {
    for (size_t i = 0; i < n; ++i)
        if (pred(i)) return long(i);
    return -1;
}

// ---------------------------------------------------------------- 步階指標
struct StepMetrics {
    double rise_time, overshoot_pct, settling_time, steady_state_error;
};

// 上升時間(10-90%)、overshoot %、2% 安定時間、穩態誤差(同 control_helpers.step_metrics)
inline StepMetrics step_metrics(const Vec& t, const Vec& y, double target = 1.0, double band = 0.02) {
    StepMetrics m{};
    long i10 = first_index(y.size(), [&](size_t i) { return y[i] >= 0.1 * target; });
    long i90 = first_index(y.size(), [&](size_t i) { return y[i] >= 0.9 * target; });
    m.rise_time = (i10 >= 0 && i90 >= 0) ? t[i90] - t[i10] : NaN;
    m.overshoot_pct = std::max(0.0, (vmax(y) - target) / std::fabs(target) * 100);
    long last_out = -1;
    for (size_t i = 0; i < y.size(); ++i)
        if (std::fabs(y[i] - target) > band * std::fabs(target)) last_out = long(i);
    if (last_out < 0) m.settling_time = t[0];
    else if (size_t(last_out) + 1 < y.size()) m.settling_time = t[last_out + 1];
    else m.settling_time = NaN;
    m.steady_state_error = target - y.back();
    return m;
}

// ---------------------------------------------------------------- 亂數
// 以固定種子產生可重現序列(數值與 numpy 不同,驗證條件為統計性質,不受影響)
struct Rng {
    std::mt19937_64 g;
    explicit Rng(unsigned long long seed) : g(seed) {}
    Vec normal(double mu, double sigma, size_t n) {
        std::normal_distribution<double> d(mu, sigma);
        Vec v(n);
        for (double& x : v) x = d(g);
        return v;
    }
    int bit() { return int(g() >> 63); }
};

// ---------------------------------------------------------------- CSV 輸出
// 輸出目錄:環境變數 CSD_OUT,預設 ./out
inline std::string out_path(const std::string& name) {
    const char* env = std::getenv("CSD_OUT");
    std::filesystem::path dir = env && *env ? env : "out";
    std::filesystem::create_directories(dir);
    return (dir / name).string();
}

// 各欄等長或不等長皆可(短欄留空),供 tools/plot_csv.py 或任何試算表 / gnuplot 使用
inline void write_csv(const std::string& name, const std::vector<std::string>& header, const std::vector<Vec>& cols) {
    std::string path = out_path(name);
    std::ofstream f(path);
    for (size_t j = 0; j < header.size(); ++j) f << (j ? "," : "") << header[j];
    f << "\n" << std::setprecision(10);
    size_t rows = 0;
    for (auto& c : cols) rows = std::max(rows, c.size());
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols.size(); ++j) {
            if (j) f << ",";
            if (i < cols[j].size()) f << cols[j][i];
        }
        f << "\n";
    }
    std::printf("  [csv] %s\n", path.c_str());
}

// ---------------------------------------------------------------- ✅ 驗證
struct Checker {
    std::string chapter;
    int total = 0, failed = 0;
    explicit Checker(std::string ch) : chapter(std::move(ch)) { std::printf("\n# ✅ 驗證\n"); }
    void operator()(bool ok, const char* expr, int line) {
        ++total;
        if (!ok) {
            ++failed;
            std::printf("  FAIL (line %d): %s\n", line, expr);
        }
    }
    int finish() const {
        if (failed == 0) {
            std::printf("%s 驗證通過 ✅ (%d/%d)\n", chapter.c_str(), total, total);
            return 0;
        }
        std::printf("%s 驗證失敗 ❌ (%d/%d 未通過)\n", chapter.c_str(), failed, total);
        return 1;
    }
};
#define CHECK(chk, cond) (chk)((cond), #cond, __LINE__)

}  // namespace csd

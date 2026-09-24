// lti.hpp — 轉移函數(連續 / 離散)與解析工具
//
// 對應 Python 版的 python-control:tf、串並聯、feedback、頻率響應、
// 穩定邊限、頻寬、離散化(ZOH / Tustin / matched)、Padé、時域響應。
#pragma once
#include <limits>
#include <string>

#include "linalg.hpp"

namespace csd {

// dt == 0 → 連續系統 H(s);dt > 0 → 離散系統 H(z),取樣週期 dt
struct TF {
    Poly num{0.0}, den{1.0};
    double dt = 0.0;
};

inline TF tf(Poly num, Poly den, double dt = 0.0) { return TF{polytrim(num), polytrim(den), dt}; }
inline TF tf_gain(double k, double dt = 0.0) { return tf({k}, {1.0}, dt); }

inline TF operator*(const TF& a, const TF& b) { return tf(polymul(a.num, b.num), polymul(a.den, b.den), a.dt); }
inline TF operator+(const TF& a, const TF& b) {
    if (a.den == b.den) return tf(polyadd(a.num, b.num), a.den, a.dt);
    return tf(polyadd(polymul(a.num, b.den), polymul(b.num, a.den)), polymul(a.den, b.den), a.dt);
}
inline TF operator*(double k, const TF& a) { return tf(polyscale(a.num, k), a.den, a.dt); }
inline TF operator+(double k, const TF& a) { return tf_gain(k, a.dt) + a; }

// 負回授:G / (1 + G H)
inline TF feedback(const TF& G, const TF& H) {
    return tf(polymul(G.num, H.den), polyadd(polymul(G.den, H.den), polymul(G.num, H.num)), G.dt);
}
inline TF feedback(const TF& G, double h = 1.0) { return feedback(G, tf_gain(h, G.dt)); }

inline cplx evalfr(const TF& g, cplx x) { return polyval(g.num, x) / polyval(g.den, x); }

// 角頻率 w [rad/s] 的頻率響應
inline cplx freqresp(const TF& g, double w) {
    if (g.dt == 0.0) return evalfr(g, cplx(0.0, w));
    return evalfr(g, std::exp(cplx(0.0, w * g.dt)));
}

inline double dcgain(const TF& g) {
    cplx x = g.dt == 0.0 ? cplx(0.0) : cplx(1.0);
    cplx d = polyval(g.den, x);
    if (std::abs(d) == 0.0) return std::numeric_limits<double>::infinity();
    return (polyval(g.num, x) / d).real();
}

inline std::vector<cplx> poles(const TF& g) { return roots(g.den); }
inline std::vector<cplx> zeros(const TF& g) { return roots(g.num); }

// ---------------------------------------------------------------- 常用方塊
inline TF first_order(double tau = 1.0, double gain = 1.0) { return tf({gain}, {tau, 1.0}); }
inline TF second_order(double wn = 10.0, double zeta = 0.7, double gain = 1.0) {
    return tf({gain * wn * wn}, {1.0, 2 * zeta * wn, wn * wn});
}
inline TF integrator(double gain = 1.0) { return tf({gain}, {1.0, 0.0}); }
inline TF inertia(double J = 1.0, double b = 0.0) { return tf({1.0}, {J, b}); }
inline TF pi_ctrl(double kp = 1.0, double ki = 0.0) { return tf({kp, ki}, {1.0, 0.0}); }
// kp + ki/s + kd*n*s/(s+n)
inline TF pid_ctrl(double kp = 1.0, double ki = 0.0, double kd = 0.0, double n = 100.0) {
    TF out = kp + tf({kd * n, 0.0}, {1.0, n});
    if (ki != 0.0) out = out + tf({ki}, {1.0, 0.0});
    return out;
}
inline TF closed_loop(const TF& c, const TF& p) { return feedback(c * p, 1.0); }

// e^{-sT} 的 Padé 近似(2 階)
inline TF pade2(double T) { return tf({T * T / 12, -T / 2, 1.0}, {T * T / 12, T / 2, 1.0}); }

// 二階類比 Butterworth 低通(scipy.signal.butter(2, wc, analog=True))
inline TF butter2_analog(double wc) { return tf({wc * wc}, {1.0, std::sqrt(2.0) * wc, wc * wc}); }

// ---------------------------------------------------------------- 狀態空間
struct SS {
    Mat A, B, C;
    double D = 0.0;
};

// 可控標準型(SISO)
inline SS tf2ss(const TF& g) {
    Poly den = polytrim(g.den), num = polytrim(g.num);
    const int n = int(den.size()) - 1;
    const double a0 = den[0];
    for (double& v : den) v /= a0;
    for (double& v : num) v /= a0;
    if (int(num.size()) > n + 1) throw std::invalid_argument("tf2ss: improper transfer function");
    Poly nf(n + 1, 0.0);
    for (size_t i = 0; i < num.size(); ++i) nf[n + 1 - num.size() + i] = num[i];
    SS s{Mat(n, n), Mat(n, 1), Mat(1, n), nf[0]};
    for (int j = 0; j < n; ++j) s.A(0, j) = -den[j + 1];
    for (int i = 1; i < n; ++i) s.A(i, i - 1) = 1.0;
    if (n > 0) s.B(0, 0) = 1.0;
    for (int j = 0; j < n; ++j) s.C(0, j) = nf[j + 1] - nf[0] * den[j + 1];
    return s;
}

// Faddeev–LeVerrier:特徵多項式與 C adj(sI-A) B
inline TF ss2tf(const SS& s, double dt) {
    const int n = s.A.r;
    Poly den(n + 1), num(n + 1, 0.0);
    den[0] = 1.0;
    Mat M = Mat::eye(n);  // M_1 = I
    for (int k = 1; k <= n; ++k) {
        Mat AM = s.A * M;
        double tr = 0.0;
        for (int i = 0; i < n; ++i) tr += AM(i, i);
        den[k] = -tr / k;
        Mat cmb = s.C * M * s.B;  // adj 係數 s^{n-k}
        num[k] = cmb(0, 0);
        M = AM + den[k] * Mat::eye(n);
    }
    // H = C adj B / det + D
    Poly full = polyadd(num, polyscale(den, s.D));
    return tf(full, den, dt);
}

// ---------------------------------------------------------------- 離散化
// method: "zoh" | "tustin" | "matched"(與 control.sample_system 相同語意)
inline TF c2d(const TF& g, double T, const std::string& method = "zoh") {
    if (method == "zoh") {
        SS s = tf2ss(g);
        const int n = s.A.r;
        Mat M(n + 1, n + 1);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) M(i, j) = s.A(i, j) * T;
            M(i, n) = s.B(i, 0) * T;
        }
        Mat E = expm(M);
        SS d{Mat(n, n), Mat(n, 1), s.C, s.D};
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) d.A(i, j) = E(i, j);
            d.B(i, 0) = E(i, n);
        }
        return ss2tf(d, T);
    }
    if (method == "tustin") {  // s = (2/T)(z-1)/(z+1)
        const int order = int(std::max(g.num.size(), g.den.size())) - 1;
        auto map = [&](const Poly& p) {
            Poly out = {0.0};
            const int deg = int(p.size()) - 1;
            for (int i = 0; i <= deg; ++i) {
                int k = deg - i;  // s^k
                Poly term = polymul(polypow({1.0, -1.0}, k), polypow({1.0, 1.0}, order - k));
                out = polyadd(out, polyscale(term, p[i] * std::pow(2.0 / T, k)));
            }
            return out;
        };
        return tf(map(g.num), map(g.den), T);
    }
    if (method == "matched") {  // 零極點 z = e^{sT},DC 增益匹配
        auto zs = zeros(g), ps = poles(g);
        std::vector<cplx> zz, zp;
        cplx gz_num = 1.0, gz_den = 1.0;
        for (cplx z : zs) { zz.push_back(std::exp(z * T)); gz_num *= 1.0 - zz.back(); }
        for (cplx p : ps) { zp.push_back(std::exp(p * T)); gz_den *= 1.0 - zp.back(); }
        double k = dcgain(g) / (gz_num / gz_den).real();
        return tf(polyscale(poly_from_roots(zz), k), poly_from_roots(zp), T);
    }
    throw std::invalid_argument("c2d: unknown method " + method);
}

// ---------------------------------------------------------------- 邊限
struct Margins {
    double gm_db, pm_deg, f_gm_hz, f_pm_hz;
};

namespace detail {
inline double wrap180(double deg) {  // → (-180, 180]
    deg = std::fmod(deg + 180.0, 360.0);
    if (deg <= 0) deg += 360.0;
    return deg - 180.0;
}
}  // namespace detail

// 以密集對數頻率格點掃描 + 二分細化求交越(與 control.stability_margins 同定義:
// 多個交越時各取「最危險」者)
inline Margins margins(const TF& L) {
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double wmax = L.dt > 0 ? PI / L.dt : 1e7;
    const int N = 40000;
    const double lw0 = std::log10(1e-4), lw1 = std::log10(wmax);
    std::vector<double> w(N), mag(N), ph(N);
    for (int i = 0; i < N; ++i) {
        w[i] = std::pow(10.0, lw0 + (lw1 - lw0) * i / (N - 1));
        cplx h = freqresp(L, w[i]);
        mag[i] = std::abs(h);
        ph[i] = std::arg(h) * 180 / PI;
        if (i > 0) {  // unwrap
            while (ph[i] - ph[i - 1] > 180) ph[i] -= 360;
            while (ph[i] - ph[i - 1] < -180) ph[i] += 360;
        }
    }
    auto phase_at = [&](double wq, double ref) {
        double p = std::arg(freqresp(L, wq)) * 180 / PI;
        while (p - ref > 180) p -= 360;
        while (p - ref < -180) p += 360;
        return p;
    };
    auto bisect = [&](double a, double b, auto f) {
        double fa = f(a);
        for (int it = 0; it < 80; ++it) {
            double m = std::sqrt(a * b), fm = f(m);
            if ((fm > 0) == (fa > 0)) { a = m; fa = fm; } else b = m;
        }
        return std::sqrt(a * b);
    };
    Margins m{inf, inf, nan, nan};
    // 增益交越 |L| = 1
    double best_pm = inf;
    for (int i = 1; i < N; ++i) {
        if ((mag[i - 1] - 1.0) * (mag[i] - 1.0) <= 0.0 && mag[i - 1] != mag[i]) {
            double wc = bisect(w[i - 1], w[i], [&](double x) { return std::abs(freqresp(L, x)) - 1.0; });
            double pm = detail::wrap180(phase_at(wc, ph[i - 1]) + 180.0);
            if (std::fabs(pm) < std::fabs(best_pm)) { best_pm = pm; m.f_pm_hz = wc / (2 * PI); }
        }
    }
    m.pm_deg = best_pm;
    // 相位交越 ∠L = -180 (mod 360)
    double best_gm = inf, best_score = inf;
    for (int i = 1; i < N; ++i) {
        double g0 = detail::wrap180(ph[i - 1] + 180.0), g1 = detail::wrap180(ph[i] + 180.0);
        if (g0 * g1 <= 0.0 && std::fabs(g0 - g1) < 90.0 && g0 != g1) {
            double ref = ph[i - 1];
            double wx = bisect(w[i - 1], w[i], [&](double x) { return detail::wrap180(phase_at(x, ref) + 180.0); });
            double gm = 1.0 / std::abs(freqresp(L, wx));
            double score = std::fabs(std::log(gm));
            if (score < best_score) { best_score = score; best_gm = gm; m.f_gm_hz = wx / (2 * PI); }
        }
    }
    m.gm_db = std::isinf(best_gm) ? inf : 20 * std::log10(best_gm);
    return m;
}

// -3 dB 頻寬 [Hz](相對 DC 增益)
inline double bandwidth_hz(const TF& T) {
    double dc_db = 20 * std::log10(std::fabs(dcgain(T)));
    auto f = [&](double w) { return 20 * std::log10(std::abs(freqresp(T, w))) - dc_db + 3.0; };
    const int N = 20000;
    double wprev = 1e-4;
    for (int i = 1; i < N; ++i) {
        double w = std::pow(10.0, -4.0 + 11.0 * i / (N - 1));
        if (f(w) < 0) {
            double a = wprev, b = w;
            for (int it = 0; it < 80; ++it) {
                double mid = std::sqrt(a * b);
                (f(mid) > 0 ? a : b) = mid;
            }
            return std::sqrt(a * b) / (2 * PI);
        }
        wprev = w;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

// ---------------------------------------------------------------- 時域響應
// 均勻時間格點上的強迫響應,輸入在樣本間線性內插(FOH,與 control.forced_response 相同假設)
inline std::vector<double> forced_response(const TF& g, const std::vector<double>& t, const std::vector<double>& u) {
    SS s = tf2ss(g);
    const int n = s.A.r;
    std::vector<double> y(t.size());
    if (n == 0) {
        for (size_t k = 0; k < t.size(); ++k) y[k] = s.D * u[k];
        return y;
    }
    const double h = t.size() > 1 ? t[1] - t[0] : 1.0;
    Mat M(n + 2, n + 2);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) M(i, j) = s.A(i, j) * h;
        M(i, n) = s.B(i, 0) * h;
    }
    M(n, n + 1) = 1.0;
    Mat E = expm(M);
    std::vector<double> x(n, 0.0), xn(n);
    for (size_t k = 0; k < t.size(); ++k) {
        double yk = s.D * u[k];
        for (int j = 0; j < n; ++j) yk += s.C(0, j) * x[j];
        y[k] = yk;
        if (k + 1 == t.size()) break;
        double du = u[k + 1] - u[k];
        for (int i = 0; i < n; ++i) {
            double v = E(i, n) * u[k] + E(i, n + 1) * du;
            for (int j = 0; j < n; ++j) v += E(i, j) * x[j];
            xn[i] = v;
        }
        x = xn;
    }
    return y;
}

inline std::vector<double> step_response(const TF& g, const std::vector<double>& t) {
    return forced_response(g, t, std::vector<double>(t.size(), 1.0));
}

// ---------------------------------------------------------------- 狀態回授工具
// SISO Ackermann 極點配置:回傳 K 使 eig(A - B K) = desired
inline Mat place_siso(const Mat& A, const Mat& B, const std::vector<cplx>& desired) {
    const int n = A.r;
    Poly phi = poly_from_roots(desired);  // s^n + a1 s^{n-1} + ... + an
    Mat Wc(n, n), col = B;
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) Wc(i, j) = col(i, 0);
        col = A * col;
    }
    Mat phiA = Mat(n, n), Ak = Mat::eye(n);
    for (int k = n; k >= 0; --k) {  // phi(A) = sum phi[n-k] A^k
        phiA = phiA + phi[k] * Ak;
        Ak = Ak * A;
    }
    Mat en(1, n);
    en(0, n - 1) = 1.0;
    return en * inv(Wc) * phiA;
}

}  // namespace csd

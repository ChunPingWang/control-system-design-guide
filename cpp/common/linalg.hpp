// linalg.hpp — 小型稠密矩陣與多項式工具(無外部相依)
//
// 只提供教材需要的最小集合:矩陣乘加、反矩陣、矩陣指數 expm、
// 多項式乘加/求值/求根。對應 Python 版背後的 numpy / scipy.linalg。
#pragma once
#include <algorithm>
#include <cmath>
#include <complex>
#include <stdexcept>
#include <vector>

namespace csd {

using cplx = std::complex<double>;
constexpr double PI = 3.14159265358979323846;

// ---------------------------------------------------------------- Mat
struct Mat {
    int r = 0, c = 0;
    std::vector<double> a;
    Mat() = default;
    Mat(int rows, int cols, double v = 0.0) : r(rows), c(cols), a(rows * cols, v) {}
    static Mat eye(int n) {
        Mat m(n, n);
        for (int i = 0; i < n; ++i) m(i, i) = 1.0;
        return m;
    }
    double& operator()(int i, int j) { return a[i * c + j]; }
    double operator()(int i, int j) const { return a[i * c + j]; }
};

inline Mat operator*(const Mat& x, const Mat& y) {
    if (x.c != y.r) throw std::invalid_argument("Mat*: dimension mismatch");
    Mat z(x.r, y.c);
    for (int i = 0; i < x.r; ++i)
        for (int k = 0; k < x.c; ++k) {
            double v = x(i, k);
            if (v == 0.0) continue;
            for (int j = 0; j < y.c; ++j) z(i, j) += v * y(k, j);
        }
    return z;
}
inline Mat operator+(Mat x, const Mat& y) {
    for (size_t i = 0; i < x.a.size(); ++i) x.a[i] += y.a[i];
    return x;
}
inline Mat operator-(Mat x, const Mat& y) {
    for (size_t i = 0; i < x.a.size(); ++i) x.a[i] -= y.a[i];
    return x;
}
inline Mat operator*(double s, Mat x) {
    for (double& v : x.a) v *= s;
    return x;
}

inline double norm1(const Mat& m) {  // 最大行絕對值和
    double best = 0.0;
    for (int j = 0; j < m.c; ++j) {
        double s = 0.0;
        for (int i = 0; i < m.r; ++i) s += std::fabs(m(i, j));
        best = std::max(best, s);
    }
    return best;
}

// 解 A X = B(部分樞軸高斯消去)
inline Mat solve(Mat A, Mat B) {
    const int n = A.r;
    for (int col = 0; col < n; ++col) {
        int piv = col;
        for (int i = col + 1; i < n; ++i)
            if (std::fabs(A(i, col)) > std::fabs(A(piv, col))) piv = i;
        if (A(piv, col) == 0.0) throw std::runtime_error("solve: singular matrix");
        if (piv != col) {
            for (int j = 0; j < n; ++j) std::swap(A(col, j), A(piv, j));
            for (int j = 0; j < B.c; ++j) std::swap(B(col, j), B(piv, j));
        }
        for (int i = 0; i < n; ++i) {
            if (i == col) continue;
            double f = A(i, col) / A(col, col);
            if (f == 0.0) continue;
            for (int j = col; j < n; ++j) A(i, j) -= f * A(col, j);
            for (int j = 0; j < B.c; ++j) B(i, j) -= f * B(col, j);
        }
    }
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < B.c; ++j) B(i, j) /= A(i, i);
    return B;
}
inline Mat inv(const Mat& A) { return solve(A, Mat::eye(A.r)); }

// 矩陣指數:scaling & squaring + Padé(6,6)
inline Mat expm(const Mat& A) {
    const int n = A.r;
    double nrm = norm1(A);
    int s = nrm > 0.5 ? std::max(0, int(std::ceil(std::log2(nrm / 0.5)))) : 0;
    Mat X = std::ldexp(1.0, -s) * A;
    const double cpade[] = {1.0, 0.5, 5.0 / 44, 1.0 / 66, 1.0 / 792, 1.0 / 15840, 1.0 / 665280};
    Mat N = Mat::eye(n), D = Mat::eye(n), P = Mat::eye(n);
    for (int k = 1; k <= 6; ++k) {
        P = P * X;
        N = N + cpade[k] * P;
        D = D + ((k % 2) ? -cpade[k] : cpade[k]) * P;
    }
    Mat E = solve(D, N);
    for (int k = 0; k < s; ++k) E = E * E;
    return E;
}

// 2x2 實矩陣特徵值
inline std::vector<cplx> eig2(const Mat& A) {
    double tr = A(0, 0) + A(1, 1);
    double det = A(0, 0) * A(1, 1) - A(0, 1) * A(1, 0);
    cplx disc = std::sqrt(cplx(tr * tr / 4 - det, 0.0));
    return {tr / 2 + disc, tr / 2 - disc};
}

// ---------------------------------------------------------------- 多項式
// 係數為降冪(與 numpy / python-control 相同):[a0, a1, ..., an] = a0*s^n + ... + an
using Poly = std::vector<double>;

inline Poly polytrim(Poly p) {
    size_t i = 0;
    while (i + 1 < p.size() && p[i] == 0.0) ++i;
    return Poly(p.begin() + i, p.end());
}
inline Poly polymul(const Poly& x, const Poly& y) {
    Poly z(x.size() + y.size() - 1, 0.0);
    for (size_t i = 0; i < x.size(); ++i)
        for (size_t j = 0; j < y.size(); ++j) z[i + j] += x[i] * y[j];
    return z;
}
inline Poly polyadd(const Poly& x, const Poly& y) {
    Poly z(std::max(x.size(), y.size()), 0.0);
    for (size_t i = 0; i < x.size(); ++i) z[z.size() - x.size() + i] += x[i];
    for (size_t i = 0; i < y.size(); ++i) z[z.size() - y.size() + i] += y[i];
    return z;
}
inline Poly polyscale(Poly p, double s) {
    for (double& v : p) v *= s;
    return p;
}
inline cplx polyval(const Poly& p, cplx x) {
    cplx y = 0.0;
    for (double c : p) y = y * x + c;
    return y;
}
inline Poly polypow(const Poly& p, int k) {
    Poly r = {1.0};
    for (int i = 0; i < k; ++i) r = polymul(r, p);
    return r;
}

// 求根(Durand–Kerner 同時迭代)
inline std::vector<cplx> roots(Poly p) {
    p = polytrim(p);
    // 去掉尾端零根
    std::vector<cplx> out;
    while (p.size() > 1 && p.back() == 0.0) {
        out.push_back(0.0);
        p.pop_back();
    }
    const int n = int(p.size()) - 1;
    if (n < 1) return out;
    std::vector<cplx> z(n);
    double radius = 0.0;
    for (int i = 1; i <= n; ++i) radius = std::max(radius, std::fabs(p[i] / p[0]));
    radius = 1.0 + radius;
    for (int i = 0; i < n; ++i) z[i] = std::polar(radius * 0.9, 2 * PI * i / n + 0.4);
    for (int it = 0; it < 2000; ++it) {
        double change = 0.0;
        for (int i = 0; i < n; ++i) {
            cplx den = p[0];
            for (int j = 0; j < n; ++j)
                if (j != i) den *= (z[i] - z[j]);
            cplx dz = polyval(p, z[i]) / den;
            z[i] -= dz;
            change = std::max(change, std::abs(dz) / std::max(1.0, std::abs(z[i])));
        }
        if (change < 1e-15) break;
    }
    for (auto& r : z) {
        if (std::fabs(r.imag()) < 1e-9 * std::max(1.0, std::abs(r))) r = cplx(r.real(), 0.0);
        out.push_back(r);
    }
    std::sort(out.begin(), out.end(),
              [](cplx a, cplx b) { return a.real() != b.real() ? a.real() < b.real() : a.imag() < b.imag(); });
    return out;
}

// 由根重建實係數多項式(首項係數 1)
inline Poly poly_from_roots(const std::vector<cplx>& rts) {
    std::vector<cplx> c = {1.0};
    for (cplx r : rts) {
        std::vector<cplx> n(c.size() + 1, 0.0);
        for (size_t i = 0; i < c.size(); ++i) {
            n[i] += c[i];
            n[i + 1] -= c[i] * r;
        }
        c = n;
    }
    Poly p(c.size());
    for (size_t i = 0; i < c.size(); ++i) p[i] = c[i].real();
    return p;
}

}  // namespace csd

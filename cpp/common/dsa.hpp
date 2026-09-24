// dsa.hpp — 動態訊號分析儀(DSA)替代品(對應 common/dsa.py)
//
// 以 chirp / PRBS 激發迴路,再用 Welch 交叉頻譜估測頻率響應:
//     H(f) = Pxy(f) / Pxx(f),coherence 告訴你哪些頻率可信。
#pragma once
#include "lti.hpp"
#include "util.hpp"

namespace csd {

// ---------------------------------------------------------------- FFT
// 長度為 2 的冪次時用 radix-2,否則退回 O(N^2) DFT(教材中只有小片段會用到)
inline void fft_inplace(std::vector<cplx>& x) {
    const size_t n = x.size();
    if (n <= 1) return;
    if (n & (n - 1)) {
        std::vector<cplx> y(n);
        for (size_t k = 0; k < n; ++k) {
            cplx s = 0.0;
            for (size_t j = 0; j < n; ++j) s += x[j] * std::polar(1.0, -2 * PI * double(k * j % n) / n);
            y[k] = s;
        }
        x = y;
        return;
    }
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(x[i], x[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        cplx wl = std::polar(1.0, -2 * PI / len);
        for (size_t i = 0; i < n; i += len) {
            cplx w = 1.0;
            for (size_t k = 0; k < len / 2; ++k) {
                cplx u = x[i + k], v = x[i + k + len / 2] * w;
                x[i + k] = u + v;
                x[i + k + len / 2] = u - v;
                w *= wl;
            }
        }
    }
}

// 實序列單邊 FFT(numpy.fft.rfft)
inline std::vector<cplx> rfft(const Vec& x) {
    std::vector<cplx> X(x.begin(), x.end());
    fft_inplace(X);
    X.resize(x.size() / 2 + 1);
    return X;
}
inline Vec rfftfreq(size_t n, double d) {
    Vec f(n / 2 + 1);
    for (size_t i = 0; i < f.size(); ++i) f[i] = i / (n * d);
    return f;
}
// numpy.hanning(對稱)
inline Vec hanning(size_t n) {
    Vec w(n);
    for (size_t i = 0; i < n; ++i) w[i] = n == 1 ? 1.0 : 0.5 - 0.5 * std::cos(2 * PI * i / (n - 1));
    return w;
}

// ---------------------------------------------------------------- 激發訊號
struct Excitation {
    Vec t, u;
};

// 對數 chirp f0 → f1 Hz(scipy.signal.chirp method="logarithmic")
inline Excitation chirp_excitation(double f0, double f1, double duration, double fs, double amplitude = 1.0) {
    Excitation e;
    e.t = arange(0.0, duration, 1.0 / fs);
    e.u.resize(e.t.size());
    const double beta = duration / std::log(f1 / f0);
    for (size_t i = 0; i < e.t.size(); ++i) {
        double phase = 2 * PI * beta * f0 * (std::pow(f1 / f0, e.t[i] / duration) - 1.0);
        e.u[i] = amplitude * std::cos(phase);
    }
    return e;
}

// 偽隨機二進位序列(±amplitude)
inline Vec prbs_excitation(size_t n, double amplitude = 1.0, unsigned long long seed = 0) {
    Rng rng(seed);
    Vec u(n);
    for (double& v : u) v = amplitude * (2.0 * rng.bit() - 1.0);
    return u;
}

// ---------------------------------------------------------------- Welch 頻譜
struct FRF {
    Vec f_hz, mag_db, phase_deg, coherence;
    std::vector<cplx> H;

    // 依遮罩挑出子集(對應 Python 的 {k: v[mask]})
    template <class Pred>
    FRF select(Pred keep) const {
        FRF o;
        for (size_t i = 0; i < f_hz.size(); ++i) {
            if (!keep(i)) continue;
            o.f_hz.push_back(f_hz[i]);
            o.mag_db.push_back(mag_db[i]);
            o.phase_deg.push_back(phase_deg[i]);
            if (!coherence.empty()) o.coherence.push_back(coherence[i]);
            o.H.push_back(H[i]);
        }
        return o;
    }
};

// scipy.signal.welch / csd / coherence 的預設行為:
// 週期型 Hann 窗、50% 重疊、去除各段平均、各段取平均
inline FRF measure_frf(const Vec& u, const Vec& y, double fs, size_t nperseg = 0) {
    if (nperseg == 0) nperseg = std::min<size_t>(u.size() / 8, 4096);
    const size_t step = nperseg - nperseg / 2;
    const size_t nseg = (u.size() - nperseg) / step + 1;
    const size_t nf = nperseg / 2 + 1;
    Vec win(nperseg);
    for (size_t i = 0; i < nperseg; ++i) win[i] = 0.5 - 0.5 * std::cos(2 * PI * i / nperseg);
    Vec pxx(nf, 0.0), pyy(nf, 0.0);
    std::vector<cplx> pxy(nf, 0.0);
    std::vector<cplx> X(nperseg), Y(nperseg);
    for (size_t s = 0; s < nseg; ++s) {
        const size_t off = s * step;
        double mu = 0, my = 0;
        for (size_t i = 0; i < nperseg; ++i) { mu += u[off + i]; my += y[off + i]; }
        mu /= nperseg; my /= nperseg;
        for (size_t i = 0; i < nperseg; ++i) {
            X[i] = (u[off + i] - mu) * win[i];
            Y[i] = (y[off + i] - my) * win[i];
        }
        fft_inplace(X);
        fft_inplace(Y);
        for (size_t k = 0; k < nf; ++k) {
            pxx[k] += std::norm(X[k]);
            pyy[k] += std::norm(Y[k]);
            pxy[k] += std::conj(X[k]) * Y[k];
        }
    }
    FRF r;
    for (size_t k = 1; k < nf; ++k) {  // 去掉 f = 0
        cplx H = pxy[k] / pxx[k];
        r.f_hz.push_back(k * fs / nperseg);
        r.H.push_back(H);
        r.mag_db.push_back(20 * std::log10(std::abs(H)));
        r.phase_deg.push_back(std::arg(H) * 180 / PI);
        r.coherence.push_back(std::norm(pxy[k]) / (pxx[k] * pyy[k]));
    }
    r.phase_deg = unwrap_deg(r.phase_deg);
    return r;
}

// 解析系統在 f_hz 的頻率響應
inline FRF frf_of_system(const TF& g, const Vec& f_hz) {
    FRF r;
    r.f_hz = f_hz;
    for (double f : f_hz) {
        cplx H = freqresp(g, 2 * PI * f);
        r.H.push_back(H);
        r.mag_db.push_back(20 * std::log10(std::abs(H)));
        r.phase_deg.push_back(std::arg(H) * 180 / PI);
    }
    r.phase_deg = unwrap_deg(r.phase_deg);
    return r;
}

inline double mag_db_at(const TF& g, double f_hz) { return frf_of_system(g, {f_hz}).mag_db[0]; }

// 將多個 Bode 曲線寫成一份 CSV(f_hz 與各系統 mag/phase 欄位)
inline void write_bode_csv(const std::string& name, const Vec& f_hz,
                           const std::vector<std::pair<std::string, TF>>& systems) {
    std::vector<std::string> hdr = {"f_hz"};
    std::vector<Vec> cols = {f_hz};
    for (auto& [label, sys] : systems) {
        FRF r = frf_of_system(sys, f_hz);
        hdr.push_back(label + "_mag_db");
        hdr.push_back(label + "_phase_deg");
        cols.push_back(r.mag_db);
        cols.push_back(r.phase_deg);
    }
    write_csv(name, hdr, cols);
}

}  // namespace csd

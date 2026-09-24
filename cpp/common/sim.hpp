// sim.hpp — 固定步長逐樣本模擬方塊(ModelQ-lite 核心,對應 common/sim.py)
//
// 每個方塊每個取樣 tick 執行一次 step(),因此能模擬 LTI 工具做不到的:
// 飽和、背隙、庫倫摩擦、量化編碼器、計算延遲、多速率迴路。
#pragma once
#include <cmath>
#include <deque>
#include <limits>

#include "linalg.hpp"

namespace csd {

inline double saturate(double u, double lo, double hi) { return std::min(std::max(u, lo), hi); }

// 量化到格距 q(編碼器 counts、DAC 階)
inline double quantize(double x, double q) { return std::floor(x / q) * q; }

// 庫倫摩擦:與速度反向,靜止時為 0
inline double coulomb(double v, double fc, double eps = 1e-9) {
    return std::fabs(v) > eps ? fc * (v > 0 ? 1.0 : -1.0) : 0.0;
}

// 教科書離散 PID:導數一階低通 + 條件積分 anti-windup
//   u = kp*e + ki*integ(e) + kd*d(e_f)/dt
struct DiscretePID {
    double kp, ki, kd, dt, out_min, out_max, alpha;
    bool anti_windup;
    double integ = 0.0, prev_ef = 0.0, ef = 0.0;
    bool first = true;

    static constexpr double INF = std::numeric_limits<double>::infinity();

    // dfilt_hz <= 0 表示導數不濾波(Python 版的 dfilt_hz=None)
    DiscretePID(double kp_, double ki_ = 0.0, double kd_ = 0.0, double dt_ = 0.001, double out_min_ = -INF,
                double out_max_ = INF, double dfilt_hz = 0.0, bool anti_windup_ = true)
        : kp(kp_), ki(ki_), kd(kd_), dt(dt_), out_min(out_min_), out_max(out_max_), anti_windup(anti_windup_) {
        if (dfilt_hz <= 0.0) {
            alpha = 1.0;
        } else {
            double a = 2 * PI * dfilt_hz * dt;
            alpha = a / (1.0 + a);  // backward-Euler LPF 極點
        }
        reset();
    }
    void reset() {
        integ = prev_ef = ef = 0.0;
        first = true;
    }
    double step(double e) {
        ef += alpha * (e - ef);
        if (first) {
            prev_ef = ef;
            first = false;
        }
        double deriv = (ef - prev_ef) / dt;
        prev_ef = ef;
        double u_unsat = kp * e + ki * integ + kd * deriv;
        double u = saturate(u_unsat, out_min, out_max);
        // 條件積分:輸出飽和且誤差仍往飽和方向推時凍結積分器
        if (!(anti_windup && u != u_unsat && e * u_unsat > 0)) integ += e * dt;
        return u;
    }
};

// 剛體慣量:J*dw/dt = T - b*w - T_dist
struct MotorPlant {
    double J, b, dt, w = 0.0, pos = 0.0;
    MotorPlant(double J_ = 1.0, double b_ = 0.0, double dt_ = 0.001) : J(J_), b(b_), dt(dt_) {}
    void reset() { w = pos = 0.0; }
    double step(double torque, double t_dist = 0.0) {
        double dw = (torque - b * w - t_dist) / J;
        w += dw * dt;
        pos += w * dt;
        return w;
    }
};

// 有刷 DC 馬達(含電氣動態)
//   L*di/dt = V - R*i - Ke*w ;  J*dw/dt = Kt*i - b*w - T_load
struct DCMotorPlant {
    double R, L, Kt, Ke, J, b, dt, i = 0.0, w = 0.0, pos = 0.0;
    DCMotorPlant(double R_ = 1.0, double L_ = 1e-3, double Kt_ = 0.1, double Ke_ = 0.1, double J_ = 1e-4,
                 double b_ = 1e-5, double dt_ = 1e-5)
        : R(R_), L(L_), Kt(Kt_), Ke(Ke_), J(J_), b(b_), dt(dt_) {}
    void reset() { i = w = pos = 0.0; }
    void step(double v, double t_load = 0.0) {
        double di = (v - R * i - Ke * w) / L;
        i += di * dt;
        double dw = (Kt * i - b * w - t_load) / J;
        w += dw * dt;
        pos += w * dt;
    }
};

// 馬達慣量經柔性軸連接負載慣量(Ch16 的經典共振受控體)
//   Jm*dwm/dt = T - ks*(thm-thl) - cs*(wm-wl)
//   Jl*dwl/dt =      ks*(thm-thl) + cs*(wm-wl)
struct TwoMassPlant {
    double Jm, Jl, ks, cs, dt, wm = 0.0, wl = 0.0, thm = 0.0, thl = 0.0;
    TwoMassPlant(double Jm_ = 1e-3, double Jl_ = 1e-3, double ks_ = 100.0, double cs_ = 0.01, double dt_ = 1e-4)
        : Jm(Jm_), Jl(Jl_), ks(ks_), cs(cs_), dt(dt_) {}
    void reset() { wm = wl = thm = thl = 0.0; }
    double resonance_hz() const { return std::sqrt(ks * (Jm + Jl) / (Jm * Jl)) / (2 * PI); }
    double antiresonance_hz() const { return std::sqrt(ks / Jl) / (2 * PI); }
    void step(double torque) {
        double tw = ks * (thm - thl) + cs * (wm - wl);
        wm += (torque - tw) / Jm * dt;
        wl += tw / Jl * dt;
        thm += wm * dt;
        thl += wl * dt;
    }
};

// 背隙(失動量),總寬度 width
struct Backlash {
    double half, out = 0.0;
    explicit Backlash(double width) : half(width / 2.0) {}
    void reset() { out = 0.0; }
    double step(double x) {
        if (x - out > half) out = x - half;
        else if (x - out < -half) out = x + half;
        return out;
    }
};

// 增量式編碼器:lines*4 counts/rev(四倍頻),位置單位 rev
struct EncoderModel {
    int counts_per_rev;
    explicit EncoderModel(int lines = 1000) : counts_per_rev(lines * 4) {}
    double read(double pos_rev) const { return std::floor(pos_rev * counts_per_rev) / counts_per_rev; }
};

// n 拍純延遲(計算 / 通訊延遲)
struct Delay {
    int n;
    std::deque<double> buf;
    explicit Delay(int n_, double initial = 0.0) : n(n_) { reset(initial); }
    void reset(double initial = 0.0) { buf.assign(std::max(1, n), initial); }
    double step(double x) {
        if (n == 0) return x;
        buf.push_back(x);
        double y = buf.front();
        buf.pop_front();
        return y;
    }
};

// 一階低通(backward Euler),各章迴路內濾波使用
struct OnePole {
    double alpha, y = 0.0;
    OnePole(double fc_hz, double dt) {
        double a = 2 * PI * fc_hz * dt;
        alpha = a / (1 + a);
    }
    double step(double x) {
        y += alpha * (x - y);
        return y;
    }
};

// 直接 I 型二階 IIR(biquad),a[0] 須為 1
struct Biquad {
    double b[3], a[3], x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    Biquad(const double bb[3], const double aa[3]) {
        for (int i = 0; i < 3; ++i) { b[i] = bb[i]; a[i] = aa[i]; }
    }
    double step(double x) {
        double y = b[0] * x + b[1] * x1 + b[2] * x2 - a[1] * y1 - a[2] * y2;
        x2 = x1; x1 = x;
        y2 = y1; y1 = y;
        return y;
    }
};

// scipy.signal.iirnotch(w0, Q) 的係數(w0 為相對 Nyquist 的正規化頻率)
inline Biquad iirnotch(double w0_norm, double Q) {
    double w0 = w0_norm * PI;
    double bw = w0 / Q;
    double beta = std::tan(bw / 2.0);  // gb = 1/sqrt(2)
    double gain = 1.0 / (1.0 + beta);
    double bb[3] = {gain, -2.0 * std::cos(w0) * gain, gain};
    double aa[3] = {1.0, -2.0 * gain * std::cos(w0), 2.0 * gain - 1.0};
    return Biquad(bb, aa);
}

}  // namespace csd

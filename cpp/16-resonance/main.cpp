// Ch16 機構柔性與共振:two-mass 受控體砍掉可用增益;LPF / notch / 機構改善的對照
#include <optional>

#include "../common/csd.hpp"
using namespace csd;

const double fs = 2000.0, dt = 1 / fs;
const int n = int(2.0 / dt);

struct Par { double Jm, Jl, ks, cs; };
enum class Filt { None, Notch, Lpf };

struct LoopResult { bool stable; Vec y; };

// 穩定 = 不發散且尾段殘留振盪 < 0.05
LoopResult run_loop(double kp, std::optional<Par> par, Filt filt = Filt::None, double ki = 5.0) {
    std::optional<MotorPlant> rigid;
    std::optional<TwoMassPlant> tm;
    if (!par) rigid.emplace(2e-3, 0.0, dt);
    else tm.emplace(par->Jm, par->Jl, par->ks, par->cs, dt);
    auto get = [&] { return rigid ? rigid->w : tm->wm; };
    auto step = [&](double u) { if (rigid) rigid->step(u); else tm->step(u); };

    DiscretePID c(kp, ki, 0.0, dt);
    Delay dly(1);
    std::optional<Biquad> notch;
    std::optional<OnePole> lpf;
    if (filt == Filt::Notch) notch.emplace(iirnotch(TwoMassPlant(par->Jm, par->Jl, par->ks, par->cs).resonance_hz() / (fs / 2), 2));
    if (filt == Filt::Lpf) lpf.emplace(60, dt);

    Vec y;
    y.reserve(n);
    for (int k = 0; k < n; ++k) {
        y.push_back(get());
        double meas = notch ? notch->step(y[k]) : lpf ? lpf->step(y[k]) : y[k];
        step(dly.step(c.step(1.0 - meas)));
        if (!std::isfinite(get()) || std::fabs(get()) > 1e4) return {false, y};
    }
    Vec tail = slice(y, size_t(0.8 * n));
    return {vmax(tail) - vmin(tail) < 0.05, y};
}

double kp_max(std::optional<Par> par, Filt filt = Filt::None) {
    Vec grid = arange(0.05, 1.01, 0.05), g2 = arange(1.1, 4.01, 0.1);
    grid.insert(grid.end(), g2.begin(), g2.end());
    double last = 0.0;
    for (double kp : grid) {
        kp = round_to(kp, 2);
        if (!run_loop(kp, par, filt).stable) break;
        last = kp;
    }
    return last;
}

int main() {
    const Par PAR{2e-4, 1.8e-3, 100.0, 0.005};
    TwoMassPlant tm0(PAR.Jm, PAR.Jl, PAR.ks, PAR.cs);
    const double f_res = tm0.resonance_hz(), f_anti = tm0.antiresonance_hz();
    std::printf("反共振 %.1f Hz | 共振 %.1f Hz | 慣量比 Jl/Jm = %.0f\n", f_anti, f_res, PAR.Jl / PAR.Jm);

    // 解析 two-mass 轉移函數(轉矩 -> 馬達速度)
    const double Jm = PAR.Jm, Jl = PAR.Jl, ks = PAR.ks, cs = PAR.cs;
    const TF P_2mass = tf({Jl, cs, ks}, {Jm * Jl, (Jm + Jl) * cs, (Jm + Jl) * ks, 0.0});
    const TF P_rigid = tf({1}, {Jm + Jl, 0});
    write_bode_csv("ch16_two_mass_bode.csv", logspace(0, 3, 600), {{"two_mass", P_2mass}, {"rigid", P_rigid}});

    // 共振之上增益抬升 = 20*log10(1 + Jl/Jm) ≈ 20 dB
    const double lift_db = mag_db_at(P_2mass, 400.0) - mag_db_at(P_rigid, 400.0);
    std::printf("400 Hz 處增益抬升:%.1f dB(理論 20·log10(1+Jl/Jm) = %.1f dB)\n", lift_db, 20 * std::log10(1 + Jl / Jm));

    const Par BAL{1e-3, 1e-3, 100.0, 0.005};  // 同總慣量、慣量比 1:1
    std::vector<std::pair<std::string, double>> table = {
        {"rigid body (dream)", kp_max(std::nullopt)},
        {"two-mass, no cure", kp_max(PAR)},
        {"two-mass + LPF 60 Hz", kp_max(PAR, Filt::Lpf)},
        {"two-mass + notch @res", kp_max(PAR, Filt::Notch)},
        {"two-mass, Jl/Jm=1 (mech!)", kp_max(BAL)},
    };
    std::printf("%-28s kp_max\n", "configuration");
    for (auto& [name, v] : table) std::printf("%-28s %6.2f\n", name.c_str(), v);
    const double rigid = table[0].second, nocure = table[1].second, lpf = table[2].second, notch = table[3].second,
                 bal = table[4].second;

    // --- 不穩定時的振盪頻率 ≈ 第二交越 kp/(2π·Jm) ---
    const double kp_unstable = nocure + 0.1;
    LoopResult osc = run_loop(kp_unstable, PAR);
    Vec seg = slice(osc.y, osc.y.size() - std::min<size_t>(2048, osc.y.size()));
    const double mu = mean(seg);
    Vec win = hanning(seg.size());
    for (size_t i = 0; i < seg.size(); ++i) seg[i] = (seg[i] - mu) * win[i];
    auto F = rfft(seg);
    Vec mag(F.size());
    for (size_t i = 0; i < F.size(); ++i) mag[i] = std::abs(F[i]);
    const double f_osc = rfftfreq(seg.size(), dt)[argmax(mag)];
    const double f_pred = kp_unstable / (2 * PI * PAR.Jm);
    std::printf("振盪頻率 %.0f Hz;第二交越預測 kp/(2π·Jm) = %.0f Hz;共振 %.0f Hz\n", f_osc, f_pred, f_res);
    LoopResult ok = run_loop(nocure, PAR);
    write_csv("ch16_unstable_vs_stable.csv", {"t_unstable", "y_unstable", "t_stable", "y_stable"},
              {sample_times(osc.y.size(), dt), osc.y, sample_times(ok.y.size(), dt), ok.y});

    Checker chk("Ch16");
    CHECK(chk, std::fabs(lift_db - 20 * std::log10(1 + Jl / Jm)) < 2);  // 增益抬升 = 慣量比
    CHECK(chk, rigid > 3);
    CHECK(chk, nocure <= rigid / 5);       // 共振砍增益 ≥5 倍
    CHECK(chk, notch >= nocure);
    CHECK(chk, lpf <= nocure);             // LPF 在此機台失敗
    CHECK(chk, bal >= 4 * nocure);
    CHECK(chk, std::fabs(f_osc - f_pred) / f_pred < 0.35);  // 振盪頻率 ≈ 第二交越
    return chk.finish();
}

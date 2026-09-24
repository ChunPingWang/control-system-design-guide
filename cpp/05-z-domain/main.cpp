// Ch5 z 域:ZOH / Tustin / matched 離散化、極點映射 z=e^{sT}、混疊
#include "../common/csd.hpp"
using namespace csd;

int main() {
    // --- 離散化方法比較 ---
    const TF P = second_order(2 * PI * 20, 0.3);  // 20 Hz、欠阻尼
    const double fs = 200.0, dt = 1 / fs;
    const TF Pz_zoh = c2d(P, dt, "zoh");
    const TF Pz_tus = c2d(P, dt, "tustin");
    const TF Pz_mat = c2d(P, dt, "matched");
    write_bode_csv("ch05_discretization.csv", logspace(0, std::log10(0.49 * fs), 300),
                   {{"continuous", P}, {"zoh", Pz_zoh}, {"tustin", Pz_tus}, {"matched", Pz_mat}});
    Vec dc = {dcgain(P), dcgain(Pz_zoh), dcgain(Pz_tus), dcgain(Pz_mat)};
    std::printf("DC gains: [%.6f %.6f %.6f %.6f]\n", dc[0], dc[1], dc[2], dc[3]);

    // --- 極點映射 ---
    const cplx s_pole(-50, 200);
    for (int f : {2000, 500, 100}) {
        cplx zp = std::exp(s_pole / double(f));
        std::printf("fs=%4d Hz: z = %.4f %+.4fj, |z| = %.3f\n", f, zp.real(), zp.imag(), std::abs(zp));
    }

    // --- 混疊:60 Hz 以 70 Hz 取樣 ---
    const double f_sig = 60.0, fs_slow = 70.0, alias_f = std::fabs(fs_slow - f_sig);
    Vec t_samp = arange(0, 0.5, 1 / fs_slow), x_samp, x_alias;
    for (double tt : t_samp) {
        x_samp.push_back(std::sin(2 * PI * f_sig * tt));
        x_alias.push_back(std::sin(2 * PI * alias_f * tt));
    }
    double corr = corrcoef(x_samp, x_alias);
    std::printf("取樣點 vs %.0f Hz 弦波相關係數: %.4f\n", alias_f, corr);
    write_csv("ch05_aliasing.csv", {"t", "x_sampled", "alias_10hz"}, {t_samp, x_samp, x_alias});

    // Tustin 在 Nyquist 附近相位比 ZOH 準(ZOH 額外半拍延遲)
    double ph_t = frf_of_system(Pz_tus, {40.0}).phase_deg[0];
    double ph_z = frf_of_system(Pz_zoh, {40.0}).phase_deg[0];
    double ph_c = frf_of_system(P, {40.0}).phase_deg[0];
    std::printf("40 Hz 相位:連續 %.1f° | Tustin %.1f° | ZOH %.1f°\n", ph_c, ph_t, ph_z);

    Checker chk("Ch5");
    for (size_t i = 1; i < dc.size(); ++i) CHECK(chk, std::fabs(dc[i] - dc[0]) < 1e-6);  // 離散化不得改變 DC 增益
    cplx zp = std::exp(s_pole / 100.0);
    CHECK(chk, std::abs(zp) < 1 && std::abs(zp) > 0.55);  // 穩定但靠近單位圓
    CHECK(chk, std::fabs(corr) > 0.99);
    CHECK(chk, std::fabs(ph_t - ph_c) < std::fabs(ph_z - ph_c));
    return chk.finish();
}

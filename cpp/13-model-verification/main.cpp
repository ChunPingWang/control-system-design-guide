// Ch13 模型驗證:對「未知」two-mass 受控體做 DSA 量測,擬合剛體慣量並找出共振
#include "../common/csd.hpp"
using namespace csd;

int main() {
    // --- 「未知」受控體(假裝我們不知道參數)---
    const double Jm = 1e-3, Jl = 1e-3, ks = 100.0, cs = 0.01;
    const double fs = 10000.0, dt = 1 / fs;
    TwoMassPlant rig(Jm, Jl, ks, cs, dt);
    Excitation ex = chirp_excitation(2, 300, 20, fs, 0.5);
    Rng rng(7);
    Vec noise = rng.normal(0, 0.02, ex.u.size());
    Vec y(ex.u.size());
    for (size_t k = 0; k < ex.u.size(); ++k) {
        rig.step(ex.u[k]);
        y[k] = rig.wm + noise[k];  // 量到的馬達速度(含雜訊)
    }
    FRF meas = measure_frf(ex.u, y, fs);
    FRF mf = meas.select([&](size_t i) { return meas.coherence[i] > 0.9 && meas.f_hz[i] > 1.5 && meas.f_hz[i] < 300; });
    std::printf("有效量測點:%zu,頻率範圍 %.1f ~ %.1f Hz\n", mf.f_hz.size(), mf.f_hz.front(), mf.f_hz.back());

    // --- 低頻段擬合剛體慣量:|H| ≈ 1/(J w) ---
    Vec J_samples, f_lo;
    for (size_t i = 0; i < mf.f_hz.size(); ++i)
        if (mf.f_hz[i] >= 2 && mf.f_hz[i] <= 10) {
            J_samples.push_back(1.0 / (std::abs(mf.H[i]) * 2 * PI * mf.f_hz[i]));
            f_lo.push_back(mf.f_hz[i]);
        }
    const double J_est = mean(J_samples), J_true = Jm + Jl;
    std::printf("擬合 J_tot = %.3f mkg·m²(真值 %.3f,誤差 %.1f%%)\n", J_est * 1000, J_true * 1000,
                std::fabs(J_est - J_true) / J_true * 100);

    const TF P_rigid = tf({1}, {J_est, 0});
    const TF P_2mass = tf({Jl, cs, ks}, {Jm * Jl, (Jm + Jl) * cs, (Jm + Jl) * ks, 0.0});
    FRF a_rigid = frf_of_system(P_rigid, mf.f_hz), a_2m = frf_of_system(P_2mass, mf.f_hz);
    write_csv("ch13_model_verification.csv", {"f_hz", "meas_mag_db", "meas_phase_deg", "rigid_mag_db", "twomass_mag_db"},
              {mf.f_hz, mf.mag_db, mf.phase_deg, a_rigid.mag_db, a_2m.mag_db});

    // --- 從量測資料自動找共振峰 / 反共振谷 ---
    Vec fb, mb;
    for (size_t i = 0; i < mf.f_hz.size(); ++i)
        if (mf.f_hz[i] > 30 && mf.f_hz[i] < 150) { fb.push_back(mf.f_hz[i]); mb.push_back(mf.mag_db[i]); }
    const double f_res_meas = fb[argmax(mb)], f_anti_meas = fb[argmin(mb)];
    TwoMassPlant tm_check(Jm, Jl, ks, cs);
    std::printf("量測共振 %.1f Hz(理論 %.1f Hz)\n", f_res_meas, tm_check.resonance_hz());
    std::printf("量測反共振 %.1f Hz(理論 %.1f Hz)\n", f_anti_meas, tm_check.antiresonance_hz());
    const double rig_at = mag_db_at(P_rigid, f_res_meas), meas_at = vmax(mb);
    std::printf("共振處剛體模型誤差:%.1f dB\n", meas_at - rig_at);

    // 低頻段 two-mass 模型與量測吻合
    FRF ana = frf_of_system(P_2mass, f_lo);
    double lo_dev = 0.0;
    for (size_t i = 0, j = 0; i < mf.f_hz.size(); ++i)
        if (mf.f_hz[i] >= 2 && mf.f_hz[i] <= 10) lo_dev = std::max(lo_dev, std::fabs(ana.mag_db[j++] - mf.mag_db[i]));

    Checker chk("Ch13");
    CHECK(chk, std::fabs(J_est - J_true) / J_true < 0.15);  // 慣量擬合 < 15%
    CHECK(chk, std::fabs(f_res_meas - tm_check.resonance_hz()) / tm_check.resonance_hz() < 0.1);
    CHECK(chk, std::fabs(f_anti_meas - tm_check.antiresonance_hz()) / tm_check.antiresonance_hz() < 0.1);
    CHECK(chk, (meas_at - rig_at) > 10);                    // 共振處剛體模型差 >10 dB
    CHECK(chk, lo_dev < 2);                                 // 低頻段 two-mass 模型與量測吻合(±2 dB)
    return chk.finish();
}

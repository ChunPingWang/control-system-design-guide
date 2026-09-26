#!/usr/bin/env python
"""Ch2 The Frequency Domain — Python simulation lab.

對應 maker-labs 講義 Chapter 2:用同一組 plant 同時觀察「時域」step response
與「頻域」Bode 圖,建立 transfer function / pole / zero 與可觀察動態的連結。

- G1 = 1/(s+1):一階系統,單一實極點,無 overshoot。
- G2 = 25/(s²+4s+25):二階系統,共軛複數極點(ωn=5, ζ=0.4),欠阻尼會 overshoot。

可從任意目錄執行:python maker-labs/02-frequency-domain/sim.py
章末以 assert 自我驗證(供 run_all.sh 判定綠/紅)。
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))  # → maker-labs/

import numpy as np
import matplotlib.pyplot as plt
import control as ct
from common.makerlab import step_metrics, show_or_save

# --- Plant:一階 vs 二階(對照講義 snippet 的 G1 / G2)---
G1 = ct.tf([1], [1, 1])            # 一階:pole 在 s=-1
G2 = ct.tf([25], [1, 4, 25])       # 二階:ωn=5, ζ=0.4,共軛複數 pole

# 二階標準式參數(供對照 Bode/step 的頻域-時域關係)
wn = np.sqrt(25.0)                 # 自然頻率 = 5 rad/s
zeta = 4.0 / (2.0 * wn)            # 阻尼比 = 0.4

p1 = ct.poles(G1)
p2 = ct.poles(G2)

print("=== Ch2 The Frequency Domain ===")
print(f"G1 = 1/(s+1)      poles: {np.round(p1, 4)}  DCgain={ct.dcgain(G1):.4f}")
print(f"G2 = 25/(s^2+4s+25) poles: {np.round(p2, 4)}  DCgain={ct.dcgain(G2):.4f}")
print(f"G2 二階參數:ωn={wn:.2f} rad/s  ζ={zeta:.2f}(欠阻尼)")

# --- 時域:step response,量化指標 ---
t = np.linspace(0, 4, 1600)
_, y1 = ct.step_response(G1, t)
_, y2 = ct.step_response(G2, t)
m1 = step_metrics(t, y1, setpoint=ct.dcgain(G1))
m2 = step_metrics(t, y2, setpoint=ct.dcgain(G2))
print(f"G1 step:rise={m1['rise_time']:.3f}s  OS={m1['overshoot_pct']:.1f}%")
print(f"G2 step:rise={m2['rise_time']:.3f}s  OS={m2['overshoot_pct']:.1f}%  "
      f"settling={m2['settling_time']:.3f}s")

# --- 頻域:magnitude / phase(手動取點,避免依賴 bode_plot 開視窗)---
w = np.logspace(-1, 2, 400)        # 0.1 .. 100 rad/s
mag1, ph1, _ = ct.frequency_response(G1, w)
mag2, ph2, _ = ct.frequency_response(G2, w)
mag1, ph1 = np.squeeze(mag1), np.squeeze(ph1)
mag2, ph2 = np.squeeze(mag2), np.squeeze(ph2)

# 二階欠阻尼的共振峰(resonant peak)是頻域最能證明「不只是感覺比較順」的證據
peak_idx = int(np.argmax(mag2))
w_peak = w[peak_idx]
mag_peak_db = 20 * np.log10(mag2[peak_idx])
print(f"G2 頻域共振峰:ω≈{w_peak:.2f} rad/s  峰值≈{mag_peak_db:.2f} dB")

# --- 圖:左 step(時域)、右上 magnitude、右下 phase(頻域),同一 plant 兩種觀點 ---
fig, axes = plt.subplots(2, 2, figsize=(11, 6.5))

ax = axes[0, 0]
ax.plot(t, y1, label="G1 (1st order)")
ax.plot(t, y2, label="G2 (2nd order, ζ=0.4)")
ax.axhline(1.0, ls=":", c="gray", label="setpoint")
ax.set_xlabel("Time (s)"); ax.set_ylabel("Output"); ax.grid(True); ax.legend()
ax.set_title("Time domain — step response")

ax = axes[1, 0]
ax.axis("off")
ax.text(0.0, 0.5,
        "Same plant, two views:\n"
        "- time domain step: overshoot / rise / settling\n"
        "- frequency domain Bode: gain / phase / resonant peak\n"
        f"G2 poles {np.round(p2, 2)}\n"
        f"-> under-damped zeta={zeta:.2f} causes step overshoot\n"
        f"   and a {mag_peak_db:.1f} dB resonant peak,\n"
        "   both are projections of the same poles.",
        fontsize=10, va="center", family="monospace")

ax = axes[0, 1]
ax.semilogx(w, 20 * np.log10(mag1), label="G1")
ax.semilogx(w, 20 * np.log10(mag2), label="G2")
ax.axvline(wn, ls="--", c="orange", label=f"ωn={wn:.0f}")
ax.set_ylabel("Magnitude (dB)"); ax.grid(True, which="both"); ax.legend()
ax.set_title("Frequency domain — Bode")

ax = axes[1, 1]
ax.semilogx(w, np.rad2deg(ph1), label="G1")
ax.semilogx(w, np.rad2deg(ph2), label="G2")
ax.axvline(wn, ls="--", c="orange")
ax.set_xlabel("Frequency (rad/s)"); ax.set_ylabel("Phase (deg)")
ax.grid(True, which="both"); ax.legend()

fig.suptitle("Ch2 1st vs 2nd order — time domain ↔ frequency domain")
fig.tight_layout()
show_or_save(plt, "ch02_time_vs_freq.png")

# --- ✅ 驗證:用解析/DC 增益/pole 性質判定,與模擬時長無關 ---
# 1) 兩個 plant DC 增益皆為 1(low-frequency gain,頻域左端 = 時域穩態)
assert abs(ct.dcgain(G1) - 1.0) < 1e-6, "G1 DC 增益應為 1"
assert abs(ct.dcgain(G2) - 1.0) < 1e-6, "G2 DC 增益應為 1"
# 2) 穩定性:所有 pole 實部 < 0
assert np.all(np.real(p1) < 0), "G1 應穩定(pole 實部<0)"
assert np.all(np.real(p2) < 0), "G2 應穩定(pole 實部<0)"
# 3) G1 一階 → 實極點;G2 二階欠阻尼 → 共軛複數極點
assert np.max(np.abs(np.imag(p1))) < 1e-9, "G1 一階應為實極點"
assert np.max(np.abs(np.imag(p2))) > 1e-6, "G2 欠阻尼應為共軛複數極點"
# 4) 阻尼比 ζ<1 → 欠阻尼;時域必有 overshoot,一階則無
assert zeta < 1.0, "G2 應為欠阻尼(ζ<1)"
assert m2["overshoot_pct"] > 1.0, "欠阻尼二階系統 step 應有明顯 overshoot"
assert m1["overshoot_pct"] < 1.0, "一階系統不應有 overshoot"
# 5) 欠阻尼(ζ<1/√2≈0.707)→ 頻域存在共振峰(峰值>0 dB 且不在最左端)
assert zeta < 1 / np.sqrt(2), "ζ<0.707 才會出現頻域共振峰"
assert mag_peak_db > 0.5 and peak_idx > 0, "G2 應在中頻出現共振峰(>0 dB)"
print("Ch2 驗證通過 ✅")

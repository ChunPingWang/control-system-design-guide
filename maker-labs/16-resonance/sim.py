#!/usr/bin/env python
"""Ch16 Compliance and Resonance — Python simulation lab.

對應 maker-labs 講義 Chapter 16:馬達與負載透過彈性聯軸器/皮帶連接時,
不能永遠視為 rigid body。兩個 inertia + spring/damper 形成
「兩慣量(two-mass)系統」,在頻域上出現 anti-resonance(dip)與
resonance(peak)。本 lab 建立 torque→motor-velocity 的傳遞函數,
畫 Bode,量出 resonance / anti-resonance 頻率,並和解析預測比較。

物理直覺:
  - 低頻:兩個慣量像被鎖在一起,系統看起來像單一 rigid body,
    torque→velocity 近似積分器 1/((Jm+Jl)·s)。
  - anti-resonance ω_ar = sqrt(k/Jl):負載端「反相」把馬達端頂住,
    馬達幾乎不動 → 量到的馬達速度出現凹陷(dip)。
  - resonance   ω_r  = sqrt(k·(Jm+Jl)/(Jm·Jl)):兩慣量對彈簧共振,
    幅值出現尖峰(peak)。提高 gain 到此頻段容易激發機械模態。

可從任意目錄執行:python maker-labs/16-resonance/sim.py
章末以 assert 自我驗證(供 run_all.sh 判定綠/紅)。
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))  # → maker-labs/

import numpy as np
import matplotlib.pyplot as plt
import control as ct
from common.makerlab import show_or_save

# --- 兩慣量參數(與講義 Chapter 16 範例一致)---
Jm, Jl, k, b = 0.002, 0.006, 20.0, 0.03   # 馬達慣量、負載慣量、彈簧、阻尼

# --- 狀態空間:x = [θm, ωm, θl, ωl];input = 馬達力矩;output = 馬達速度 ωm ---
A = np.array([[0, 1, 0, 0],
              [-k / Jm, -b / Jm, k / Jm, b / Jm],
              [0, 0, 0, 1],
              [k / Jl, b / Jl, -k / Jl, -b / Jl]])
B = np.array([[0], [1 / Jm], [0], [0]])
C = np.array([[0, 1, 0, 0]])          # 量測 = 馬達端速度(collocated sensor)
D = np.zeros((1, 1))
sys = ct.ss(A, B, C, D)

# --- 解析預測 ---
w_ar_pred = np.sqrt(k / Jl)                       # anti-resonance(零點)
w_r_pred = np.sqrt(k * (Jm + Jl) / (Jm * Jl))     # resonance(極點)

# --- 頻率響應(手動取 |G(jω)|,不依賴繪圖 API)---
w = np.logspace(np.log10(2.0), np.log10(1000.0), 6000)   # rad/s
resp = np.squeeze(np.asarray(sys(1j * w)))
mag = np.abs(resp)
phase_deg = np.unwrap(np.angle(resp)) * 180.0 / np.pi

# rigid-body(無彈性)近似:兩慣量鎖死 → 1/((Jm+Jl)·s)
mag_rigid = 1.0 / ((Jm + Jl) * w)


def _peak_in_band(values, center, half_frac, want_max=True):
    """在 center 的 ±half_frac 頻帶內找局部極值,回傳 (freq, value)。"""
    lo, hi = center * (1 - half_frac), center * (1 + half_frac)
    band = (w >= lo) & (w <= hi)
    idx_band = np.where(band)[0]
    sub = values[idx_band]
    j = np.argmax(sub) if want_max else np.argmin(sub)
    i = idx_band[j]
    return w[i], values[i]


w_res, mag_res = _peak_in_band(mag, w_r_pred, 0.4, want_max=True)
w_ar, mag_ar = _peak_in_band(mag, w_ar_pred, 0.4, want_max=False)
mag_rigid_at_res = 1.0 / ((Jm + Jl) * w_res)

print("=== Ch16 Compliance and Resonance ===")
print(f"參數:Jm={Jm} Jl={Jl} k={k} b={b}")
print(f"anti-resonance:量測 {w_ar:6.2f} rad/s ({w_ar/2/np.pi:5.2f} Hz)  "
      f"預測 sqrt(k/Jl)={w_ar_pred:6.2f} rad/s")
print(f"resonance     :量測 {w_res:6.2f} rad/s ({w_res/2/np.pi:5.2f} Hz)  "
      f"預測 sqrt(k(Jm+Jl)/(Jm·Jl))={w_r_pred:6.2f} rad/s")
print(f"峰值/凹陷幅值比 = {mag_res/mag_ar:8.1f}  "
      f"(peak {20*np.log10(mag_res):+.1f} dB, dip {20*np.log10(mag_ar):+.1f} dB)")
print(f"resonance 峰值超過 rigid-body 趨勢線 = "
      f"{20*np.log10(mag_res/mag_rigid_at_res):+.1f} dB")

# --- 參數 sweep:改變 k(彈簧剛度)觀察 resonance peak 移動 ---
k2 = 60.0
A2 = A.copy()
A2[1, 0], A2[1, 2] = -k2 / Jm, k2 / Jm
A2[3, 0], A2[3, 2] = k2 / Jl, -k2 / Jl
sys2 = ct.ss(A2, B, C, D)
mag2 = np.abs(np.squeeze(np.asarray(sys2(1j * w))))
w_r2_pred = np.sqrt(k2 * (Jm + Jl) / (Jm * Jl))
w_res2, _ = _peak_in_band(mag2, w_r2_pred, 0.4, want_max=True)
print(f"sweep:k {k}→{k2}  resonance {w_res:.1f}→{w_res2:.1f} rad/s "
      f"(剛度↑ → 共振頻率↑)")

# --- 圖:Bode(magnitude + phase),標出 resonance / anti-resonance ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.5, 6.5), sharex=True)

ax1.semilogx(w, 20 * np.log10(mag), label=f"k={k:.0f} (baseline)")
ax1.semilogx(w, 20 * np.log10(mag2), "-", color="tab:green",
             alpha=0.7, label=f"k={k2:.0f} (sweep)")
ax1.semilogx(w, 20 * np.log10(mag_rigid), "--", color="gray",
             label="rigid-body 1/((Jm+Jl)s)")
ax1.plot(w_res, 20 * np.log10(mag_res), "rv", ms=9,
         label=f"resonance {w_res:.0f} rad/s")
ax1.plot(w_ar, 20 * np.log10(mag_ar), "b^", ms=9,
         label=f"anti-resonance {w_ar:.0f} rad/s")
ax1.set_ylabel("Magnitude (dB)"); ax1.grid(True, which="both"); ax1.legend(fontsize=8)
ax1.set_title("Ch16 two-mass resonance — torque → motor velocity")

ax2.semilogx(w, phase_deg)
ax2.axvline(w_res, color="r", ls=":", alpha=0.6)
ax2.axvline(w_ar, color="b", ls=":", alpha=0.6)
ax2.set_xlabel("Frequency (rad/s)"); ax2.set_ylabel("Phase (deg)")
ax2.grid(True, which="both")

show_or_save(plt, "ch16_resonance_bode.png")

# --- ✅ 驗證(解析、與繪圖無關)---
# 1) resonance / anti-resonance 頻率貼近解析預測
assert abs(w_res - w_r_pred) / w_r_pred < 0.10, \
    "resonance 頻率應接近 sqrt(k(Jm+Jl)/(Jm·Jl))"
assert abs(w_ar - w_ar_pred) / w_ar_pred < 0.10, \
    "anti-resonance 頻率應接近 sqrt(k/Jl)"
# 2) anti-resonance 一定在 resonance 之下(collocated:先零點後極點)
assert w_ar < w_res, "collocated 系統:anti-resonance 應低於 resonance"
# 3) 確有共振尖峰:峰值遠高於凹陷,且高於 rigid-body 趨勢線(柔性放大)
assert mag_res > 5.0 * mag_ar, "resonance 峰值應明顯高於 anti-resonance 凹陷"
assert mag_res > mag_rigid_at_res, \
    "resonance 峰值應高於 rigid-body 趨勢線(否則就沒有共振放大)"
# 4) 峰值必須是頻帶內真正的局部極大(左右鄰點都比它低)
i_res = int(np.argmin(np.abs(w - w_res)))
assert mag[i_res] >= mag[i_res - 5] and mag[i_res] >= mag[i_res + 5], \
    "resonance 應為局部極大值"
# 5) 剛度提高 → 共振頻率上升
assert w_res2 > w_res, "彈簧剛度提高後 resonance 頻率應上升"
print("Ch16 驗證通過 ✅")

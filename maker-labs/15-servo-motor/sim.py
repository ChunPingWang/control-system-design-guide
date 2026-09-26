#!/usr/bin/env python
"""Ch15 Basics of the Electric Servomotor and Drive — Python simulation lab.

對應 maker-labs 講義 Chapter 15:以 state-space 建立 DC 伺服馬達模型,
同時保留電氣 dynamics(R,L,Ke,Kt)與機械 dynamics(J,b),觀察:
  1. 電氣時間常數 τe = L/R  遠小於  機械時間常數 τm = J/b(快慢分層)。
  2. Cascade 控制:內層電流(torque)迴路頻寬 > 外層速度迴路頻寬,
     所以「先快速把電流控好,外層再把速度控好」的串級架構才成立。

可從任意目錄執行:python maker-labs/15-servo-motor/sim.py
章末以 assert 自我驗證(供 run_all.sh 判定綠/紅)。
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))  # → maker-labs/

import numpy as np
import matplotlib.pyplot as plt
import control as ct
from common.makerlab import show_or_save

# ---------------------------------------------------------------------------
# 1. DC 伺服馬達參數(對齊講義)
#    V = R i + L di/dt + Ke ω
#    J dω/dt = Kt i - B ω - τload
# ---------------------------------------------------------------------------
R, L = 2.0, 0.01      # 電樞電阻 (Ω)、電感 (H)
Ke, Kt = 0.08, 0.08   # 反電動勢常數 (V·s/rad)、轉矩常數 (N·m/A)
J, B = 0.002, 0.002   # 轉動慣量 (kg·m²)、黏滯摩擦 (N·m·s/rad)

tau_e = L / R         # 電氣時間常數
tau_m = J / B         # 機械時間常數(僅摩擦主導的一階近似)

# 狀態 x = [i, ω]^T,輸入 u = V,輸出取 [i; ω]
A = np.array([[-R / L, -Ke / L],
              [Kt / J, -B / J]])
Bv = np.array([[1.0 / L], [0.0]])
C = np.eye(2)
D = np.zeros((2, 1))
sys = ct.ss(A, Bv, C, D)

print("=== Ch15 Basics of the Electric Servomotor and Drive ===")
print(f"電氣時間常數 τe = L/R = {tau_e*1e3:.2f} ms")
print(f"機械時間常數 τm = J/B = {tau_m*1e3:.2f} ms")
print(f"τm/τe 比值        = {tau_m/tau_e:.0f}x  (電氣遠快於機械)")

# ---------------------------------------------------------------------------
# 2. 開迴路 step:一個電壓 step 下,電流先急衝再回落、速度緩慢爬升
# ---------------------------------------------------------------------------
t = np.linspace(0, 0.5, 4000)
_, yoc = ct.step_response(sys, t)
yoc = np.squeeze(np.asarray(yoc))   # → shape (2, N)
i_oc = yoc[0]     # 電流對電壓 step 的響應
w_oc = yoc[1]     # 速度對電壓 step 的響應

# 電流峰值時間 vs 速度到 63% 的時間 —— 量化「快慢分層」
i_peak_t = t[np.argmax(np.abs(i_oc))]
w_final = w_oc[-1]
w_63_idx = np.where(w_oc >= 0.63 * w_final)[0]
w_63_t = t[w_63_idx[0]] if len(w_63_idx) else np.nan
print(f"開迴路:電流峰值時間 ≈ {i_peak_t*1e3:.2f} ms, "
      f"速度達 63% 時間 ≈ {w_63_t*1e3:.1f} ms")

# ---------------------------------------------------------------------------
# 3. Cascade:內層電流迴路 vs 外層速度迴路頻寬
# ---------------------------------------------------------------------------
# 由 state-space 抽出兩個 SISO 傳遞函數
G_Vi = ct.ss2tf(ct.ss(A, Bv, np.array([[1.0, 0.0]]), np.array([[0.0]])))  # V → i
G_iw = ct.tf([Kt], [J, B])   # i → ω(轉矩/機械一階)


def bandwidth_3db(T, wmax=1e5):
    """回傳閉迴路 -3dB 頻寬 (rad/s):由 DC 增益下降 1/sqrt(2) 的頻率。"""
    w = np.logspace(-2, np.log10(wmax), 6000)
    mag = np.abs(np.squeeze(ct.frequency_response(T, w).frdata))
    dc = mag[0]
    thresh = dc / np.sqrt(2.0)
    below = np.where(mag < thresh)[0]
    return w[below[0]] if len(below) else w[-1]


# 內層:比例電流控制器 Kp_i(V per A),閉合 V→i 迴路
Kp_i = 20.0
T_inner = ct.feedback(Kp_i * G_Vi, 1)
bw_inner = bandwidth_3db(T_inner)

# 外層:速度控制器 Kp_w(輸出電流命令 i_ref),內層電流迴路視為快速近似
Kp_w = 2.0
L_outer = Kp_w * T_inner * G_iw          # 外層開迴路(含內層閉迴路)
T_outer = ct.feedback(L_outer, 1)
bw_outer = bandwidth_3db(T_outer)

print(f"內層電流迴路頻寬 ≈ {bw_inner:.0f} rad/s "
      f"({bw_inner/2/np.pi:.0f} Hz)")
print(f"外層速度迴路頻寬 ≈ {bw_outer:.1f} rad/s "
      f"({bw_outer/2/np.pi:.1f} Hz)")
print(f"內/外頻寬比      = {bw_inner/bw_outer:.0f}x")

# ---------------------------------------------------------------------------
# 4. 圖
# ---------------------------------------------------------------------------
fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))

# (左)開迴路電氣 vs 機械 dynamics —— 雙 y 軸凸顯時間尺度差
ax1 = axes[0]
ax1.plot(t * 1e3, i_oc, "C0", label="current i(t)")
ax1.set_xlabel("Time (ms)")
ax1.set_ylabel("Current (A)", color="C0")
ax1.tick_params(axis="y", labelcolor="C0")
ax1.set_xlim(0, 60)
ax1.grid(True)
ax1b = ax1.twinx()
ax1b.plot(t * 1e3, w_oc, "C1", label="speed ω(t)")
ax1b.set_ylabel("Speed (rad/s)", color="C1")
ax1b.tick_params(axis="y", labelcolor="C1")
ax1.axvline(tau_e * 1e3, ls=":", c="C0", alpha=0.7)
ax1.set_title("Open loop: fast electrical vs slow mechanical")

# (右)內層 vs 外層閉迴路 Bode 幅值 —— 頻寬分離
w = np.logspace(0, 5, 800)
mag_in = np.abs(np.squeeze(ct.frequency_response(T_inner, w).frdata))
mag_out = np.abs(np.squeeze(ct.frequency_response(T_outer, w).frdata))
ax2 = axes[1]
ax2.semilogx(w, 20 * np.log10(mag_in), label="inner current loop")
ax2.semilogx(w, 20 * np.log10(mag_out), label="outer velocity loop")
ax2.axhline(-3, ls=":", c="gray", label="-3 dB")
ax2.axvline(bw_inner, ls="--", c="C0", alpha=0.6)
ax2.axvline(bw_outer, ls="--", c="C1", alpha=0.6)
ax2.set_xlabel("Frequency (rad/s)")
ax2.set_ylabel("Magnitude (dB)")
ax2.set_ylim(-40, 10)
ax2.grid(True, which="both")
ax2.legend()
ax2.set_title("Cascade: inner bandwidth >> outer bandwidth")

fig.suptitle("Ch15 DC servomotor: electrical/mechanical + current/velocity cascade")
fig.tight_layout()
show_or_save(plt, "ch15_servo_motor.png")

# ---------------------------------------------------------------------------
# 5. ✅ 驗證(analytic + 模型結果)
# ---------------------------------------------------------------------------
# (a) 電氣遠快於機械:時間常數至少差一個數量級
assert tau_e < tau_m, "電氣時間常數應小於機械時間常數"
assert tau_m / tau_e > 10.0, "τm/τe 應遠大於 1(快慢分層)"

# (b) 開迴路:電流峰值出現得比速度到 63% 早很多
assert i_peak_t < w_63_t, "電流動態應比速度動態快"

# (c) Cascade:內層電流迴路頻寬明顯大於外層速度迴路頻寬
assert bw_inner > bw_outer, "內層電流迴路頻寬應大於外層速度迴路"
assert bw_inner > 5.0 * bw_outer, "串級要成立,內層頻寬應至少為外層數倍"

# (d) 兩個閉迴路都穩定(所有極點實部 < 0)
assert np.all(np.real(ct.poles(T_inner)) < 0), "內層閉迴路應穩定"
assert np.all(np.real(ct.poles(T_outer)) < 0), "外層閉迴路應穩定"

print("Ch15 驗證通過 ✅")

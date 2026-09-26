#!/usr/bin/env python
"""Ch11 Introduction to Modeling — Python simulation lab.

對應 maker-labs 講義 Chapter 11:模型的目的、frequency-domain modeling、
time-domain modeling。核心觀念:同一個 plant 可以用兩種方式描述,
各自能回答不同的工程問題。

以一階馬達近似 G(s)=K/(τs+1) 為例:
  - Time-domain model:step 響應 y(t)=K·(1-e^{-t/τ}),看 transient/時間常數。
  - Frequency-domain model:Bode(magnitude/phase),看 bandwidth/loop/stability。
兩者是同一組 (K, τ) 的兩張臉。

可從任意目錄執行:python maker-labs/11-modeling/sim.py
章末以 assert 自我驗證(供 run_all.sh 判定綠/紅)。
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))  # → maker-labs/

import numpy as np
import matplotlib.pyplot as plt
import control as ct
from common.makerlab import step_metrics, show_or_save

# --- Plant:一階馬達近似 G(s)=K/(τs+1) ---
#   K  = DC 增益(穩態 RPM / 單位輸入)
#   τ  = time constant(響應到穩態 63.2% 所需時間)
K = 4.0
tau = 0.25
G = ct.tf([K], [tau, 1])

# 解析真值(assert 用,與模擬時長無關):
dc_gain_true = K                 # DC 增益 = G(0) = K
pole_true = -1.0 / tau           # 極點 = -1/τ

print("=== Ch11 Introduction to Modeling ===")
print(f"Plant G(s) = {K} / ({tau}·s + 1)")
print(f"DC gain = {ct.dcgain(G):.4f}  (理論 K = {dc_gain_true:.4f})")
print(f"poles   = {ct.poles(G)}  (理論 -1/τ = {pole_true:.4f})")

# --- 描述法 1:Time-domain model(step 響應) ---
t = np.linspace(0, 2.0, 500)
_, y_sim = ct.step_response(G, t)          # 數值 step 響應
y_analytic = K * (1.0 - np.exp(-t / tau))  # 解析 step 響應
m = step_metrics(t, y_sim, setpoint=dc_gain_true)
print(f"time-domain:rise={m['rise_time']:.3f}s  "
      f"settling={m['settling_time']:.3f}s  τ(量測)≈{tau:.3f}s")

# --- 描述法 2:Frequency-domain model(Bode) ---
w = np.logspace(-1, 3, 400)                # rad/s
mag, phase, omega = ct.frequency_response(G, w)
mag = np.asarray(mag).squeeze()
phase = np.asarray(phase).squeeze()
# 一階系統 -3 dB bandwidth 就落在轉折頻率 ω_c = 1/τ
wc_true = 1.0 / tau
mag_at_wc = np.abs(ct.frequency_response(G, [wc_true])[0].squeeze())
print(f"freq-domain:轉折頻率 ω_c = 1/τ = {wc_true:.2f} rad/s  "
      f"→ |G(jω_c)|/K = {mag_at_wc / K:.3f} (理論 1/√2 = {1/np.sqrt(2):.3f})")

# --- 圖:同一 plant,左邊 time-domain、右邊 frequency-domain ---
fig, (ax_t, ax_f) = plt.subplots(1, 2, figsize=(11, 4.2))

# (左)time-domain
ax_t.plot(t, y_sim, label="step response (sim)")
ax_t.plot(t, y_analytic, "--", label=r"$K(1-e^{-t/\tau})$")
ax_t.axhline(dc_gain_true, ls=":", c="gray", label="DC gain (K)")
ax_t.axhline(0.632 * K, ls=":", c="tab:orange")
ax_t.axvline(tau, ls=":", c="tab:orange", label=r"$t=\tau$ (63.2%)")
ax_t.set_xlabel("Time (s)"); ax_t.set_ylabel("Output (RPM/unit)")
ax_t.grid(True); ax_t.legend(loc="lower right")
ax_t.set_title("Time-domain model")

# (右)frequency-domain(magnitude,dB)
ax_f.semilogx(omega, 20 * np.log10(mag), label="|G(jω)|")
ax_f.axhline(20 * np.log10(K) - 3.0, ls=":", c="tab:orange",
             label="-3 dB from DC")
ax_f.axvline(wc_true, ls=":", c="tab:orange", label=r"$\omega_c=1/\tau$")
ax_f.set_xlabel("Frequency (rad/s)"); ax_f.set_ylabel("Magnitude (dB)")
ax_f.grid(True, which="both"); ax_f.legend(loc="lower left")
ax_f.set_title("Frequency-domain model")

fig.suptitle("Ch11 same plant, two descriptions")
fig.tight_layout()
show_or_save(plt, "ch11_two_descriptions.png")

# --- ✅ 驗證(解析真值:DC gain 與 pole 位置) ---
assert abs(ct.dcgain(G) - dc_gain_true) < 1e-9, "DC 增益應等於 K"
poles = ct.poles(G)
assert len(poles) == 1, "一階模型應只有一個極點"
assert abs(poles[0].real - pole_true) < 1e-9, "極點應位於 -1/τ"
assert abs(poles[0].imag) < 1e-9, "一階實極點虛部應為 0"
# time-domain 與 frequency-domain 是同一模型的兩種描述:互相對照
assert np.max(np.abs(y_sim - y_analytic)) < 1e-2, "數值 step 應吻合解析 step 響應"
assert abs(mag_at_wc / K - 1 / np.sqrt(2)) < 1e-2, "ω_c=1/τ 應為 -3 dB 點(1/√2)"
print("Ch11 驗證通過 ✅")

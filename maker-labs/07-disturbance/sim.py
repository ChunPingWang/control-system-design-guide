#!/usr/bin/env python
"""Ch7 Disturbance Response — Python simulation lab.

對應 maker-labs 講義 Chapter 7:分開量測 command response 與 disturbance
response,理解「好的 setpoint tracking 不等於好的 disturbance rejection」。

核心:sensitivity function S = feedback(1, C*G)。
  - command response   T = feedback(C*G, 1)   → reference → output 路徑
  - disturbance response G*S                   → plant-input 擾動 → output 路徑
含積分器的 controller 會讓 S(0)=0,故 disturbance 穩態誤差 → 0(擾動被完全抑制)。

可從任意目錄執行:python maker-labs/07-disturbance/sim.py
章末以 assert 自我驗證(供 run_all.sh 判定綠/紅)。
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))  # → maker-labs/

import numpy as np
import matplotlib.pyplot as plt
import control as ct
from common.makerlab import show_or_save

# --- Plant 與 controller(取自講義 snippet)---
G = ct.tf([1], [0.3, 1])       # 一階馬達近似
C = ct.tf([3, 6], [1, 0])      # PI:含積分器 → 具擾動抑制能力

S = ct.feedback(1, C * G)      # sensitivity(reference→error / disturbance→output 的核心)
T = ct.feedback(C * G, 1)      # complementary sensitivity(command response)
Gd = G * S                     # plant-input disturbance → output

t = np.linspace(0, 4, 2000)
_, yc = ct.step_response(T, t)    # 對 setpoint 做 step
_, yd = ct.step_response(Gd, t)   # 對 plant 輸入端 disturbance 做 step

# --- 指標:command 穩態 vs disturbance 峰值/回復 ---
dc_cmd = ct.dcgain(T)          # 應 ≈ 1(能追上 setpoint)
dc_dist = ct.dcgain(Gd)        # 應 ≈ 0(穩態把擾動抑制掉)

peak_dev = np.max(np.abs(yd))                 # disturbance 造成的最大偏移
peak_idx = int(np.argmax(np.abs(yd)))
peak_time = t[peak_idx]

# recovery time:峰值後回落到「峰值 5%」以內(視為已抑制回穩態)並不再離開
recov_band = 0.05 * peak_dev
recovery_time = float("nan")
for i in range(peak_idx, len(t)):
    if np.all(np.abs(yd[i:]) <= recov_band):
        recovery_time = t[i] - peak_time
        break

print("=== Ch7 Disturbance Response ===")
print(f"Plant  G(s) = 1/(0.3 s + 1)")
print(f"Ctrl   C(s) = (3 s + 6)/s   (PI,含積分器)")
print(f"command response DC 增益 T(0)  = {dc_cmd:.4f}  (理想 ≈ 1,追得上 setpoint)")
print(f"disturbance response DC 增益 Gd(0) = {dc_dist:.4f}  (理想 ≈ 0,穩態抑制擾動)")
print(f"disturbance 峰值偏移 = {peak_dev:.4f}  @ t = {peak_time:.3f} s")
print(f"disturbance 回復時間(回到峰值5%內) = {recovery_time:.3f} s")

# --- 圖:command vs disturbance response 疊圖 ---
fig, ax = plt.subplots(figsize=(7, 4.5))
ax.plot(t, yc, label="command response  T=C·G/(1+C·G)")
ax.plot(t, yd, label="plant-input disturbance  G·S")
ax.axhline(1.0, ls=":", c="gray", label="setpoint")
ax.axhline(0.0, ls=":", c="lightgray")
ax.plot(peak_time, yd[peak_idx], "rv", label=f"dist peak={peak_dev:.3f}")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Output")
ax.grid(True)
ax.legend()
ax.set_title("Ch7 command vs disturbance response")
show_or_save(plt, "ch07_disturbance.png")

# --- ✅ 驗證 ---
# 1. command response 穩態能追上 setpoint(DC 增益 ≈ 1)
assert abs(dc_cmd - 1.0) < 1e-3, "command response 穩態應追上 setpoint(T(0)≈1)"
# 2. 含積分器 → disturbance 穩態被完全抑制(DC 增益 ≈ 0)
assert abs(dc_dist) < 1e-3, "含積分器的 controller 應在穩態完全抑制 plant-input 擾動(Gd(0)≈0)"
# 3. disturbance response 末端確實回到 ≈0(數值層面)
assert abs(yd[-1]) < 1e-2, "disturbance response 末端應回落到 ≈0"
# 4. 擾動瞬間確有偏移,但被 controller 拉回(峰值 > 末端偏移)
assert peak_dev > abs(yd[-1]) + 1e-3, "controller 應把擾動峰值拉回穩態(rejection 有效)"
# 5. 回復時間可量測(有限值)
assert np.isfinite(recovery_time) and recovery_time > 0, "disturbance 應在有限時間內回復"
print("Ch7 驗證通過 ✅")

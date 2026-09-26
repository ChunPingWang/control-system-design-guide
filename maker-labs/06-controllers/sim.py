#!/usr/bin/env python
"""Ch6 Four Types of Controllers — Python simulation lab.

對應 maker-labs 講義 Chapter 6:比較 P / PI / PD / PID 在同一 plant 的表現,
建立 P/I/D 各自角色的因果直覺。

可從任意目錄執行:python maker-labs/06-controllers/sim.py
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))

import numpy as np
import matplotlib.pyplot as plt
import control as ct
from common.makerlab import step_metrics, show_or_save

G = ct.tf([1], [0.4, 1])          # 一階 plant
s = ct.tf([1, 0], [1])            # s 運算子

controllers = {
    "P":   ct.tf([2], [1]),
    "PI":  2 + 1.5 / s,
    "PD":  2 + 0.08 * s,
    "PID": 2 + 1.5 / s + 0.08 * s,
}

t = np.linspace(0, 12, 2000)
results = {}
fig, ax = plt.subplots(figsize=(7, 4.5))
for name, C in controllers.items():
    T = ct.feedback(C * G, 1)
    _, y = ct.step_response(T, t)
    results[name] = (y, step_metrics(t, y, setpoint=1.0), ct.dcgain(T))
    ax.plot(t, y, label=name)

ax.axhline(1.0, ls=":", c="gray", label="setpoint")
ax.set_xlabel("Time (s)"); ax.set_ylabel("Output"); ax.grid(True); ax.legend()
ax.set_title("Ch6 P / PI / PD / PID")
show_or_save(plt, "ch06_controllers.png")

print("=== Ch6 Four Types of Controllers ===")
for name, (y, m, dc) in results.items():
    print(f"{name:>3}: DCgain={dc:.3f}  sse={m['steady_state_error']:+.4f}  "
          f"OS={m['overshoot_pct']:.1f}%  rise={m['rise_time']:.3f}s")

# 用 DC 增益(解析、與模擬時長無關)判定穩態誤差:
#   含積分器的 controller → 閉迴路 DC 增益 = 1 → 零穩態誤差。
dc_p = results["P"][2]
dc_pi = results["PI"][2]
dc_pid = results["PID"][2]

# --- ✅ 驗證:I 消除穩態誤差;純 P 有殘差 ---
assert dc_p < 0.99, "純 P 對一階 plant 應留有 steady-state error(DC 增益<1)"
assert abs(dc_pi - 1.0) < 1e-6, "PI 應消除 steady-state error(DC 增益=1)"
assert abs(dc_pid - 1.0) < 1e-6, "PID 應消除 steady-state error(DC 增益=1)"
assert (1 - dc_p) > (1 - dc_pid), "加入 I 後穩態誤差應下降"
print("Ch6 驗證通過 ✅")

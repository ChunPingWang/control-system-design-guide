#!/usr/bin/env python
"""Ch1 Introduction to Controls — Python simulation lab.

對應 maker-labs 講義 Chapter 1:以一階馬達近似建立第一個閉迴路,
觀察 feedback 如何降低 steady-state error 與對 plant 變化的敏感度。

可從任意目錄執行:python maker-labs/01-introduction/sim.py
章末以 assert 自我驗證(供 run_all.sh 判定綠/紅)。
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))  # → maker-labs/

import numpy as np
import matplotlib.pyplot as plt
import control as ct
from common.makerlab import step_metrics, show_or_save

# --- Plant:一階馬達近似 G(s)=1/(tau s + 1) ---
tau = 0.5
G = ct.tf([1], [tau, 1])
K = 2.0

# 開迴路(直接把 command 當輸出)vs 閉迴路
T_closed = ct.feedback(K * G, 1)
t = np.linspace(0, 4, 800)
_, y_closed = ct.step_response(T_closed, t)
_, y_open = ct.step_response(K * G, t)   # 未回授,增益 K 直接乘 plant

print("=== Ch1 Introduction to Controls ===")
print(f"閉迴路 DC 增益 = {ct.dcgain(T_closed):.4f}  (理論 K/(1+K) = {K/(1+K):.4f})")
m = step_metrics(t, y_closed, setpoint=ct.dcgain(T_closed))
print(f"閉迴路 step:{m}")

# --- Plant 變化 20%,比較開/閉迴路輸出漂移 ---
G2 = ct.tf([1], [tau, 1.2])              # plant 增益/極點變化
y_open2 = ct.step_response(K * G2, t)[1]
y_closed2 = ct.step_response(ct.feedback(K * G2, 1), t)[1]
drift_open = abs(y_open2[-1] - y_open[-1]) / y_open[-1]
drift_closed = abs(y_closed2[-1] - y_closed[-1]) / y_closed[-1]
print(f"plant 變化後穩態漂移:開迴路 {drift_open*100:.1f}%  閉迴路 {drift_closed*100:.1f}%")

# --- 圖 ---
fig, ax = plt.subplots(figsize=(7, 4))
ax.plot(t, y_closed, label="closed loop")
ax.plot(t, y_open, "--", label="open loop (K·G)")
ax.axhline(1.0, ls=":", c="gray", label="setpoint")
ax.set_xlabel("Time (s)"); ax.set_ylabel("Output"); ax.grid(True); ax.legend()
ax.set_title("Ch1 open vs closed loop")
show_or_save(plt, "ch01_open_vs_closed.png")

# --- ✅ 驗證 ---
assert abs(ct.dcgain(T_closed) - K / (1 + K)) < 1e-6, "閉迴路 DC 增益應為 K/(1+K)"
assert drift_closed < drift_open, "feedback 應降低對 plant 變化的敏感度"
assert m["overshoot_pct"] < 1.0, "一階閉迴路不應有明顯 overshoot"
print("Ch1 驗證通過 ✅")

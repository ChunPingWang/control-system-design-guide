#!/usr/bin/env python
"""Ch12 Nonlinear Behavior and Time Variation — Python simulation lab.

對應 maker-labs 講義 Chapter 12:LTI vs non-LTI。用明確的 function/block
建模三種常見非線性(deadband、saturation、rate limit),而不是把誤差歸咎於
「noise」:
  1) 靜態非線性曲線:command → effective input(deadband + saturation)。
  2) 對閉迴路的影響:同一 PI controller,LTI plant vs 加入致動器
     deadband/saturation 的非 LTI plant,比較步階響應。

可從任意目錄執行:python maker-labs/12-nonlinear/sim.py
章末以 assert 自我驗證(供 run_all.sh 判定綠/紅)。
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))  # → maker-labs/

import numpy as np
import matplotlib.pyplot as plt
from common.makerlab import show_or_save


# --- 非線性 block(與 firmware/util.h 的 deadband / saturate 對應) ---
def deadband(u, dead):
    """死區:|u|<dead → 0,否則平移使輸出連續(斜率保持 1)。"""
    u = np.asarray(u, float)
    return np.where(np.abs(u) < dead, 0.0, np.sign(u) * (np.abs(u) - dead))


def saturate(u, limit):
    """對稱飽和:clip 到 ±limit。"""
    return np.clip(u, -limit, limit)


# ============================================================
# Part 1:靜態非線性曲線(對照講義 snippet)
# ============================================================
DEAD = 0.15
SAT = 1.0
GAIN = 2.0                     # deadband 後再放大,凸顯 saturation
u = np.linspace(-1.0, 1.0, 401)
y_db = deadband(u, DEAD)              # 只有 deadband
y_full = saturate(GAIN * y_db, SAT)  # deadband + gain + saturation

print("=== Ch12 Nonlinear Behavior and Time Variation ===")
print(f"deadband 寬度 = ±{DEAD}, gain = {GAIN}, saturation = ±{SAT}")
# 有效輸出開始飽和的 command 門檻:GAIN*(|u|-DEAD)=SAT → |u|=SAT/GAIN+DEAD
u_sat_onset = SAT / GAIN + DEAD
print(f"進入飽和的 command 門檻 ≈ ±{u_sat_onset:.3f}")


# ============================================================
# Part 2:非線性對閉迴路的影響
#   離散 PI 控制一階 plant;致動器指令先過 deadband + saturation。
# ============================================================
def sim_loop(setpoint, nonlinear, n=600, dt=0.01, tau=0.4, kp=1.2, ki=4.0):
    """回傳 (t, y, u_cmd)。nonlinear=True 時致動器有 deadband+saturation。"""
    a = np.exp(-dt / tau)          # 一階 plant 離散化(DC 增益=1)
    b = 1.0 - a
    y = 0.0
    integ = 0.0
    ys, us = [], []
    for _ in range(n):
        e = setpoint - y
        integ += e * dt
        u = kp * e + ki * integ    # PI 輸出(致動器指令)
        u_eff = u
        if nonlinear:
            u_eff = saturate(deadband(u, DEAD), SAT)
            # anti-windup:飽和時凍結積分,避免 wind-up 使響應惡化
            if abs(u) > SAT:
                integ -= e * dt
        y = a * y + b * u_eff      # plant 對「有效」致動輸出反應
        ys.append(y)
        us.append(u_eff)
    t = np.arange(n) * dt
    return t, np.array(ys), np.array(us)


SP = 0.5
t, y_lin, _ = sim_loop(SP, nonlinear=False)
_, y_nl, _ = sim_loop(SP, nonlinear=True)
sse_lin = SP - y_lin[-1]
sse_nl = SP - y_nl[-1]
print(f"步階 setpoint = {SP}")
print(f"  LTI plant     最終值 = {y_lin[-1]:.4f}  sse = {sse_lin:+.4f}")
print(f"  非 LTI plant  最終值 = {y_nl[-1]:.4f}  sse = {sse_nl:+.4f}")


# ============================================================
# 圖
# ============================================================
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))

ax1.plot(u, y_db, label="deadband only")
ax1.plot(u, y_full, label="+ gain + saturation")
ax1.plot(u, u, ":", c="gray", label="ideal LTI (y=u)")
ax1.axvspan(-DEAD, DEAD, color="orange", alpha=0.15, label="deadband zone")
ax1.set_xlabel("command"); ax1.set_ylabel("effective input")
ax1.set_title("Ch12 static nonlinearity"); ax1.grid(True); ax1.legend(fontsize=8)

ax2.plot(t, y_lin, label="LTI actuator")
ax2.plot(t, y_nl, label="deadband + saturation")
ax2.axhline(SP, ls=":", c="gray", label="setpoint")
ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Output")
ax2.set_title("Ch12 effect on closed loop"); ax2.grid(True); ax2.legend(fontsize=8)

show_or_save(plt, "ch12_nonlinear.png")


# ============================================================
# ✅ 驗證
# ============================================================
# --- deadband ---
inside = np.abs(u) < DEAD
assert np.all(y_db[inside] == 0.0), "deadband 內輸出必須恆為 0"
assert np.any(np.abs(y_db[~inside]) > 0.0), "deadband 外輸出應非零"
# 連續性:在邊界附近輸出應連續(無跳變),最大相鄰差 ~ 斜率*步距
assert np.max(np.abs(np.diff(y_db))) < 0.02, "deadband 輸出應連續(不得有跳變)"
# 對稱奇函數
assert np.allclose(y_db, -deadband(-u, DEAD)), "deadband 應為奇函數(對稱)"

# --- saturation ---
big = np.array([-100.0, -SAT - 5, SAT + 5, 100.0])
assert np.all(np.abs(saturate(big, SAT)) <= SAT + 1e-12), "saturation 必須夾到 ±limit"
assert np.isclose(saturate(SAT + 10, SAT), SAT), "超過上限應夾到 +limit"
assert np.isclose(saturate(-SAT - 10, SAT), -SAT), "低於下限應夾到 -limit"
# 線性區(未飽和)應維持原值
small = np.linspace(-SAT * 0.5, SAT * 0.5, 11)
assert np.allclose(saturate(small, SAT), small), "未飽和區 saturation 不應改變輸入"
# 有效輸出整體被夾在 ±SAT
assert np.all(np.abs(y_full) <= SAT + 1e-9), "deadband+saturation 輸出應在 ±SAT 內"

# --- 閉迴路:含積分器故 LTI 應近似零穩態誤差;非線性造成可見差異 ---
assert abs(sse_lin) < 1e-3, "LTI + PI 應達近似零穩態誤差"
assert np.max(np.abs(y_nl - y_lin)) > 1e-3, "非線性應對閉迴路造成可觀察差異"

print("Ch12 驗證通過 ✅")

#!/usr/bin/env python
"""Ch17 Position-Control Loops — Python simulation lab.

對應 maker-labs 講義 Chapter 17:位置控制迴路。

核心觀念:位置 = 速度的積分,所以「速度 plant」串上一個積分器 (1/s) 後
就變成「位置 plant」,整個受控體本身已含一個積分器 (type-1 系統)。

  → 這帶來一個和前面章節不同的結論:對「位置階躍命令」而言,
    純 P 控制器就已經有零穩態誤差(不需要靠 I 消除殘差),
    因為受控體內建的積分器扮演了消除 steady-state error 的角色。

本章比較 P / PI / PID 三種位置控制器對相同目標角度的階躍響應,
觀察 overshoot / damping 的差異:
  - P   :已零穩態誤差,但阻尼由 Kp 與 plant 極點決定,會有 overshoot。
  - PI  :再疊一個積分器(type-2),命令響應通常 overshoot 更大。
  - PID :導數項提供阻尼,把 PI 的 overshoot 壓下來。

可從任意目錄執行:python maker-labs/17-position-control/sim.py
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))

import numpy as np
import matplotlib.pyplot as plt
import control as ct
from common.makerlab import step_metrics, show_or_save

# ---- Plant ----
# 速度 plant(馬達:電壓命令 → 轉速),一階慣性:
TAU = 0.15
Gv = ct.tf([1], [TAU, 1])
# 位置 = 速度積分 → 串一個積分器 1/s:
Gp = Gv * ct.tf([1], [1, 0])       # 位置 plant(type-1,內建一個積分器)

s = ct.tf([1, 0], [1])

# ---- 三種位置控制器 ----
Kp = 8.0
Ki = 6.0
Kd = 1.2
controllers = {
    "P":   ct.tf([Kp], [1]),
    "PI":  Kp + Ki / s,
    "PID": Kp + Ki / s + Kd * s,
}

TARGET_DEG = 90.0                  # 目標角度(度)
t = np.linspace(0, 4, 3000)

results = {}
fig, ax = plt.subplots(figsize=(7.5, 4.5))
for name, C in controllers.items():
    T = ct.feedback(C * Gp, 1)
    _, y = ct.step_response(T, t)
    y = y * TARGET_DEG             # 縮放到目標角度
    m = step_metrics(t, y, setpoint=TARGET_DEG)
    dc = float(ct.dcgain(T))       # 閉迴路 DC 增益(解析,與模擬時長無關)
    results[name] = (y, m, dc)
    ax.plot(t, y, label=name)

ax.axhline(TARGET_DEG, ls=":", c="gray", label="target angle")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Position (deg)")
ax.set_title("Ch17 Position Loop: P / PI / PID step to target angle")
ax.grid(True)
ax.legend()
show_or_save(plt, "ch17_position_control.png")

print("=== Ch17 Position-Control Loops ===")
print(f"Plant = 1/(s*({TAU}s+1))  (velocity plant + integrator),  target = {TARGET_DEG:.0f} deg")
for name, (y, m, dc) in results.items():
    print(f"{name:>3}: DCgain={dc:.4f}  sse={m['steady_state_error']:+.3f} deg  "
          f"OS={m['overshoot_pct']:.1f}%  rise={m['rise_time']:.3f}s  "
          f"settle={m['settling_time']:.3f}s")

dc_p, dc_pi, dc_pid = results["P"][2], results["PI"][2], results["PID"][2]
os_p = results["P"][1]["overshoot_pct"]
os_pi = results["PI"][1]["overshoot_pct"]
os_pid = results["PID"][1]["overshoot_pct"]

# ---- ✅ 驗證 ----
# (1) 解析:受控體內建積分器 → 三種控制器閉迴路 DC 增益皆為 1 → 零穩態誤差。
#     特別是「純 P 已零穩態誤差」是位置迴路和一般 plant 最大的差別。
assert abs(dc_p - 1.0) < 1e-6, "位置迴路的純 P 應零穩態誤差(閉迴路 DC 增益=1)"
assert abs(dc_pi - 1.0) < 1e-6, "PI 位置迴路 DC 增益應為 1"
assert abs(dc_pid - 1.0) < 1e-6, "PID 位置迴路 DC 增益應為 1"

# (2) 三者穩態角度都應收斂到目標(數值上殘差極小)。
for name in ("P", "PI", "PID"):
    sse = results[name][1]["steady_state_error"]
    assert abs(sse) < 0.5, f"{name} 穩態誤差應趨近 0(得 {sse:.3f} deg)"

# (3) 阻尼比較(可量測):
#     疊加積分器的 PI 命令響應 overshoot 應大於純 P。
assert os_pi > os_p, f"PI 的 overshoot 應大於純 P(PI={os_pi:.1f}%, P={os_p:.1f}%)"
#     PID 的導數項提供阻尼,把 PI 的 overshoot 壓下來。
assert os_pid < os_pi, f"PID(有導數阻尼)overshoot 應小於 PI(PID={os_pid:.1f}%, PI={os_pi:.1f}%)"
#     所有控制器都會有一定 overshoot(欠阻尼二階以上),純 P 也不例外。
assert os_p > 0.0, "本組增益下純 P 位置迴路應為欠阻尼、有 overshoot"

print("Ch17 驗證通過 ✅")

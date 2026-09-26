#!/usr/bin/env python
"""Ch8 Feed-Forward — Python simulation lab.

對應 maker-labs 講義 Chapter 8:比較「純 feedback」與「feedback + feed-forward」
的 step 響應。Feedback 要看到 error 才修正;feed-forward 則利用已知 plant/command
先提供所需 control effort,因此命令追蹤更快。重點:FF 不取代 FB
(model mismatch / disturbance 仍需 feedback 收尾),兩者穩態相同,但 FF 讓 rise 更快。

可從任意目錄執行:python maker-labs/08-feedforward/sim.py
章末以 assert 自我驗證(供 run_all.sh 判定綠/紅)。
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))  # → maker-labs/

import numpy as np
import matplotlib.pyplot as plt
import control as ct
from common.makerlab import step_metrics, show_or_save

# --- Plant 與 controller(對應講義 Ch8 snippet)---
# G(s)=1/(0.3s+1) 一階馬達近似,DC 增益=1。
# C(s)=(3s+6)/s = 3 + 6/s,含積分器 → feedback 已可零穩態誤差。
G = ct.tf([1], [0.3, 1])
C = ct.tf([3, 6], [1, 0])

# 純 feedback 閉迴路
T_fb = ct.feedback(C * G, 1)

# feedback + 靜態 feed-forward:F = 1/DCgain(G) = 1,把命令前饋到 plant。
# 2-DOF 結構:u = C(r - y) + F r → Y = [GC/(1+GC) + GF/(1+GC)] R
# 第二項 G*feedback(1, C*G) = G/(1+GC) 即 feed-forward 對輸出的貢獻。
T_ff = ct.feedback(C * G, 1) + G * ct.feedback(1, C * G)

t = np.linspace(0, 3, 1500)
_, y_fb = ct.step_response(T_fb, t)
_, y_ff = ct.step_response(T_ff, t)

dc_fb = float(ct.dcgain(T_fb))
dc_ff = float(ct.dcgain(T_ff))
m_fb = step_metrics(t, y_fb, setpoint=1.0)
m_ff = step_metrics(t, y_ff, setpoint=1.0)

# 命令追蹤誤差積分(IAE):量化「追得快、追得準」的整體優劣。
iae_fb = float(np.trapezoid(np.abs(1.0 - y_fb), t))
iae_ff = float(np.trapezoid(np.abs(1.0 - y_ff), t))

# --- 圖:feedback vs feedback+FF step response overlay ---
fig, ax = plt.subplots(figsize=(7, 4.5))
ax.plot(t, y_fb, label="feedback only")
ax.plot(t, y_ff, label="feedback + feed-forward")
ax.axhline(1.0, ls=":", c="gray", label="setpoint")
ax.set_xlabel("Time (s)"); ax.set_ylabel("Output"); ax.grid(True); ax.legend()
ax.set_title("Ch8 feed-forward improves command tracking speed")
show_or_save(plt, "ch08_feedforward.png")

print("=== Ch8 Feed-Forward ===")
print(f"feedback     : DCgain={dc_fb:.4f}  rise={m_fb['rise_time']:.3f}s  "
      f"OS={m_fb['overshoot_pct']:.1f}%  IAE={iae_fb:.4f}")
print(f"feedback+FF  : DCgain={dc_ff:.4f}  rise={m_ff['rise_time']:.3f}s  "
      f"OS={m_ff['overshoot_pct']:.1f}%  IAE={iae_ff:.4f}")
print(f"rise 改善 {(1 - m_ff['rise_time']/m_fb['rise_time'])*100:.1f}%,"
      f"IAE 改善 {(1 - iae_ff/iae_fb)*100:.1f}%")

# --- ✅ 驗證 ---
# 1. 兩者穩態相同:FF 只改變暫態,不改變 DC(含積分器 → 皆為 1)。
assert abs(dc_fb - 1.0) < 1e-6, "feedback 閉迴路 DC 增益應為 1"
assert abs(dc_ff - dc_fb) < 1e-6, "加入 feed-forward 不應改變穩態(DC 增益相同)"
# 2. FF 讓命令追蹤更快:rise time 明顯下降。
assert m_ff["rise_time"] < m_fb["rise_time"], "feed-forward 應使 rise time 變快"
assert m_ff["rise_time"] < 0.7 * m_fb["rise_time"], "feed-forward 的 rise 改善應顯著(>30%)"
# 3. 整體命令追蹤誤差(IAE)下降。
assert iae_ff < iae_fb, "feed-forward 應降低命令追蹤誤差積分(IAE)"
print("Ch8 驗證通過 ✅")

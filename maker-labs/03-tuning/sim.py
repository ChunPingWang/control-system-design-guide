#!/usr/bin/env python
"""Ch3 Tuning a Control System — Python simulation lab.

對應 maker-labs 講義 Chapter 3:掃描 loop gain K,用 ct.margin(L) 量化
gain/phase margin,並疊圖比較各 K 的閉迴路 step 響應。
核心觀念:調參不是只看曲線「順不順」,而是看離失穩還有多少餘裕(GM/PM)。

Plant  G(s)=1/(0.2 s^2 + s)=1/(s(0.2s+1)),type-1(含積分器)。
Loop   L(s)=K·G(s);K 越大 → 交越頻率越高 → phase margin 越小 → overshoot 越大。

可從任意目錄執行:python maker-labs/03-tuning/sim.py
章末以 assert 自我驗證(供 run_all.sh 判定綠/紅),與模擬時長無關。
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))  # → maker-labs/

import numpy as np
import matplotlib.pyplot as plt
import control as ct
from common.makerlab import step_metrics, show_or_save

# --- Plant:含積分器的二階近似(對應講義 tf([1],[0.2,1,0]))---
G = ct.tf([1], [0.2, 1, 0])

# --- 掃描 loop gain K ---
K_list = [0.5, 2.0, 8.0]
t = np.linspace(0, 6, 3000)

print("=== Ch3 Tuning a Control System ===")
print(f"Plant G(s) = {G}")

results = {}
fig, ax = plt.subplots(figsize=(7.5, 4.5))
for K in K_list:
    L = K * G
    gm, pm, wcg, wcp = ct.margin(L)      # GM(倍率)、PM(度)、對應頻率
    T = ct.feedback(L, 1)
    _, y = ct.step_response(T, t)
    m = step_metrics(t, y, setpoint=1.0)
    results[K] = dict(gm=gm, pm=pm, wcp=wcp, overshoot=m["overshoot_pct"],
                      settling=m["settling_time"])
    gm_db = 20 * np.log10(gm) if np.isfinite(gm) else np.inf
    print(f"K={K:<4}  GM={gm_db:6.2f} dB  PM={pm:6.2f} deg  "
          f"wc={wcp:5.2f} rad/s  OS={m['overshoot_pct']:5.1f}%")
    ax.plot(t, y, label=f"K={K} (PM={pm:.0f}°)")

ax.axhline(1.0, ls=":", c="gray", label="setpoint")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Output")
ax.grid(True)
ax.legend()
ax.set_title("Ch3 loop-gain sweep — closed-loop step")
show_or_save(plt, "ch03_loop_gain_sweep.png")

# --- Bode(L)疊圖:直接看 gain/phase margin 隨 K 變化 ---
fig2 = plt.figure(figsize=(7.5, 6))
for K in K_list:
    ct.bode_plot(K * G, label=f"K={K}", dB=True, Hz=False)
plt.suptitle("Ch3 loop transfer L(s)=K·G(s) Bode")
show_or_save(plt, "ch03_loop_bode.png")

# --- ✅ 驗證(horizon-independent:靠 margin / overshoot 的單調關係)---
pm_vals = [results[K]["pm"] for K in K_list]
os_vals = [results[K]["overshoot"] for K in K_list]

# 1) phase margin 隨 K 單調下降(K 越大越接近失穩)
assert pm_vals[0] > pm_vals[1] > pm_vals[2], \
    f"phase margin 應隨 loop gain 上升而下降:{pm_vals}"
# 2) 所有 PM 為正(此 plant 對這些 K 仍閉迴路穩定)
assert all(pm > 0 for pm in pm_vals), f"這些 K 應仍穩定(PM>0):{pm_vals}"
# 3) overshoot 隨 K 上升(margin 變小 → 阻尼變差)
assert os_vals[0] < os_vals[1] < os_vals[2], \
    f"overshoot 應隨 loop gain 上升而增加:{os_vals}"
# 4) 最小的 K 幾乎無 overshoot,最大的 K 明顯 overshoot
assert os_vals[0] < 10.0, "最小 K 應接近臨界阻尼、overshoot 很小"
assert os_vals[-1] > 20.0, "最大 K 應有明顯 overshoot"
print("Ch3 驗證通過 ✅")

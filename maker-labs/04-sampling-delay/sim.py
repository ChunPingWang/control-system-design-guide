#!/usr/bin/env python
"""Ch4 Delay in Digital Controllers — Python simulation lab.

對應 maker-labs 講義 Chapter 4:同一 controller(比例增益 C),只改
sample time Ts,用 ZOH 離散化 plant,觀察取樣/保持帶來的 phase lag
如何讓閉迴路 overshoot 變大、stability margin 變差。

可從任意目錄執行:python maker-labs/04-sampling-delay/sim.py
章末以 assert 自我驗證(供 run_all.sh 判定綠/紅)。
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))  # → maker-labs/

import numpy as np
import matplotlib.pyplot as plt
import control as ct
from common.makerlab import step_metrics, show_or_save

# --- Plant:一階馬達近似 G(s)=1/(0.15 s + 1),固定 P 控制器 C=5 ---
G = ct.tf([1], [0.15, 1])
C = 5.0

# 連續(理想、無取樣延遲)閉迴路作為 baseline
T_cont = ct.feedback(C * G, 1)
sp = float(ct.dcgain(T_cont))          # 比例控制的穩態值 = C/(1+C)

Ts_list = [0.002, 0.01, 0.03, 0.05]    # sample time 由小到大 sweep
t = np.linspace(0, 3, 1500)

print("=== Ch4 Delay in Digital Controllers ===")
print(f"Plant G(s)=1/(0.15 s+1), 控制器 C={C:g}, 閉迴路穩態值 sp={sp:.4f}")
print(f"{'Ts(s)':>7} | {'overshoot%':>10} | {'settling(s)':>11} | {'gain margin(dB)':>15} | {'phase margin(deg)':>17}")

results = []
fig, ax = plt.subplots(figsize=(7.5, 4.2))

# 連續 baseline 疊圖
_, y_cont = ct.step_response(T_cont, t)
ax.plot(t, y_cont, "k--", lw=1.2, label="continuous (Ts→0)")

for Ts in Ts_list:
    Gd = ct.sample_system(G, Ts, method="zoh")     # ZOH 離散化 plant
    Td = ct.feedback(C * Gd, 1)                     # 數位閉迴路
    Ld = C * Gd                                     # 開迴路(算 margin 用)

    # step 響應(離散,以 zero-order hold 呈現 → plt.step where="post")
    td, yd = ct.step_response(Td, T=t[-1])
    m = step_metrics(td, yd, setpoint=sp)

    # stability margin(離散開迴路)
    gm, pm, _, _ = ct.margin(Ld)
    gm_db = 20 * np.log10(gm) if np.isfinite(gm) and gm > 0 else np.inf

    results.append({
        "Ts": Ts,
        "overshoot": m["overshoot_pct"],
        "settling": m["settling_time"],
        "gm_db": gm_db,
        "pm_deg": pm,
    })
    print(f"{Ts:>7.3f} | {m['overshoot_pct']:>10.2f} | "
          f"{m['settling_time']:>11.3f} | {gm_db:>15.2f} | {pm:>17.2f}")

    ax.step(td, yd, where="post", label=f"Ts={Ts*1000:.0f} ms")

ax.axhline(sp, ls=":", c="gray", label="setpoint (steady-state)")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Output")
ax.set_title("Ch4 digital control: larger Ts → more overshoot / less margin")
ax.grid(True)
ax.legend(loc="lower right", fontsize=8)
show_or_save(plt, "ch04_sampling_delay.png")

# --- 結論說明 ---
print("\n觀察:sample time Ts 越大,ZOH 造成的等效延遲越大,phase lag 增加,")
print("      → overshoot 變大、phase margin 變小,最後可能失穩。")
print("      這就是「同一 controller,只改 sample time」會惡化 performance 的原因。")

# --- ✅ 驗證:Ts 越大 → performance/穩定度惡化 ---
overshoots = [r["overshoot"] for r in results]
pms = [r["pm_deg"] for r in results]

# 1) 比例控制無穩態積分,穩態值應接近連續理論值 C/(1+C)
assert abs(sp - C / (1 + C)) < 1e-6, "P 控制閉迴路穩態值應為 C/(1+C)"

# 2) overshoot 隨 Ts 單調不減(取樣延遲惡化阻尼)
for a, b in zip(overshoots, overshoots[1:]):
    assert b >= a - 1e-6, f"Ts 變大 overshoot 不應變小:{overshoots}"

# 3) 最大 Ts 的 overshoot 明顯高於最小 Ts(至少多 3 個百分點)
assert overshoots[-1] > overshoots[0] + 3.0, \
    f"最大 Ts 應有明顯更大的 overshoot:{overshoots}"

# 4) phase margin 隨 Ts 單調不增(穩定度變差)
for a, b in zip(pms, pms[1:]):
    assert b <= a + 1e-6, f"Ts 變大 phase margin 不應變好:{pms}"

# 5) 連續 baseline 幾乎無 overshoot,離散最大 Ts 明顯有 overshoot
assert overshoots[0] < overshoots[-1], "離散化延遲應讓 overshoot 相對 baseline 增大"

print("Ch4 驗證通過 ✅")

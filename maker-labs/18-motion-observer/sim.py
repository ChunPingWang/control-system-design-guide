#!/usr/bin/env python
"""Ch18 Using the Luenberger Observer in Motion Control — Python simulation lab.

對應 maker-labs 講義 Chapter 18:把 Ch10 的 Luenberger observer 從「離線估計」
推進到「真正進 feedback loop」。這裡跑一個 velocity 控制迴路(motion loop):
只有 position 可量測(含噪),速度回授(velocity feedback)有兩種來源:

  A. finite-difference:直接對 noisy position 差分 → 噪聲被 1/dt 放大。
  B. observer:2 狀態 Luenberger observer 由同一 noisy position 估 velocity。

兩迴路用「完全相同」的 plant、controller、量測噪聲(FIXED seed),只差在
velocity feedback 的來源。要證明的事:observer 給的速度回授更貼近 true velocity,
因此控制命令(control effort)更平滑、迴路仍穩定,而差分回授把噪聲灌進命令。

可從任意目錄執行:python maker-labs/18-motion-observer/sim.py
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))

import numpy as np
import matplotlib.pyplot as plt
import control as ct
from common.makerlab import show_or_save

# --- 連續 plant 模型:state = [position, velocity],只量測 position ---
#   pdot = v
#   vdot = -2 v + 2 u   (一階馬達速度動態:steady-state v = u)
A = np.array([[0, 1], [0, -2.]])
B = np.array([[0], [2.]])
C = np.array([[1, 0]])          # 只量測 position

# --- observer pole placement:poles 比 plant(0, -2)快很多 ---
DESIRED_POLES = np.array([-20., -25.])
L = ct.place(A.T, C.T, DESIRED_POLES).T
obs_poles = np.linalg.eigvals(A - L @ C)

print("=== Ch18 Using the Luenberger Observer in Motion Control ===")
print("Observer gain L =", L.ravel())
print("Requested poles :", np.sort(DESIRED_POLES))
print("A-LC eigenvalues:", np.sort(obs_poles.real))

# --- 模擬設定 ---
rng = np.random.default_rng(20180924)   # FIXED seed → 可重現
dt = 0.001
t = np.arange(0, 4.0, dt)
N = len(t)

# velocity setpoint:0.5 s 後階躍到 5(counts/s 之類的一致單位)
r = np.where(t >= 0.5, 5.0, 0.0)

# 量測噪聲(加在 position 上);差分會把它放大 1/dt 倍
MEAS_NOISE = 0.01

# velocity PI controller(兩迴路共用同一組增益)
KP, KI = 0.6, 6.0
U_MIN, U_MAX = -20.0, 20.0      # 命令飽和(模擬 driver 限制)


def run_loop(feedback="observer"):
    """跑一個 velocity 迴路;feedback ∈ {'observer','fd'} 決定速度回授來源。

    回傳 dict:true velocity、量測、回授訊號、控制命令、position。
    每次呼叫都用同一顆固定種子 → 兩迴路看到完全相同的噪聲序列。
    """
    noise = rng_shared.normal(0, MEAS_NOISE, N)  # 共用噪聲序列

    x = np.zeros((2, N))            # true state [pos, vel]
    xhat = np.zeros((2, N))         # observer 估計
    y = np.zeros(N)                 # noisy position 量測
    v_fb = np.zeros(N)              # 回授用的速度
    u = np.zeros(N)                 # 控制命令
    integ = 0.0
    prev_y = 0.0

    for k in range(N):
        y[k] = x[0, k] + noise[k]

        # --- 取得速度回授 ---
        if feedback == "fd":
            v_fb[k] = 0.0 if k == 0 else (y[k] - prev_y) / dt
        else:  # observer
            v_fb[k] = xhat[1, k]
        prev_y = y[k]

        # --- PI 控制器(velocity loop)+ 飽和 + 條件 anti-windup ---
        e = r[k] - v_fb[k]
        integ += e * dt
        u_unsat = KP * e + KI * integ
        u[k] = min(max(u_unsat, U_MIN), U_MAX)
        if u[k] != u_unsat and e * u_unsat > 0:
            integ -= e * dt

        if k < N - 1:
            # --- true plant 前向 Euler ---
            xdot = A @ x[:, k] + B.ravel() * u[k]
            x[:, k + 1] = x[:, k] + dt * xdot
            # --- observer 前向 Euler(用同一個 u 與 noisy 量測 y)---
            err = y[k] - (C @ xhat[:, k])[0]
            xhatdot = A @ xhat[:, k] + B.ravel() * u[k] + L.ravel() * err
            xhat[:, k + 1] = xhat[:, k] + dt * xhatdot

    return dict(x=x, xhat=xhat, y=y, v_fb=v_fb, u=u)


# 兩迴路共用同一噪聲:各自建立同種子的 generator
rng_shared = np.random.default_rng(20180924)
res_obs = run_loop("observer")
rng_shared = np.random.default_rng(20180924)
res_fd = run_loop("fd")

true_v = res_obs["x"][1, :]     # 兩迴路 true velocity 幾乎一致,取 observer 迴路的

# --- 量化指標(取 setpoint 施加後的穩態視窗)---
win = t >= 1.5

# 1) 回授速度訊號 vs true velocity 的 RMS 誤差
rmse_fb_obs = np.sqrt(np.mean((res_obs["v_fb"][win] - res_obs["x"][1, win]) ** 2))
rmse_fb_fd = np.sqrt(np.mean((res_fd["v_fb"][win] - res_fd["x"][1, win]) ** 2))

# 2) 控制命令抖動(相鄰命令差的 std → 噪聲灌進命令的程度)
chatter_obs = np.std(np.diff(res_obs["u"][win]))
chatter_fd = np.std(np.diff(res_fd["u"][win]))

# 3) 穩態 true velocity 平均(兩者都應收到 setpoint 附近 → 迴路穩定且無穩態誤差)
vmean_obs = res_obs["x"][1, win].mean()
vmean_fd = res_fd["x"][1, win].mean()

print(f"回授速度 RMS 誤差 vs true:observer={rmse_fb_obs:.4f}  finite-diff={rmse_fb_fd:.4f}")
print(f"控制命令抖動 std(Δu):observer={chatter_obs:.4f}  finite-diff={chatter_fd:.4f}")
print(f"穩態 true velocity 平均:observer={vmean_obs:.3f}  finite-diff={vmean_fd:.3f}  (setpoint=5)")

# --- 繪圖 ---
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 9), sharex=True)

ax1.plot(t, res_fd["v_fb"], color="0.75", lw=0.5, label="finite-diff velocity (feedback)")
ax1.plot(t, res_obs["v_fb"], "C0", lw=1.2, label="observer velocity (feedback)")
ax1.plot(t, true_v, "k", lw=1.6, label="true velocity")
ax1.plot(t, r, "r:", lw=1.2, label="velocity setpoint")
ax1.set_ylabel("Velocity")
ax1.set_ylim(-8, 14)
ax1.grid(True); ax1.legend(loc="upper right", fontsize=8)
ax1.set_title("Ch18 Observer velocity feedback vs finite-difference in a motion loop")

ax2.plot(t, res_fd["u"], color="0.75", lw=0.5, label="command u (finite-diff loop)")
ax2.plot(t, res_obs["u"], "C1", lw=1.2, label="command u (observer loop)")
ax2.set_ylabel("Control command u")
ax2.grid(True); ax2.legend(loc="upper right", fontsize=8)

ax3.plot(t, res_fd["x"][1, :], color="0.55", lw=1.0, label="true velocity (finite-diff loop)")
ax3.plot(t, res_obs["x"][1, :], "C2", lw=1.4, label="true velocity (observer loop)")
ax3.plot(t, r, "r:", lw=1.2, label="setpoint")
ax3.set_xlabel("Time (s)"); ax3.set_ylabel("True velocity")
ax3.grid(True); ax3.legend(loc="lower right", fontsize=8)

show_or_save(plt, "ch18_motion_observer.png")

# --- ✅ 驗證 ---
# 1. observer pole placement 正確
assert np.allclose(np.sort(obs_poles.real), np.sort(DESIRED_POLES), atol=1e-6), \
    "A-LC 的 eigenvalues 應等於指定的 observer poles"
assert np.allclose(np.sort(obs_poles.imag), 0.0, atol=1e-9), \
    "本例指定實數 poles,eigenvalues 不應有虛部"

# 2. observer 回授速度明顯比差分回授更貼近 true velocity(抗噪)
assert rmse_fb_obs < 0.5 * rmse_fb_fd, \
    "observer 速度回授對 true velocity 的 RMS 誤差應遠小於差分回授"

# 3. observer 迴路的控制命令明顯更平滑(噪聲沒有被灌進命令)
assert chatter_obs < 0.5 * chatter_fd, \
    "observer 迴路的控制命令抖動應遠小於差分回授迴路"

# 4. 兩迴路都必須維持穩定(true state 有界,不發散)
for name, res in [("observer", res_obs), ("finite-diff", res_fd)]:
    assert np.all(np.isfinite(res["x"])), f"{name} 迴路 state 出現 NaN/Inf"
    assert np.max(np.abs(res["x"][1, :])) < 50.0, f"{name} 迴路 velocity 發散"

# 5. observer 迴路穩態 velocity 追到 setpoint;差分迴路因噪聲被命令飽和整流而偏低,
#    追蹤也較差 → observer 對 true velocity 的追蹤誤差應小於差分迴路。
assert abs(vmean_obs - 5.0) < 0.5, "observer 迴路穩態 velocity 應追到 setpoint"
assert abs(vmean_fd - 5.0) < 2.0, "finite-diff 迴路仍有界,但穩態 velocity 已被噪聲拉偏"
track_err_obs = abs(vmean_obs - 5.0)
track_err_fd = abs(vmean_fd - 5.0)
assert track_err_obs < track_err_fd, \
    "observer 迴路的穩態追蹤誤差應小於差分迴路(observer 控制更準)"

print("Ch18 驗證通過 ✅")

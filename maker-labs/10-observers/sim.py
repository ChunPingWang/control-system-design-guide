#!/usr/bin/env python
"""Ch10 Introduction to Observers — Python simulation lab.

對應 maker-labs 講義 Chapter 10:用 Luenberger observer 由「只有 position 的
含噪量測」估計整個 state(position + velocity),建立 state estimation 的因果直覺。

流程:
  1. 由連續模型 (A,B,C) 用 ct.place 設計 observer gain L,使 error dynamics
     A-LC 的 poles 落在指定位置(比 plant 快)。
  2. 驗證 eig(A-LC) 確實等於指定 poles。
  3. 模擬含噪 position 量測,比較 true state 與 observer 估計(position/velocity),
     並對照「有限差分速度」(finite-difference)看 observer 如何抑制噪聲。

可從任意目錄執行:python maker-labs/10-observers/sim.py
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))

import numpy as np
import matplotlib.pyplot as plt
import control as ct
from common.makerlab import show_or_save

# --- 連續模型(與講義 snippet 一致):state = [position, velocity] ---
A = np.array([[0, 1], [0, -2.]])
B = np.array([[0], [2.]])
C = np.array([[1, 0]])          # 只量測 position

# --- observer pole placement:poles 比 plant(0, -2)快 ---
DESIRED_POLES = np.array([-6., -7.])
L = ct.place(A.T, C.T, DESIRED_POLES).T
obs_poles = np.linalg.eigvals(A - L @ C)

print("=== Ch10 Introduction to Observers ===")
print("Observer gain L =", L.ravel())
print("Requested poles :", np.sort(DESIRED_POLES))
print("A-LC eigenvalues:", np.sort(obs_poles.real))

# --- 含噪量測模擬 ---
rng = np.random.default_rng(0)
dt = 0.001
t = np.arange(0, 6.0, dt)
N = len(t)

# 輸入:1 s 後施加階躍輸入,推動 plant 產生非平凡的 position/velocity
u = np.where(t >= 1.0, 1.0, 0.0)

# true state:前向 Euler 積分 xdot = A x + B u
x = np.zeros((2, N))
x[:, 0] = [0.0, 0.0]
for k in range(N - 1):
    xdot = A @ x[:, k] + (B.ravel() * u[k])
    x[:, k + 1] = x[:, k] + dt * xdot

# 量測:position + 高斯噪聲
meas_noise = 0.01
y = x[0, :] + rng.normal(0, meas_noise, N)

# observer:故意給錯誤初始估計,觀察 error 收斂
xhat = np.zeros((2, N))
xhat[:, 0] = [0.5, -3.0]        # 與 true(0,0)有明顯偏差
for k in range(N - 1):
    err = y[k] - (C @ xhat[:, k])[0]
    xhatdot = A @ xhat[:, k] + (B.ravel() * u[k]) + L.ravel() * err
    xhat[:, k + 1] = xhat[:, k] + dt * xhatdot

# 有限差分速度(直接對含噪 position 差分)——對照組,噪聲被放大
fd_vel = np.zeros(N)
fd_vel[1:] = (y[1:] - y[:-1]) / dt

# 估計誤差(true - estimate)
err_pos = x[0, :] - xhat[0, :]
err_vel = x[1, :] - xhat[1, :]

# --- 繪圖 ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)

ax1.plot(t, y, color="0.7", lw=0.6, label="measured position (noisy)")
ax1.plot(t, x[0, :], "k", lw=1.6, label="true position")
ax1.plot(t, xhat[0, :], "C0--", lw=1.4, label="observer position")
ax1.set_ylabel("Position"); ax1.grid(True); ax1.legend(loc="best")
ax1.set_title("Ch10 Luenberger Observer — state estimation from noisy position")

ax2.plot(t, fd_vel, color="0.7", lw=0.5, label="finite-difference velocity")
ax2.plot(t, x[1, :], "k", lw=1.6, label="true velocity")
ax2.plot(t, xhat[1, :], "C3--", lw=1.6, label="observer velocity")
ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Velocity")
ax2.grid(True); ax2.legend(loc="best")
ax2.set_ylim(x[1, :].min() - 2, x[1, :].max() + 2)

show_or_save(plt, "ch10_observer.png")

# --- 量化指標 ---
# 收斂:比較「暫態初期」與「穩態末段」的估計誤差範數
warmup = t < 0.5           # observer 剛啟動,初始誤差主導
settled = t > 4.0          # error dynamics 應已衰減
e_early = np.sqrt(err_pos[warmup] ** 2 + err_vel[warmup] ** 2).mean()
e_late = np.sqrt(err_pos[settled] ** 2 + err_vel[settled] ** 2).mean()

# observer 速度 vs 有限差分速度(對 true velocity 的 RMSE,取穩態後)
rmse_obs = np.sqrt(np.mean((xhat[1, settled] - x[1, settled]) ** 2))
rmse_fd = np.sqrt(np.mean((fd_vel[settled] - x[1, settled]) ** 2))

print(f"估計誤差範數:early={e_early:.4f} → late={e_late:.4f}")
print(f"速度估計 RMSE:observer={rmse_obs:.4f}  finite-diff={rmse_fd:.4f}")

# --- ✅ 驗證 ---
# 1. observer poles 必須等於指定 poles(pole placement 正確)
assert np.allclose(np.sort(obs_poles.real), np.sort(DESIRED_POLES), atol=1e-6), \
    "A-LC 的 eigenvalues 應等於指定的 observer poles"
assert np.allclose(np.sort(obs_poles.imag), 0.0, atol=1e-9), \
    "本例指定實數 poles,eigenvalues 不應有虛部"

# 2. 估計誤差應隨時間收斂(末段遠小於初期)
assert e_late < 0.1 * e_early, "估計誤差應隨 error dynamics 收斂(late << early)"
assert e_late < 0.05, "穩態估計誤差應接近 0"

# 3. observer 對含噪量測估速度,應明顯優於直接有限差分
assert rmse_obs < rmse_fd, "observer 速度估計應比有限差分更抗噪(RMSE 更小)"

print("Ch10 驗證通過 ✅")

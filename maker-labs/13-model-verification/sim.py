#!/usr/bin/env python
"""Ch13 Model Development and Verification — Python simulation lab.

對應 maker-labs 講義 Chapter 13:從「量測資料」建立一階馬達模型,
並用「未參與 fitting 的第二筆資料」驗證模型,而不是用同一筆資料自證。

核心流程(七步建模的可執行縮影):
  1. 定義用途:預測馬達轉速 step 響應。
  2. 選結構:一階 y(t) = K·(1 - exp(-t/τ))。
  3. 產生「量測」:真實 plant + 感測雜訊(FIXED seed → 可重現)。
  4. Run A(train)以 curve_fit 估 K, τ。
  5. Run B(validation)另一組雜訊 → simulate 模型並比對。
  6. compare:計算 RMSE 與 VAF(percent fit)。
  7. revise / 判定:validation 指標達標才算通過。

可從任意目錄執行:python maker-labs/13-model-verification/sim.py
章末以 assert 自我驗證(供 run_all.sh 判定綠/紅)。
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))  # → maker-labs/

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from common.makerlab import show_or_save

# --- 真實(未知)一階 plant 參數:量測時我們「假裝」不知道 ---
K_TRUE = 1200.0      # 穩態轉速 (rpm)
TAU_TRUE = 0.28      # 時間常數 (s)
NOISE_STD = 18.0     # 感測雜訊 std (rpm)

t = np.linspace(0.0, 2.0, 100)          # 2 s step test,取樣 100 點


def first_order_step(t, K, tau):
    """一階系統對單位 step 的響應結構。"""
    return K * (1.0 - np.exp(-t / tau))


def measure(t, seed):
    """模擬一次 step test:真實響應 + 高斯感測雜訊(FIXED seed → 可重現)。"""
    rng = np.random.default_rng(seed)
    true = first_order_step(t, K_TRUE, TAU_TRUE)
    return true + rng.normal(0.0, NOISE_STD, len(t))


def vaf(y_meas, y_model):
    """Variance Accounted For(百分比擬合度);100% 表示模型完全解釋量測變異。"""
    err_var = np.var(y_meas - y_model)
    sig_var = np.var(y_meas)
    return max(0.0, (1.0 - err_var / sig_var) * 100.0)


def rmse(y_meas, y_model):
    return float(np.sqrt(np.mean((y_meas - y_model) ** 2)))


print("=== Ch13 Model Development and Verification ===")

# --- Run A(train):量測 → fitting ---
meas_A = measure(t, seed=13)            # FIXED seed
p0 = [1000.0, 0.2]                       # 初始猜測
popt, _ = curve_fit(first_order_step, t, meas_A, p0=p0)
K_fit, tau_fit = popt
model_A = first_order_step(t, *popt)
print(f"真實參數:K={K_TRUE:.1f} rpm, τ={TAU_TRUE:.3f} s")
print(f"估計參數:K={K_fit:.1f} rpm, τ={tau_fit:.3f} s (由 Run A fitting)")
print(f"Run A (train)      : RMSE={rmse(meas_A, model_A):6.2f} rpm, VAF={vaf(meas_A, model_A):5.2f}%")

# --- Run B(validation):另一組雜訊,模型「不重新 fitting」直接預測 ---
meas_B = measure(t, seed=1313)          # 不同 FIXED seed → 獨立驗證資料
model_B = first_order_step(t, *popt)    # 沿用 Run A 的參數
rmse_B = rmse(meas_B, model_B)
vaf_B = vaf(meas_B, model_B)
print(f"Run B (validation) : RMSE={rmse_B:6.2f} rpm, VAF={vaf_B:5.2f}%")

# 參數估計誤差
err_K = abs(K_fit - K_TRUE) / K_TRUE * 100.0
err_tau = abs(tau_fit - TAU_TRUE) / TAU_TRUE * 100.0
print(f"參數誤差:K {err_K:.2f}%, τ {err_tau:.2f}%")

# --- 圖:量測 vs 模型(train 與 validation overlay)---
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4), sharey=True)
ax1.scatter(t, meas_A, s=12, c="tab:blue", alpha=0.6, label="measured (Run A)")
ax1.plot(t, model_A, "r-", lw=2, label="fitted model")
ax1.set_title(f"Run A — fitting\nK={K_fit:.0f}, τ={tau_fit:.3f}")
ax1.set_xlabel("Time (s)"); ax1.set_ylabel("Speed (rpm)")
ax1.grid(True); ax1.legend(loc="lower right")

ax2.scatter(t, meas_B, s=12, c="tab:green", alpha=0.6, label="measured (Run B)")
ax2.plot(t, model_B, "r-", lw=2, label="model (no re-fit)")
ax2.set_title(f"Run B — validation\nRMSE={rmse_B:.1f}, VAF={vaf_B:.1f}%")
ax2.set_xlabel("Time (s)")
ax2.grid(True); ax2.legend(loc="lower right")
fig.suptitle("Ch13 Model vs Measurement (train / validation)")
show_or_save(plt, "ch13_model_verification.png")

# --- ✅ 驗證:重點是「validation」資料達標,而非只擬合 train ---
assert err_K < 5.0, "K 估計誤差應 < 5%"
assert err_tau < 15.0, "τ 估計誤差應 < 15%"
assert rmse_B < 3.0 * NOISE_STD, "validation RMSE 應接近雜訊水準(未 overfit)"
assert vaf_B > 90.0, "validation VAF 應 > 90%(模型能解釋獨立資料)"
print(f"驗證資料 VAF={vaf_B:.1f}% (> 90%), RMSE={rmse_B:.1f} rpm")
print("Ch13 驗證通過 ✅")

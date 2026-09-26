#!/usr/bin/env python
"""Ch14 Encoders and Resolvers — Python simulation lab.

對應 maker-labs 講義 Chapter 14:編碼器 position 是量化值,velocity 由
量化後的 count 在一段量測窗內差分推估。本模擬展示三件事:

  1. 低速 / 短窗 時,量化雜訊在 velocity 估測上被放大(counts/window 太少)。
  2. 加長量測窗可降低量化 velocity 誤差,但代價是延遲(response 變慢)。
  3. 對 raw velocity 做一階低通(OnePoleLP)可壓低 RMS 雜訊,但引入 lag。

可從任意目錄執行:python maker-labs/14-encoder/sim.py
章末以 assert 自我驗證(供 run_all.sh 判定綠/紅)。
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))  # → maker-labs/

import numpy as np
import matplotlib.pyplot as plt
from common.makerlab import show_or_save

RNG = np.random.default_rng(20260924)  # 固定 seed:結果可重現

# --- 編碼器與取樣參數 ---
CPR = 400            # counts per revolution(4x 解碼後)
FS = 1000.0          # 底層 count 取樣頻率 (Hz)
DT = 1.0 / FS
T_END = 2.0
t = np.arange(0, T_END, DT)


def true_velocity(rpm):
    """回傳等速 rpm 的真實角位置(counts)與真實速度(rpm)時間序列。"""
    cps = rpm / 60.0 * CPR                 # 真實 counts/sec
    true_pos = cps * t                     # 連續(未量化)位置 counts
    return true_pos, np.full_like(t, rpm)


def quantized_counts(true_pos):
    """編碼器只能回報整數 count → floor 量化。"""
    return np.floor(true_pos).astype(np.int64)


def velocity_from_window(counts, window_samples):
    """固定窗差分法估速度:每 window_samples 取一次 (Δcount / Δt) → rpm。

    回傳 (窗中心時間, 估測 rpm)。這是硬體上最常見的 fixed-window count 法。
    """
    edges = np.arange(0, len(counts), window_samples)
    tw, vw = [], []
    for i in range(1, len(edges)):
        i0, i1 = edges[i - 1], edges[i]
        dcount = counts[i1] - counts[i0]
        dt = (i1 - i0) * DT
        cps = dcount / dt
        rpm = cps / CPR * 60.0
        tw.append(t[i1])
        vw.append(rpm)
    return np.array(tw), np.array(vw)


def onepole_lp(x, fc_hz, dt):
    """一階 IIR 低通,對齊 firmware filter.h 的 OnePoleLP。"""
    rc = 1.0 / (2.0 * np.pi * fc_hz)
    alpha = dt / (dt + rc)
    y = np.empty_like(x, dtype=float)
    acc = x[0]
    for i, xi in enumerate(x):
        acc += alpha * (xi - acc)
        y[i] = acc
    return y


print("=== Ch14 Encoders and Resolvers ===")
print(f"CPR={CPR}, 底層取樣 {FS:.0f} Hz")

# --- 實驗 A:量測窗長度 vs 量化 velocity 誤差(中速) ---
RPM_A = 120.0
pos_a, vtrue_a = true_velocity(RPM_A)
cnt_a = quantized_counts(pos_a)
windows = [5, 10, 20, 50, 100]     # 每窗取樣數(越大 = 窗越長)
rms_by_window = []
for w in windows:
    _, vw = velocity_from_window(cnt_a, w)
    err = vw - RPM_A
    rms = float(np.sqrt(np.mean(err ** 2)))
    rms_by_window.append(rms)
    print(f"窗 {w:3d} 取樣 ({w*DT*1000:5.1f} ms):量化 velocity RMS 誤差 = {rms:6.3f} rpm")

# --- 實驗 B:低速 vs 中速,同一短窗下的雜訊放大 ---
WIN_B = 10
RPM_LOW, RPM_MID = 15.0, 150.0
_, v_low = velocity_from_window(quantized_counts(true_velocity(RPM_LOW)[0]), WIN_B)
_, v_mid = velocity_from_window(quantized_counts(true_velocity(RPM_MID)[0]), WIN_B)
rms_low = float(np.sqrt(np.mean((v_low - RPM_LOW) ** 2)))
rms_mid = float(np.sqrt(np.mean((v_mid - RPM_MID) ** 2)))
# 以相對誤差(%)比較:低速的量化雜訊佔比明顯較大
rel_low = rms_low / RPM_LOW * 100.0
rel_mid = rms_mid / RPM_MID * 100.0
print(f"短窗 {WIN_B} 取樣:低速 {RPM_LOW} rpm 相對雜訊 {rel_low:.1f}% | "
      f"中速 {RPM_MID} rpm 相對雜訊 {rel_mid:.1f}%")

# --- 實驗 C:raw velocity vs 低通濾波(noise/lag trade-off) ---
# 用逐樣本差分(最短窗=1)得到最吵的 raw velocity,再低通。
cps_raw = np.diff(cnt_a, prepend=cnt_a[0]) / DT
vraw = cps_raw / CPR * 60.0
FC = 8.0  # 低通截止頻率 (Hz)
vfilt = onepole_lp(vraw, FC, DT)
# 跳過暖機段(前 5%)避免初值影響 RMS 統計
warm = int(0.05 * len(t))
rms_raw = float(np.sqrt(np.mean((vraw[warm:] - RPM_A) ** 2)))
rms_filt = float(np.sqrt(np.mean((vfilt[warm:] - RPM_A) ** 2)))
print(f"逐樣本差分:raw RMS 雜訊 {rms_raw:.3f} rpm → 低通 {FC:.0f} Hz 後 {rms_filt:.3f} rpm")

# --- 圖:三張子圖 ---
fig, axs = plt.subplots(3, 1, figsize=(8, 10))

# (1) 窗長 vs RMS 誤差
axs[0].plot([w * DT * 1000 for w in windows], rms_by_window, "o-")
axs[0].set_xlabel("Measurement window (ms)")
axs[0].set_ylabel("Velocity RMS error (rpm)")
axs[0].set_title(f"Ch14 (a) window length vs quantization noise @ {RPM_A:.0f} rpm")
axs[0].grid(True)

# (2) 低速 vs 中速估測波形(同短窗)
tw_low, _ = velocity_from_window(quantized_counts(true_velocity(RPM_LOW)[0]), WIN_B)
tw_mid, _ = velocity_from_window(quantized_counts(true_velocity(RPM_MID)[0]), WIN_B)
axs[1].step(tw_low, v_low, where="post", label=f"{RPM_LOW:.0f} rpm est")
axs[1].axhline(RPM_LOW, ls="--", c="C0")
axs[1].step(tw_mid, v_mid, where="post", label=f"{RPM_MID:.0f} rpm est")
axs[1].axhline(RPM_MID, ls="--", c="C1")
axs[1].set_xlabel("Time (s)"); axs[1].set_ylabel("Estimated RPM")
axs[1].set_title(f"Ch14 (b) low vs mid speed, same {WIN_B*DT*1000:.0f} ms window")
axs[1].grid(True); axs[1].legend()

# (3) raw vs filtered velocity
axs[2].plot(t, vraw, c="0.7", lw=0.8, label="raw diff velocity")
axs[2].plot(t, vfilt, c="C3", lw=1.5, label=f"OnePoleLP {FC:.0f} Hz")
axs[2].axhline(RPM_A, ls="--", c="k", label="true")
axs[2].set_xlabel("Time (s)"); axs[2].set_ylabel("RPM")
axs[2].set_title("Ch14 (c) low-pass trades noise for lag")
axs[2].grid(True); axs[2].legend()

fig.tight_layout()
show_or_save(plt, "ch14_encoder.png")

# --- ✅ 驗證 ---
# 1) 加長量測窗應降低量化 velocity 誤差(單調不增,允許數值抖動容忍)。
assert rms_by_window[-1] < rms_by_window[0], "加長量測窗應降低量化 velocity 誤差"
for a, b in zip(rms_by_window, rms_by_window[1:]):
    assert b <= a + 1e-6, "RMS 誤差應隨窗長單調不增"
# 2) 同短窗下,低速的相對量化雜訊應大於中速。
assert rel_low > rel_mid, "低速的相對量化雜訊應大於中速"
# 3) 低通濾波後的 RMS 雜訊應明顯小於 raw。
assert rms_filt < rms_raw, "低通濾波應降低 velocity RMS 雜訊"
assert rms_filt < 0.5 * rms_raw, "低通應顯著壓低雜訊(<50% raw)"
print("Ch14 驗證通過 ✅")

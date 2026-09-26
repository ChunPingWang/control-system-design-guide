#!/usr/bin/env python
"""Ch9 Filters in Control Systems — Python simulation lab.

對應 maker-labs 講義 Chapter 9:對含噪訊號套用 Butterworth low-pass,
觀察 filter 如何降低 sensor noise,以及「截止頻率越低 → 噪聲衰減越多、
但 phase lag 越大」這個控制系統無法迴避的 trade-off。

可從任意目錄執行:python maker-labs/09-filters/sim.py
章末以 assert 自我驗證(供 run_all.sh 判定綠/紅)。
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))  # → maker-labs/

import numpy as np
import scipy.signal as sig
import matplotlib.pyplot as plt
from common.makerlab import show_or_save

# --- 產生含噪訊號(固定 seed 保證可重現)---
# 情境:量測一顆約 1000 RPM 的馬達,真實轉速只有緩慢變化(1 Hz,遠低於任一
# cutoff,屬 passband 內),疊加高頻感測雜訊(白噪)。這樣「filter 拿掉的」
# 幾乎純粹是 noise,不會誤傷 signal,噪聲衰減量才可乾淨比較。
fs = 200                                  # 取樣頻率 (Hz)
t = np.arange(0, 3, 1 / fs)
rng = np.random.default_rng(9)            # ← 固定 seed,結果 deterministic
clean = 1000 + 30 * np.sin(2 * np.pi * 1.0 * t)  # 真訊號:DC + 1 Hz 緩變(passband 內)
noise = 25 * rng.normal(size=len(t))             # 高頻量測雜訊
x = clean + noise

# --- 兩種截止頻率的 2 階 Butterworth low-pass ---
# 低 cutoff → 噪聲衰減更多,但相位延遲(phase lag)更大。
fc_low = 5      # Hz,較保守:平滑但延遲大
fc_high = 20    # Hz,較快:延遲小但殘餘噪聲多


def butter_lp(signal, fc):
    b, a = sig.butter(2, fc / (fs / 2))
    return sig.lfilter(b, a, signal), (b, a)


y_low, (b_lo, a_lo) = butter_lp(x, fc_low)
y_high, (b_hi, a_hi) = butter_lp(x, fc_high)

# --- 量化指標 ---
# 用穩態段(略過起始暫態)量殘餘噪聲 RMS:估測值 - 真訊號。
mask = t > 0.5
raw_noise_rms = float(np.sqrt(np.mean((x[mask] - clean[mask]) ** 2)))
low_noise_rms = float(np.sqrt(np.mean((y_low[mask] - clean[mask]) ** 2)))
high_noise_rms = float(np.sqrt(np.mean((y_high[mask] - clean[mask]) ** 2)))


def group_delay_ms(b, a, fc):
    """在該 filter 通帶內(取一個低頻點)估 group delay,換算成毫秒。"""
    w, gd = sig.group_delay((b, a), w=[2 * np.pi * 1.0 / fs])  # 1 Hz 處
    return float(gd[0]) / fs * 1000.0


lag_low_ms = group_delay_ms(b_lo, a_lo, fc_low)
lag_high_ms = group_delay_ms(b_hi, a_hi, fc_high)

print("=== Ch9 Filters in Control Systems ===")
print(f"取樣頻率 fs = {fs} Hz,seed = 9")
print(f"殘餘噪聲 RMS:raw={raw_noise_rms:.2f}  "
      f"fc={fc_low}Hz→{low_noise_rms:.2f}  fc={fc_high}Hz→{high_noise_rms:.2f}")
print(f"通帶 group delay:fc={fc_low}Hz→{lag_low_ms:.1f} ms  "
      f"fc={fc_high}Hz→{lag_high_ms:.1f} ms")

# --- 圖 1:raw vs filtered 時間序列(放大前 0.5 s 看波形與延遲)---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.5, 7))
ax1.plot(t, x, alpha=0.4, label="raw (noisy)")
ax1.plot(t, y_high, label=f"low-pass fc={fc_high} Hz")
ax1.plot(t, y_low, label=f"low-pass fc={fc_low} Hz")
ax1.plot(t, clean, ":", c="black", lw=1, label="true signal")
ax1.set_xlim(0, 0.5)
ax1.set_xlabel("Time (s)"); ax1.set_ylabel("RPM")
ax1.grid(True); ax1.legend(loc="upper right")
ax1.set_title("Ch9 raw vs low-pass filtered (noise ↓ but lag ↑)")

# --- 圖 2:兩個 cutoff 的頻率響應(Bode 幅值),看 passband/attenuation ---
for (b, a), fc in [((b_hi, a_hi), fc_high), ((b_lo, a_lo), fc_low)]:
    w, h = sig.freqz(b, a, worN=2048, fs=fs)
    ax2.semilogx(w, 20 * np.log10(np.abs(h) + 1e-12), label=f"fc={fc} Hz")
ax2.axhline(-3, ls=":", c="gray", label="-3 dB")
ax2.set_xlim(0.5, fs / 2)
ax2.set_xlabel("Frequency (Hz)"); ax2.set_ylabel("Magnitude (dB)")
ax2.grid(True, which="both"); ax2.legend()
ax2.set_title("Butterworth low-pass frequency response")
show_or_save(plt, "ch09_filters.png")

# --- ✅ 驗證 ---
# 1) 兩個 filter 都要降低殘餘噪聲。
assert low_noise_rms < raw_noise_rms, "low-pass 應降低殘餘噪聲 RMS"
assert high_noise_rms < raw_noise_rms, "low-pass 應降低殘餘噪聲 RMS"
# 2) 截止頻率越低 → 噪聲衰減越多(殘餘 RMS 越小)。
assert low_noise_rms < high_noise_rms, "cutoff 越低,殘餘噪聲應越少"
# 3) 但截止頻率越低 → phase lag(group delay)越大,這是 trade-off。
assert lag_low_ms > lag_high_ms, "cutoff 越低,通帶延遲應越大(noise vs lag trade-off)"
print("Ch9 驗證通過 ✅")

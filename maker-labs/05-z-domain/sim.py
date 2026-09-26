#!/usr/bin/env python
"""Ch5 The z-Domain — Python simulation lab.

對應 maker-labs 講義 Chapter 5:離散化(c2d/ZOH)、z-domain 極點、
aliasing、quantization,以及「transfer function → difference equation」。

兩個實驗:
  1. 對同一 plant 用不同取樣週期 Ts 做 ZOH 離散化,印出離散係數,
     並把離散模型化成 MCU 可直接執行的 difference equation。
  2. Aliasing demo:以取樣率 fs 取樣一個高於 Nyquist 的正弦,
     觀察它被摺疊成的低頻假頻 f_alias = |f - k*fs|。

可從任意目錄執行:python maker-labs/05-z-domain/sim.py
章末以 assert 自我驗證(供 run_all.sh 判定綠/紅)。
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))  # → maker-labs/

import numpy as np
import matplotlib.pyplot as plt
import control as ct
from common.makerlab import show_or_save


def alias_freq(f, fs):
    """回傳連續頻率 f 以 fs 取樣後觀察到的假頻(0..fs/2)。"""
    r = f % fs
    return min(r, fs - r)


# --- Plant:一階系統 G(s) = 1/(0.2 s + 1) ---
tau = 0.2
G = ct.tf([1], [tau, 1])

print("=== Ch5 The z-Domain ===")
print(f"連續 plant  G(s) = 1/({tau}s + 1),連續極點 s = {-1/tau:.3f}")

# --- 實驗 1:不同 Ts 的 ZOH 離散化,印出係數 + difference equation ---
Ts_list = [0.005, 0.02, 0.1]
disc_models = {}
print("\n-- ZOH 離散化(c2d, method='zoh') --")
for Ts in Ts_list:
    Gd = ct.c2d(G, Ts, method="zoh")
    num = np.asarray(Gd.num[0][0], float)
    den = np.asarray(Gd.den[0][0], float)
    poles = Gd.poles()
    disc_models[Ts] = (Gd, num, den, poles)
    # 對一階 ZOH:Gd(z) = b1 / (z - a1),對應 y[k] = a1*y[k-1] + b1*u[k-1]
    b = num[-1]          # 分子常數項
    a = -den[-1]         # z^0 係數搬到右側(den = [1, -a1])
    print(f"Ts={Ts:<6}  num={np.round(num,5)}  den={np.round(den,5)}  "
          f"|pole|={abs(poles[0]):.5f}")
    print(f"          difference eq:  y[k] = {a:.5f}*y[k-1] + {b:.5f}*u[k-1]")
    # 預期離散極點 = exp(-Ts/tau)(一階 ZOH 的解析結果)
    print(f"          exp(-Ts/tau) = {np.exp(-Ts/tau):.5f}  (應等於離散極點)")

# --- 實驗 2:Aliasing ---
# 取樣率 fs=50 Hz → Nyquist = 25 Hz。取一個 10 Hz(合法)與 40 Hz(超過 Nyquist)。
fs = 50.0
f_ok = 10.0        # < Nyquist,不會 alias
f_hi = 40.0        # > Nyquist,會摺疊成 |40-50| = 10 Hz
f_alias_pred = alias_freq(f_hi, fs)
# 有號假頻:sin(2π·f·t) 在 t=n/fs 上等於 sin(2π·f_signed·t),
# 對 40 Hz@50 Hz 而言 f_signed = -10(即 10 Hz 但相位反轉)。
f_signed = f_hi - round(f_hi / fs) * fs
print("\n-- Aliasing demo --")
print(f"fs={fs} Hz  Nyquist={fs/2} Hz")
print(f"{f_ok} Hz 取樣後仍為 {alias_freq(f_ok, fs)} Hz(合法)")
print(f"{f_hi} Hz 取樣後摺疊成 {f_alias_pred} Hz 假頻(|f - fs| = {abs(f_hi-fs)})")

# 用高密度連續時間畫真實波形,再疊上以 fs 取樣的點。
t_cont = np.linspace(0, 1, 5000)
t_samp = np.arange(0, 1, 1 / fs)

# --- 圖:兩個子圖 ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7))

# 子圖 1:離散極點在 z 平面單位圓內
theta = np.linspace(0, 2 * np.pi, 400)
ax1.plot(np.cos(theta), np.sin(theta), "k--", lw=1, label="unit circle")
for Ts, (_, _, _, poles) in disc_models.items():
    ax1.plot(np.real(poles), np.imag(poles), "o", label=f"Ts={Ts}")
ax1.set_aspect("equal")
ax1.set_xlabel("Re(z)"); ax1.set_ylabel("Im(z)"); ax1.grid(True); ax1.legend()
ax1.set_title("Ch5 discrete poles vs Ts (inside unit circle → stable)")

# 子圖 2:aliasing —— 40 Hz 取樣後看起來像 10 Hz
ax2.plot(t_cont, np.sin(2 * np.pi * f_hi * t_cont), c="lightgray",
         label=f"true {f_hi:.0f} Hz")
ax2.plot(t_cont, np.sin(2 * np.pi * f_signed * t_cont), "b-", lw=1,
         label=f"apparent {f_alias_pred:.0f} Hz")
ax2.plot(t_samp, np.sin(2 * np.pi * f_hi * t_samp), "ro", ms=5,
         label=f"samples @ {fs:.0f} Hz")
ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Amplitude")
ax2.grid(True); ax2.legend(loc="upper right")
ax2.set_title(f"Ch5 aliasing: {f_hi:.0f} Hz sampled @ {fs:.0f} Hz → {f_alias_pred:.0f} Hz")

fig.tight_layout()
show_or_save(plt, "ch05_z_domain.png")

# --- ✅ 驗證 ---
# 1) 所有選定 Ts 的離散模型都應穩定(極點在單位圓內),且極點 = exp(-Ts/tau)。
for Ts, (_, _, _, poles) in disc_models.items():
    assert np.all(np.abs(poles) < 1.0), f"Ts={Ts} 離散模型應穩定(極點在單位圓內)"
    assert abs(abs(poles[0]) - np.exp(-Ts / tau)) < 1e-6, \
        f"Ts={Ts} 一階 ZOH 離散極點應為 exp(-Ts/tau)"
# 2) Ts 越大極點越靠近原點(取樣越慢,一步衰減越多)。
mags = [abs(disc_models[Ts][3][0]) for Ts in Ts_list]
assert mags[0] > mags[1] > mags[2], "Ts 越大,離散極點模應越小"
# 3) 離散 DC 增益應與連續 plant 一致(ZOH 保持 DC gain)。
for Ts, (Gd, _, _, _) in disc_models.items():
    assert abs(ct.dcgain(Gd) - ct.dcgain(G)) < 1e-6, f"Ts={Ts} ZOH 應保持 DC 增益"
# 4) Aliasing:40 Hz 以 50 Hz 取樣的假頻應精確等於 |f - fs| = 10 Hz,
#    且取樣點應與預測的 10 Hz 波形完全重合。
assert abs(f_alias_pred - abs(f_hi - fs)) < 1e-9, "假頻應等於 |f - fs|"
assert np.allclose(np.sin(2 * np.pi * f_hi * t_samp),
                   np.sin(2 * np.pi * f_signed * t_samp), atol=1e-9), \
    "取樣點應與(有號)假頻波形重合(混疊)"
assert alias_freq(f_ok, fs) == f_ok, "低於 Nyquist 的訊號不應 alias"
print("\nCh5 驗證通過 ✅")

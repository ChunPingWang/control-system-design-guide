# Ch9 控制系統中的濾波器 — C++ 版

> **理論與完整說明**:[`python/09-filters/README.md`](../../python/09-filters/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch09`
> **上一章**:[Ch8 前饋](../08-feedforward/README.md) · **下一章**:[Ch10 觀測器入門](../10-observers/README.md)

## 本章做什麼

以剛體 $P=1/(0.002s+0.01)$ 加 PI $C=0.5+5/s$(交越約 40 Hz)為基準迴路:
(1)在回授路徑串入二階 Butterworth 低通(200 Hz / 50 Hz),計算 PM 損失;
(2)比較 200 Hz notch($Q=5$)與 200 Hz 低通的 Bode 與 PM;
(3)$f_s=5$ kHz 逐樣本模擬,量測雜訊 σ=0.05 經 200 Hz 一階低通後,轉矩命令雜訊下降多少。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch09 && ./cpp/build/ch09
c++ -std=c++17 -O2 cpp/09-filters/main.cpp -o ch09 && ./ch09   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| `notch_tf` | $N(s)=(s^2+\omega_0^2)/(s^2+\frac{\omega_0}{Q}s+\omega_0^2)$ | `tf` |
| 實驗 1 | `C * P * butter2_analog(2π·fc)`,fc ∈ {200, 50};`margins(...).pm_deg`;寫 Bode CSV | `inertia`、`pi_ctrl`、`butter2_analog`、`margins`、`write_bode_csv`、`logspace` |
| 實驗 2 | `margins(C*P*N)`;`frf_of_system(N, linspace(190, 210, 201))` 取最小 dB 作為深度 | `frf_of_system`、`vmin` |
| 實驗 3 | `Rng(42).normal(0, 0.05, n)`;`run(use_filter)` 逐樣本跑 `MotorPlant` + `DiscretePID` + `OnePole(200, dt)`;0.5 s 後的 `stdev` | `Rng`、`MotorPlant`、`DiscretePID`、`OnePole`、`stdev`、`slice` |
| ✅ 驗證 | 6 條 `CHECK`,與 notebook 的 assertion 一一對應 | `Checker`、`CHECK` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `signal.butter(2, 2π·fc, 'low', analog=True)` → `ct.tf` | `butter2_analog(2 * PI * fc)` |
| `inertia(J=J, b=b)`、`pi(kp=0.5, ki=5.0)` | `inertia(J, b)`、`pi_ctrl(0.5, 5.0)` |
| `margins(L)['pm_deg']` | `margins(L).pm_deg` |
| `frf_of_system(N, f)['mag_db'].min()` | `vmin(frf_of_system(N, f).mag_db)` |
| notebook 內定義的 `class OnePole` | 函式庫的 `OnePole`(`sim.hpp`,同一後向 Euler 公式) |
| `np.random.default_rng(42).normal(0, 0.05, n)` | `Rng(42).normal(0, 0.05, n)`(mt19937_64) |
| `np.std(u[settle])` | `stdev(slice(u, int(0.5/dt)))` |
| `bode_compare` 圖、時域圖 | 三份 CSV(下表) |

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch09_lpf_loops.csv` | `f_hz`, `no_filter_mag_db`, `no_filter_phase_deg`, `LPF200_mag_db`, `LPF200_phase_deg`, `LPF50_mag_db`, `LPF50_phase_deg` | 實驗 1:三條迴路增益 Bode(1~1000 Hz) |
| `ch09_lpf_vs_notch.csv` | `f_hz`, `LPF200_mag_db`, `LPF200_phase_deg`, `notch200_mag_db`, `notch200_phase_deg` | 實驗 2:濾波器本身的 Bode |
| `ch09_noise.csv` | `t`, `u_raw`, `u_filtered`, `w_raw`, `w_filtered` | 實驗 3:轉矩命令與速度波形 |

```bash
python3 cpp/tools/plot_csv.py out/ch09_lpf_loops.csv
```

### 實際執行結果

```
PM 比較(迴路頻寬 ~40 Hz):
  no filter    PM =   88.9°
  LPF 200 Hz   PM =   72.5°
  LPF 50 Hz    PM =   24.9°
  Notch 200 Hz PM =   86.5°
notch 深度 @200 Hz ≈ -inf dB
轉矩雜訊 RMS:不濾 0.0252 → 濾波 0.0078(降 3.3 倍)

# ✅ 驗證
Ch9 驗證通過 ✅ (6/6)
```

PM(88.9° / 72.5° / 24.9° / 86.5°)與 notch 深度(−inf dB,掃描格點恰好含 200.0 Hz)與 Python 逐位相同。
**轉矩雜訊 RMS 有差異**:Python 為 0.0251 → 0.0082(3.1 倍),C++ 為 0.0252 → 0.0078(3.3 倍)。
原因是雜訊序列不同(C++ `mt19937_64` vs numpy PCG64,同為 seed 42 但序列無關),0.5 s × 5 kHz = 2500 點的
標準差估計本身就有數 % 的統計起伏。兩者都接近理論值:不濾 $K_p\sigma=0.025$、一階低通縮減 $\sqrt{\alpha/(2-\alpha)}\approx0.33$。

## ✅ 驗證條件(6 條)

| # | 條件 | 意義 |
|---|---|---|
| 1 | `pm[no filter] > pm[LPF 200] > pm[LPF 50]` | 濾波越低,相位代價越大 |
| 2 | `pm[LPF 50 Hz] < 30` | 濾在 1.2 倍頻寬 → 邊限崩潰 |
| 3 | `pm[LPF 200 Hz] > 60` | 濾在 5 倍頻寬 → 損失可控 |
| 4 | `pm_notch > pm[LPF 200 Hz]` | notch 在交越處吃相位較少 |
| 5 | `depth < -15` | notch 凹口深度足夠 |
| 6 | `rms_flt < rms_raw / 2` | 低通確實壓住轉矩雜訊 |

## 動手改改看

- 把 `for (int fc : {200, 50})` 改成掃描 50~1000 Hz,印出 PM 損失 vs `fc/40`,驗證 5~10 倍法則。
- `notch_tf(200, 5)` 的 `Q` 改成 1 與 20,比較 `ch09_lpf_vs_notch.csv` 的凹口寬度與 `pm_notch`。
- 把 `OnePole lpf(200, dt)` 改成 100 Hz 或 400 Hz,看雜訊 RMS 如何隨 $\sqrt{\alpha/(2-\alpha)}$ 變化。

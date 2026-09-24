# Ch2 頻域分析 — C++ 版

> **理論與完整說明**:[`python/02-frequency-domain/README.md`](../../python/02-frequency-domain/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch02`
> **上一章**:[Ch1 回授控制導論](../01-introduction/README.md) · **下一章**:[Ch3 調機](../03-tuning/README.md)

## 本章做什麼

PI 速度迴路($P=1/(0.002s+0.01)$、$C=0.5+5/s$):
(1)解析求開迴路 GM / PM 與閉迴路 −3 dB 頻寬;(2)以 5 kHz 離散迴路 + 對數 chirp(1→300 Hz)做 DSA 量測,
用 Welch 交叉頻譜估計閉迴路 FRF,與解析 $T(s)$ 交叉比對。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch02 && ./cpp/build/ch02
c++ -std=c++17 -O2 cpp/02-frequency-domain/main.cpp -o ch02 && ./ch02   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| 參數 | `J=0.002`、`b=0.01`、`P = inertia(J,b)`、`C = pi_ctrl(0.5, 5)`、`L = C*P`、`T = closed_loop(C,P)` | `inertia`、`pi_ctrl`、`closed_loop` |
| 解析 Bode | `margins(L)` 取 GM/PM/穿越頻率,`bandwidth_hz(T)`;0.1~1000 Hz 600 點寫 CSV | `margins`、`bandwidth_hz`、`write_bode_csv`、`logspace` |
| DSA 激發 | `chirp_excitation(1, 300, 20, 5000, 1.0)`;逐樣本跑 `MotorPlant` + `DiscretePID` | `chirp_excitation`、`MotorPlant`、`DiscretePID` |
| FRF 估計 | `measure_frf(r, y, fs)`,`select()` 只留 coherence > 0.95 的頻點;找第一個 < −3 dB 的頻點 | `measure_frf`、`FRF::select`、`first_index`、`frf_of_system` |
| ✅ 驗證 | 5 條 `CHECK`,與 notebook 的 assertion 一一對應 | `Checker`、`CHECK` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `inertia(J=J, b=b)`、`pi(kp=0.5, ki=5.0)` | `inertia(J, b)`、`pi_ctrl(0.5, 5.0)` |
| `margins(L)` 回傳 dict | `margins(L)` 回傳 `Margins{gm_db, pm_deg, f_gm_hz, f_pm_hz}` |
| `chirp_excitation(...)` 回傳 `(t, u)` | `chirp_excitation(...)` 回傳 `Excitation{t, u}` |
| `measure_frf(r, y, fs)`(scipy `welch`/`csd`/`coherence`) | `measure_frf(ex.u, y, fs)`(自帶 FFT,重現 scipy 預設:週期 Hann 窗、50% 重疊、去平均) |
| `{k: v[good] for k, v in meas.items()}` | `meas.select([&](size_t i){ return meas.coherence[i] > 0.95; })` |
| `np.nonzero(mag_m < -3)[0][0]` | `first_index(n, pred)` |
| `bode_compare(...)` 兩張圖 | 兩份 CSV(下表) |

chirp 是確定性訊號、迴路中沒有隨機雜訊,且 `measure_frf` 逐步重現 scipy 的 Welch 演算法,
因此量測頻寬也與 notebook 完全相同。

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch02_bode_open_closed.csv` | `f_hz`, `L_mag_db`, `L_phase_deg`, `T_mag_db`, `T_phase_deg` | 開 / 閉迴路解析 Bode |
| `ch02_dsa_vs_analytic.csv` | `f_hz`, `meas_mag_db`, `meas_phase_deg`, `coherence`, `ana_mag_db`, `ana_phase_deg` | DSA 量測(coherence > 0.95 的頻點)vs 解析 |

```bash
python3 cpp/tools/plot_csv.py out/ch02_dsa_vs_analytic.csv
```

### 實際執行結果

```
開迴路邊限: GM=inf dB, PM=88.86° @ 39.81 Hz
閉迴路 -3dB 頻寬: 40.5 Hz
解析頻寬 40.5 Hz | DSA 量測頻寬 42.7 Hz

# ✅ 驗證
Ch2 驗證通過 ✅ (5/5)
```

與 Python notebook 的數值完全相同(notebook 以 dict 形式印出 `pm_deg: 88.8558…`、`f_pm_hz: 39.8126…`,
C++ 取兩位小數;頻寬 40.5 / 42.7 Hz 逐位一致)。

## ✅ 驗證條件(5 條)

| # | 條件 | 意義 |
|---|---|---|
| 1 | `85 < PM < 92` | PI 速度迴路 PM ≈ 89° |
| 2 | `isinf(GM)` | 相位永遠不到 −180°,GM 無限大 |
| 3 | `35 < bw < 46` | 頻寬 ≈ $k_p/(2\pi J)$ ≈ 40 Hz |
| 4 | `|bw_meas − bw| / bw < 0.3` | DSA 量測與解析誤差 < 30% |
| 5 | 5 Hz 以下量測 `|mag_db| < 1` | 低頻 \|T\| ≈ 0 dB |

## 動手改改看

- 把 `pi_ctrl(0.5, 5.0)` 與 `DiscretePID(0.5, 5.0, ...)` 的 kp 一起改成 1.0,看 PM 與兩個頻寬如何變化。
- 把 `chirp_excitation` 的振幅改成 0.01,並在 `y[k]` 上加一點雜訊(例如 `Rng(0).normal(0, 1e-3, n)` 產生的高斯雜訊),
  觀察 `ch02_dsa_vs_analytic.csv` 中 coherence 在哪些頻段先掉下 0.95。
- 用 `measure_frf(ex.u, y, fs, 1024)` 改變 `nperseg`,看頻率解析度與量測頻寬的關係。

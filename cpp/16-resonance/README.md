# Ch16 機構柔性與共振 — C++ 版

> **理論與完整說明**:[`python/16-resonance/README.md`](../../python/16-resonance/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch16`
> **上一章**:[Ch15 伺服馬達與驅動器](../15-servo-motor/README.md) · **下一章**:[Ch17 位置控制迴路](../17-position-control/README.md)

## 本章做什麼

two-mass 機台($J_m=2\times10^{-4}$、$J_l=1.8\times10^{-3}$、$k_s=100$、$c_s=0.005$)的 2 kHz 速度迴路(PI + 一拍延遲):
(1)解析 Bode 圖與 400 Hz 增益抬升;(2)五種組態的 $k_{p,max}$ 掃描(剛體 / 無處置 / 60 Hz LPF / notch / 慣量比 1:1);
(3)失穩時用 FFT 找振盪頻率,對照第二交越 $k_p/(2\pi J_m)$。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch16 && ./cpp/build/ch16
c++ -std=c++17 -O2 cpp/16-resonance/main.cpp -o ch16 && ./ch16   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| `run_loop(kp, par, filt)` | `std::optional<Par>` 空 = 剛體 `MotorPlant(2e-3)`,否則 `TwoMassPlant`;回授可選 notch / LPF;發散即提早回傳;穩定 = 尾段 20% 峰對峰 < 0.05 | `MotorPlant`、`TwoMassPlant`、`DiscretePID`、`Delay`、`Biquad`、`iirnotch`、`OnePole` |
| `kp_max(par, filt)` | 格點 `arange(0.05,1.01,0.05)` + `arange(1.1,4.01,0.1)`,`round_to(kp,2)`,第一次失敗即停 | `arange`、`round_to` |
| Bode / 抬升 | `P_2mass`、`P_rigid` 的 `tf`;`mag_db_at(P, 400)` 相減 | `tf`、`write_bode_csv`、`mag_db_at`、`logspace` |
| 掃描表 | 五種組態,存成 `std::vector<pair<string,double>>` 依序印出 | — |
| 失穩頻率 | `kp = nocure + 0.1`,最後 2048 點去均值 × Hanning → `rfft` → `argmax` | `hanning`、`rfft`、`rfftfreq`、`argmax` |
| ✅ 驗證 | 7 條 `CHECK`,與 notebook 的 assertion 一一對應 | `Checker`、`CHECK` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `TwoMassPlant(**PAR)` / `.resonance_hz` | `TwoMassPlant(Jm, Jl, ks, cs)` / `.resonance_hz()` |
| notebook 內定義的 `Biquad`、`OnePole` 類別 | 函式庫 `Biquad`、`OnePole`(`sim.hpp`) |
| `signal.iirnotch(f0/(fs/2), 2)` | `iirnotch(f0/(fs/2), 2)`(同公式的係數) |
| `run_loop(kp, rigid=True)` | `run_loop(kp, std::nullopt)` |
| `filt='notch'` / `'lpf'` 字串 | `enum class Filt { None, Notch, Lpf }` |
| `frf_of_system(P, [400.0])['mag_db'][0]` | `mag_db_at(P, 400.0)` |
| `np.fft.rfft(seg * np.hanning(len(seg)))` | `rfft(seg)`(先逐點乘 `hanning`) |
| `bode_compare(...)` 與時域圖 | 兩份 CSV(下表) |

模擬全為確定性,$k_p$ 格點以 `round_to` 對齊 notebook 的 `round(kp, 2)`,因此掃描結果與 notebook 相同。

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch16_two_mass_bode.csv` | `f_hz`, `two_mass_mag_db`, `two_mass_phase_deg`, `rigid_mag_db`, `rigid_phase_deg` | 實驗 1:two-mass vs 剛體 Bode(1~1000 Hz,600 點) |
| `ch16_unstable_vs_stable.csv` | `t_unstable`, `y_unstable`, `t_stable`, `y_stable` | 實驗 3:$k_p=0.40$ 失穩 vs $k_p=0.30$ 最大穩定(失穩那欄在發散時提早截斷,長度較短) |

```bash
python3 cpp/tools/plot_csv.py out/ch16_two_mass_bode.csv
```

### 實際執行結果

```
反共振 37.5 Hz | 共振 118.6 Hz | 慣量比 Jl/Jm = 9
400 Hz 處增益抬升:20.7 dB(理論 20·log10(1+Jl/Jm) = 20.0 dB)
configuration                kp_max
rigid body (dream)             3.90
two-mass, no cure              0.30
two-mass + LPF 60 Hz           0.10
two-mass + notch @res          0.35
two-mass, Jl/Jm=1 (mech!)      1.90
振盪頻率 348 Hz;第二交越預測 kp/(2π·Jm) = 318 Hz;共振 119 Hz

# ✅ 驗證
Ch16 驗證通過 ✅ (7/7)
```

與 Python notebook 的輸出逐位相同。

## ✅ 驗證條件(7 條)

| # | 條件 | 意義 |
|---|---|---|
| 1 | `|lift_db − 20·log10(1+Jl/Jm)| < 2` | 增益抬升 = 慣量比 |
| 2 | `rigid > 3` | 剛體基準夠高 |
| 3 | `nocure ≤ rigid / 5` | 共振砍增益 ≥ 5 倍 |
| 4 | `notch ≥ nocure` | notch 不會變差 |
| 5 | `lpf ≤ nocure` | 60 Hz LPF 在此機台失敗 |
| 6 | `bal ≥ 4 · nocure` | 機構改善(慣量比 1:1)最有效 |
| 7 | `|f_osc − f_pred| / f_pred < 0.35` | 振盪頻率 ≈ 第二交越 |

## 動手改改看

- 把 `PAR.ks` 改成 `1000.0`(剛性聯軸器),共振升到約 $\sqrt{10}$ 倍(375 Hz),看 `kp_max` 回升多少、何時撞上延遲極限。
- 把 `lpf.emplace(60, dt)` 改成 `lpf.emplace(600, dt)`,低通截止拉高後是否還比「不濾」差?
- 在 `run_loop` 把 `get()` 改回傳 `tm->wl`(負載端回授,non-collocated),重跑掃描表。

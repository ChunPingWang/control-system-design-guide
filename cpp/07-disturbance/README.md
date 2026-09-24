# Ch7 擾動響應 — C++ 版

> **理論與完整說明**:[`python/07-disturbance/README.md`](../../python/07-disturbance/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch07`
> **上一章**:[Ch6 四種控制器](../06-controllers/README.md) · **下一章**:[Ch8 前饋](../08-feedforward/README.md)

## 本章做什麼

PI 速度迴路($k_p = 0.5$)的擾動響應 $G_d = P/(1+CP)$:
(1)$k_i$ = 1 / 5 / 20 的擾動 Bode 與開迴路 $P$ 比較;(2)t = 0.2 s 施加 0.1 Nm 步階負載的連續時域跌落;
(3)1 kHz 離散迴路上,以「延遲 1 ms、增益誤差 10%」的負載量測做擾動解耦前饋,與純回授比較;
(4)驗證低頻 $|G_d| \approx \omega/k_i$ 漸近線。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch07 && ./cpp/build/ch07
c++ -std=c++17 -O2 cpp/07-disturbance/main.cpp -o ch07 && ./ch07   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| 頻域 | 對 ki ∈ {1, 5, 20}:`feedback(P, pi_ctrl(0.5, ki))` 即 $G_d$;加上 `P` 本身,0.1~1000 Hz 寫 Bode CSV | `feedback`、`pi_ctrl`、`write_bode_csv` |
| 時域 | `linspace(0, 2, 4000)`,t ≥ 0.2 s 時輸入 −0.1;`forced_response(Gd, t, d)`,跌落 = `-vmin(y)` | `forced_response`、`vmin` |
| 解耦 | lambda `run(decouple, meas_delay_ms=1, meas_gain=0.9)`:`Delay` 延遲量測、乘 0.9 後加到 `u` | `MotorPlant`、`DiscretePID`、`Delay`、`slice` |
| 低頻漸近線 | `mag_db_at(Gd, 0.1)` 對 ki = 1、20;漸近線 `20·log10(2π·0.1/20)` | `mag_db_at` |
| ✅ 驗證 | 4 條 `CHECK`,與 notebook 的 assertion 一一對應 | `Checker`、`CHECK` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `ct.feedback(P, C)` | `feedback(P, pi_ctrl(0.5, ki))` |
| `_, y = ct.forced_response(Gd, t, d)` | `Vec y = forced_response(Gd, t, d)` |
| `def run(decouple, meas_delay_ms=1, meas_gain=0.9)` | 同簽名的 lambda `run` |
| `Delay(int(meas_delay_ms * 1e-3 / dt))` | `Delay dly(int(meas_delay_ms * 1e-3 / dt))` |
| `frf_of_system(sys, [0.1])['mag_db'][0]` | `mag_db_at(sys, 0.1)` |
| `dips = {ki: ...}` dict | `std::map<int,double> dips` |
| 三張圖(Bode、時域、解耦) | 三份 CSV(下表) |

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch07_disturbance_bode.csv` | `f_hz`, `ki1_mag_db`, `ki1_phase_deg`, `ki5_mag_db`, `ki5_phase_deg`, `ki20_mag_db`, `ki20_phase_deg`, `open_loop_P_mag_db`, `open_loop_P_phase_deg` | 擾動響應 $G_d$ Bode(含開迴路 P 參考) |
| `ch07_disturbance_time.csv` | `t`, `ki1`, `ki5`, `ki20` | 0.1 Nm 步階負載的速度偏差 |
| `ch07_decoupling.csv` | `t`, `feedback_only`, `with_decoupling` | 純回授 vs 加解耦的速度響應 |

```bash
python3 cpp/tools/plot_csv.py out/ch07_disturbance_bode.csv
```

### 實際執行結果

```
max dip by ki: 1→0.1902  5→0.1779  20→0.1552
速度跌落:純回授 0.1786 | 加解耦 0.0472(改善 3.8x)
|Gd(0.1 Hz)|:ki=1 -4.5 dB | ki=20 -30.1 dB(漸近線 -30.1 dB)

# ✅ 驗證
Ch7 驗證通過 ✅ (4/4)
```

跌落與解耦結果與 Python notebook 逐位相同;$|G_d(0.1\,\text{Hz})|$ 一行為 C++ 版額外印出(notebook 只在 assertion 中使用)。
兩者相差 25.6 dB,接近理論的 $20\log_{10}20 = 26$ dB。

## ✅ 驗證條件(4 條)

| # | 條件 | 意義 |
|---|---|---|
| 1 | `dips[1] > dips[5] > dips[20]` | ki 越大跌落越小 |
| 2 | `|(g1 − g20) − 20·log10(20)| < 2` | 低頻 \|Gd\| 與 ki 成反比(差約 26 dB) |
| 3 | `|g20 − asym| < 1.5` | 低頻 \|Gd\| 貼合 $s/k_i$ 漸近線 |
| 4 | `dip_dc < dip_fb / 2` | 解耦至少改善 2 倍 |

## 動手改改看

- 把 `run(true)` 改成 `run(true, 50)`(量測延遲 50 ms),看 `with_decoupling` 還有沒有改善(練習 2)。
- 把 `meas_gain` 改成 1.0 與 0.5,比較跌落:延遲與增益誤差哪個影響大?
- 把時域的步階負載改成 5 / 40 / 200 Hz 的正弦,量輸出振幅,對照 `ch07_disturbance_bode.csv` 的預測(練習 3)。

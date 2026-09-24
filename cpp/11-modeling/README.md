# Ch11 建模入門 — C++ 版

> **理論與完整說明**:[`python/11-modeling/README.md`](../../python/11-modeling/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch11`
> **上一章**:[Ch10 觀測器入門](../10-observers/README.md) · **下一章**:[Ch12 非線性行為](../12-nonlinear/README.md)

## 本章做什麼

從物理方程建立 DC 馬達($R=1\,\Omega$、$L=1$ mH、$K_t=K_e=0.1$、$J=10^{-4}$、$b=10^{-5}$)的電壓 → 轉速模型:
(1)完整二階模型 vs 忽略電感的一階降階模型,計算 $\tau_e$、$\tau_m$、極點與 Bode 差異;
(2)用 `DCMotorPlant` 1 µs Euler 逐樣本積分與解析步階響應交叉驗證(1 V 步階、50 ms)。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch11 && ./cpp/build/ch11
c++ -std=c++17 -O2 cpp/11-modeling/main.cpp -o ch11 && ./ch11   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| 模型 | `P_full = tf({Kt}, (Ls+R)(Js+b) + KtKe)`;`P_red = tf({Kt}, R(Js+b) + KtKe)`,以多項式運算組分母 | `tf`、`polymul`、`polyadd`、`polyscale` |
| 時間常數 | $\tau_e = L/R$、$\tau_m = RJ/(Rb+K_tK_e)$;`poles(P_full)`;寫 Bode CSV(1 Hz~10 kHz) | `poles`、`write_bode_csv`、`logspace` |
| 逐樣本 | `DCMotorPlant(R, L, Kt, Ke, J, b, 1e-6)`,50000 步 1 V 步階 | `DCMotorPlant` |
| 解析 | 每 50 點取一個時間點做 `step_response(P_full, t_tf)`,`interp` 回 1 µs 時間軸算最大相對誤差 | `step_response`、`interp`、`dcgain` |
| 頻域對照 | `mag_db_at(P, 1 Hz)`、`mag_db_at(P, 1 kHz)` 的完整 − 降階差 | `mag_db_at` |
| ✅ 驗證 | 5 條 `CHECK`,與 notebook 的 assertion 一一對應 | `Checker`、`CHECK` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `s = ct.tf('s')`;`Kt / ((L*s + R)*(J*s + b) + Kt*Ke)` | `tf({Kt}, polyadd(polymul({L, R}, {J, b}), {Kt*Ke}))` |
| `ct.poles(P_full)` | `poles(P_full)`(回傳 `cplx` 向量) |
| `ct.dcgain(P)` | `dcgain(P)` |
| `DCMotorPlant(R=..., dt=1e-6)`;`plant.step(1.0)` | `DCMotorPlant(R, L, Kt, Ke, J, b, 1e-6)`;`plant.step(1.0)` |
| `ct.step_response(P_full, t_sim[::50])` | `step_response(P_full, t_tf)` |
| `np.interp(t_sim, t_tf, w_tf)` | `interp(t_sim, t_tf, w_tf)` |
| `frf_of_system(P, [1000.0])['mag_db'][0]` | `mag_db_at(P, 1000.0)` |
| `bode_compare` 圖、步階比較圖 | 兩份 CSV(下表) |

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch11_bode_full_vs_reduced.csv` | `f_hz`, `full_mag_db`, `full_phase_deg`, `reduced_mag_db`, `reduced_phase_deg` | 完整 vs 降階 Bode(1 Hz~10 kHz,600 點) |
| `ch11_step_crosscheck.csv` | `t_analytic`, `w_analytic`, `t_sim_decimated`, `w_sim_decimated` | 解析步階 vs 逐樣本模擬(抽點到 50 µs 間隔) |

```bash
python3 cpp/tools/plot_csv.py out/ch11_bode_full_vs_reduced.csv
```

### 實際執行結果

```
電氣時間常數 τe = 1.00 ms(極點 1000 rad/s ≈ 159 Hz)
機械時間常數 τm = 9.99 ms(極點 100 rad/s ≈ 15.9 Hz)
完整模型極點: -887.28+0.00j -112.82+0.00j
兩實作最大相對誤差:0.008%
穩態轉速:模擬 9.949 | 解析 DC 增益 9.990 rad/s
完整 vs 降階增益差:1 Hz 0.003 dB | 1 kHz -16.05 dB

# ✅ 驗證
Ch11 驗證通過 ✅ (5/5)
```

時間常數、極點(−887.28 / −112.82)、交叉驗證誤差 0.008%、穩態轉速 9.949 / 9.990 皆與 Python notebook 的輸出一致
(極點的顯示位數不同:Python 印 4 位小數)。最後一行「1 Hz / 1 kHz 增益差」是 C++ 版額外印出的,notebook 只在 assertion 內計算。

## ✅ 驗證條件(5 條)

| # | 條件 | 意義 |
|---|---|---|
| 1 | `max_err < 0.01` | 解析與逐樣本兩條獨立實作一致(< 1%) |
| 2 | `|dc_full − dc_red| / dc_full < 1e-6` | 降階不改變 DC 增益 |
| 3 | `|d1| < 0.1` dB | 1 Hz:兩模型重合 |
| 4 | `|d1k| > 3` dB | 1 kHz:降階模型失效 |
| 5 | `|w_sim(50 ms) − dc_full| / dc_full < 0.01` | 模擬穩態符合 DC 增益 |

## 動手改改看

- 把 `L_ind` 改成 `1e-2`(10 mH),看完整模型極點變成複數還是仍為實數,以及 `d1k` 如何變化。
- 把 `dt_sim` 放大到 `1e-4`,觀察 Euler 積分在 $dt$ 接近 $\tau_e$ 的量級時 `max_err` 如何惡化。
- 把 `Ke` 設為 0(拿掉反電動勢),$\tau_m$ 變成 $J/b = 10$ s —— 體會反電動勢提供的阻尼。

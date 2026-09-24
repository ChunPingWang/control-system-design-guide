# Ch5 z 域 — C++ 版

> **理論與完整說明**:[`python/05-z-domain/README.md`](../../python/05-z-domain/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch05`
> **上一章**:[Ch4 數位控制器與延遲](../04-sampling-delay/README.md) · **下一章**:[Ch6 四種控制器](../06-controllers/README.md)

## 本章做什麼

(1)把 20 Hz、ζ = 0.3 的二階受控體以 $f_s$ = 200 Hz 做 ZOH / Tustin / matched 三種離散化,比較 Bode、DC 增益與 40 Hz 相位;
(2)把 $s = -50 + j200$ 以 $z = e^{sT}$ 映射到 2000 / 500 / 100 Hz 的 z 平面;
(3)60 Hz 正弦以 70 Hz 取樣,驗證它混疊成 10 Hz。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch05 && ./cpp/build/ch05
c++ -std=c++17 -O2 cpp/05-z-domain/main.cpp -o ch05 && ./ch05   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| 離散化 | `second_order(2π·20, 0.3)`;`c2d(P, dt, "zoh" / "tustin" / "matched")`;1 Hz ~ 0.49 fs 寫 Bode CSV;`dcgain` 比對 | `second_order`、`c2d`、`dcgain`、`write_bode_csv` |
| 極點映射 | `std::exp(s_pole / fs)`,印出實部、虛部、`|z|` | `std::complex`(`cplx`) |
| 混疊 | `arange(0, 0.5, 1/70)` 取樣 60 Hz 與 10 Hz 正弦,求相關係數 | `arange`、`corrcoef`、`write_csv` |
| 40 Hz 相位 | `frf_of_system(sys, {40.0}).phase_deg[0]`:連續、Tustin、ZOH | `frf_of_system` |
| ✅ 驗證 | 6 條 `CHECK`(DC 增益 3 條 + 極點 + 混疊 + 相位) | `Checker`、`CHECK` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `ct.sample_system(P, dt, method='zoh')` | `c2d(P, dt, "zoh")`(擴充矩陣 `expm` 精確離散化) |
| `ct.sample_system(P, dt, method='tustin')` | `c2d(P, dt, "tustin")`(代入 $s = \frac{2}{T}\frac{z-1}{z+1}$ 展開多項式) |
| `ct.sample_system(P, dt, method='matched')` | `c2d(P, dt, "matched")`(零極點 $e^{sT}$ 映射 + DC 增益匹配) |
| `ct.dcgain(sys)` | `dcgain(sys)`(離散系統取 $z = 1$) |
| `np.exp(s_pole / fs)` | `std::exp(s_pole / double(fs))` |
| `np.corrcoef(a, b)[0, 1]` | `corrcoef(a, b)` |
| z 平面圖(圖例標 `|z|`) | 以 `printf` 印出極點座標 |

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch05_discretization.csv` | `f_hz`, `continuous_mag_db`, `continuous_phase_deg`, `zoh_mag_db`, `zoh_phase_deg`, `tustin_mag_db`, `tustin_phase_deg`, `matched_mag_db`, `matched_phase_deg` | 四種模型 Bode 比較 |
| `ch05_aliasing.csv` | `t`, `x_sampled`, `alias_10hz` | 70 Hz 取樣點 vs 10 Hz 混疊弦波 |

```bash
python3 cpp/tools/plot_csv.py out/ch05_discretization.csv
```

### 實際執行結果

```
DC gains: [1.000000 1.000000 1.000000 1.000000]
fs=2000 Hz: z = 0.9704 +0.0974j, |z| = 0.975
fs= 500 Hz: z = 0.8334 +0.3524j, |z| = 0.905
fs= 100 Hz: z = -0.2524 +0.5515j, |z| = 0.607
取樣點 vs 10 Hz 弦波相關係數: -1.0000
40 Hz 相位:連續 -158.2° | Tustin -162.3° | ZOH 166.1°

# ✅ 驗證
Ch5 驗證通過 ✅ (6/6)
```

DC 增益與相關係數與 Python notebook 逐位相同(notebook 印 `[1. 1. 1. 1.]`)。極點座標與 40 Hz 相位為 C++ 版額外印出;
以 python-control 計算 notebook 的同一組模型,40 Hz 相位同樣是 −158.2° / −162.3° / 166.1°。
ZOH 的 166.1° 即 −193.9°,比連續多落後約半拍(36°)。

## ✅ 驗證條件(6 條)

| # | 條件 | 意義 |
|---|---|---|
| 1–3 | `|dc[i] − dc[0]| < 1e-6`,i = ZOH、Tustin、matched | 離散化不得改變 DC 增益 |
| 4 | `0.55 < |e^{sT}| < 1`(fs = 100 Hz) | 穩定但靠近單位圓 |
| 5 | `|corr| > 0.99` | 60 Hz 以 70 Hz 取樣混疊成 10 Hz |
| 6 | `|ph_t − ph_c| < |ph_z − ph_c|`(40 Hz) | Tustin 相位比 ZOH 準(ZOH 額外半拍延遲) |

## 動手改改看

- 把 `fs = 200.0` 改成 2000,重畫 `ch05_discretization.csv`:三種方法在 0.49 fs 以下幾乎重合(練習 1)。
- 把 `f_sig` 改成 130,預測混疊頻率後,把 `alias_f` 改成你的答案,看相關係數是否仍為 ±1(練習 3)。
- 在 40 Hz 相位那段加入 `Pz_mat`,看 matched 的相位落後多少,並對照它的分子階數。

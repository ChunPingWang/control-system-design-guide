# Ch4 數位控制器與延遲 — C++ 版

> **理論與完整說明**:[`python/04-sampling-delay/README.md`](../../python/04-sampling-delay/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch04`
> **上一章**:[Ch3 調機](../03-tuning/README.md) · **下一章**:[Ch5 z 域](../05-z-domain/README.md)

## 本章做什麼

同一組 PI 增益($k_p=0.5$、$k_i=5$)在不同取樣率下的相位邊限與振鈴,用三種方法交叉驗證:
(1)直線公式 $\Delta\phi = 360° \cdot f_c \cdot 1.5/f_s$;(2)逐樣本模擬 500 / 1000 / 5000 Hz 的階躍響應;
(3)二階 Padé 延遲模型串入開迴路,以 `margins()` 求 PM。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch04 && ./cpp/build/ch04
c++ -std=c++17 -O2 cpp/04-sampling-delay/main.cpp -o ch04 && ./ch04   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| 連續邊限 | `margins(C*P)` 取 PM 與 `fc = f_pm_hz` | `inertia`、`pi_ctrl`、`margins` |
| 相位損失表 | 對 fs ∈ {500, 1000, 2000, 5000, 20000}:`loss = 360·fc·1.5/fs`,存入 `theory[fs]` | `std::map` |
| 時域模擬 | fs ∈ {500, 1000, 5000}:`MotorPlant` + `DiscretePID` + `Delay(1)` 跑 0.25 s;另算連續 `step_response` | `MotorPlant`、`DiscretePID`、`Delay`、`step_metrics`、`step_response` |
| Padé | `C * P * pade2(1.5/fs)`,fs ∈ {500, 2000},印出 PM 與直線公式 | `pade2`、`margins`、`write_bode_csv` |
| ✅ 驗證 | 4 條 `CHECK`,與 notebook 的 assertion 一一對應 | `Checker`、`CHECK` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `rows = [(fs, loss, pm), ...]` | `std::map<int,double> theory` |
| `num, den = ct.pade(1.5/fs, 2); ct.tf(num, den)` | `pade2(1.5 / fs)`(直接回傳 `TF`) |
| `ct.step_response(T_ideal, np.linspace(0, 0.25, 1000))` | `step_response(closed_loop(C, P), linspace(0, 0.25, 1000))` |
| `os_by_fs = {}` dict | `std::map<int,double> os_by_fs` |
| Padé PM 只標在圖例 | 以 `printf` 明確印出 Padé PM 與直線公式 |
| `bode_compare` / 時域圖 | 五份 CSV(下表) |

`pade2(T)` 即 $\dfrac{T^2s^2/12 - Ts/2 + 1}{T^2s^2/12 + Ts/2 + 1}$,與 `ct.pade(T, 2)` 相同。

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch04_step_fs500.csv` | `t`, `y` | 500 Hz 階躍響應 |
| `ch04_step_fs1000.csv` | `t`, `y` | 1000 Hz 階躍響應 |
| `ch04_step_fs5000.csv` | `t`, `y` | 5000 Hz 階躍響應 |
| `ch04_step_continuous.csv` | `t`, `y` | 連續(理想)閉迴路階躍響應 |
| `ch04_bode_delay.csv` | `f_hz`, `no_delay_mag_db`, `no_delay_phase_deg`, `fs500_mag_db`, `fs500_phase_deg`, `fs2000_mag_db`, `fs2000_phase_deg` | 延遲吃相位不吃增益 |

```bash
python3 cpp/tools/plot_csv.py out/ch04_step_fs500.csv
python3 cpp/tools/plot_csv.py out/ch04_bode_delay.csv
```

### 實際執行結果

```
連續系統:PM=88.9° @ fc=39.8 Hz
 fs [Hz] 相位損失[°] 數位 PM[°]
     500         43.0         45.9
    1000         21.5         67.4
    2000         10.7         78.1
    5000          4.3         84.6
   20000          1.1         87.8
overshoot by fs: 500→27.5%  1000→1.8%  5000→1.6%
Padé fs=500 Hz:PM=45.9°(直線公式 45.9°)
Padé fs=2000 Hz:PM=78.1°(直線公式 78.1°)

# ✅ 驗證
Ch4 驗證通過 ✅ (4/4)
```

相位損失表與 overshoot 與 Python notebook 逐位相同;Padé 兩行為 C++ 版額外印出(notebook 只在圖例顯示取整後的 PM)。

## ✅ 驗證條件(4 條)

| # | 條件 | 意義 |
|---|---|---|
| 1 | `OS(500) > OS(1000) > OS(5000)` | 取樣率越低振鈴越大 |
| 2 | `|PM_Padé(500) − theory[500]| < 6` | Padé vs 直線公式一致 |
| 3 | `|PM_Padé(2000) − theory[2000]| < 6` | 同上,較高取樣率 |
| 4 | `OS(5000) < 8` | 高取樣率接近連續 |

## 動手改改看

- 把時域模擬的 `Delay delay(1)` 改成 `Delay delay(2)`,同時把公式的 1.5 改成 2.5,500 Hz 還穩定嗎?(練習 1)
- 在時域迴圈的 fs 清單加入 250,觀察 overshoot 與 CSV 波形。
- 把 `pi_ctrl` 與 `DiscretePID` 的 kp 同時減半,看 500 Hz 的 overshoot 與穿越頻率各變多少(練習 3)。

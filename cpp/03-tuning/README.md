# Ch3 調機 — C++ 版

> **理論與完整說明**:[`python/03-tuning/README.md`](../../python/03-tuning/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch03`
> **上一章**:[Ch2 頻域分析](../02-frequency-domain/README.md) · **下一章**:[Ch4 數位控制器與延遲](../04-sampling-delay/README.md)

## 本章做什麼

在 1 kHz 離散速度迴路(含一拍計算延遲)上實作 Ellis 的 zone-based tuning:
(1)`ki=0` 掃 kp = 0.1~2.0,選 overshoot 首次 ≥ 4% 者;(2)固定 kp 掃 ki = 0~80,選 overshoot 首次 ≥ 15% 者;
(3)算類比 PM 並扣掉 1.5 拍延遲的相位損失,檢查數位 PM。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch03 && ./cpp/build/ch03
c++ -std=c++17 -O2 cpp/03-tuning/main.cpp -o ch03 && ./ch03   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| 參數 | 全域 `J=0.002`、`b=0.01`、`fs=1000`、`dt=1/fs` | — |
| `run_step(kp, ki)` | `MotorPlant` + `DiscretePID` + `Delay(1)`,單位步階跑 0.4 s,回傳 `Run{t, y}` | `MotorPlant`、`DiscretePID`、`Delay`、`sample_times` |
| 步驟 1 | `arange(0.1, 2.01, 0.1)` 經 `round_to(v, 2)` 得 kp 清單;`first_index` 找 OS ≥ 4% | `arange`、`round_to`、`step_metrics`、`first_index` |
| 步驟 2 | `arange(0, 81, 5)` 掃 ki;`first_index` 找 OS ≥ 15%;寫掃描 CSV 與六條步階響應 CSV | `write_csv` |
| 頻域檢查 | `margins(pi_ctrl(kp_sel, ki_sel) * inertia(J,b))`,相位損失 `360·f_pm·1.5·dt` | `margins`、`pi_ctrl`、`inertia` |
| ✅ 驗證 | 5 條 `CHECK`,與 notebook 的 assertion 一一對應 | `Checker`、`CHECK` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `np.round(np.arange(0.1, 2.01, 0.1), 2)` | `arange(0.1, 2.01, 0.1)` + `round_to(v, 2)` |
| `kps[np.nonzero(np.array(os_p) >= 4.0)[0][0]]` | `kps[first_index(n, [&](size_t i){ return os_p[i] >= 4.0; })]` |
| `step_metrics(t, y)['overshoot_pct']`(dict) | `step_metrics(t, y).overshoot_pct`(`StepMetrics` 結構) |
| `Delay(1)` | `Delay dly(1)` |
| `np.all(np.diff(os_p) > -0.5)` | 迴圈累積 `mono` 旗標 |
| Matplotlib 兩張子圖 | `ch03_steps.csv` 六條響應 + `ch03_sweeps.csv` 掃描表 |

`DiscretePID`、`MotorPlant`(前向 Euler)、`Delay` 與 `python/common/sim.py` 逐行對應,
全程沒有亂數,因此掃描表與最終指標和 notebook 逐位相同。

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch03_sweeps.csv` | `kp`, `os_p`, `ki`, `os_i` | 步驟 1 / 2 的 overshoot 掃描(kp 20 列、ki 17 列,較短欄位留空) |
| `ch03_steps.csv` | `t`, `kp0.200000`, `kp0.700000`, `kp1.400000`, `ki0.000000`, `ki30.000000`, `ki80.000000` | 代表性步階響應(欄名由 `std::to_string` 產生) |

```bash
python3 cpp/tools/plot_csv.py out/ch03_steps.csv
```

### 實際執行結果

```
kp sweep overshoot%: 0.1:0.0 0.2:0.0 0.3:0.0 0.4:0.0 0.5:0.0 0.6:0.0 0.7:4.5 0.8:11.0 0.9:18.1 1.0:24.3 1.1:33.9 1.2:43.1 1.3:51.8 1.4:60.0 1.5:67.6 1.6:74.8 1.7:81.5 1.8:87.7 1.9:93.3 2.0:114.5
選定 kp = 0.7(overshoot 首次 ≥ 4%)
ki sweep overshoot%: 0:4.5 5:6.3 10:8.1 15:9.9 20:11.7 25:13.5 30:15.5 35:17.6 40:19.6 45:21.6 50:23.6 55:25.7 60:27.7 65:29.7 70:31.6 75:33.6 80:35.6
選定 ki = 30(overshoot 首次 ≥ 15%)
類比 PM = 83.9° @ 56.1 Hz
取樣延遲相位損失 ≈ 30.3° → 數位 PM ≈ 53.6°
最終調機結果: rise=0.0020 OS=15.5405% settle=0.0420 sse=-0.0000

# ✅ 驗證
Ch3 驗證通過 ✅ (5/5)
```

與 Python notebook 的數值逐位相同(僅格式不同:notebook 印 dict 與 `ki = 30.0`)。

## ✅ 驗證條件(5 條)

| # | 條件 | 意義 |
|---|---|---|
| 1 | 相鄰 kp 的 overshoot 差 > −0.5% | overshoot 隨 kp 大致遞增 |
| 2 | `4 ≤ os_p[kp_sel] < 30` | 步驟 1 選到合理的 P 增益 |
| 3 | `14 ≤ 最終 OS < 30` | 步驟 2 達到 ~15% 目標 |
| 4 | `30 < 數位 PM < 75` | 調機結果的邊限合理 |
| 5 | `|穩態誤差| < 0.01` | 積分器消除穩態誤差 |

## 動手改改看

- 把全域 `fs` 改成 500,重跑整個流程:一拍延遲的時間變長,選出的 kp 會變小(練習 1)。
- 把 `Delay dly(1)` 改成 `Delay dly(0)`(無計算延遲),kp 掃描的 overshoot 表會怎麼變?還選得到 kp 嗎?
- 把步驟 2 的門檻 `15.0` 改成 `10.0`,比較選出的 ki 與數位 PM。

# Ch8 前饋 — C++ 版

> **理論與完整說明**:[`python/08-feedforward/README.md`](../../python/08-feedforward/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch08`
> **上一章**:[Ch7 擾動響應](../07-disturbance/README.md) · **下一章**:[Ch9 控制系統中的濾波器](../09-filters/README.md)

## 本章做什麼

以梯形速度規劃(加速 0.1 s、等速 0.3 s @ 2 rad/s、減速 0.1 s)驅動「比例位置迴路 + PI 速度迴路 + 剛體 $J=0.002$」的串級系統,
逐樣本($f_s=1$ kHz)比較:(1)無前饋 / 速度前饋 / 速度 + 加速度前饋的峰值追隨誤差;
(2)速度前饋比例 $K_{vff}=0\sim1.2$ 掃描;(3)無前饋等速段誤差對照理論值 $v/K_p$。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch08 && ./cpp/build/ch08
c++ -std=c++17 -O2 cpp/08-feedforward/main.cpp -o ch08 && ./ch08   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| 參數 | `J=0.002`、`b=0.01`、`fs=1000`;`acc_ref` 逐點賦值,`vel_ref`、`pos_ref` 以累加 × `dt` 積分 | `sample_times`、`cumsum` |
| `run` lambda | 每次新建 `MotorPlant(J, b, dt)` 與 `DiscretePID(0.5, 5.0, 0.0, dt)`;`v_cmd = kp_pos·err + kvff·vel_ref`,`u = PI(v_cmd − w) + kaff·J·acc_ref` | `MotorPlant`、`DiscretePID` |
| 實驗 1 | 三種組態 `{no FF, vel FF, vel + acc FF}`,記錄 `max_abs(err)` | `max_abs` |
| 實驗 2 | `arange(0, 1.21, 0.2)` 掃描 $K_{vff}$,記錄峰值誤差與端點 overshoot `max(pos) − max(pos_ref)` | `arange`、`vmax` |
| 等速段誤差 | $t\in(0.15, 0.35)$ s 的平均誤差 vs `max(vel_ref)/30` | `mean` |
| ✅ 驗證 | 4 條 `CHECK`,與 notebook 的 assertion 一一對應 | `Checker`、`CHECK` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `MotorPlant(J=J, b=b, dt=dt)` | `MotorPlant(J, b, dt)` |
| `DiscretePID(kp=0.5, ki=5.0, dt=dt)` | `DiscretePID(0.5, 5.0, 0.0, dt)`(`kd` 為位置參數) |
| `np.cumsum(acc_ref) * dt` | `cumsum(acc_ref)` 後逐元素乘 `dt` |
| `def run(kvff, kaff, kp_pos=30)` 回傳 `(pos, err)` | `run(kvff, kaff, &pos_out, kp_pos)` lambda,回傳 `err`,`pos` 經指標輸出 |
| `cases` dict | `std::vector<Case>`;結果存 `std::map<std::string, Vec>` |
| `np.max(np.abs(err))` | `max_abs(err)` |
| Matplotlib 兩張圖 | 兩份 CSV(下表) |

逐樣本模擬使用與 `python/common/sim.py` 相同的 Euler 更新式與條件積分 PI,沒有亂數,因此數值與 notebook 一致。

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch08_feedforward.csv` | `t`, `pos_ref`, `pos_noFF`, `err_noFF`, `pos_velFF`, `err_velFF`, `pos_velaccFF`, `err_velaccFF` | 實驗 1 位置與追隨誤差曲線 |
| `ch08_kvff_sweep.csv` | `kvff`, `peak_err`, `endpoint_overshoot` | 實驗 2 前饋比例取捨曲線(單位 rad) |

```bash
python3 cpp/tools/plot_csv.py out/ch08_feedforward.csv
```

### 實際執行結果

```
峰值追隨誤差 [mrad]: no FF 66.58 | vel FF 2.01 | vel + acc FF 0.67
Kvff 掃描峰值誤差 [mrad]: 0.0:66.58 0.2:53.25 0.4:39.92 0.6:26.59 0.8:13.27 1.0:2.01 1.2:13.97
等速段誤差 65.80 mrad(理論 v/kp = 66.67 mrad)

# ✅ 驗證
Ch8 驗證通過 ✅ (4/4)
```

峰值誤差與 $K_{vff}$ 掃描表與 Python notebook 的輸出逐位相同;等速段誤差一行是 C++ 版額外印出的
(notebook 只在 assertion 內計算),差 1.3%,在 15% 容許範圍內。
`ch08_kvff_sweep.csv` 中端點 overshoot 在 $K_{vff}\le0.8$ 皆為 0,$K_{vff}=1.0$ 為 1.95 mrad、$1.2$ 為 6.69 mrad。

## ✅ 驗證條件(4 條)

| # | 條件 | 意義 |
|---|---|---|
| 1 | `peak[no FF] > peak[vel FF] > peak[vel + acc FF]` | 每多一層前饋都有改善 |
| 2 | `peak[vel FF] < 0.25 · peak[no FF]` | 速度前饋至少改善 4 倍 |
| 3 | `peak[vel + acc FF] < 0.5 · peak[vel FF]` | 加速度前饋處理加減速段殘差 |
| 4 | `|e_cruise − v/kp| / (v/kp) < 0.15` | 無前饋時追隨誤差 = $v/K_p$ |

## 動手改改看

- 把 `run` 的預設 `kp_pos = 30.0` 改成 60,無前饋等速段誤差應減半為約 33 mrad —— 但想想速度迴路頻寬夠不夠。
- 在 `run` 內用 `kaff·1.3·J·acc_ref` 模擬慣量估錯 +30%,看 vel + acc FF 的 0.67 mrad 會回升多少。
- 在 `u` 再加上黏滯前饋 `b·vel_ref`(完整逆模型),觀察殘餘誤差是否再下降。

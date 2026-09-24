# Ch17 位置控制迴路 — C++ 版

> **理論與完整說明**:[`python/17-position-control/README.md`](../../python/17-position-control/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch17`
> **上一章**:[Ch16 機構柔性與共振](../16-resonance/README.md) · **下一章**:[Ch18 運動控制中的觀測器](../18-motion-observer/README.md)

## 本章做什麼

在 `MotorPlant(J=0.002, b=0.01)`、1 kHz 上比較兩種位置迴路:
(1)串級 P 位置($k_{p,pos}=30$,速度命令夾 ±2 rad/s)+ PI 速度(0.5, 5);
(2)單迴路 PID(15, 20, 0.5,導數濾波 100 Hz)。1 rad 步階、$t=1.2$ s 加 0.05 Nm 負載;
另以 1 rad/s 斜坡驗證追隨誤差 $= v/k_{p,pos}$。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch17 && ./cpp/build/ch17
c++ -std=c++17 -O2 cpp/17-position-control/main.cpp -o ch17 && ./ch17   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| 常數 | `J`、`b`、`dt=1e-3`、`n=2000`、`V_MAX=2`、`U_MAX=1`、`DIST_T=1.2` | — |
| `run_cascade` | `v_cmd = saturate(kp_pos·(step − pos), ±V_MAX)` → `vel.step(v_cmd − w)`(±U_MAX) | `MotorPlant`、`DiscretePID`、`saturate` |
| `run_pid` | `DiscretePID c(15, 20, 0.5, dt, −1, 1, 100)`(最後一個參數 = 導數濾波 Hz) | `DiscretePID` |
| 擾動偏移 | $t \ge 1.2$ s 起的 $\max|1-\theta|$ | — |
| 斜坡 | `kp_pos ∈ {15, 30, 60}`,`err = t − pos`,取 $t \ge 1$ s 的 `mean` | `mean`、`slice` |
| ✅ 驗證 | 8 條 `CHECK`(notebook 的 for 迴圈展開成 3 條) | `Checker`、`CHECK`、`max_abs`、`vmax` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `MotorPlant(J=J, b=b, dt=dt)` | `MotorPlant plant(J, b, dt)` |
| `DiscretePID(0.5, 5.0, dt=dt, out_min=-U_MAX, out_max=U_MAX)` | `DiscretePID vel(0.5, 5.0, 0.0, dt, -U_MAX, U_MAX)` |
| `DiscretePID(15, 20, 0.5, dt=dt, dfilt_hz=100, ...)` | `DiscretePID c(15.0, 20.0, 0.5, dt, -U_MAX, U_MAX, 100)` |
| `np.clip(x, -V_MAX, V_MAX)` | `saturate(x, -V_MAX, V_MAX)` |
| `plant.step(u, t_dist=...)` | `plant.step(u, t[k] >= DIST_T ? 0.05 : 0.0)` |
| `np.abs(w_c).max()` | `max_abs(c.w)` |
| `ferr` dict | `std::map<double, double> ferr` |
| Matplotlib 兩張圖 | 兩份 CSV(下表) |

同一套前向 Euler 受控體與 `DiscretePID`,無亂數,因此數值與 notebook 一致。

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch17_cascade_vs_pid.csv` | `t`, `pos_cascade`, `w_cascade`, `pos_pid`, `w_pid` | 實驗 1:位置與速度波形(串級的梯形速度 vs PID) |
| `ch17_ramp_following_error.csv` | `t`, `err_kp15`, `err_kp30`, `err_kp60` | 實驗 2:斜坡追隨誤差 |

```bash
python3 cpp/tools/plot_csv.py out/ch17_cascade_vs_pid.csv
```

### 實際執行結果

```
串級:max|w|=2.03(極限 2.0),擾動偏移 2.01 mrad
PID :max|w|=18.30(未受控!),擾動偏移 6.27 mrad
kp_pos=15: 量測 66.67 mrad,理論 66.67 mrad
kp_pos=30: 量測 33.33 mrad,理論 33.33 mrad
kp_pos=60: 量測 16.67 mrad,理論 16.67 mrad

# ✅ 驗證
Ch17 驗證通過 ✅ (8/8)
```

與 Python notebook 的輸出逐位相同。

## ✅ 驗證條件(8 條)

| # | 條件 | 意義 |
|---|---|---|
| 1 | `max_abs(c.w) ≤ 1.05 · V_MAX` | 串級尊重速度極限 |
| 2 | `max_abs(p.w) > 3 · V_MAX` | PID 無速度概念 |
| 3 | `vmax(c.pos) < 1.005` | 串級無 overshoot(梯形到位) |
| 4 | `|c.pos.back() − 1| < 1e-3` 且 `|p.pos.back() − 1| < 5e-3` | 兩者皆到位 |
| 5 | `dev_c < dev_p` | 串級剛性較好(內環先擋) |
| 6–8 | `|e − v/kp_pos| / (v/kp_pos) < 0.15`,kp_pos = 15, 30, 60 | 追隨誤差 = $v/k_v$ |

## 動手改改看

- 速度前饋:在 `main()` 的斜坡迴圈把 `v_cmd` 改成 `saturate(kp_pos * err[k] + ramp_v, ...)`,追隨誤差應趨近 0。
- 把 `kp_pos` 掃到 `120.0`,觀察 `w_cascade` 的梯形開始振鈴(外環追上內環頻寬)。
- 把 `V_MAX` 改成 `0.5`,比較兩種架構的到位時間與 `max|w|`。

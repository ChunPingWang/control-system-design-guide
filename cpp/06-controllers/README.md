# Ch6 四種控制器 — C++ 版

> **理論與完整說明**:[`python/06-controllers/README.md`](../../python/06-controllers/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch06`
> **上一章**:[Ch5 z 域](../05-z-domain/README.md) · **下一章**:[Ch7 擾動響應](../07-disturbance/README.md)

## 本章做什麼

1 kHz 離散速度迴路上讓 P / PI / PID / PID+(2-DOF,設定點加權 b = 0.6)同場對決:
t = 0 單位步階命令、t = 0.5 s 施加 0.1 Nm 負載轉矩,比較 overshoot、命令穩態誤差、擾動跌落與最終誤差;
另以連續模型算 P / PI / PID 的開迴路 PM。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch06 && ./cpp/build/ch06
c++ -std=c++17 -O2 cpp/06-controllers/main.cpp -o ch06 && ./ch06   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| 參數 | 全域 `J`、`b`、`fs=1000`、`n=1000`;`struct Design{name, kp, ki, kd, sp_weight}` | — |
| `run(d, dist)` | 2-DOF:`pd_part`(kp、kd,D 濾波 100 Hz)作用在 `sp_weight·r − y`,`i_part`(ki)作用在 `r − y`;`plant.step(u, dist[k])` | `MotorPlant`、`DiscretePID` |
| 指標表 | 前半段 `step_metrics`;後半段 `1 − vmin(y)` 為跌落、`1 − y.back()` 為最終誤差 | `step_metrics`、`slice`、`vmin` |
| 開迴路 PM | `pid_ctrl(0.5)`、`pid_ctrl(0.5, 15)`、`pid_ctrl(0.8, 25, 0.004, 2π·100)` 乘 `inertia` | `pid_ctrl`、`margins`、`write_bode_csv` |
| ✅ 驗證 | 6 條 `CHECK`,與 notebook 的 assertion 一一對應 | `Checker`、`CHECK` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `designs = {name: dict(kp=..., ...)}` | `std::vector<Design>`(`sp_weight` 明確給 1.0 或 0.6) |
| `DiscretePID(kp, ki=0.0, kd=kd, dt=dt, dfilt_hz=100)` | `DiscretePID(kp, 0.0, kd, dt, -INF, INF, dfilt_hz)` |
| `np.where(t >= 0.5, 0.1, 0.0)` | 迴圈逐點賦值 |
| `y[half:].min()` | `vmin(slice(y, half))` |
| `pid(0.8, 25, 0.004, n=2*np.pi*100)` | `pid_ctrl(0.8, 25, 0.004, 2 * PI * 100)` |
| PM 標在 Bode 圖例 | `printf` 印出各控制器 PM |
| 時域圖 + Bode 圖 | 兩份 CSV(下表) |

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch06_controllers.csv` | `t`, `P`, `PI`, `PID`, `PIDplus` | 四種控制器的速度響應(命令步階 + 負載擾動) |
| `ch06_open_loop.csv` | `f_hz`, `P_mag_db`, `P_phase_deg`, `PI_mag_db`, `PI_phase_deg`, `PID_mag_db`, `PID_phase_deg` | 開迴路 Bode 形狀 |

```bash
python3 cpp/tools/plot_csv.py out/ch06_controllers.csv
```

### 實際執行結果

```
controller                      OS%  sse(cmd)  dist dip  sse(end)
P    (kp=0.5)                   0.0    0.0196    0.2157    0.2157
PI   (kp=0.5, ki=15)            7.0   -0.0000    0.1676    0.0000
PID  (kp=0.8, ki=25, kd=4m)     6.3    0.0000    0.0889   -0.0000
PID+ (2-DOF b=0.6)              0.0   -0.0000    0.0889   -0.0000
P    PM = 91°
PI   PM = 84°
PID  PM = 106°

# ✅ 驗證
Ch6 驗證通過 ✅ (6/6)
```

指標表與 Python notebook 逐位相同(連 `-0.0000` 的符號都一致);PM 三行為 C++ 版額外印出,與 notebook 圖例相同。

## ✅ 驗證條件(6 條)

| # | 條件 | 意義 |
|---|---|---|
| 1 | P:`dip > 0.05` 且 `|sse_end| > 0.05` | P 控制對擾動留下穩態誤差 |
| 2 | PI:`|sse_end| < 0.005` | 積分作用讓擾動誤差歸零 |
| 3 | PID:`|sse_end| < 0.005` | 同上 |
| 4 | `PID.dip < PI.dip` | PID 增益較高,擾動跌落較小 |
| 5 | `PID+.os < PID.os − 2` | 2-DOF 設定點加權壓 overshoot |
| 6 | `|PID+.dip − PID.dip| < 0.01` | 且不犧牲擾動抑制 |

## 動手改改看

- 把 `run(d, dist)` 的 `dfilt_hz` 改成 20,比較 PID 的 overshoot 與擾動跌落(練習 1)。
- 在 `run()` 裡把回授訊號換成 `plant.w + noise[k]`(`Rng(0).normal(0, 0.01, n)`),累積 `u` 的 RMS 比較四種控制器(練習 2)。
- 把 PID+ 的 `sp_weight` 從 0.4 掃到 1.0,每次印出 overshoot,畫 overshoot vs b(練習 3)。

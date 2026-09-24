# Ch12 非線性行為 — C++ 版

> **理論與完整說明**:[`python/12-nonlinear/README.md`](../../python/12-nonlinear/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch12`
> **上一章**:[Ch11 建模入門](../11-modeling/README.md) · **下一章**:[Ch13 模型建立與驗證](../13-model-verification/README.md)

## 本章做什麼

在剛體 $J=0.002$、$b=0.01$($f_s=1$ kHz、1.5 s)上逐樣本模擬三種 LTI 工具做不到的非線性:
(1)轉矩飽和 ±0.3 N·m 下 5 rad/s 大步階,naive 積分器 vs 條件積分 anti-windup 的 overshoot;
(2)1 Hz 正弦速度命令下,0.05 N·m 庫倫摩擦造成的換向誤差尖峰;
(3)總寬 0.02 rad 背隙的失動量與遲滯。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch12 && ./cpp/build/ch12
c++ -std=c++17 -O2 cpp/12-nonlinear/main.cpp -o ch12 && ./ch12   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| 實驗 1 | `run_windup(anti_windup)`:`DiscretePID(0.5, 20.0, 0.0, dt, -u_max, u_max, 0.0, anti_windup)` + `MotorPlant`;overshoot = `vmax(y) − target` | `DiscretePID`、`MotorPlant`、`vmax` |
| 實驗 2 | `run_friction(fc)`:`plant.step(u, coulomb(plant.w, fc))`,fc ∈ {0, 0.05};0.3 s 後的 \|誤差\| | `coulomb`、`vmax` |
| 實驗 3 | `Backlash bl(0.02)` 逐點推動;`gap = motor − load` | `Backlash`、`vmin`、`vmax` |
| 過零點 | 以 `sgn` 找 `w_ref` 過零索引,與 `argmax(err_fric)` 的最小距離 | `argmax` |
| ✅ 驗證 | 6 條 `CHECK`,與 notebook 的 assertion 一一對應 | `Checker`、`CHECK` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `DiscretePID(kp, ki, dt=dt, out_min=-u_max, out_max=u_max, anti_windup=aw)` | `DiscretePID(kp, ki, 0.0, dt, -u_max, u_max, 0.0, aw)`(位置參數;`dfilt_hz = 0` 表示不濾波) |
| `plant.step(u, t_dist=coulomb(plant.w, fc))` | `plant.step(u, coulomb(plant.w, fc))` |
| `Backlash(width=0.02)`;`[bl.step(x) for x in motor_pos]` | `Backlash bl(0.02)`;迴圈 `bl.step(motor_pos[k])` |
| `np.nonzero(np.diff(np.sign(w_ref[k0:])))[0]` | 迴圈比較相鄰 `sgn` 值 |
| `np.all(np.abs(u_naive) <= u_max + 1e-9)` | `max_abs(u_naive) <= u_max + 1e-9` |
| Matplotlib 三組圖 | 三份 CSV(下表) |

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch12_windup.csv` | `t`, `y_naive`, `u_naive`, `y_antiwindup`, `u_antiwindup` | 實驗 1:速度與(飽和後)轉矩 |
| `ch12_friction.csv` | `t`, `w_ref`, `w_no_friction`, `w_friction` | 實驗 2:有無摩擦的速度追蹤 |
| `ch12_backlash.csv` | `t`, `motor_pos`, `load_pos` | 實驗 3:以 `motor_pos` 對 `load_pos` 作圖即得遲滯迴線 |

```bash
python3 cpp/tools/plot_csv.py out/ch12_windup.csv
```

### 實際執行結果

```
overshoot:naive 2.428 rad/s | anti-windup 0.002 rad/s
最大追蹤誤差:無摩擦 0.0086 | 有摩擦 0.1604
失動量範圍:[-0.0100, 0.0100](理論 ±0.01 rad)

# ✅ 驗證
Ch12 驗證通過 ✅ (6/6)
```

本章無亂數,與 Python notebook 的輸出逐位相同。從 `ch12_windup.csv` 可讀出 windup 的時序:
naive 版本在 37 ms 到達 5 rad/s,但轉矩仍卡在 +0.3 N·m 直到 55 ms,速度在 58 ms 衝到峰值 7.43 rad/s。

## ✅ 驗證條件(6 條)

| # | 條件 | 意義 |
|---|---|---|
| 1 | `os_naive > 2 · max(os_aw, 0.05)` | windup 顯著惡化 overshoot |
| 2 | `os_aw < 0.5` | 條件積分有效 |
| 3 | `max_abs(u_naive) <= u_max + 1e-9` | 飽和確實生效 |
| 4 | `vmax(err_fric) > 3 · vmax(err_nof)` | 摩擦造成明顯誤差尖峰 |
| 5 | 尖峰距最近過零點 < 150 樣本(0.15 s) | 尖峰發生在速度換向處 |
| 6 | `|max(gap) − 0.01| < 1e-3` 且 `|min(gap) + 0.01| < 1e-3` | 背隙半寬 = 0.01 rad |

## 動手改改看

- 在 `run_friction` 的 `u` 加上摩擦前饋 `0.05 * sgn(w_ref[k])`,看 0.1604 的尖峰能壓到多少;再把前饋量故意設成 0.04 或 0.06。
- 把 `run_windup` 的 `target` 改成 0.5(小步階,幾乎不進飽和),比較兩種積分器的 overshoot 差距是否消失 —— windup 只在大訊號出現。
- 把 `Backlash bl(0.02)` 放進一個以 `load_pos` 回授的比例位置迴路,逐步提高增益,找出極限循環出現的點。

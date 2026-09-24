# Ch18 運動控制中的觀測器 — C++ 版

> **理論與完整說明**:[`python/18-motion-observer/README.md`](../../python/18-motion-observer/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch18`
> **上一章**:[Ch17 位置控制迴路](../17-position-control/README.md) · **下一章**:[Ch19 快速控制原型](../19-rapid-control-prototyping/README.md)

## 本章做什麼

`MotorPlant(J=0.002, b=0.01)` + 2500 線編碼器的 1 kHz 速度迴路(PI,$k_i=5$),速度回授兩選一:
編碼器差分(FD)或 Luenberger 觀測器(極點 −200、−220 rad/s)。
(1)$k_p=0.5$ 比較轉矩雜訊;(2)$k_p \in \{0.25, 0.5, 1.0, 1.5\}$ 掃描,對照理論 $k_pq/(\sqrt6T_s)$。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch18 && ./cpp/build/ch18
c++ -std=c++17 -O2 cpp/18-motion-observer/main.cpp -o ch18 && ./ch18   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| 觀測器增益 | 對偶設計:對 $(A^T, C^T)$ 做 `place_siso` 得 $K$,$L = K^T$ → `L0`、`L1` | `Mat`、`place_siso` |
| `vel_loop(use_observer, kp)` | 每拍:量化位置 → FD 速度;innovation `y − x0`;前向 Euler 更新 `x0`、`x1`(輸入為 `u_prev`);回授選 `x1` 或 `w_fd` | `MotorPlant`、`EncoderModel`、`DiscretePID` |
| 實驗 1 | `kp0 = 0.5`,穩態段($t \ge 1$ s)轉矩 `stdev` | `stdev`、`slice` |
| 實驗 2 | 四個 $k_p$ × 兩種回授;多印一欄理論值 | — |
| ✅ 驗證 | 11 條 `CHECK`(notebook 兩個 for 迴圈展開成 4 + 4 條) | `Checker`、`CHECK`、`mean` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `ct.place(A.T, Cm.T, [-200, -220]).T` | `place_siso(At, Ct, {-200.0, -220.0})`(Ackermann) |
| `xhat = xhat + (A@xhat + B*u_prev + L*innov)*dt`(矩陣) | 展開成 `dx0 = x1 + L0·innov`、`dx1 = −b/J·x1 + u_prev/J + L1·innov` |
| `vel_loop('fd' / 'obs', kp)` | `vel_loop(false / true, kp)` |
| `float(enc.read(pos/(2π)))*2π` | `enc.read(plant.pos / (2 * PI)) * 2 * PI` |
| `np.std(u[settle])` | `stdev(slice(u, settle))` |
| 理論值只畫在圖上 | 表格多一欄 `theory` |
| Matplotlib 兩張圖 | 兩份 CSV(下表) |

觀測器以純量展開後與 notebook 的矩陣運算逐項相同,無亂數,因此數值一致。

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch18_fd_vs_observer.csv` | `t`, `fb_fd`, `fb_obs`, `u_fd`, `u_obs` | 實驗 1:$k_p=0.5$ 的回授訊號與轉矩命令 |
| `ch18_noise_vs_kp.csv` | `kp`, `noise_fd`, `noise_obs`, `theory` | 實驗 2:雜訊 vs 增益(建議以對數 y 軸作圖) |

```bash
python3 cpp/tools/plot_csv.py out/ch18_fd_vs_observer.csv
```

### 實際執行結果

```
kp=0.5:轉矩雜訊 RMS 差分 0.1536 → 觀測器 0.0036(改善 43 倍)
    kp        FD  observer   ratio    theory
  0.25    0.0764    0.0018    43.5    0.0641
  0.50    0.1536    0.0036    43.0    0.1283
  1.00    0.3080    0.0077    40.2    0.2565
  1.50    0.4975    0.0125    39.8    0.3848

# ✅ 驗證
Ch18 驗證通過 ✅ (11/11)
```

所有數值與 Python notebook 的輸出相同;差別只在 C++ 表格多印了 `theory` 欄、$k_p$ 以兩位小數顯示。

## ✅ 驗證條件(11 條)

| # | 條件 | 意義 |
|---|---|---|
| 1 | `rms_ob < rms_fd / 10` | $k_p=0.5$ 時至少改善一個數量級 |
| 2 | `|mean(ob.w[settle]) − 1| < 0.01` | 觀測器回授的追蹤沒有犧牲 |
| 3 | `|mean(fd.w[settle]) − 1| < 0.02` | FD 回授平均也到位 |
| 4–7 | `noise_ob[i] < noise_fd[i] / 5`,四個 $k_p$ | 改善在所有增益成立 |
| 8–11 | `0.3 < noise_fd[i] / theory[i] < 3`,四個 $k_p$ | FD 雜訊符合 $k_pq/(\sqrt6T_s)$ |

## 動手改改看

- 在 `dx1` 裡把 `/ J` 改成 `/ (1.3 * J)`(觀測器模型慣量錯 +30%),看轉矩雜訊與 `w` 的暫態變化。
- 把極點改成 `{-1000.0, -1100.0}`,觀測器雜訊會往 FD 靠攏多少?
- 在 FD 分支加一個 `OnePole lpf(200, dt)` 濾波後再回授,比較它的雜訊與 step response 延遲。

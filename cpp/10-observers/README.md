# Ch10 觀測器入門 — C++ 版

> **理論與完整說明**:[`python/10-observers/README.md`](../../python/10-observers/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch10`
> **上一章**:[Ch9 控制系統中的濾波器](../09-filters/README.md) · **下一章**:[Ch11 建模入門](../11-modeling/README.md)

## 本章做什麼

對剛體馬達 $x=[\theta,\omega]^T$(只量位置)設計 Luenberger 觀測器:
(1)檢查可觀測性並用 Ackermann 公式把 $A-LC$ 極點放到 −200 / −220;
(2)開迴路正弦轉矩下,從錯誤的初始速度估測(−2 rad/s)收斂;
(3)位置經 10000 counts/rev 編碼器量化,比較有限差分速度與觀測器速度的誤差 RMS。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch10 && ./cpp/build/ch10
c++ -std=c++17 -O2 cpp/10-observers/main.cpp -o ch10 && ./ch10   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| 狀態空間 | `Mat A(2,2), B(2,1), Cm(1,2)` 逐元素賦值 | `Mat` |
| 可觀測性 | 直接算 $[C; CA]$ 的 2×2 行列式(≠0 ⇔ 秩 2) | `Mat` 乘法 |
| 極點配置 | 對偶:`place_siso(Aᵀ, Cᵀ, {-200, -220})` 得 $L^T$;`eig2(A − L·C)` 驗證 | `place_siso`、`eig2` |
| `obs_step` lambda | 前向 Euler:`x̂ += (A x̂ + B u + L (y − C x̂))·dt`,展開成純量運算 | — |
| 實驗 1 | `MotorPlant` + 正弦轉矩;初值 `x1 = -2.0`;記錄 `w_true`、`w_hat` | `MotorPlant`、`sample_times` |
| 實驗 2 | `EncoderModel(2500)` 量化位置;差分速度 vs 觀測器速度;`k ≥ 200` 的誤差 `stdev` | `EncoderModel`、`stdev` |
| ✅ 驗證 | 4 條 `CHECK`,與 notebook 的 assertion 一一對應 | `Checker`、`CHECK` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `np.linalg.matrix_rank(ct.obsv(A, Cm))` | 手算 `det([C; CA])`(2×2 足夠) |
| `ct.place(A.T, Cm.T, obs_poles).T` | `place_siso(At, Ct, obs_poles)`(Ackermann) |
| `np.linalg.eigvals(A - L @ Cm)` | `eig2(A - Lm * Cm)` |
| `xhat = xhat + (A @ xhat + B*u + L*innov) * dt` | `obs_step(x0, x1, u, y, dt)` 展開後的純量式 |
| `EncoderModel(lines=2500).read(pos_rev)` | `EncoderModel(2500).read(pos_rev)` |
| `np.std(w_fd[200:] - w_true2[200:])` | 迴圈收集誤差後 `stdev` |
| Matplotlib 兩張圖 | 兩份 CSV(下表) |

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch10_convergence.csv` | `t`, `w_true`, `w_hat` | 實驗 1:觀測器從錯誤初值收斂 |
| `ch10_quantized.csv` | `t`, `w_true`, `w_fd`, `w_obs` | 實驗 2:量化下差分與觀測器速度 |

```bash
python3 cpp/tools/plot_csv.py out/ch10_quantized.csv
```

### 實際執行結果

```
可觀測性矩陣行列式 = 1(≠0 → 秩 2)
觀測器增益 L = [415.0, 41925.0]
驗證極點: -200.0000, -220.0000
初始速度誤差 2.00 → 後半段最大誤差 0.02224
速度誤差 RMS:差分 0.2574 | 觀測器 0.0321(改善 8.0 倍)
差分雜訊理論值 ~ q/(sqrt(6)·Ts) = 0.2565

# ✅ 驗證
Ch10 驗證通過 ✅ (4/4)
```

本章無亂數,$L$、收斂誤差、雜訊 RMS 與 Python notebook 的輸出逐位相同;
只有可觀測性一行的呈現不同(Python 印矩陣秩 2,C++ 印行列式 1,兩者等價)。

## ✅ 驗證條件(4 條)

| # | 條件 | 意義 |
|---|---|---|
| 1 | 排序後極點與 −220 / −200 的相對誤差 < 1e-6 | Ackermann 極點配置正確 |
| 2 | `err_early > 1.5 && err_late < 0.05` | 從 2 rad/s 誤差收斂(殘差為 Euler 離散化不一致) |
| 3 | `noise_obs < noise_fd / 3` | 觀測器速度比差分乾淨得多 |
| 4 | `noise_fd > q_rad / dt / 10` | 差分雜訊量級符合量化理論 $q/(\sqrt6 T_s)$ |

## 動手改改看

- 把 `obs_poles` 改成 `{-50, -60}` 與 `{-2000, -2200}`:慢極點收斂慢但速度更平滑;快極點在 1 kHz 前向 Euler 下 $|1 + pT_s| > 1$,離散觀測器直接發散。
- 在 `obs_step` 裡把 `u / J` 改成 `u / (1.5 * J)`(模型慣量錯 +50%),觀察 `w_obs` 的系統性誤差。
- 把 `EncoderModel enc(2500)` 改成 `enc(500)`,驗證差分雜訊隨 $q$ 放大 5 倍,而觀測器的改善倍數維持。

# Ch1 回授控制導論 — C++ 版

> **理論與完整說明**:[`python/01-introduction/README.md`](../../python/01-introduction/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch01`
> **下一章**:[Ch2 頻域分析](../02-frequency-domain/README.md)

## 本章做什麼

以一階受控體 $P(s)=K/(0.1s+1)$ 比較開迴路與 PI 閉迴路($C=2+20/s$):
(1)受控體增益 $K$ 漂移 1~3 時的穩態誤差;(2)$t=0.75$ s 注入 −0.5 階躍擾動後的誤差。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch01 && ./cpp/build/ch01
c++ -std=c++17 -O2 cpp/01-introduction/main.cpp -o ch01 && ./ch01   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| 參數 | `tau=0.1`、`K_nom=2`、`C = pi_ctrl(2, 20)`、`t = linspace(0, 1.5, 1500)` | `pi_ctrl`、`linspace` |
| 實驗 1 | 對 $K\in\{1,2,3\}$:開迴路 `forced_response(P, t, 1/K_nom)`,閉迴路 `step_response(closed_loop(C, P), t)`;記錄最終誤差 | `first_order`、`forced_response`、`step_response`、`closed_loop` |
| 實驗 2 | 擾動 `d(t)`;閉迴路 = 命令響應 + `forced_response(feedback(P, C), t, d)`(線性疊加) | `feedback` |
| ✅ 驗證 | 7 條 `CHECK`,與 notebook 的 assertion 一一對應 | `Checker`、`CHECK` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `first_order(tau=tau, gain=K)` | `first_order(tau, K)` |
| `pi(kp=2.0, ki=20.0)` | `pi_ctrl(2.0, 20.0)` |
| `ct.forced_response(P, t, u)` | `forced_response(P, t, u)`(回傳 `Vec` 輸出) |
| `ct.step_response(T, t)` | `step_response(T, t)` |
| `ct.feedback(P, C)` | `feedback(P, C)` |
| `np.where(t >= 0.75, -0.5, 0.0)` | 迴圈逐點賦值 |
| Matplotlib 兩張圖 | 兩份 CSV(下表) |

`forced_response` 以矩陣指數做精確離散化,輸入在樣本間線性內插,與 python-control 相同假設,
因此數值與 notebook 一致。

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch01_gain_variation.csv` | `t`, `open_K1`, `closed_K1`, `open_K2`, `closed_K2`, `open_K3`, `closed_K3` | 實驗 1 六條響應曲線 |
| `ch01_disturbance.csv` | `t`, `open`, `closed` | 實驗 2 擾動響應 |

```bash
python3 cpp/tools/plot_csv.py out/ch01_gain_variation.csv
```

### 實際執行結果

```
   K 開迴路穩態誤差 閉迴路穩態誤差
 1.0         0.5000       0.000000
 2.0         0.0000       0.000000
 3.0        -0.5000       0.000000
開迴路最終誤差 : 0.9994
閉迴路最終誤差 : 0.000184

# ✅ 驗證
Ch1 驗證通過 ✅ (7/7)
```

與 Python notebook 的輸出逐位相同。

## ✅ 驗證條件(7 條)

| # | 條件 | 意義 |
|---|---|---|
| 1 | `|開迴路誤差(K=2)| < 1e-3` | 模型正確時開迴路也準 |
| 2 | `|開迴路誤差(K=1) − 0.5| < 0.01` | 模型錯 50% → 輸出錯 50% |
| 3–5 | `|閉迴路誤差(K)| < 1e-3`,K = 1, 2, 3 | 回授對參數漂移不敏感 |
| 6 | 擾動後開迴路誤差明顯(> 0.2) | 開迴路無法反應擾動 |
| 7 | 擾動後閉迴路誤差 < 1e-3 | 積分作用消除擾動穩態誤差 |

## 動手改改看

- 把 `pi_ctrl(2.0, 20.0)` 改成 `pi_ctrl(2.0, 0.0)`(純 P),看閉迴路誤差變成 $1/(1+2K)$。
- 把 `ki` 改成 200,重新產生 `ch01_gain_variation.csv` 畫圖,觀察振鈴。

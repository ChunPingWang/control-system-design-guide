# Ch19 快速控制原型(RCP)— C++ 版

> **理論與完整說明**:[`python/19-rapid-control-prototyping/README.md`](../../python/19-rapid-control-prototyping/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch19`
> **上一章**:[Ch18 運動控制中的觀測器](../18-motion-observer/README.md) · **下一步**:[實機平台 `hardware/`](../../hardware/README.md)(全書最後一章)

## 本章做什麼

完整走一遍 RCP 流程:(1)產生一段「ESP32 開迴路步階」遙測 CSV(格式同韌體序列輸出);
(2)讀回 CSV,以兩參數 Levenberg–Marquardt 擬合 $K/(\tau s+1)$;(3)λ-tuning(λ = 0.1 s)算 PI;
(4)100 Hz 離散閉迴路驗證(±255 PWM);(5a)韌體演算法 vs `DiscretePID` 差異 ≤ $k_i|e|T_s$;
(5b)**直接 `#include` 真正的韌體 `hardware/esp32/src/pid.h`**,以 float32 跑同一序列,與 double 版比對。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch19 && ./cpp/build/ch19
c++ -std=c++17 -O2 cpp/19-rapid-control-prototyping/main.cpp -o ch19 && ./ch19   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。`pid.h` 以相對路徑 `../../hardware/esp32/src/pid.h` 引入,單檔編譯時請在 repo 根目錄執行。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| `CStylePID` | 逐行翻譯 `pid_step()` 的 double 版:積分先更新、導數不濾波 | `saturate` |
| `fit_first_order` | 手寫 LM:解析 Jacobian($\partial/\partial K$、$\partial/\partial\tau$)、阻尼 λ 自適應(×0.3 / ×10) | — |
| 步驟 1 | `Rng rng(1)` 產生雜訊,`snprintf` 寫 `millis,target,actual,pwm` | `Rng`、`out_path` |
| 步驟 2 | `ifstream` + `getline(',')` 解析 CSV,初值 $K=1$、$\tau=0.1$ | — |
| 步驟 3–4 | `kp = tau/(K·lam)`、`ki = kp/tau`;真實一階馬達 Euler 更新 | `DiscretePID`、`step_metrics` |
| 步驟 5a | `Rng rng2(3)` 產生 500 點誤差,`CStylePID` vs `DiscretePID(..., anti_windup=false)` | — |
| 步驟 5b | `pid_t_` + `pid_init` / `pid_step`(float32)vs `CStylePID` 輸出 | `pid.h` |
| ✅ 驗證 | 7 條 `CHECK`:notebook 的 6 條 + float32 韌體一致性 | `Checker`、`CHECK`、`first_index` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `np.random.default_rng(1).normal(0, 3, n)`(PCG64) | `Rng(1).normal(0, 3.0, n)`(`std::mt19937_64`) |
| 遙測存成字串、`pd.read_csv(io.StringIO(...))` | 寫出 `ch19_telemetry.csv` 再用 `ifstream` 讀回 |
| `scipy.optimize.curve_fit` | `fit_first_order`(兩參數 LM) |
| `step_metrics(...)` 回傳 dict | `StepMetrics` 結構(`rise_time`、`overshoot_pct`…) |
| `class CStylePID`(Python 翻譯) | `struct CStylePID`(double 翻譯)**+ 真正的 `pid.h`(float32)** |
| `DiscretePID(..., anti_windup=False)` | `DiscretePID(kp, ki, 0.0, dt, -255, 255, 0.0, false)` |
| Matplotlib 兩張圖 | 三份 CSV(下表) |

**亂數不同 → 數字略有不同**:C++ 標準亂數產生器無法重現 numpy 的 PCG64 序列,
雜訊樣本不同,因此擬合結果與其後的增益、閉迴路指標、等價性最大差異都與 notebook 有小差距(見下方比較);
所有驗證條件在兩邊都通過。

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch19_telemetry.csv` | `millis`, `target`, `actual`, `pwm` | 步驟 1:模擬遙測(與韌體序列輸出同格式,可直接換成實機記錄) |
| `ch19_identification.csv` | `t`, `telemetry`, `fitted` | 步驟 2:遙測點與擬合曲線 |
| `ch19_closed_loop.csv` | `t`, `rpm` | 步驟 4:λ-tuning 閉迴路步階(目標 400 rpm) |

```bash
python3 cpp/tools/plot_csv.py out/ch19_identification.csv
```

### 實際執行結果

```
遙測寫入 out/ch19_telemetry.csv(格式同韌體序列輸出)
擬合:K=3.000(真值 3.0),tau=248.6 ms(真值 250 ms)
λ-tuning:kp=0.8289 pwm/rpm,ki=3.3338
閉迴路指標:rise=0.2900 OS=0.0000% settle=0.6100 sse=0.0249
最大差異 5.6284 pwm-unit;理論上限 max(ki·|e|·Ts) = 5.6284
超出理論上限的樣本數:0 / 500
韌體 pid.h(float32)vs double 版最大差異 2.66e-05 pwm-unit

# ✅ 驗證
Ch19 驗證通過 ✅ (7/7)
```

(第一行的路徑取決於 `CSD_OUT`,此處以預設 `out/` 顯示。)與 Python notebook 的比較:

| 項目 | Python | C++ | 說明 |
|---|---:|---:|---|
| $K_{fit}$ | 2.996 | 3.000 | 雜訊序列不同 |
| $\tau_{fit}$ | 249.9 ms | 248.6 ms | 同上 |
| $k_p$ / $k_i$ | 0.8341 / 3.3373 | 0.8289 / 3.3338 | 由擬合值算出 |
| 上升時間 / OS / 安定 | 0.28 s / 0% / 0.61 s | 0.29 s / 0% / 0.61 s | 飽和主導,幾乎相同 |
| 穩態誤差 | 0.0267 rpm | 0.0249 rpm | |
| 最大差異 = 理論上限 | 5.5450 | 5.6284 | 誤差序列不同,但兩邊都恰好等於上限、0 個超出 |
| float32 韌體 vs double | — | 2.66e-05 | C++ 獨有的檢查 |

## ✅ 驗證條件(7 條)

| # | 條件 | 意義 |
|---|---|---|
| 1 | `|K_fit − K_true| / K_true < 0.05` | 辨識 K 誤差 < 5% |
| 2 | `|tau_fit − tau_true| / tau_true < 0.10` | 辨識 τ 誤差 < 10% |
| 3 | `overshoot_pct < 15` | λ-tuning 閉迴路溫和 |
| 4 | `|steady_state_error| < 5` | 穩態到位(rpm) |
| 5 | `|t(63%) − λ| / λ < 0.5` | 閉迴路時間常數 ≈ λ |
| 6 | `over == 0` | 韌體演算法與 `DiscretePID` 差異全在 $k_i|e|T_s$ 內 |
| 7 | `fw_worst < 1e-3` | 真正的 float32 `pid.h` 與 double 翻譯版一致(C++ 獨有) |

## 動手改改看

- 把 `lam` 改成 `0.05` 與 `0.2`,比較 `ch19_closed_loop.csv` 的三條響應與飽和時間。
- 在閉迴路模擬裡用一個 `Delay`(20 ms = 2 拍)延遲 `u`,λ = 0.05 還穩嗎?
- 用實機:把 ESP32 `STEP,100` 的序列輸出存成 `ch19_telemetry.csv` 格式,改讀那個檔案,直接擬合你的馬達。
- 把 `pid_init` 的 `kd` 改成非零,5b 的差異會突然變大嗎?(double 版 `CStylePID` 也要同步改)

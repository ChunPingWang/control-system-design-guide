# Ch19 快速控制原型(Rapid Control Prototyping, RCP)

> **對應原書**:Ellis, *Control System Design Guide* 4/e, Chapter 19(原書用商用 RCP 硬體;這裡用 **ESP32 + Python** 全開源重現)
> **本章檔案**:[`lab.ipynb`](lab.ipynb) · C++ 版:[`cpp/19-rapid-control-prototyping`](../../cpp/19-rapid-control-prototyping/README.md)
> **前置章節**:[Ch18 運動控制中的觀測器](../18-motion-observer/README.md) · **下一步**:[實機平台 `hardware/`](../../hardware/README.md)(全書最後一章)

## 一句話重點

RCP 是一條「記錄 → 辨識 → 設計 → 模擬驗證 → 下載實測」的閉環工作流程:
即時迴路交給 ESP32,其餘全交給 Python;而且在上機之前,先證明桌上的演算法與要燒進去的韌體**逐樣本等價**。

## 學習目標

1. 走完 RCP 工作流程:遙測記錄、一階系統辨識、λ-tuning、離散閉迴路模擬驗證。
2. 用 `curve_fit` 從開迴路步階擬合 $K/(\tau s+1)$,並理解雜訊對辨識精度的影響。
3. 理解 **λ-tuning(IMC)**:用一個「期望閉迴路時間常數」λ 直接算出 PI 增益。
4. 驗證韌體 PID 與教材 `DiscretePID` 的差異**恰好只有一拍積分量** $k_i e T_s$,不多也不少。

## 理論重點

### RCP 工作流程

```
 ┌──────── ESP32(100 Hz 即時迴路)────────┐        ┌──────────── Python(離線)────────────┐
 │ 編碼器 → 速度 → pid_step() → PWM → 馬達 │ ─CSV─► │ ① 記錄 ② 辨識 K,τ ③ λ-tuning ④ 模擬驗證 │
 │        ▲  SET,KP / SET,KI / STEP,100    │ ◄─命令─ │ ⑤ 韌體等價性(逐樣本比對)           │
 └─────────────────────────────────────────┘        └──────────────────────────────────────┘
```

- 遙測(115200 baud):`millis,target,actual,pwm`
- 命令:`SET,KP,2.0` / `SET,KI,1.0` / `SET,KD,0.05` / `SET,TARGET,500` / `STEP,100`(開迴路 PWM 步階)

### 一階系統辨識

開迴路 PWM 步階 $u_0$ 下,一階受控體的響應為

$$y(t) = K u_0\left(1 - e^{-t/\tau}\right)$$

以非線性最小平方(`scipy.optimize.curve_fit`,Levenberg–Marquardt)對 $K$、$\tau$ 擬合。
$K$ 由穩態值決定(資料多、雜訊平均掉,很準);$\tau$ 由上升段曲率決定(資料點較少,誤差略大)。

### λ-tuning(IMC 調機)

PI $C(s) = k_p\dfrac{\tau_i s + 1}{\tau_i s}$,令 $\tau_i = \tau$ 對消受控體極點:

$$C(s)P(s) = k_p\frac{\tau s+1}{\tau s}\cdot\frac{K}{\tau s+1} = \frac{k_pK}{\tau s} \equiv \frac{1}{\lambda s}
\quad\Rightarrow\quad k_p = \frac{\tau}{K\lambda},\qquad k_i = \frac{k_p}{\tau}$$

閉迴路 $T(s) = \dfrac{1}{\lambda s + 1}$:一階、無 overshoot、63% 時間 = λ。**λ 就是「你想要多快」的旋鈕**。
本章取 λ = 100 ms(約 τ/2.5)。

### 韌體積分順序:差一拍積分量

| | 輸出 | 積分器 |
|---|---|---|
| `DiscretePID`(Python) | $u_k = k_pe_k + k_iI_{k-1}$ | 先算輸出,再 $I_k = I_{k-1} + e_kT_s$ |
| 韌體 `pid.h` | $u_k = k_pe_k + k_iI_k$ | 先 $I_k = I_{k-1} + e_kT_s$,再算輸出 |

兩者的積分狀態 $I$ 完全相同,輸出只差 $k_i e_k T_s$(前向 vs 後向 Euler 積分)。
輸出飽和只會把差異夾小,所以

$$|u^{fw}_k - u^{py}_k| \le k_i\,|e_k|\,T_s$$

## 實驗設計

| 項目 | 設定 |
|---|---|
| 真實馬達 | $K = 3$ rpm/pwm、$\tau = 250$ ms(辨識時「不知道」) |
| RCP 迴路 | ESP32 100 Hz($T_s = 10$ ms) |
| 步驟 1 | 開迴路 PWM 步階 100,記錄 3 s(300 點),量測雜訊 $\mathcal N(0, 3^2)$ rpm,`default_rng(1)` |
| 步驟 2 | 以遙測字串 → `pandas` 解析 → `curve_fit`,初值 $K=1$、$\tau=0.1$ |
| 步驟 3–4 | λ = 0.1 s;`DiscretePID`(±255 PWM 飽和)驅動**真實**受控體,目標 400 rpm,模擬 2 s |
| 步驟 5 | 500 點隨機誤差序列 $\mathcal N(0, 50^2)$(`default_rng(3)`),韌體翻譯版 `CStylePID` vs `DiscretePID(anti_windup=False)` |

## 結果

| 辨識 | 擬合值 | 真值 |
|---|---:|---:|
| $K$ | 2.996 | 3.0 |
| $\tau$ | 249.9 ms | 250 ms |

| λ-tuning / 閉迴路 | 數值 |
|---|---:|
| $k_p$ | 0.8341 pwm/rpm |
| $k_i$ | 3.3373 |
| 上升時間(10–90%) | 0.28 s |
| Overshoot | 0.0% |
| 2% 安定時間 | 0.61 s |
| 穩態誤差 | 0.0267 rpm |

| 韌體等價性 | 數值 |
|---|---:|
| 最大差異 | 5.5450 pwm-unit |
| 理論上限 $\max(k_i\lvert e\rvert T_s)$ | 5.5450 pwm-unit |
| 超出上限的樣本 | 0 / 500 |

- **辨識誤差 < 0.2%**:300 點、雜訊 3 rpm 相對穩態 300 rpm 只有 1%,一階模型又與真實結構相同,擬合幾乎完美。
- **Overshoot 0%**:λ-tuning 的閉迴路本質上是一階系統。
- **上升時間 0.28 s 比理論 $2.2\lambda = 0.22$ s 慢**:第一拍 $k_p \times 400 \approx 334$ PWM 超過 ±255,
  起步被飽和限制;安定時間 0.61 s 也因此比一階的 $4\lambda = 0.4$ s 長。這正是「先模擬再上機」要抓的東西。
- **最大差異 = 理論上限(到小數第四位)**:差異就是 $k_ie_kT_s$ 這一項,在 $|e|$ 最大的那一拍取到上限;沒有任何樣本超出。

## ✅ 驗證條件

| 條件 | 意義 |
|---|---|
| $|K_{fit} - K|/K < 5\%$ | 辨識增益準確 |
| $|\tau_{fit} - \tau|/\tau < 10\%$ | 辨識時間常數準確 |
| Overshoot < 15% | λ-tuning 閉迴路溫和 |
| $|$穩態誤差$| < 5$ rpm | 積分器讓穩態到位 |
| 達到 63% 目標的時間與 λ 相差 < 50% | 閉迴路時間常數 ≈ λ |
| 所有樣本 $|u^{fw} - u^{py}| \le k_i|e|T_s$ | 韌體與 Python 差異全在理論上限內 |

## 工程意義與常見誤區

- **RCP 的價值是縮短「改參數 → 看結果」的迴圈**:增益用序列埠命令即時下發,不必重新編譯燒錄。
- **辨識模型只要「夠用」**:一階模型捕捉主極點即可設計 PI;若擬合殘差不是白雜訊(練習 2),代表漏掉了動態(電氣極點、延遲、死區)。
- **λ 不能無限縮小**:λ 越小 $k_p$ 越大,PWM 飽和、量測雜訊、序列埠與排程延遲(練習 3,對照第 4 章)都會浮現。經驗上 λ 取 τ/3 ~ τ 較穩健。
- **「演算法一樣」不等於「程式碼一樣」**:積分順序、導數濾波、anti-windup、浮點精度,每一項都會讓實機與模擬不同。
  本章把差異**量化到一個可證明的上限**,這才叫等價性驗證。
- **`pid.h` 與 `DiscretePID` 的已知差異**:(1)積分先更新再算輸出;(2)導數不濾波(練習 4)。
  此外讀程式碼可發現 `pid.h` 沒有 anti-windup 積分夾限 —— 這也是本章比對時把 `DiscretePID` 設成 `anti_windup=False` 的原因;
  實機大步階時要留意積分飽和。
- **韌體用 float32**:本 notebook 比對的是 double 翻譯版;C++ 版另外直接 `#include` 真正的 `pid.h` 以 float32 驗證,
  並由 `hardware/esp32/test_host/` 在 host 端逐樣本比對。

## 上實機(硬體到位後)

```bash
cd ../../hardware/esp32
pio run -t upload && pio device monitor        # 燒錄 + 看遙測
```

1. `STEP,100` 記錄開迴路步階 → 用本 notebook 步驟 2 擬合你的馬達。
2. 把算出的 kp/ki 用 `SET,KP,…`/`SET,KI,…` 下發,`SET,TARGET,400` 看階躍。
3. 用第 2 章的 DSA:把 chirp 疊加在 target 上,量實機閉迴路 FRF。

## 練習

1. 把 λ 掃 0.05/0.1/0.2,模擬三條 step response —— λ 就是「你想要多快」的旋鈕。
2. 對擬合殘差畫直方圖,雜訊是白的嗎?如果不是,漏掉了什麼動態?
3. 在模擬加入 20 ms 的傳輸延遲(序列埠+排程),λ=0.05 還穩嗎?對照第 4 章。
4. 讀 `hardware/esp32/src/pid.h`,找出它與 `DiscretePID` 的第二個差異(提示:導數濾波)。

## 執行

```bash
jupyter lab python/19-rapid-control-prototyping/lab.ipynb   # 互動執行
./python/run_all.sh                                         # 批次驗證全部章節
```

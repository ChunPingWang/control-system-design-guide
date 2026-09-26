# Control System Design Guide --- 19 章現代 Maker 實驗教材

**副標題：以 Python / Jupyter / python-control / SciPy / ESP32 取代
Visual ModelQ 與 LabVIEW 的完整實作路線**

版本：1.0\
對應書籍：George Ellis, *Control System Design Guide: Using Your
Computer to Understand and Diagnose Feedback Controllers*, 4th Edition.

> 本教材是獨立撰寫的 companion lab
> manual，不重製原書正文、圖表或解答。章名與實驗主題依第 4
> 版公開目錄對齊；請搭配合法取得的原書閱讀。

## 使用方式

每章依序完成：**理論 → Python simulation → 圖表觀察 → 練習題 →
原書實驗概念對照 → ESP32/實體實驗 → Lab Report**。

### 共用軟體

``` bash
python -m venv .venv
source .venv/bin/activate       # Windows: .venv\Scripts\activate
pip install numpy scipy matplotlib pandas control ipywidgets pyserial jupyterlab
jupyter lab
```

### 共用硬體

-   ESP32 DevKit
-   低壓 DC gear motor + quadrature encoder
-   合適的低壓 H-bridge motor driver（例如 TB6612FNG 類）
-   獨立低壓 DC 電源
-   麵包板/導線（僅低功率訊號）
-   選配：current sensor、MPU6050、彈性聯軸/小型機構

### 安全原則

-   馬達與驅動實驗限低壓 DC；不要直接操作市電。
-   首次啟動先設定 PWM/output limit，確認正反轉與 encoder polarity。
-   高速旋轉件加護罩；不要以手直接對高速軸施加負載。
-   Firmware 必須有 stop/timeout；PC 斷線時輸出回安全狀態。
-   RCP 中真正的 fixed-period control loop 放在 ESP32；Python PC 端負責
    supervisory control、logging 與 plotting。

### 共用量測欄位

``` text
timestamp,setpoint,measurement,error,control_output,p_term,i_term,d_term,saturated
```

### 共用評估指標

-   Rise time
-   Peak / overshoot
-   Settling time
-   Steady-state error
-   RMS error / RMSE
-   Control effort
-   Sampling period / jitter

------------------------------------------------------------------------

# Chapter 1 --- Introduction to Controls

## 1. 學習目標

控制系統、Plant、Controller、Feedback；以一階馬達近似建立第一個閉迴路。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

G(s)=1/(τs+1)。閉迴路的核心是以量測輸出 y 與目標 r 的誤差 e=r-y
驅動控制器。先把「控制」理解成資訊流：Setpoint → Error → Controller →
Plant → Sensor → Feedback。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立 Notebook：`ch01_introduction_to_controls.ipynb`

``` python
import numpy as np
import matplotlib.pyplot as plt
import control as ct

G = ct.tf([1], [0.5, 1])
K = 2.0
T = ct.feedback(K*G, 1)
t, y = ct.step_response(T)
plt.plot(t, y, label="closed loop")
plt.axhline(1, ls="--", label="setpoint")
plt.xlabel("Time (s)"); plt.ylabel("Output"); plt.grid(); plt.legend(); plt.show()
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：控制系統、Plant、Controller、Feedback；以一階馬達近似建立第一個閉迴路。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

把 Visual ModelQ 的第一個控制方塊圖改成 `python-control` 的
`tf()`、`feedback()` 與 `step_response()`；重點不是重畫
UI，而是驗證相同的 input/plant/controller/feedback 關係。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

Potentiometer 作 setpoint、ESP32 ADC 讀值、PWM 控制 LED 或低功率 DC
motor。第一週只做 open-loop，再加入 encoder 才形成真正 feedback。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# Chapter 2 --- The Frequency Domain

## 1. 學習目標

Laplace、Transfer Function、Pole/Zero、Bode、Step
Response、頻域與時域的關係。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

對 LTI 系統，Transfer Function G(s)=Y(s)/U(s)
將微分方程轉成代數關係。Pole 決定自然動態；Zero 改變響應形狀。Bode
圖用頻率掃描觀察 magnitude 與 phase，是後續穩定度與 tuning 的共同語言。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立 Notebook：`ch02_the_frequency_domain.ipynb`

``` python
import numpy as np
import matplotlib.pyplot as plt
import control as ct

G1 = ct.tf([1], [1, 1])
G2 = ct.tf([25], [1, 4, 25])
print("poles G2:", ct.poles(G2))
ct.bode_plot(G2, dB=True, grid=True)
plt.show()
t, y = ct.step_response(G2)
plt.plot(t, y); plt.grid(); plt.xlabel("Time (s)"); plt.ylabel("y"); plt.show()
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：Laplace、Transfer
    Function、Pole/Zero、Bode、Step Response、頻域與時域的關係。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

對照原書的 transfer-function、block diagram、phase/gain 與 Bode
實驗；Python 版要求同一 Plant 同時畫 step 與
Bode，並寫下兩種觀點對同一動態的解釋。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

對 DC motor 做多組 PWM step，記錄 RPM。先不做正式 frequency sweep；用
step response 估 time constant，為後續建立 motor model 做準備。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# Chapter 3 --- Tuning a Control System

## 1. 學習目標

開迴路、閉迴路、Gain/Phase Margin、Loop Gain、飽和、Cascade。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

Loop transfer
L(s)=C(s)G(s)。穩定度不只看閉迴路「現在會不會震」，還要看離失穩還有多少餘裕。Gain
margin 與 phase margin 是實務 tuning 的重要診斷量；調參時要避免 actuator
saturation 掩蓋真實 loop dynamics。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立 Notebook：`ch03_tuning_a_control_system.ipynb`

``` python
import control as ct
import matplotlib.pyplot as plt
G = ct.tf([1], [0.2, 1, 0])
for K in [0.5, 2, 8]:
    L = K*G
    gm, pm, wcg, wcp = ct.margin(L)
    print(K, "GM=", gm, "PM=", pm)
    t,y = ct.step_response(ct.feedback(L,1))
    plt.plot(t,y,label=f"K={K}")
plt.grid(); plt.legend(); plt.show()
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：開迴路、閉迴路、Gain/Phase Margin、Loop
    Gain、飽和、Cascade。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

對照原書 open-loop method、stability margins 與 zone-based tuning；用
`margin()` 量化，而不是只靠肉眼看曲線。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

Encoder motor speed loop：逐步提高 Kp，記錄 overshoot、settling time
與是否 oscillate。設定 PWM clamp，並把 saturation 狀態一起送回 PC。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# Chapter 4 --- Delay in Digital Controllers

## 1. 學習目標

Sampling、sample-and-hold、calculation delay、sample time 選擇。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

數位控制器一定有延遲。取樣、計算、通訊與輸出更新都會增加 phase lag；loop
越快，固定延遲佔一個週期的比例越高。實驗重點是「同一 controller，只改
sample time/額外 delay」會發生什麼。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立 Notebook：`ch04_delay_in_digital_controllers.ipynb`

``` python
import control as ct
import matplotlib.pyplot as plt
G = ct.tf([1], [0.15, 1])
C = 5
for Ts in [0.002,0.01,0.05]:
    Gd = ct.sample_system(G, Ts, method="zoh")
    Td = ct.feedback(C*Gd,1)
    t,y = ct.step_response(Td, T=3)
    plt.step(t,y,where="post",label=f"Ts={Ts}s")
plt.grid(); plt.legend(); plt.show()
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：Sampling、sample-and-hold、calculation
    delay、sample time 選擇。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

直接重做原書 Experiment 4A 的精神：固定 Plant/Controller，改 digital
delay/sample time，比較穩定度與 command response。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

ESP32 用 `micros()` 建固定週期 loop。分別跑 1/5/10/50 ms；記錄實際 loop
period、jitter、RPM response。不要用長時間 `delay()` 當正式控制排程。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# Chapter 5 --- The z-Domain

## 1. 學習目標

z-domain、aliasing、離散化、quantization、從 transfer function 到
algorithm。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

z-domain 是離散時間系統的自然表示。Sampling frequency 不足會
alias；ADC、encoder、PWM 都有 quantization。控制器最後必須從數學表示落到
difference equation。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立 Notebook：`ch05_the_z-domain.ipynb`

``` python
import numpy as np, control as ct, matplotlib.pyplot as plt
G = ct.tf([1],[0.2,1])
for Ts in [0.005,0.02,0.1]:
    Gd=ct.c2d(G,Ts,method="zoh")
    print("Ts",Ts, Gd)
# aliasing demo
fs=50; t=np.arange(0,1,1/fs)
for f in [10,40]:
    plt.plot(t,np.sin(2*np.pi*f*t),'.-',label=f"{f} Hz")
plt.legend(); plt.grid(); plt.show()
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：z-domain、aliasing、離散化、quantization、從
    transfer function 到 algorithm。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

對照原書 z phasor、aliasing、digital functions、calculation
delay、quantization。Python
版額外把離散模型係數印出來，要求學生能說明如何轉成 MCU difference
equation。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

以 encoder pulse count 示範低速量測 quantization；改變 measurement
window，觀察 resolution 與 latency trade-off。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# Chapter 6 --- Four Types of Controllers

## 1. 學習目標

P、I、D、PI/PD/PID 的角色與 tuning。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

P 對現在誤差反應；I 累積誤差以消除 steady-state error；D
對變化率反應、可增加 damping，但對 noise 敏感。實作時 derivative 常需要
filter，integrator 需要 anti-windup。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立 Notebook：`ch06_four_types_of_controllers.ipynb`

``` python
import control as ct, matplotlib.pyplot as plt
G=ct.tf([1],[0.4,1])
controllers={
"P":ct.tf([2],[1]),
"PI":2+1.5/ct.tf([1,0],[1]),
"PD":2+0.08*ct.tf([1,0],[1]),
"PID":2+1.5/ct.tf([1,0],[1])+0.08*ct.tf([1,0],[1])
}
for name,C in controllers.items():
    t,y=ct.step_response(ct.feedback(C*G,1),T=5)
    plt.plot(t,y,label=name)
plt.grid(); plt.legend(); plt.show()
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：P、I、D、PI/PD/PID 的角色與 tuning。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

對照原書 Experiments 6A--6D：每次只改一個 control action，建立 P/I/D
的因果直覺；不要直接用 auto-tune 跳過比較。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

完成 DC Motor Speed PID。Serial
CSV：`t,setpoint,rpm,error,pwm,p,i,d,saturated`。加入 output
clamp、integral clamp、derivative low-pass。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# Chapter 7 --- Disturbance Response

## 1. 學習目標

負載擾動、command response 與 disturbance response 的差異、disturbance
decoupling。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

好的 setpoint tracking 不代表好的 disturbance rejection。負載 torque 是
motor control 最直觀的 disturbance；應分開測 command step 與 load step。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立 Notebook：`ch07_disturbance_response.ipynb`

``` python
import control as ct, matplotlib.pyplot as plt
G=ct.tf([1],[0.3,1]); C=ct.tf([3,6],[1,0])
S=ct.feedback(1,C*G)       # sensitivity
T=ct.feedback(C*G,1)
t,yc=ct.step_response(T,T=4)
_,yd=ct.step_response(G*S,T=4)
plt.plot(t,yc,label="command")
plt.plot(t,yd,label="plant-input disturbance")
plt.grid(); plt.legend(); plt.show()
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：負載擾動、command response 與 disturbance
    response 的差異、disturbance decoupling。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

對照原書 velocity controller disturbance response 與 decoupling；Python
用 sensitivity function 清楚區分 reference path 與 disturbance path。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

馬達穩定轉速後施加可重複負載（例如小摩擦輪/固定機構，不用手碰高速旋轉件），量測
RPM dip、recovery time、最大 error。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# Chapter 8 --- Feed-Forward

## 1. 學習目標

Plant-based feed-forward、converter compensation、command
delay、double-integrating plant。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

Feedback 是看到 error 後修正；feed-forward 利用已知 plant/command
預先提供所需 control effort。Feed-forward 不應取代 feedback，因為 model
mismatch 與 disturbance 仍存在。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立 Notebook：`ch08_feed-forward.ipynb`

``` python
import numpy as np, control as ct, matplotlib.pyplot as plt
G=ct.tf([1],[0.3,1]); C=ct.tf([3,6],[1,0])
Tfb=ct.feedback(C*G,1)
# simple static feed-forward F=1 for plant DC gain=1
Tff=ct.feedback(C*G,1) + G*ct.feedback(1,C*G)
t,y1=ct.step_response(Tfb,T=3); _,y2=ct.step_response(Tff,T=3)
plt.plot(t,y1,label="feedback"); plt.plot(t,y2,label="feedback+FF")
plt.grid(); plt.legend(); plt.show()
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：Plant-based feed-forward、converter
    compensation、command delay、double-integrating plant。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

對照原書 plant-based feed-forward；重點放在 model accuracy、power
converter 與 command shaping 對效果的影響。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

由 open-loop 實驗建立 `target_rpm → baseline_pwm` lookup/linear
model，再讓 PID 只修正 residual error；比較 PID-only 與 FF+PID。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# Chapter 9 --- Filters in Control Systems

## 1. 學習目標

Low-pass、passband、noise vs phase lag、數位濾波。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

Filter 能降低 sensor noise，但會引入 phase lag。控制迴路中的 filter
不是「越平滑越好」；必須同時看 noise attenuation 與 stability margin。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立 Notebook：`ch09_filters_in_control_systems.ipynb`

``` python
import numpy as np, scipy.signal as sig, matplotlib.pyplot as plt
fs=200; t=np.arange(0,3,1/fs)
x=1000+30*np.sin(2*np.pi*30*t)+10*np.random.default_rng(1).normal(size=len(t))
fc=10
b,a=sig.butter(2,fc/(fs/2))
y=sig.lfilter(b,a,x)
plt.plot(t,x,alpha=.4,label="raw"); plt.plot(t,y,label="filtered")
plt.xlim(0,.5); plt.grid(); plt.legend(); plt.show()
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：Low-pass、passband、noise vs phase
    lag、數位濾波。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

對照原書 filter passband 與 implementation；Python 版同時畫 raw/filtered
time series，並用 Bode/frequency response 檢查截止頻率。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

ESP32 實作一階 IIR：`y += alpha*(x-y)`；記錄 raw RPM 與 filtered
RPM。逐步降低 cutoff，觀察控制是否因延遲而變差。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# Chapter 10 --- Introduction to Observers in Control Systems

## 1. 學習目標

Observer、Luenberger Observer、state estimation、observer tuning。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

Observer 用 model、input 與有限 measurement 估計無法直接或不宜直接量測的
state。Luenberger observer 的 error dynamics 由 A-LC 決定；observer
poles 通常需比主要 plant dynamics 快，但太快會放大 noise/model
mismatch。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立 Notebook：`ch10_introduction_to_observers_in_control_systems.ipynb`

``` python
import numpy as np, control as ct, matplotlib.pyplot as plt
A=np.array([[0,1],[0,-2.]])
B=np.array([[0],[2.]])
C=np.array([[1,0]])
L=ct.place(A.T,C.T,[-6,-7]).T
print("Observer gain L=\n",L)
print("Observer poles:",np.linalg.eigvals(A-L@C))
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：Observer、Luenberger Observer、state
    estimation、observer tuning。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

對照原書 Experiments 10A--10C 與 Luenberger observer 設計；Python
版先驗證 eigenvalues，再做 noisy measurement simulation。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

先不把 observer 放進 safety-critical loop。用 encoder position 作
measurement，離線比較 finite-difference velocity 與 observer-estimated
velocity；確認後才用於控制。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# Chapter 11 --- Introduction to Modeling

## 1. 學習目標

模型目的、frequency-domain modeling、time-domain modeling。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

Model
不是越複雜越好，而是要足以回答工程問題。控制設計常先用低階模型，再用實測驗證。Frequency-domain
model 適合 loop/stability；time-domain model 適合
transient、nonlinearity 與 implementation。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立 Notebook：`ch11_introduction_to_modeling.ipynb`

``` python
import control as ct, numpy as np, matplotlib.pyplot as plt
G=ct.tf([1],[0.25,1])
t=np.linspace(0,2,500)
_,y=ct.step_response(G,t)
plt.plot(t,y,label="1st-order model")
plt.grid(); plt.legend(); plt.show()
print("DC gain:",ct.dcgain(G),"poles:",ct.poles(G))
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：模型目的、frequency-domain
    modeling、time-domain modeling。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

對照原書 frequency-domain 與 time-domain modeling；同一 plant
必須用兩種方式描述並說明各自能回答什麼問題。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

做 motor open-loop step test。固定供電、不同 PWM，記錄 RPM。估 DC gain
與 time constant，建立第一個可辨識的 motor model。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# Chapter 12 --- Nonlinear Behavior and Time Variation

## 1. 學習目標

LTI vs non-LTI、saturation、deadband、friction、backlash、time
variation。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

真實機器常不是 LTI：PWM 有死區、driver 有 saturation、齒輪有
backlash、摩擦有 stiction、電池電壓會變。這些效應若被忽略，simulation
可能漂亮但實機失敗。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立 Notebook：`ch12_nonlinear_behavior_and_time_variation.ipynb`

``` python
import numpy as np, matplotlib.pyplot as plt
u=np.linspace(-1,1,401)
dead=.15
y=np.where(np.abs(u)<dead,0,np.sign(u)*(np.abs(u)-dead))
ysat=np.clip(2*y,-1,1)
plt.plot(u,y,label="deadband"); plt.plot(u,ysat,label="+ saturation")
plt.grid(); plt.legend(); plt.xlabel("command"); plt.ylabel("effective input"); plt.show()
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：LTI vs
    non-LTI、saturation、deadband、friction、backlash、time variation。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

對照原書十類 nonlinear behavior 的分析方式；Python 版要求每個非線性用
function/block 明確建模，而不是把誤差歸咎於「noise」。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

量 motor 起轉 PWM、正反轉 deadband、最大 RPM、低速 stiction。建立
deadband compensation，但保留 output safety clamp。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# Chapter 13 --- Model Development and Verification

## 1. 學習目標

七步建模流程、驗證、simulation→deployment、RCP/HIL。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

模型必須用未參與 fitting 的資料驗證。基本流程：定義用途→選
states/inputs/outputs→建立結構→量測參數→simulate→compare→revise。Validation
比「模型能跑」重要。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立 Notebook：`ch13_model_development_and_verification.ipynb`

``` python
import numpy as np, matplotlib.pyplot as plt
from scipy.optimize import curve_fit
t=np.linspace(0,2,100)
rng=np.random.default_rng(2)
true=1200*(1-np.exp(-t/.28))
meas=true+rng.normal(0,18,len(t))
def f(t,K,tau): return K*(1-np.exp(-t/tau))
p,_=curve_fit(f,t,meas,p0=[1000,.2])
pred=f(t,*p)
print("K,tau=",p)
plt.scatter(t,meas,s=10,label="measurement"); plt.plot(t,pred,label="model")
plt.grid(); plt.legend(); plt.show()
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：七步建模流程、驗證、simulation→deployment、RCP/HIL。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

對照原書 seven-step process 與 RCP/HIL；Python 版加入 train/validation
runs，禁止只用同一筆資料 fitting 後宣稱驗證成功。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

收兩次 motor step test：Run A 估 K/τ，Run B 驗證。計算
RMSE、steady-state error；若差異大，檢查電源、負載、friction、sampling。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# Chapter 14 --- Encoders and Resolvers

## 1. 學習目標

accuracy、resolution、response、encoder/resolver、velocity
estimation、cyclic error。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

Encoder position 是量化值；velocity 通常是由 position 差分或 pulse
timing 推估，因此低速時 resolution/noise
問題特別明顯。Accuracy、resolution、repeatability 不應混為一談。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立 Notebook：`ch14_encoders_and_resolvers.ipynb`

``` python
import numpy as np, matplotlib.pyplot as plt
cpr=400; rpm=30; Ts=.01
counts_per_sample=rpm/60*cpr*Ts
print("ideal counts/sample:",counts_per_sample)
t=np.arange(0,1,Ts)
pos=np.round((rpm/60*cpr)*t)
vel=np.diff(pos,prepend=pos[0])/cpr/Ts*60
plt.step(t,vel,where="post"); plt.axhline(rpm,ls="--")
plt.ylabel("Estimated RPM"); plt.xlabel("Time"); plt.grid(); plt.show()
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：accuracy、resolution、response、encoder/resolver、velocity
    estimation、cyclic error。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

對照原書 position resolution、velocity estimation/noise 與 cyclical
errors；Python 版可精確控制 CPR、Ts，觀察 quantization。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

使用 ESP32 hardware pulse counter/interrupt 讀 quadrature encoder。比較
fixed-window count 與 pulse-period method；低速時特別記錄 zero-count
問題。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# Chapter 15 --- Basics of the Electric Servomotor and Drive

## 1. 學習目標

Drive、servo system、PM brush/brushless motor、torque、back EMF、drive
dynamics。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

電機不是單純 gain。簡化 DC motor：V=Ri+L di/dt+Keω；J dω/dt=Kt
i-Bω-τload。電氣與機械 dynamics 共同決定 response。Maker 階段可先用
brushed DC gearmotor，再理解 brushless servo。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立 Notebook：`ch15_basics_of_the_electric_servomotor_and_drive.ipynb`

``` python
import numpy as np, control as ct, matplotlib.pyplot as plt
R,L,Kt,Ke,J,B=2.0,0.01,0.08,0.08,0.002,0.002
A=np.array([[-R/L,-Ke/L],[Kt/J,-B/J]])
Bv=np.array([[1/L],[0]])
C=np.array([[0,1]]); D=np.array([[0]])
sys=ct.ss(A,Bv,C,D)
t,y=ct.step_response(sys,T=1)
plt.plot(t,y); plt.grid(); plt.xlabel("s"); plt.ylabel("rad/s per volt step"); plt.show()
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：Drive、servo system、PM brush/brushless
    motor、torque、back EMF、drive dynamics。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

對照原書 servo motor/drive 基礎；Python 版以 state-space 將 current 與
speed 都保留，方便之後做 observer/cascade。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

量供電電壓、PWM、RPM；若有合適 current sensor 可額外記
current，但不要求初學者自行接觸高功率/市電。所有 motor 實驗使用低壓 DC。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# Chapter 16 --- Compliance and Resonance

## 1. 學習目標

柔性負載、兩慣量系統、resonance/anti-resonance、notch、降低 bandwidth。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

馬達與負載若透過彈性聯軸器/皮帶連接，不能永遠視為 rigid body。兩個
inertia 加 spring/damper 會形成 resonance；提高 gain 可能激發機械模態。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立 Notebook：`ch16_compliance_and_resonance.ipynb`

``` python
import numpy as np, control as ct, matplotlib.pyplot as plt
Jm,Jl,k,b=.002,.006,20,.03
A=np.array([[0,1,0,0],[-k/Jm,-b/Jm,k/Jm,b/Jm],
            [0,0,0,1],[k/Jl,b/Jl,-k/Jl,-b/Jl]])
B=np.array([[0],[1/Jm],[0],[0]])
C=np.array([[0,1,0,0]]); D=np.zeros((1,1))
sys=ct.ss(A,B,C,D)
ct.bode_plot(sys,dB=True,grid=True); plt.show()
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：柔性負載、兩慣量系統、resonance/anti-resonance、notch、降低
    bandwidth。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

對照原書 tuned resonance、inertial-reduction instability 與 curing
resonance；Python 版改 J、k、b 觀察 resonance peak 移動。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

硬體延伸採低能量柔性 coupling demo；若沒有安全機構，以 simulation
為主。可在 encoder motor + elastic belt
小車上量測振動，不做高速裸露旋轉實驗。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# Chapter 17 --- Position-Control Loops

## 1. 學習目標

P/PI、PI/P、PID position loop、profile generation、Bode。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

位置控制常是外位置迴路 + 內速度迴路。不同 controller placement 會影響
command response、disturbance response 與 overshoot。Profile generator
可避免不合理的瞬間速度/加速度命令。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立 Notebook：`ch17_position-control_loops.ipynb`

``` python
import control as ct, matplotlib.pyplot as plt
# simplified velocity plant -> position adds integrator
Gv=ct.tf([1],[0.15,1])
Gp=Gv*ct.tf([1],[1,0])
for Kp in [2,5,10]:
    T=ct.feedback(Kp*Gp,1)
    t,y=ct.step_response(T,T=4)
    plt.plot(t,y,label=f"Kp={Kp}")
plt.grid(); plt.legend(); plt.ylabel("position"); plt.show()
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：P/PI、PI/P、PID position loop、profile
    generation、Bode。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

對照原書 P/PI、PI/P、PID 與 position profile；Python 版先以簡化 plant
比較架構，再用 Ch.15 motor model 重做。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

ESP32 建 position setpoint（encoder count）。先限制 max PWM，再加入
trapezoidal position profile；測 overshoot、settling、final position
error。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# Chapter 18 --- Using the Luenberger Observer in Motion Control

## 1. 學習目標

用 observer 估 velocity、降低差分相位延遲、acceleration feedback。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

直接對 noisy position 做差分會放大 noise；強低通又增加 lag。Observer
可利用 plant model 在 noise 與 lag 間取得不同折衷。關鍵是 model quality
與 observer bandwidth。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立
Notebook：`ch18_using_the_luenberger_observer_in_motion_control.ipynb`

``` python
import numpy as np, control as ct
A=np.array([[0,1],[0,-5.]])
B=np.array([[0],[5.]])
C=np.array([[1,0]])
for poles in [[-10,-12],[-30,-35]]:
    L=ct.place(A.T,C.T,poles).T
    print("target",poles,"L=",L.ravel(),
          "actual",np.linalg.eigvals(A-L@C))
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：用 observer 估
    velocity、降低差分相位延遲、acceleration feedback。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

對照原書 velocity observation 與 acceleration feedback；Python 版比較
slow/fast observer poles 對 noise sensitivity 的影響。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

以 encoder position + motor command 餵 observer，Serial 同時送
`velocity_fd`、`velocity_filtered`、`velocity_observer`。先離線比較，再決定是否進
feedback loop。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# Chapter 19 --- Rapid Control Prototyping (RCP) for a Motion System

## 1. 學習目標

RCP、rigid/compliant load、PC tooling、parameter tuning、logging、HIL
思維。

完成本章後，你應該能把概念分成三層：**物理現象、數學/模型表示、可量測的實驗結果**。

## 2. 理論說明

RCP 的價值是縮短 model→controller→experiment→diagnosis 的迭代。現代
Maker 版採 ESP32 執行 deterministic control；Python/Jupyter
做設定、資料擷取、視覺化與報告。PC 不應透過一般 USB serial 承擔硬即時
motor loop。

### 必須回答的工程問題

1.  本章的 Plant、Input、Output、Measurement 分別是什麼？
2.  哪些假設屬於 LTI？真實硬體可能在哪裡違反？
3.  哪一張圖最能證明 controller/模型真的改善，而不是「感覺比較順」？
4.  哪些參數改變會影響 stability、noise 或 performance？

## 3. Python Simulation Lab

建立
Notebook：`ch19_rapid_control_prototyping_(rcp)_for_a_motion_system.ipynb`

``` python
# PC-side telemetry skeleton
import serial, csv, time
# ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
# with open("run.csv","w",newline="") as f:
#     w=csv.writer(f); w.writerow(["t","setpoint","actual","pwm"])
#     t0=time.time()
#     while time.time()-t0 < 10:
#         row=ser.readline().decode(errors="ignore").strip().split(",")
#         if len(row)==3:
#             w.writerow([time.time()-t0,*row])
print("Keep the real-time loop on the ESP32; use Python for supervisory control.")
```

### 圖表要求

Notebook 不只要能執行，至少要留下：

-   清楚的 title / x-label / y-label / legend / grid。
-   Setpoint 與 response（適用時）。
-   改參數前後的 overlay comparison。
-   對 frequency-domain 章節保留 Bode/相關頻域圖。
-   對硬體資料保留 raw 與 processed data，避免只保存截圖。

### 實驗紀錄

``` text
Hypothesis:
Parameters:
What changed:
Observed result:
Measured metrics:
Why:
Next experiment:
```

## 4. 練習題

1.  用自己的話解釋本章的核心：RCP、rigid/compliant load、PC
    tooling、parameter tuning、logging、HIL 思維。
2.  至少改變兩個參數，先寫下預測，再執行 simulation；比較預測與結果。
3.  從圖表量出至少兩個可量化指標，不接受只寫「看起來比較好」。
4.  列出 simulation 與真實硬體之間至少三個 model mismatch 來源。
5.  提出一個故障或反例，說明本章方法在什麼條件下可能失效。

## 5. 與原書實驗/主題的對照

對照原書以 LabVIEW/RCP 驗證 rigidly- 與 compliantly-coupled
loads；本教材用 Python telemetry + ESP32
firmware，保留相同工程目的而非複製 GUI。

> 原則：重現**控制工程問題與可觀察現象**，而不是逐像素模仿 Visual ModelQ
> / LabVIEW。若原書參數與本教材示例不同，以原書參數另開一個 notebook
> cell 重做比較。

## 6. ESP32 / Maker Lab

完成 Mini RCP：PC 可設定 setpoint/Kp/Ki/Kd、開始/停止 run、下載
CSV；ESP32 固定週期 PID。加入 emergency stop/command timeout/output
limit。

### Firmware 共通骨架

``` cpp
const uint32_t CONTROL_US = 10000; // 例：10 ms，依章節實驗調整
uint32_t nextTick;

void setup() {
  Serial.begin(115200);
  nextTick = micros();
  // init ADC / encoder / PWM / motor driver
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += CONTROL_US;

    // 1. read sensor / encoder
    // 2. estimate state / filter if required
    // 3. calculate controller
    // 4. clamp output + safety checks
    // 5. update PWM
    // 6. emit CSV telemetry
  }

  // non-real-time command parsing can be handled separately
}
```

### 必交資料

-   Firmware commit/hash
-   `run_*.csv`
-   Python analysis notebook
-   至少一張 response plot
-   Parameters/BOM/接線說明
-   一段「simulation vs hardware」差異分析

## 7. 完成本章的 Exit Criteria

-   [ ] 我能不用背公式，解釋本章的物理意義。
-   [ ] Python notebook 從乾淨 kernel 可完整執行。
-   [ ] 圖表有單位、label 與可比較的 baseline。
-   [ ] 至少做過一次 parameter sweep。
-   [ ] 若本章有硬體實驗，保存了原始 CSV。
-   [ ] 我能指出至少一個 model mismatch。
-   [ ] 我能說明下一章如何建立在本章結果上。

------------------------------------------------------------------------

# 附錄 A --- 共用 PID 實作建議

離散 PID 的基本形式：

``` cpp
error = setpoint - measurement;
integral += error * dt;
derivative = (error - prevError) / dt;

u = Kp * error + Ki * integral + Kd * derivative;
u = constrain(u, -outputLimit, outputLimit);

prevError = error;
```

正式實驗應逐步加入：

1.  output saturation
2.  integral anti-windup
3.  derivative filtering
4.  derivative-on-measurement（視架構需要）
5.  bumpless parameter update
6.  fixed sample time
7.  command timeout / safe stop

# 附錄 B --- Python 硬體資料分析模板

``` python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("run_001.csv")

plt.plot(df["timestamp"], df["setpoint"], label="setpoint")
plt.plot(df["timestamp"], df["measurement"], label="measurement")
plt.xlabel("Time (s)")
plt.ylabel("Output")
plt.grid()
plt.legend()
plt.show()

df["error"] = df["setpoint"] - df["measurement"]
rmse = (df["error"].pow(2).mean()) ** 0.5
print("RMSE:", rmse)
```

# 附錄 C --- 每章 Lab Report 模板

``` markdown
# Lab N — Title

## Objective
## System / Block Diagram
## Hypothesis
## Model and Assumptions
## Parameters
## Simulation
## Hardware Setup
## Raw Data
## Results
## Metrics
## Simulation vs Hardware
## Diagnosis
## Conclusion
## Next Experiment
```

# 附錄 D --- 最終整合專案

完成 Chapter 19 後，把成果整合成：

``` text
Python/Jupyter
    │
    ├── Parameter configuration
    ├── Experiment runner
    ├── CSV logging
    ├── Plot / metrics
    │
  Serial
    │
  ESP32
    ├── Fixed-rate scheduler
    ├── Encoder
    ├── Filter / Observer
    ├── PID / Cascade
    ├── Safety clamp
    └── PWM
      │
 Motor Driver
      │
 Motor + Encoder + Load
```

## Capstone A：DC Motor Control Bench

必須展示：

-   Open loop
-   P / PI / PID
-   Sampling effect
-   Disturbance rejection
-   Feed-forward
-   Filter
-   Model identification
-   Position control
-   Observer
-   RCP logging

## Capstone B：Self-Balancing Robot（延伸）

在 Motor Control Bench 穩定後再進入：

``` text
IMU → Sensor Fusion → Tilt Estimate → Controller
                                  ↓
                         Left/Right Motor
                                  ↑
                              Encoders
```

此專案將前 19 章的 sampling、filter、model、PID、observer、motor
drive、telemetry 與 diagnosis 整合在同一個 real-time system。

# 建議學習節奏

每章 1--2 週；Chapter 6、10、13、16--19 建議各保留 2
週。不要以「讀完章節」作為進度，而以 **Notebook 可重現 + Hardware data
可分析 + Exit Criteria 通過** 作為進度。

完成整套教材後，你應能從：

**Plant → Model → Simulation → Controller → Discretization → Embedded
Implementation → Measurement → Diagnosis → Model Revision**

完整走過一次控制系統工程循環。

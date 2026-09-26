# Maker 自動控制完整學習計畫

> **主軸：從 Maker 實作進入 Feedback Control，再走到 Digital
> Control、Motor Control、Observer 與 Self-Balancing Robot**
>
> 建議對象：已有程式設計能力，希望以 Arduino / ESP32、Python
> 與實體硬體真正理解自動控制的人。\
> 建議週期：**24 週（約 6 個月）**\
> 建議投入：每週 **5--7 小時**，以「30% 閱讀、20% 模擬、50%
> 實作」為原則。

------------------------------------------------------------------------

## 1. 學習目標

完成這份計畫後，應能：

1.  理解
    Open-loop、Closed-loop、Feedback、Plant、Controller、Sensor、Disturbance
    等核心概念。
2.  使用 Python 建立 Transfer Function、Step Response、Bode Plot、Root
    Locus 與 State-Space 模型。
3.  理解並實作 P、PI、PD、PID Controller。
4.  理解 Sampling、Discrete-Time Control、z-domain 與數位 PID。
5.  使用 ESP32 / Arduino 控制 DC Motor，讀取
    Encoder，建立速度與位置閉迴路控制。
6.  從實際量測資料建立簡單 Plant Model，並比較模型與真實系統。
7.  理解 Noise、Filter、Feedforward、Disturbance Rejection、Observer。
8.  完成 Motor Speed Control、Servo Position Control 與 Self-Balancing
    Robot。
9.  建立一套 Python + ESP32 的簡易 Rapid Control Prototyping 環境。
10. 能閱讀較正式的控制工程教材，繼續進入 Modern Control、Robotics 或
    Mechatronics。

------------------------------------------------------------------------

# 2. 核心學習策略

不要採用：

> 數學 → 更多數學 → Laplace → 題目 → 考試 → 最後才碰硬體

本計畫採用：

``` text
實際現象
   ↓
Python Simulation
   ↓
控制理論
   ↓
ESP32 / Arduino 實作
   ↓
量測真實資料
   ↓
比較 Model vs Reality
   ↓
修改 Controller
```

每個主題都盡可能經過以下循環：

``` text
Understand → Simulate → Build → Measure → Diagnose → Improve
```

------------------------------------------------------------------------

# 3. 建議教材

## 3.1 主教材

### Control System Design Guide --- George Ellis

**Control System Design Guide: Using Your Computer to Understand and
Diagnose Feedback Controllers, 4th Edition**

定位：

-   本計畫的控制系統主教材
-   重視工程直覺與 Simulation
-   適合搭配 Python 取代原書 Visual ModelQ
-   後半部非常適合 Motor / Encoder / Servo / Observer 實作

本計畫會將原書軟體環境替換為：

``` text
Visual ModelQ → Python + python-control + SciPy
LabVIEW       → Python + Jupyter + PySerial
RCP           → Python + ESP32
```

------------------------------------------------------------------------

## 3.2 Maker 參考教材

### Arduino Workshop, 2nd Edition --- John Boxall

用途：

-   Arduino / MCU 基礎
-   Sensor / Actuator
-   Motor
-   電子實作
-   快速建立硬體直覺

不需要逐章讀完；把它當作硬體入門與實驗參考。

### Arduino Cookbook, 3rd Edition

用途：

-   Sensor
-   Encoder
-   PWM
-   DC Motor
-   H-Bridge
-   Servo
-   Stepper
-   Serial Communication

定位為「Maker 工具書」，遇到硬體問題時查閱。

### Advanced Arduino Techniques in Science

用途：

-   Data Acquisition
-   Python
-   PID
-   Measurement
-   Arduino 與 PC 整合

適合在中期開始閱讀。

------------------------------------------------------------------------

## 3.3 理論補強

### Feedback Systems --- Åström & Murray

當你已經做過 PID 與 Motor Control 後，再用這本補強理論。

### Modern Control Engineering --- Katsuhiko Ogata

不建議作為第一本。

等完成本計畫後，如果希望進一步學：

-   Root Locus
-   State Space
-   Modern Control
-   Control Theory

再系統性閱讀。

------------------------------------------------------------------------

# 4. 軟體環境

## Python

建議：

``` text
Python 3.x
JupyterLab
VS Code
```

主要套件：

``` text
numpy
scipy
matplotlib
pandas
control
ipywidgets
pyserial
```

安裝：

``` bash
python -m venv .venv

source .venv/bin/activate

pip install \
    numpy \
    scipy \
    matplotlib \
    pandas \
    control \
    ipywidgets \
    pyserial \
    jupyterlab
```

------------------------------------------------------------------------

## Embedded

建議：

``` text
VS Code
PlatformIO
ESP32 Arduino Framework
```

ESP32 比傳統 Arduino UNO 更適合作為後續主要控制平台：

-   運算能力較高
-   Timer 資源較完整
-   Serial / Wi-Fi / Bluetooth
-   適合較高頻率 control loop
-   後續容易做 telemetry

但最初幾個簡單實驗也可以使用 Arduino Uno。

------------------------------------------------------------------------

# 5. 建議硬體

## 第一階段

-   ESP32 DevKit
-   Breadboard
-   Jumper wires
-   LED
-   Push buttons
-   Potentiometer
-   Multimeter
-   基本 resistor / capacitor

## Motor Control

-   DC Gear Motor
-   Quadrature Encoder
-   TB6612FNG 或適合的 Motor Driver
-   獨立 DC Power Supply
-   Encoder 適用電源
-   USB cable

## Sensor

-   MPU6050 或其他 IMU
-   HC-SR04（選配）
-   Temperature sensor（選配）

## 後期

-   Self-balancing robot chassis
-   兩顆帶 Encoder 的 Gear Motor
-   Wheels
-   Battery
-   Voltage regulator
-   IMU
-   Motor driver

------------------------------------------------------------------------

# 6. 24 週完整 Roadmap

------------------------------------------------------------------------

# Phase 0 --- 環境與電子基礎

## Week 1：建立 Maker Control Lab

### 學習

了解：

-   Voltage
-   Current
-   Resistance
-   PWM
-   ADC
-   Digital Input / Output
-   Ground
-   MCU

### 實作

完成：

``` text
Potentiometer
      ↓
     ADC
      ↓
    ESP32
      ↓
     PWM
      ↓
     LED
```

### Python

建立第一個 Jupyter Notebook：

``` text
00-python-basics.ipynb
```

練習：

-   NumPy array
-   Matplotlib
-   CSV
-   Pandas

### 完成標準

能讓旋鈕控制 LED 亮度，並能解釋 ADC 與 PWM 的差異。

------------------------------------------------------------------------

# Phase 1 --- Motor 與 Feedback 直覺

## Week 2：DC Motor

### 學習

了解：

-   DC Motor
-   PWM
-   H-Bridge
-   Motor Driver
-   Back EMF
-   Load

### 實作

``` text
ESP32
  ↓ PWM
Motor Driver
  ↓
DC Motor
```

控制：

-   Start
-   Stop
-   Direction
-   Speed

### 關鍵問題

思考：

> PWM = 50%，Motor 是否一定是 50% RPM？

答案通常不是。

這就是開始學控制的入口。

------------------------------------------------------------------------

## Week 3：Encoder

### 學習

理解：

-   Incremental Encoder
-   Quadrature Encoder
-   Pulse
-   CPR / PPR
-   Position
-   Velocity

### 實作

``` text
Motor
 ↓
Encoder
 ↓
ESP32
 ↓
RPM
```

### Python

透過 Serial 收集：

``` text
timestamp,pwm,rpm
```

畫：

``` text
RPM
│
│       ______
│     /
│____/
└──────────── time
```

### 完成標準

可以從 Encoder pulse 計算 RPM。

------------------------------------------------------------------------

# Phase 2 --- Feedback Control 基礎

## Week 4：Open Loop vs Closed Loop

開始正式閱讀 **Control System Design Guide Chapter 1**。

理解：

``` text
Open Loop

Command → Controller → Plant → Output
```

以及：

``` text
Closed Loop

             ┌─────────────┐
             │             ↓
Target → (+) → Controller → Plant → Output
          ↑                         │
          └──────── Sensor ─────────┘
```

### 實驗

Motor 固定 PWM。

手指輕微增加 motor load。

觀察 RPM。

接著思考：

> 如果希望負載增加後仍保持相同 RPM，系統缺少什麼？

------------------------------------------------------------------------

## Week 5：Transfer Function 與 Step Response

閱讀 Chapter 2 相關內容。

不要一開始追求完整數學推導。

先理解：

``` text
Input → System → Output
```

使用：

``` python
import control as ct

G = ct.tf([1], [1, 1])
t, y = ct.step_response(G)
```

學習：

-   Transfer Function
-   Pole
-   Zero
-   Time Constant
-   Step Response

理解：

-   Rise Time
-   Settling Time
-   Overshoot
-   Steady-State Error

### Lab

建立：

``` text
01-step-response.ipynb
```

------------------------------------------------------------------------

## Week 6：Frequency Response

學：

-   Frequency
-   Gain
-   Phase
-   Bandwidth
-   Bode Plot

Python：

``` python
ct.bode_plot(G)
```

不要急著背公式。

先建立直覺：

> 不同頻率的 input 通過 system 後，振幅與相位會如何改變？

### Lab

比較：

-   First-order system
-   Second-order system
-   不同 damping ratio

------------------------------------------------------------------------

# Phase 3 --- PID

## Week 7：P Controller

閱讀 Controller / Tuning 相關章節。

建立：

``` text
error = target - actual

output = Kp × error
```

Python simulation。

然後實際放進 ESP32。

### Motor Lab

``` text
Target RPM
    ↓
Error
    ↓
P Controller
    ↓
PWM
    ↓
Motor
    ↓
Encoder
    └───────────── feedback
```

改變 Kp。

記錄：

-   response speed
-   oscillation
-   steady-state error

------------------------------------------------------------------------

## Week 8：PI Controller

加入：

``` text
Integral
```

理解：

> 為什麼 P Controller 可能永遠差一點點？

觀察 PI 如何消除 steady-state error。

同時學習：

-   Integral windup
-   Saturation

### 必做

加入 Anti-Windup。

------------------------------------------------------------------------

## Week 9：PD 與 PID

加入 derivative。

理解：

``` text
P → 現在差多少
I → 過去累積差多少
D → 誤差變化多快
```

比較：

``` text
P
PI
PD
PID
```

### Project 1

# DC Motor PID Speed Controller

成果必須包含：

-   Target RPM
-   Actual RPM
-   PWM
-   Kp / Ki / Kd
-   Step Response
-   Overshoot
-   Settling Time
-   Steady-State Error

------------------------------------------------------------------------

# Phase 4 --- Digital Control

## Week 10：Sampling

閱讀 Sampling / Delay。

實驗：

``` text
Ts = 1 ms
Ts = 5 ms
Ts = 10 ms
Ts = 50 ms
Ts = 100 ms
```

比較控制效果。

理解：

-   Sampling Rate
-   Control Loop Frequency
-   Delay
-   Jitter

### 關鍵問題

> PID loop 為什麼不能「想到才跑一次」？

------------------------------------------------------------------------

## Week 11：Discrete PID

從：

``` text
Continuous PID
```

走到：

``` text
Digital PID
```

實際理解：

``` cpp
error = target - actual;

integral += error * dt;

derivative =
    (error - previousError) / dt;
```

學習：

-   z-domain 基本概念
-   c2d
-   Zero Order Hold

Python：

``` python
ct.c2d()
```

### Project 2

# Digital PID Controller

要求 ESP32 使用固定 sampling period 執行。

------------------------------------------------------------------------

# Phase 5 --- Disturbance / Feedforward / Filter

## Week 12：Disturbance Rejection

對 motor 人為增加負載。

比較：

``` text
Open Loop
P
PI
PID
```

觀察 recovery。

------------------------------------------------------------------------

## Week 13：Feedforward

理解：

``` text
Feedback:
發生誤差後修正

Feedforward:
預先補償
```

建立：

``` text
             ┌→ Feedforward ─┐
Target ──────┤               ├→ Plant
             └→ PID ─────────┘
```

比較：

``` text
Feedback only
vs
Feedforward + Feedback
```

------------------------------------------------------------------------

## Week 14：Noise 與 Filter

刻意加入 noise。

學：

-   Low-pass Filter
-   High-pass Filter
-   Moving Average
-   Notch Filter

觀察：

``` text
Raw Encoder
     ↓
Filter
     ↓
Controller
```

同時注意：

> Filter 可以減少 noise，但也會帶來 delay。

這是非常重要的工程 trade-off。

------------------------------------------------------------------------

# Phase 6 --- Modeling

## Week 15：Motor Model

建立 DC Motor model。

基本模型：

``` text
Voltage
   ↓
Electrical Dynamics
   ↓
Torque
   ↓
Mechanical Dynamics
   ↓
Angular Velocity
```

學習：

-   Resistance
-   Inductance
-   Torque Constant
-   Back EMF
-   Inertia
-   Friction

Python 建立 Transfer Function / State Space model。

------------------------------------------------------------------------

## Week 16：System Identification

不要只相信理論參數。

實際測量：

``` text
PWM → Motor → Encoder
```

收集：

``` csv
time,pwm,rpm
0.00,0,0
0.01,100,20
0.02,100,48
...
```

Python：

``` text
CSV
 ↓
Pandas
 ↓
Plot
 ↓
Estimate Model
 ↓
Simulation
 ↓
Compare Reality
```

### Project 3

# Motor Digital Twin Lite

至少做到：

``` text
Real Step Response
vs
Simulated Step Response
```

------------------------------------------------------------------------

# Phase 7 --- Position / Servo Control

## Week 17：Position Control

從 RPM control 進入 position control。

``` text
Target Position
      ↓
Position Controller
      ↓
Motor
      ↓
Encoder
      ↓
Actual Position
```

實驗：

``` text
0°
90°
180°
360°
```

比較 P / PI / PID。

------------------------------------------------------------------------

## Week 18：Cascaded Control

建立：

``` text
Position Loop
      ↓
Velocity Target
      ↓
Velocity Loop
      ↓
PWM
      ↓
Motor
```

理解：

``` text
Outer Loop
Inner Loop
```

這是工業 Servo Control 很重要的架構。

### Project 4

# Servo Position Controller

------------------------------------------------------------------------

# Phase 8 --- State Space / Observer

## Week 19：State Space

現在才正式進入：

\[ `\dot{x}`{=tex}=Ax+Bu \]

\[ y=Cx+Du \]

理解 State 不只是數學。

例如 Motor：

``` text
State:

Position
Velocity
Current
```

使用：

``` python
ct.ss()
```

------------------------------------------------------------------------

## Week 20：Observer

學習：

-   Observability
-   State Estimation
-   Luenberger Observer

Python：

``` python
ct.obsv()
ct.place()
```

比較：

``` text
Actual Velocity
Measured Velocity
Estimated Velocity
```

再把 Observer 放進 Motor 實驗。

------------------------------------------------------------------------

# Phase 9 --- Rapid Control Prototyping

## Week 21：Python Control Dashboard

建立：

``` text
MacBook / PC
     │
   Python
     │
  PySerial
     │
    USB
     │
   ESP32
     │
Motor + Encoder
```

Python 負責：

-   Setpoint
-   Parameter configuration
-   Logging
-   Plot
-   Experiment

ESP32 負責：

-   Sensor sampling
-   PID
-   PWM
-   deterministic loop

------------------------------------------------------------------------

## Week 22：Experiment Automation

建立自動實驗：

``` text
1. Set Kp Ki Kd
2. Set target
3. Start experiment
4. Collect 10 sec data
5. Calculate metrics
6. Save CSV
7. Plot response
8. Generate report
```

### Project 5

# Mini Rapid Control Prototyping Platform

到這裡，你已經完成原書 Visual ModelQ / LabVIEW 思想的現代 Maker 版本。

------------------------------------------------------------------------

# Phase 10 --- Capstone

## Week 23--24：Self-Balancing Robot

最終專案：

``` text
          MPU6050
             ↓
      Accelerometer
        + Gyroscope
             ↓
      Sensor Fusion
             ↓
       Tilt Angle
             ↓
       PID / Control
          ↙     ↘
     Left       Right
     Motor      Motor
       ↑          ↑
     Encoder   Encoder
```

需要整合：

-   IMU
-   Sampling
-   Filtering
-   Sensor Fusion
-   PID
-   Motor Driver
-   Encoder
-   Real-Time Loop
-   Telemetry
-   Tuning

------------------------------------------------------------------------

# 7. 五個里程碑專案

## Project 1 --- DC Motor PID Speed Control

證明你理解：

-   Feedback
-   Encoder
-   P / PI / PID
-   Step Response

------------------------------------------------------------------------

## Project 2 --- Digital PID Controller

證明你理解：

-   Sampling
-   Discretization
-   Real-time loop
-   Embedded implementation

------------------------------------------------------------------------

## Project 3 --- Motor Model / Digital Twin Lite

證明你理解：

-   Modeling
-   Data Acquisition
-   System Identification
-   Model Verification

------------------------------------------------------------------------

## Project 4 --- Servo Position Control

證明你理解：

-   Position loop
-   Velocity loop
-   Cascaded control
-   Encoder

------------------------------------------------------------------------

## Project 5 --- Self-Balancing Robot

整合：

``` text
Sensor
+
Filter
+
Estimator
+
Controller
+
Motor
+
Real-Time Software
```

------------------------------------------------------------------------

# 8. 每週固定學習方式

每週建議 3 個 session。

## Session A --- 理論，約 90 分鐘

``` text
30 min  閱讀 Control System Design Guide
30 min  整理概念
30 min  Python Simulation
```

不要抄大量筆記。

每章只回答：

1.  這個問題是什麼？
2.  為什麼會發生？
3.  Controller 如何改善？
4.  我如何用 simulation 證明？
5.  我如何用硬體證明？

------------------------------------------------------------------------

## Session B --- Simulation，約 90 分鐘

Jupyter Notebook 固定包含：

``` text
Objective
Theory
Plant Model
Simulation
Plot
Observation
Experiment
Conclusion
```

------------------------------------------------------------------------

## Session C --- Maker Lab，2--4 小時

固定流程：

``` text
Build
 ↓
Measure
 ↓
Log
 ↓
Plot
 ↓
Diagnose
 ↓
Tune
 ↓
Repeat
```

不要只以「Motor 有轉」當作成功。

一定要留下 data。

------------------------------------------------------------------------

# 9. 建議 GitHub Repository

``` text
maker-control-lab/
│
├── README.md
├── ROADMAP.md
│
├── notebooks/
│   ├── 01-step-response/
│   ├── 02-frequency-response/
│   ├── 03-p-controller/
│   ├── 04-pi-controller/
│   ├── 05-pid-controller/
│   ├── 06-sampling/
│   ├── 07-digital-control/
│   ├── 08-disturbance/
│   ├── 09-feedforward/
│   ├── 10-filter/
│   ├── 11-modeling/
│   ├── 12-system-identification/
│   ├── 13-state-space/
│   └── 14-observer/
│
├── firmware/
│   ├── motor-open-loop/
│   ├── encoder/
│   ├── speed-pid/
│   ├── position-control/
│   └── balancing-robot/
│
├── hardware/
│   ├── bom.md
│   ├── wiring/
│   └── datasheets/
│
├── data/
│   ├── raw/
│   └── processed/
│
└── reports/
```

------------------------------------------------------------------------

# 10. 每個實驗的 Engineering Notebook Template

每個 Lab 都回答以下內容。

## Objective

我要驗證什麼？

## System

``` text
Input → Controller → Plant → Output
```

## Hypothesis

我預期會發生什麼？

## Parameters

記錄：

``` text
Kp
Ki
Kd
Sampling Time
Motor Voltage
Target RPM
Load
```

## Measurement

記錄：

``` text
Timestamp
Setpoint
Actual
Error
Control Output
```

## Result

至少畫：

``` text
Setpoint vs Actual
Control Output
Error
```

## Metrics

計算：

-   Rise Time
-   Overshoot
-   Settling Time
-   Steady-State Error

## Observation

結果與預期是否相同？

## Conclusion

下一次要改什麼？

------------------------------------------------------------------------

# 11. 不建議的學習方式

## 不要一開始追求完整數學證明

第一輪目標是：

> 看懂、模擬、做出來、量到。

第二輪才深入 derivation。

------------------------------------------------------------------------

## 不要只看 Arduino 範例

例如：

``` cpp
analogWrite(motor, 128);
```

這只是 actuator control，不等於 feedback control。

你真正要問：

``` text
Target 是多少？
Actual 是多少？
Error 是多少？
Controller 如何修正？
```

------------------------------------------------------------------------

## 不要只靠 Serial Monitor

應該把資料送進 Python。

從：

``` text
「看起來差不多」
```

升級成：

``` text
Rise Time = ?
Overshoot = ?
Settling Time = ?
Steady-State Error = ?
```

------------------------------------------------------------------------

## 不要過早做 Self-Balancing Robot

如果連：

``` text
Encoder
Motor Speed PID
Sampling
Filter
```

都還沒有做熟，自平衡車很容易變成不停 trial-and-error 調參數。

先完成 Motor PID，再做 Robot。

------------------------------------------------------------------------

# 12. 六個月後應具備的能力

完成 Roadmap 後，應該能看到一個控制問題，就自然拆成：

``` text
What is the Plant?

What is the Input?

What is the Output?

What can I measure?

What is the Setpoint?

What disturbances exist?

What is the sampling rate?

What controller should I use?

How do I measure performance?
```

而不是只想到：

> 「PID 的 Kp、Ki、Kd 要設多少？」

這代表已經開始具備真正的 Control System Engineering 思維。

------------------------------------------------------------------------

# 13. 後續進階方向

完成本計畫後，可以選擇三條路。

## A. Robotics

``` text
Self-Balancing Robot
 ↓
Differential Drive
 ↓
Odometry
 ↓
Kalman Filter
 ↓
Path Tracking
 ↓
ROS 2
```

## B. Modern Control

``` text
State Space
 ↓
Controllability
 ↓
Observability
 ↓
Pole Placement
 ↓
LQR
 ↓
Kalman Filter
 ↓
LQG
```

## C. Industrial / Motion Control

``` text
Motor Control
 ↓
Current Loop
 ↓
Velocity Loop
 ↓
Position Loop
 ↓
Servo
 ↓
Trajectory
 ↓
Industrial Motion Control
```

------------------------------------------------------------------------

# 14. 建議閱讀順序

不需要一本讀完才讀下一本。

建議：

``` text
Arduino Workshop
     │
     ├── 前期硬體基礎
     ↓
Control System Design Guide
     │
     ├── 全程主教材
     ↓
Arduino Cookbook
     │
     ├── 遇到硬體問題查閱
     ↓
Advanced Arduino Techniques in Science
     │
     ├── Python / DAQ / PID
     ↓
Feedback Systems
     │
     ├── 補強理論
     ↓
Modern Control Engineering
         進階理論
```

------------------------------------------------------------------------

# 15. 最終學習路徑總圖

``` text
Electronics Basics
        ↓
Arduino / ESP32
        ↓
PWM + DC Motor
        ↓
Encoder
        ↓
Open-loop Control
        ↓
Feedback
        ↓
Transfer Function
        ↓
Step Response
        ↓
Bode Plot
        ↓
P → PI → PID
        ↓
Motor Speed Control
        ↓
Sampling
        ↓
Digital PID
        ↓
Disturbance / Feedforward
        ↓
Noise / Filter
        ↓
Motor Modeling
        ↓
System Identification
        ↓
Position Control
        ↓
Cascaded Control
        ↓
State Space
        ↓
Observer
        ↓
Python + ESP32 RCP
        ↓
Self-Balancing Robot
```

------------------------------------------------------------------------

## 最重要的原則

> **不要把這個計畫當成「讀完一本控制工程書」。**

真正的學習單位是：

``` text
一個概念
+
一個 Simulation
+
一個 Experiment
+
一份 Measurement
+
一次 Diagnosis
```

完成 24 週後，你應該不只是「知道
PID」，而是能從模型、模擬、嵌入式控制、感測器、資料分析到實體機構，完整地建立並診斷一個
Feedback Control System。

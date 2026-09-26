# Ch7 Disturbance Response — Lab Report

> 對應 maker-labs 講義 Chapter 7。填完本表即完成本章 engineering notebook。

## Objective
分開量測 command response 與 disturbance response,理解「好的 setpoint tracking
不等於好的 disturbance rejection」,並用 sensitivity function 說明負載擾動如何被抑制。

## System
```
                    disturbance d (plant-input load)
                              │
Setpoint → (+) → Controller(C) → (+) → Plant(G) → Output
             ↑                                        │
             └──────────────── Sensor ────────────────┘

  command response      T = C·G/(1+C·G)
  sensitivity           S = 1/(1+C·G)
  disturbance response  Gd = G·S = G/(1+C·G)
```
- Plant / Input / Output / Measurement:______
- 哪些假設屬於 LTI?真實硬體在哪裡違反(摩擦、齒隙、飽和)?______

## Hypothesis
（執行 simulation 前先寫下預測:含積分器的 PI 會不會把 load 造成的穩態誤差歸零?dip 多大?）

## Parameters
| 參數 | 值 |
|---|---|
| Plant G(s) | 1/(0.3 s + 1) |
| Controller C(s) | (3 s + 6)/s（PI） |
| Setpoint (硬體) | 300 RPM |
| Control loop freq | 100 Hz |
| Counts / rev | 1000 |

## Simulation
- 指令:`python maker-labs/07-disturbance/sim.py`
- 產生圖:`out/ch07_disturbance.png`
- 記錄:command response DC 增益 T(0)≈1、disturbance response DC 增益 Gd(0)≈0、
  disturbance 峰值偏移與回復時間。

## Hardware (ESP32)
- 韌體:`firmware/main.cpp`(encoder → RPM → PID → PWM,速度閉迴路)。
- 擾動施加:馬達穩定轉速後施加**可重複**負載(小摩擦輪/固定夾具);
  不要用手碰高速旋轉件。CSV 內 `load` 欄標記負載開/關。
- BOM:ESP32 DevKit、附正交編碼器的 DC motor、馬達驅動板(如 TB6612/DRV8871)、
  可重複負載機構、電源、麵包板。
- 接線:ENC_A→GPIO32、ENC_B→GPIO33、PWM→GPIO25、DIR→GPIO26。
- 收集 `run_*.csv`(欄位 `t,setpoint,rpm,error,pwm,load,dip,recovered`)。

## Result / Metrics
| 指標 | Sim | Hardware |
|---|---|---|
| Command DC gain T(0) | ≈1 | — |
| Disturbance DC gain Gd(0) | ≈0 | — |
| Disturbance 峰值偏移 / RPM dip | | |
| Recovery time | | |
| Max error | | |

## Observation & Conclusion
- command response 與 disturbance response 的形狀為何不同?______
- 積分器如何讓 disturbance 穩態誤差 → 0?若改純 P 會如何?______
- 結果與預期是否相同?至少一個 model mismatch(摩擦非線性/量化/驅動飽和):______
- 下一章(feed-forward)如何在本章結果上再降低 dip:______

## Exit Criteria
- [ ] 能不用背公式解釋 command vs disturbance response 的差異。
- [ ] `sim.py` 從乾淨環境可完整執行並通過 ✅。
- [ ] 圖表有單位、label 與可比較 baseline(command / disturbance 疊圖)。
- [ ] 至少做過一次 parameter sweep(改 C 的 ki 或 load 大小)。
- [ ] 若做硬體:保存原始 CSV,並標記 load step 時間。
- [ ] 能指出至少一個 model mismatch。

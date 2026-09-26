# Ch1 Introduction to Controls — Lab Report

> 對應 maker-labs 講義 Chapter 1。填完本表即完成本章 engineering notebook。

## Objective
建立第一個閉迴路,理解 feedback 如何降低 steady-state error 與對 plant 變化的敏感度。

## System
```
Setpoint → (+) → Controller(K) → Plant(1/(τs+1)) → Output
             ↑                                        │
             └──────────────── Sensor ────────────────┘
```
- Plant / Input / Output / Measurement:______
- 哪些假設屬於 LTI?真實硬體在哪裡違反?______

## Hypothesis
（執行 simulation 前先寫下預測）

## Parameters
| 參數 | 值 |
|---|---|
| τ (plant time constant) | 0.5 s |
| K (controller gain) | 2.0 |
| Control loop freq | 100 Hz |

## Simulation
- 指令:`python maker-labs/01-introduction/sim.py`
- 產生圖:`out/ch01_open_vs_closed.png`
- 記錄:閉迴路 DC 增益、step metrics、plant 變化後的開/閉迴路漂移。

## Hardware (ESP32)
- 韌體:`firmware/main.cpp`(pot → ADC → PWM → LED,open-loop）
- BOM:ESP32 DevKit、電位器、LED（或低壓 DC motor + driver）、麵包板。
- 接線:POT wiper→GPIO34、PWM→GPIO25。
- 收集 `run_*.csv`(欄位 `t,setpoint_raw,duty`)。

## Result / Metrics
| 指標 | Sim | Hardware |
|---|---|---|
| Rise time | | |
| Overshoot | | |
| Settling time | | |
| Steady-state error | | |

## Observation & Conclusion
- 結果與預期是否相同?至少一個 model mismatch:______
- 下一章如何建立在本章結果上:______

## Exit Criteria
- [ ] 能不用背公式解釋本章物理意義。
- [ ] `sim.py` 從乾淨環境可完整執行並通過 ✅。
- [ ] 圖表有單位、label 與可比較 baseline。
- [ ] 至少做過一次 parameter sweep（改 K 或 τ）。
- [ ] 若做硬體:保存原始 CSV。
- [ ] 能指出至少一個 model mismatch。

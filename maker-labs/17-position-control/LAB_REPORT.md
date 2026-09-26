# Ch17 Position-Control Loops — Lab Report

> 對應 maker-labs 講義 Chapter 17。填完本表即完成本章 engineering notebook。

## Objective
建立位置閉迴路,理解「位置 = 速度積分」讓受控體內建一個積分器 (type-1),
因此純 P 對定位命令即可零穩態誤差;比較 P / PI / PID 在 overshoot 與阻尼上的差異,
並認識外位置迴路 + 內速度迴路(cascade)的角色分工。

## System
```
Target angle → (+) → Position Controller → PWM → Motor(velocity) → 1/s → Position
                 ↑                                                          │
                 └──────────────── Encoder (counts) ────────────────────────┘
```
- Plant / Input / Output / Measurement:位置 plant `1/(s(τs+1))` / 目標角度(count) / 角度位置 / encoder count。
- 哪些假設屬於 LTI?真實硬體在哪裡違反?______(靜摩擦、背隙、PWM 飽和、量化)

## Hypothesis
（執行 simulation 前先寫下預測）
- 純 P 是否有 steady-state error?為什麼?______
- 加 I 對定位命令的 overshoot 是變好還變壞?______

## Parameters
| 參數 | 值 |
|---|---|
| τ (velocity plant time constant) | 0.15 s |
| Kp (position) | 8.0 (sim) / 0.8 (fw, count 單位) |
| Ki (position) | 6.0 (sim) / 0.05 (fw) |
| Kd (position) | 1.2 (sim) / 6.0 (fw) |
| Target angle | 90 deg |
| Counts per rev | 1000 |
| Control loop freq | 100 Hz |

## Simulation
- 指令:`python maker-labs/17-position-control/sim.py`
- 產生圖:`out/ch17_position_control.png`
- 記錄:三種控制器的閉迴路 DC 增益、step metrics、overshoot/damping overlay。
- 關鍵結論:位置 plant 內建積分器 → 純 P 已零穩態誤差;PI 命令響應 overshoot 更大;
  PID 導數項提供阻尼把 overshoot 壓下來。

## Hardware (ESP32)
- 韌體:`firmware/main.cpp`(target count → 位置 PID → PWM,`pid.h` + `encoder.h`)
- BOM:ESP32 DevKit、附正交編碼器的 DC 馬達、馬達驅動器(如 TB6612)、電源、麵包板。
- 接線:ENC_A→GPIO32、ENC_B→GPIO33、PWM→GPIO25、DIR→GPIO26。
- 收集 `run_*.csv`(欄位 `t,target_cnt,pos_cnt,error,pwm,p,i,d,saturated`)。
- 先限制 max PWM,再考慮加入 trapezoidal position profile 避免瞬間大速度命令。

## Result / Metrics
| 指標 | Sim (P / PI / PID) | Hardware |
|---|---|---|
| Rise time | | |
| Overshoot | | |
| Settling time | | |
| Steady-state error (final angle error) | | |

## Observation & Conclusion
- 結果與預期是否相同?至少一個 model mismatch:______（靜摩擦造成的殘差、背隙、encoder 量化）
- 純 P 零穩態誤差在硬體上還成立嗎?為什麼可能不成立?______
- 下一章(Luenberger observer)如何建立在本章結果上:______

## Exit Criteria
- [ ] 能不用背公式解釋為何位置迴路純 P 即可零穩態誤差。
- [ ] `sim.py` 從乾淨環境可完整執行並通過 ✅。
- [ ] 圖表有單位、label 與可比較 baseline(P/PI/PID overlay)。
- [ ] 至少做過一次 parameter sweep(改 Kp/Ki/Kd 或目標角度)。
- [ ] 若做硬體:保存原始 CSV。
- [ ] 能指出至少一個 model mismatch。

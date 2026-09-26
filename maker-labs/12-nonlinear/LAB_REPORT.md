# Ch12 Nonlinear Behavior and Time Variation — Lab Report

> 對應 maker-labs 講義 Chapter 12。填完本表即完成本章 engineering notebook。

## Objective
把真實機器的非線性分成三層理解:**物理現象 → 數學/模型表示 → 可量測結果**。
重點:LTI vs non-LTI、saturation、deadband、friction/stiction、backlash、
time variation,並學會用明確的 function/block 建模,而不是把誤差歸咎於「noise」。

## System
```
Setpoint → (+) → Controller(P/PI) → [deadband comp] → [rate limit] → [saturation]
             ↑                                                            │
             │                                                        Motor/Plant
             │                                                            │
             └──────────────────── Encoder (RPM) ─────────────────────────┘
```
- Plant / Input / Output / Measurement:______
- 哪些假設屬於 LTI?真實硬體在哪裡違反?(PWM 死區、driver 飽和、齒輪 backlash、
  stiction、電池電壓下降造成的 time variation)______

## Hypothesis
（執行 simulation 前先寫下預測:deadband 會造成小訊號無反應/穩態殘差;
saturation 會限制上升速度並可能引發 integral wind-up）

## Parameters
| 參數 | 值 |
|---|---|
| Deadband 寬度 | ±0.15 (sim) / 30 duty (fw) |
| Saturation limit | ±1.0 (sim) / 255 duty (fw) |
| Gain after deadband | 2.0 (sim) |
| Rate limit (每步) | 20 duty / 10 ms (fw) |
| Control loop freq | 100 Hz |

## Simulation
- 指令:`python maker-labs/12-nonlinear/sim.py`
- 產生圖:`out/ch12_nonlinear.png`
- 記錄:
  - 靜態曲線 command → effective input(deadband zone、飽和門檻 ≈ ±0.65)。
  - 閉迴路 overlay:LTI 致動器 vs deadband+saturation 致動器的步階響應差異。

## Hardware (ESP32)
- 韌體:`firmware/main.cpp`(encoder 測 RPM → P 控制 → deadband 補償 →
  rate limit → saturation clamp → PWM)。
- BOM:ESP32 DevKit、低壓 DC motor + driver(如 TB6612)、正交編碼器、電源、麵包板。
- 接線:ENC_A→GPIO32、ENC_B→GPIO33、PWM→GPIO25、DIR→GPIO26。
- 校準步驟:量測(a) motor 起轉 PWM、(b) 正反轉 deadband、(c) 最大 RPM、
  (d) 低速 stiction 發生的 duty。用結果填 `PWM_DEADBAND` / `MAX_RATE`。
- 收集 `run_*.csv`(欄位 `t,setpoint,rpm,u_raw,u_comp,u_rl,duty,dir,saturated`)。

## Result / Metrics
| 指標 | Sim (LTI) | Sim (non-LTI) | Hardware |
|---|---|---|---|
| 步階最終值 / 穩態誤差 | | | |
| 低速死區(最小可動 RPM） | | | |
| Rise time | | | |
| 飽和發生比例 | | | |

## Observation & Conclusion
- deadband 補償前後,低速可動範圍差多少?______
- saturation 是否引發 wind-up?anti-windup 有無改善?______
- 至少一個 model mismatch:______
- time variation(電壓下降、發熱)如何影響重複實驗?______

## Exit Criteria
- [ ] 能用自己的話解釋 LTI vs non-LTI、deadband、saturation、backlash、stiction。
- [ ] `sim.py` 從乾淨環境可完整執行並通過 ✅。
- [ ] 圖表有單位、label、legend 與可比較 baseline(LTI vs non-LTI overlay)。
- [ ] 至少做過一次 parameter sweep(改 deadband 或 saturation limit)。
- [ ] 每個非線性都用 function/block 明確建模,未歸咎於「noise」。
- [ ] 若做硬體:保存原始 CSV,並校準 deadband / 起轉 PWM。
- [ ] 能指出至少三個 sim 與硬體之間的 model mismatch 來源。

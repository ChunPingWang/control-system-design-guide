# Ch3 Tuning a Control System — Lab Report

> 對應 maker-labs 講義 Chapter 3。填完本表即完成本章 engineering notebook。

## Objective
理解 tuning 的本質:調 loop gain K 時,不是只看 step 曲線「順不順」,而是用
gain margin / phase margin 量化「離失穩還有多少餘裕」,並注意 actuator
saturation 不要掩蓋真實 loop dynamics。

## System
```
Setpoint → (+) → Controller(K) → Plant(1/(s(0.2s+1))) → Output
             ↑                                             │
             └──────────────── Sensor(encoder) ────────────┘

Loop transfer  L(s) = K·G(s)
```
- Plant / Input / Output / Measurement:馬達 / PWM duty / 轉速 rpm / encoder。
- 哪些假設屬於 LTI?真實硬體在哪裡違反?______(飽和、摩擦、量化、延遲)

## Hypothesis
（執行 simulation 前先寫下預測）
- K 上升 → phase margin ______、overshoot ______、settling time ______。

## Parameters
| 參數 | 值 |
|---|---|
| Plant G(s) | 1/(0.2 s² + s) = 1/(s(0.2s+1)) |
| Loop gain sweep K | 0.5, 2, 8 |
| Control loop freq | 100 Hz (DT=10 ms) |
| PWM clamp | ±255 (8-bit) |
| Setpoint (HW) | 300 rpm |
| Kp sweep (HW) | 0.2, 0.6, 1.5, 3.0 |

## Simulation
- 指令:`python maker-labs/03-tuning/sim.py`
- 產生圖:`out/ch03_loop_gain_sweep.png`、`out/ch03_loop_bode.png`
- 記錄:每個 K 的 GM(dB)、PM(deg)、gain crossover 頻率、overshoot。
- 觀念:此 plant 為 type-1(含積分器,只有兩個極點),phase 不會越過 −180°,
  故 gain margin 為 ∞;真正隨 K 惡化的是 **phase margin**。

## Hardware (ESP32)
- 韌體:`firmware/main.cpp`(encoder → RPM → P 控制 → PWM,含 clamp 與 saturation 回報)
- BOM:ESP32 DevKit、帶正交 encoder 的 DC motor、馬達驅動器(H-bridge)、電源、麵包板。
- 接線:ENC_A→GPIO32、ENC_B→GPIO33、PWM→GPIO25、DIR→GPIO26。
- 實驗:韌體自動逐步提高 Kp(0.2→3.0),每段重跑同一 setpoint step。
- 收集 `run_*.csv`(欄位 `t,kp,setpoint,rpm,error,pwm,saturated`)。

## Result / Metrics
| 指標 | Sim (K=0.5) | Sim (K=2) | Sim (K=8) | Hardware |
|---|---|---|---|---|
| Phase margin | | | | — |
| Gain margin | ∞ | ∞ | ∞ | — |
| Overshoot | | | | |
| Settling time | | | | |
| Saturation? | — | — | — | |

## Observation & Conclusion
- K 越大 → PM 越小 → overshoot 越大,與預測是否相同?______
- 是否出現 PWM 飽和?飽和是否讓「看起來的 overshoot」失真?______
- 至少一個 model mismatch:______(如靜摩擦造成低速無法轉、encoder 量化)
- 下一章如何建立在本章結果上:______

## Exit Criteria
- [ ] 能不用背公式解釋 gain margin / phase margin 的物理意義。
- [ ] `sim.py` 從乾淨環境可完整執行並通過 ✅。
- [ ] 圖表有單位、label 與可比較 baseline(多個 K overlay)。
- [ ] 至少做過一次 loop-gain sweep,並用 `margin()` 量化而非肉眼判斷。
- [ ] 若做硬體:保存原始 CSV,並標註哪些段落發生 saturation。
- [ ] 能指出至少一個 model mismatch。

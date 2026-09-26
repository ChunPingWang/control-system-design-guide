# Ch11 Introduction to Modeling — Lab Report

> 對應 maker-labs 講義 Chapter 11。填完本表即完成本章 engineering notebook。

## Objective
理解「模型的目的」:模型不是越複雜越好,而是要足以回答工程問題。學會把同一個 plant 用
**time-domain**(step 響應、時間常數)與 **frequency-domain**(Bode、bandwidth)兩種方式描述,
並說明各自能回答什麼問題。

## System
```
固定供電 → PWM (duty step) → DC Motor (open-loop) → Encoder → RPM
                                   │
                          G(s) = K / (τs + 1)
```
- Plant / Input / Output / Measurement:DC 馬達 / PWM duty / 轉速 RPM / encoder counts。
- 哪些假設屬於 LTI?真實硬體在哪裡違反?______(dead-zone、飽和、摩擦、負載變動)

## Hypothesis
（執行 simulation 前先寫下預測:K、τ 大約多少?time-domain 與 frequency-domain 描述應如何對應?）

## Parameters
| 參數 | 值 |
|---|---|
| K (DC gain) | 4.0 |
| τ (time constant) | 0.25 s |
| 轉折頻率 ω_c = 1/τ | 4.0 rad/s |
| Counts per rev | 1000 |
| Control/量測窗 | 100 Hz (10 ms) |
| Open-loop step duties | 0 / 64 / 128 / 192 / 255 |

## Simulation
- 指令:`python maker-labs/11-modeling/sim.py`
- 產生圖:`out/ch11_two_descriptions.png`(左 time-domain step、右 frequency-domain Bode magnitude）
- 記錄:DC gain、poles(-1/τ)、time-domain 的 rise/settling、frequency-domain 的 -3 dB 轉折頻率。
- 觀念:同一組 (K, τ) 在兩張圖上是同一模型的兩種描述 —— time-domain 看 transient/實作,
  frequency-domain 看 loop/stability/bandwidth。

## Hardware (ESP32)
- 韌體:`firmware/main.cpp`(open-loop step test:固定供電、依序施加不同 PWM，encoder 量 RPM）。
- BOM:ESP32 DevKit、附 encoder 的低壓 DC motor、motor driver(如 TB6612/DRV8833）、電源、麵包板。
- 接線:ENC_A→GPIO32、ENC_B→GPIO33、PWM→GPIO25、DIR→GPIO26。
- 收集 `run_*.csv`(欄位 `t_ms,duty,pos,rpm`)。每個 duty step 各取一段離線擬合 K、τ。

## Result / Metrics
| 指標 | Sim | Hardware |
|---|---|---|
| DC gain K | 4.0 | |
| Time constant τ | 0.25 s | |
| Pole 位置 (-1/τ) | -4.0 | |
| -3 dB 轉折頻率 | 4.0 rad/s | |
| Rise time | | |

## Observation & Conclusion
- 結果與預期是否相同?至少一個 model mismatch:______(如 dead-zone 讓低 duty 不轉、飽和、量測噪聲）
- Time-domain 與 frequency-domain 各回答了什麼工程問題?______
- 下一章如何建立在本章結果上:______(用此 model 做 nonlinear/time-variation 分析)

## Exit Criteria
- [ ] 能不用背公式解釋本章物理意義(模型目的、time vs frequency domain)。
- [ ] `sim.py` 從乾淨環境可完整執行並通過 ✅。
- [ ] 圖表有單位、label 與可比較 baseline。
- [ ] 至少做過一次 parameter sweep（改 K 或 τ，比較兩種描述如何變化）。
- [ ] 若做硬體:保存原始 CSV,並用同一 plant 的 time/frequency 兩種描述各量出一個指標。
- [ ] 能指出至少一個 model mismatch。

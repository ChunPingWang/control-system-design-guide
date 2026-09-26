# Ch4 Delay in Digital Controllers — Lab Report

> 對應 maker-labs 講義 Chapter 4。填完本表即完成本章 engineering notebook。

## Objective
理解數位控制器一定存在的延遲(sampling、sample-and-hold、calculation delay),
並用「同一 controller,只改 sample time」量化 Ts 增大如何惡化 stability 與 overshoot。

## System
```
Setpoint → (+) → Controller(C) → ZOH/Ts → Plant(1/(0.15s+1)) → Output
             ↑                                                    │
             └───────────────── Sampler(Ts) ──────────────────────┘
```
- Plant / Input / Output / Measurement:一階馬達近似 / PWM duty / 轉速 RPM / encoder counts。
- 哪些假設屬於 LTI?真實硬體在哪裡違反?______（PWM 飽和、量化、摩擦死區、迴路 jitter）

## Hypothesis
（執行 simulation 前先寫下預測）Ts 越大,ZOH 等效延遲越大 → phase margin 下降、overshoot 上升。

## Parameters
| 參數 | 值 |
|---|---|
| Plant time constant τ | 0.15 s |
| Controller gain C (P) | 5.0 |
| Sample time sweep Ts | 2 / 10 / 30 / 50 ms |
| 韌體控制週期選項 | 1 / 5 / 10 / 50 ms |

## Simulation
- 指令:`python maker-labs/04-sampling-delay/sim.py`
- 產生圖:`out/ch04_sampling_delay.png`
- 記錄:各 Ts 的 overshoot、settling time、gain/phase margin,與連續 baseline 疊圖比較。
- 觀察:Ts 由 2 ms → 50 ms,overshoot 由 ~0% 升至 ~70%,phase margin 由 ~100° 降至 ~41°。

## Hardware (ESP32)
- 韌體:`firmware/main.cpp`(encoder → RPM → PID → PWM,可切換控制週期)。
- 排程重點:用 `micros()` + `nextTick += CONTROL_US` 建固定週期,**不要**用長 `delay()`。
- BOM:ESP32 DevKit、帶 encoder 的 DC motor、馬達驅動(如 TB6612/DRV8833)、電源、麵包板。
- 接線:ENC_A→GPIO32、ENC_B→GPIO33、PWM→GPIO25、DIR→GPIO26。
- 收集 `run_*.csv`(欄位 `t_ms,control_us,actual_dt_us,jitter_us,setpoint,rpm,pwm`)。
- 對 1/5/10/50 ms 各跑一段,記錄實際 loop period 與 jitter 分布。

## Result / Metrics
| 指標 | Sim (Ts=10ms) | Sim (Ts=50ms) | Hardware |
|---|---|---|---|
| Rise time | | | |
| Overshoot | | | |
| Settling time | | | |
| Phase margin | | | |
| 實際週期 / jitter | — | — | |

## Observation & Conclusion
- 結果與預期是否相同?至少一個 model mismatch:______（sim 假設固定 Ts 無 jitter,硬體有排程抖動與計算延遲）
- 下一章如何建立在本章結果上:進入 Ch5 z-domain,把「延遲/取樣」正式用離散轉移函數與 aliasing 分析。

## Exit Criteria
- [ ] 能不用背公式解釋 sampling / sample-and-hold / calculation delay / sample-time 選擇。
- [ ] `sim.py` 從乾淨環境可完整執行並通過 ✅。
- [ ] 圖表有單位、label 與可比較 baseline(連續 vs 各 Ts)。
- [ ] 至少做過一次 parameter sweep(改 Ts)。
- [ ] 若做硬體:保存原始 CSV,並附 jitter 統計。
- [ ] 能指出至少一個 model mismatch。

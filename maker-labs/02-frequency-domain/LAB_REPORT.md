# Ch2 The Frequency Domain — Lab Report

> 對應 maker-labs 講義 Chapter 2。填完本表即完成本章 engineering notebook。

## Objective
理解同一個 LTI 系統的兩種觀點:時域(step response)與頻域(Bode)。
從 transfer function 的 pole/zero 看懂 overshoot、rise time 與共振峰的來源,
並在硬體上用多組 PWM step 量 DC motor 的 time constant τ,為建立 motor model 做準備。

## System
```
        時域觀點                          頻域觀點
   u(step) → G(s) → y(t)            u(sin ω) → G(s) → |G|∠φ
   看 rise / overshoot / settling   看 magnitude / phase / 共振峰
             └────── 同一組 pole 的不同投影 ──────┘
```
- Plant / Input / Output / Measurement:G(s)=1/(τs+1) 的 DC motor / PWM duty / 轉速 RPM / encoder 計數
- 哪些假設屬於 LTI?真實硬體在哪裡違反?______(如:摩擦死區、PWM 飽和、量測量化)

## Hypothesis
（執行 simulation 前先寫下預測）
- 一階 G1 無 overshoot;二階欠阻尼 G2(ζ=0.4)會 overshoot 並在 Bode 出現共振峰。
- 預測 G2 共振峰頻率接近 ωn=5 rad/s。

## Parameters
| 參數 | 值 |
|---|---|
| G1 | 1/(s+1),pole = -1 |
| G2 | 25/(s²+4s+25),ωn=5 rad/s、ζ=0.4 |
| Frequency sweep 範圍 | 0.1 – 100 rad/s |
| Control / sample freq | 100 Hz |
| Step levels (PWM duty) | 0 → 80 → 160 → 255 → 120 → 0 |
| 每個 step 維持 | 2.0 s |

## Simulation
- 指令:`python maker-labs/02-frequency-domain/sim.py`
- 產生圖:`out/ch02_time_vs_freq.png`(左 step、右上 magnitude、右下 phase)
- 記錄:G1/G2 的 poles、DC 增益、step metrics(rise/overshoot/settling)、G2 頻域共振峰。

## Hardware (ESP32)
- 韌體:`firmware/main.cpp`(多組 PWM step → DC motor,encoder 讀 RPM,開迴路)
- BOM:ESP32 DevKit、帶 quadrature encoder 的 DC motor、馬達驅動板(如 TB6612 / DRV8833)、電源、麵包板。
- 接線:ENC_A→GPIO32、ENC_B→GPIO33、PWM→GPIO25、DIR→GPIO26。
- 收集 `run_*.csv`(欄位 `t,pwm_duty,rpm`)。
- 分析:對每個 step 擬合一階響應,τ ≈ 到達終值 63.2% 所需時間。

## Result / Metrics
| 指標 | Sim (G2) | Hardware |
|---|---|---|
| Rise time | | |
| Overshoot | | |
| Settling time | | |
| Time constant τ (估) | (一階 G1: 1.0 s) | |
| 共振峰頻率 / 峰值 | ≈5 rad/s / >0 dB | |

## Observation & Conclusion
- 時域 overshoot 與頻域共振峰是否指向同一組 pole?______
- 結果與預期是否相同?至少一個 model mismatch:______(如死區、飽和、量化)
- 下一章如何建立在本章結果上:用本章估得的 τ 建立 motor model,做 tuning。

## Exit Criteria
- [ ] 能不用背公式解釋 pole/zero 與 time/frequency domain 的關係。
- [ ] `sim.py` 從乾淨環境可完整執行並通過 ✅。
- [ ] 圖表有單位、label 與可比較 baseline(同時保留 step 與 Bode)。
- [ ] 至少做過一次 parameter sweep(改 ζ 或 ωn / 改 PWM step)。
- [ ] 若做硬體:保存原始 CSV,並用 raw + processed 兩種形式。
- [ ] 能指出至少一個 model mismatch。
- [ ] 能說明下一章如何用本章估得的 τ 建立 motor model。

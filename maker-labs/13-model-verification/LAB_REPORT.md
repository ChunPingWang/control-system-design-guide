# Ch13 Model Development and Verification — Lab Report

> 對應 maker-labs 講義 Chapter 13。填完本表即完成本章 engineering notebook。

## Objective
從量測資料建立馬達一階模型,並用**未參與 fitting 的第二筆資料**驗證模型,
理解「模型能跑」不等於「模型正確」——validation 比 fitting 重要。

## System
```
PWM step → Motor(1/(τs+1)·K) → Encoder → RPM  ──記錄──▶ CSV
                                                          │
                                       Python curve_fit ◀─┘
                                       估 K, τ → simulate → compare
```
- Plant / Input / Output / Measurement:馬達 / PWM duty / 轉速 rpm / encoder 差分速度
- 哪些假設屬於 LTI?真實硬體在哪裡違反?______(friction、dead-zone、飽和、電池電壓下降)

## Hypothesis
（執行 simulation 前先寫下預測:K≈? τ≈? validation VAF 能否 > 90%?）

## Parameters
| 參數 | 值 |
|---|---|
| K (true steady-state speed) | 1200 rpm |
| τ (true time constant) | 0.28 s |
| 感測雜訊 std | 18 rpm |
| 取樣 / control loop | 100 Hz |
| Step 長度 | 2 s |
| Run A / Run B seed | 13 / 1313 |

## Simulation
- 指令:`python maker-labs/13-model-verification/sim.py`
- 產生圖:`out/ch13_model_verification.png`(左:Run A fitting;右:Run B validation)
- 記錄:估計 K/τ、參數誤差、Run A/B 的 RMSE 與 VAF。

## Hardware (ESP32)
- 韌體:`firmware/main.cpp`(自動跑 Run A→Run B 兩段 PWM step,記錄 system-ID 資料)
- BOM:ESP32 DevKit、附正交編碼器的 DC motor、馬達驅動(如 TB6612/DRV8833)、電源、麵包板。
- 接線:ENC_A→GPIO32、ENC_B→GPIO33、PWM→GPIO25、DIR→GPIO26。
- 收集 `run_*.csv`(欄位 `run,t_ms,pwm,rpm`);Run 0 = train,Run 1 = validation。

## Result / Metrics
| 指標 | Sim (Run B / validation) | Hardware |
|---|---|---|
| 估計 K | | |
| 估計 τ | | |
| RMSE (rpm) | | |
| VAF (%) | | |

## Observation & Conclusion
- Run A(train)與 Run B(validation)指標是否接近?若 train 好而 validation 差 → overfit。
- 結果與預期是否相同?至少一個 model mismatch:______
- 下一章如何建立在本章結果上:______

## Exit Criteria
- [ ] 能不用背公式解釋七步建模流程與「用獨立資料驗證」的重要性。
- [ ] `sim.py` 從乾淨環境可完整執行並通過 ✅。
- [ ] 圖表有單位、label 與 train/validation 可比較 baseline。
- [ ] 至少做過一次 parameter sweep（改 K、τ 或雜訊 std）。
- [ ] 若做硬體:保存 Run A / Run B 原始 CSV。
- [ ] 能指出至少一個 model mismatch(sim 理想 LTI vs 硬體 friction/飽和)。

# Ch14 Encoders and Resolvers — Lab Report

> 對應 maker-labs 講義 Chapter 14。填完本表即完成本章 engineering notebook。

## Objective
理解編碼器 position 是量化值,velocity 為差分/pulse-timing 推估;量測
accuracy / resolution / response 的分別,並觀察 quantization noise 在低速 /
短窗時如何被放大,以及低通濾波在 noise 與 lag 之間的取捨。

## System
```
真實角位置 θ(t) ──floor 量化──> 整數 count ──固定窗差分──> raw velocity
                                                              │ OnePoleLP
                                                              ▼
                                                        filtered velocity
```
- Plant / Input / Output / Measurement:______
  (提示:Plant=旋轉軸;Input=角速度;Output=角位置;Measurement=量化 count)
- 哪些假設屬於 LTI?真實硬體在哪裡違反?______
  (提示:量化本身非線性;低速 zero-count、cyclic error、A/B 相位不對稱)

## Hypothesis
（執行 simulation 前先寫下預測:窗長加倍後 RMS 誤差約如何變化?低速相對雜訊?）

## Parameters
| 參數 | 值 |
|---|---|
| CPR (counts per rev, 4x) | 400 |
| 底層 count 取樣頻率 | 1000 Hz |
| 量測窗掃描 | 5 / 10 / 20 / 50 / 100 取樣 (5–100 ms) |
| Velocity 低通截止 fc | 8 Hz |
| Firmware 量測窗 | 100 Hz (10 ms) |

## Simulation
- 指令:`python maker-labs/14-encoder/sim.py`
- 產生圖:`out/ch14_encoder.png`(三張子圖:窗長 vs RMS、低速 vs 中速、raw vs filtered)
- 記錄:各量測窗的 velocity RMS 誤差、低速/中速相對雜訊 %、低通前後 RMS。

## Hardware (ESP32)
- 韌體:`firmware/main.cpp`(quadrature encoder → QuadDecoder 4x 解碼 →
  VelocityEstimator 固定窗差分 → OnePoleLP)。
- BOM:ESP32 DevKit、增量式正交編碼器(或帶編碼器的 gearmotor)、麵包板。
- 接線:ENC_A→GPIO32、ENC_B→GPIO33(INPUT_PULLUP),GND/VCC 共地。
- 收集 `run_*.csv`(欄位 `t_ms,position,rpm_raw,rpm_filt`)。
- 低速時特別記錄 zero-count(某些窗 Δcount=0)問題,對照 pulse-period 法。

## Result / Metrics
| 指標 | Sim | Hardware |
|---|---|---|
| 短窗 velocity RMS 誤差 (rpm) | | |
| 長窗 velocity RMS 誤差 (rpm) | | |
| 低速相對雜訊 (%) | | |
| raw vs filtered RMS 比 | | |
| 濾波引入的估測 lag | | |

## Observation & Conclusion
- 加長量測窗如何降低量化雜訊?代價(latency)為何?______
- 低速為何相對雜訊更大(counts/window 太少)?______
- 低通濾波把 noise 換成 lag 的取捨點在哪?______
- 結果與預期是否相同?至少一個 model mismatch:______
- 下一章(15 Servomotor & Drive)如何建立在本章 velocity 估測上:______

## Exit Criteria
- [ ] 能不用背公式解釋 accuracy / resolution / response 的差別。
- [ ] `sim.py` 從乾淨環境可完整執行並通過 ✅。
- [ ] 圖表有單位、label 與可比較 baseline。
- [ ] 至少做過一次 parameter sweep（改量測窗或 fc）。
- [ ] 若做硬體:保存原始 CSV(含低速 zero-count 案例)。
- [ ] 能指出至少一個 model mismatch(量化、cyclic error、相位不對稱)。

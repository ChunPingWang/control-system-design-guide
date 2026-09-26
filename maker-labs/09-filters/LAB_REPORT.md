# Ch9 Filters in Control Systems — Lab Report

> 對應 maker-labs 講義 Chapter 9。填完本表即完成本章 engineering notebook。

## Objective
理解 low-pass filter 如何降低 sensor noise,以及 noise attenuation 與 phase lag 的
trade-off:filter 不是「越平滑越好」,截止頻率必須和控制迴路頻寬/穩定裕度一起看。

## System
```
noisy RPM (encoder) → Low-pass filter (Butterworth / 一階 IIR) → filtered RPM → Controller
                                        │
                        cutoff 越低 → noise ↓,但 phase lag ↑
```
- Plant / Input / Output / Measurement:______
- 哪些假設屬於 LTI?真實硬體在哪裡違反?______
- Low-pass / passband / noise vs phase lag / 數位濾波 各自的物理意義:______

## Hypothesis
（執行 simulation 前先寫下預測:降低 cutoff 後噪聲與延遲各會怎麼變？）

## Parameters
| 參數 | 值 |
|---|---|
| 取樣頻率 fs | 200 Hz (sim) / 100 Hz (firmware) |
| Butterworth 階數 | 2 |
| Cutoff（比較用） | 5 Hz vs 20 Hz (sim) |
| 一階 IIR cutoff | 15 Hz (`FILTER_FC_HZ`) |
| numpy seed | 9（deterministic） |

## Simulation
- 指令:`python maker-labs/09-filters/sim.py`
- 產生圖:`out/ch09_filters.png`（上:raw vs filtered 時序;下:頻率響應）
- 記錄:各 cutoff 的殘餘噪聲 RMS 與通帶 group delay(ms)。

## Hardware (ESP32)
- 韌體:`firmware/main.cpp`(encoder → RPM → `OnePoleLP` 一階 IIR)
- BOM:ESP32 DevKit、附正交編碼器的 DC motor、motor driver、麵包板。
- 接線:ENC_A→GPIO32、ENC_B→GPIO33。
- 收集 `run_*.csv`(欄位 `t,raw_rpm,filt_rpm`)。
- 實驗:逐步調低 `FILTER_FC_HZ`(如 30→15→5 Hz),比對 raw 與 filtered 的平滑度與延遲。

## Result / Metrics
| 指標 | Sim (fc 低) | Sim (fc 高) | Hardware |
|---|---|---|---|
| 殘餘噪聲 RMS | | | |
| 通帶 group delay | | | |
| 對 step 的落後 | | | |
| 迴路是否變不穩 | | | |

## Observation & Conclusion
- 降低 cutoff 後,noise 與 lag 的取捨是否符合預期?______
- 若把 filtered RPM 餵回 controller,lag 對穩定裕度的影響:______
- 至少一個 model mismatch(sim 的高斯白噪 vs 硬體實際雜訊譜):______
- 下一章如何建立在本章結果上:______

## Exit Criteria
- [ ] 能不用背公式解釋 low-pass / passband / noise vs phase lag / 數位濾波。
- [ ] `sim.py` 從乾淨環境可完整執行並通過 ✅。
- [ ] 圖表有單位、label 與可比較 baseline(兩個 cutoff overlay)。
- [ ] 至少做過一次 parameter sweep(改 cutoff)。
- [ ] 若做硬體:保存原始 CSV(raw 與 filtered 皆有)。
- [ ] 能指出至少一個 model mismatch。

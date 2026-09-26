# Ch5 The z-Domain — Lab Report

> 對應 maker-labs 講義 Chapter 5。填完本表即完成本章 engineering notebook。

## Objective
理解 z-domain、aliasing、離散化(c2d/ZOH)與 quantization,並能把連續 transfer function
落到 MCU 可執行的 difference equation。

## System
```
連續 plant  G(s) = 1/(τs+1)
     │  ZOH 取樣 (Ts)
     ▼
離散 plant  Gd(z) = b1/(z - a1)   →   difference eq:  y[k] = a1·y[k-1] + b1·u[k-1]

量測端:encoder pulse count 在量測窗 (window) 內累積 → 速度,受 quantization 限制。
```
- Plant / Input / Output / Measurement:______
- 哪些假設屬於 LTI?真實硬體在哪裡違反?______

## Hypothesis
（執行前先寫下預測）
- Ts 越大,離散極點 exp(-Ts/τ) 會往哪裡移動?
- 40 Hz 訊號以 50 Hz 取樣後會看到幾 Hz?
- 量測窗拉長後 resolution 與 latency 各如何變化?

## Parameters
| 參數 | 值 |
|---|---|
| τ (plant time constant) | 0.2 s |
| Ts sweep (離散化) | 0.005 / 0.02 / 0.1 s |
| 取樣率 fs (aliasing demo) | 50 Hz（Nyquist 25 Hz） |
| 測試頻率 | 10 Hz（合法）/ 40 Hz（混疊） |
| Encoder CPR | 1000 |
| 量測窗 sweep (firmware) | 10 / 20 / 50 / 100 ms |

## Simulation
- 指令:`python maker-labs/05-z-domain/sim.py`
- 產生圖:`out/ch05_z_domain.png`（上:z 平面離散極點;下:aliasing 波形）
- 記錄:各 Ts 的離散係數、difference equation、離散極點 vs exp(-Ts/τ)、假頻 = |f - fs|。

## Hardware (ESP32)
- 韌體:`firmware/main.cpp`（共用 `encoder.h`,示範低速 encoder count quantization）
- BOM:ESP32 DevKit、正交編碼器馬達(或手轉編碼器)、麵包板。
- 接線:ENC_A→GPIO32、ENC_B→GPIO33（均 INPUT_PULLUP）。
- CSV 欄位:`t_ms,window_ms,raw_counts,cps,rpm,resolution_cps,latency_ms`
- 做法:低速轉動,比較不同 `window_ms` 下 raw_counts 的量化跳動與延遲。

## Result / Metrics
| Window (ms) | raw_counts | resolution (cps) | latency (ms) | 量化跳動觀察 |
|---|---|---|---|---|
| 10 | | 100.0 | 10 | |
| 20 | | 50.0 | 20 | |
| 50 | | 20.0 | 50 | |
| 100 | | 10.0 | 100 | |

| Ts (s) | 離散極點 |a1| | exp(-Ts/τ) | 是否穩定 |
|---|---|---|---|---|
| 0.005 | | | |
| 0.02 | | | |
| 0.1 | | | |

## Observation & Conclusion
- resolution 與 latency 的 trade-off 是否如預期?______
- aliasing:40 Hz 看起來是幾 Hz?若要正確量測需要多高的 fs?______
- 至少一個 model mismatch(如 ISR 抖動、count 邊緣噪聲、非等速):______
- 下一章(Ch6 controllers)如何用本章的 difference equation / Ts:______

## Exit Criteria
- [ ] 能不用背公式解釋 aliasing 與 quantization 的物理意義。
- [ ] `sim.py` 從乾淨環境可完整執行並通過 ✅。
- [ ] 圖表有單位、label 與可比較 baseline。
- [ ] 至少做過一次 parameter sweep（Ts 或 window_ms）。
- [ ] 韌體 host 編譯通過（`verify_firmware.sh`）。
- [ ] 若做硬體:保存原始 CSV。
- [ ] 能把一個離散 transfer function 手寫成 difference equation。
- [ ] 能指出至少一個 model mismatch。

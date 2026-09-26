# Ch16 Compliance and Resonance — Lab Report

> 對應 maker-labs 講義 Chapter 16。填完本表即完成本章 engineering notebook。

## Objective
理解柔性負載(彈性聯軸器/皮帶)造成的兩慣量系統:在頻域上出現
anti-resonance(dip)與 resonance(peak),並知道提高 gain 為何會激發機械模態、
如何用濾波(低通 / notch)與降低 bandwidth 來緩解。

## System
```
        馬達力矩 τ
           │
        ┌──▼──┐   k, b (spring/damper)   ┌─────┐
  τ ───►│ Jm  │~~~~~~~~~~~~~~~~~~~~~~~~~~~►│ Jl  │
        └──┬──┘        彈性 coupling      └─────┘
           │ ωm(collocated 量測:馬達端速度)
           ▼
        encoder → 速度回授 → (OnePoleLP 抗共振) → P 控制 → PWM
```
- Plant / Input / Output / Measurement:
  Plant = 兩慣量 + 彈簧阻尼;Input = 馬達力矩(PWM);
  Output = 負載端運動;Measurement = 馬達端速度 ωm(collocated encoder)。
- 哪些假設屬於 LTI?真實硬體在哪裡違反?
  假設 k、b 為常數線性;實際皮帶有非線性剛度、背隙(backlash)、
  Coulomb 摩擦、溫度漂移。

## Hypothesis
（執行 simulation 前先寫下預測）
- resonance 出現在 ω_r = sqrt(k(Jm+Jl)/(Jm·Jl)) ≈ ____ rad/s。
- anti-resonance 出現在 ω_ar = sqrt(k/Jl) ≈ ____ rad/s。
- 彈簧剛度 k 提高 → 共振頻率 ____(升/降)。

## Parameters
| 參數 | 值 |
|---|---|
| Jm(馬達慣量) | 0.002 kg·m² |
| Jl(負載慣量) | 0.006 kg·m² |
| k(彈簧剛度) | 20 N·m/rad |
| b(阻尼) | 0.03 N·m·s/rad |
| 速度回授低通截止 VEL_LP_HZ | 8 Hz |
| Control loop freq | 200 Hz(firmware) |

## Simulation
- 指令:`python maker-labs/16-resonance/sim.py`
- 產生圖:`out/ch16_resonance_bode.png`(Bode magnitude + phase,標出 peak/dip)。
- 記錄:量測 vs 解析的 resonance / anti-resonance 頻率、峰值/凹陷幅值比、
  峰值超過 rigid-body 趨勢線多少 dB、k sweep 後共振頻率移動。
- 參考輸出:anti-resonance ≈ 57.7 rad/s (9.2 Hz)、resonance ≈ 116 rad/s (18.5 Hz)、
  peak/dip ≈ 76 倍(+25.5 dB vs −12.1 dB)、k 20→60 使 resonance 116→200 rad/s。

## Hardware (ESP32)
- 韌體:`firmware/main.cpp`(encoder 速度回授 → OnePoleLP 抗共振 → P 控制 → PWM)。
- 使用共用 lib:`encoder.h`(QuadDecoder / VelocityEstimator)、`filter.h`(OnePoleLP)。
- BOM:ESP32 DevKit、帶 encoder 的低壓 DC motor、彈性 coupling 或皮帶帶動一個
  慣量盤(負載)、馬達驅動板、麵包板。
- 接線:ENC_A→GPIO32、ENC_B→GPIO33、PWM→GPIO25、DIR→GPIO26。
- 安全:採低能量柔性 demo,不做高速裸露旋轉;若無防護請以 simulation 為主。
- 收集 `run_*.csv`(欄位 `t,setpoint,rpm_raw,rpm_filt,error,pwm,saturated`)。

## Result / Metrics
| 指標 | Sim | Hardware |
|---|---|---|
| resonance 頻率 (rad/s) | | |
| anti-resonance 頻率 (rad/s) | | |
| peak/dip 幅值比 | | |
| 濾波後共振殘量 (rpm_raw vs rpm_filt) | — | |
| 提高 KP 到不穩定的臨界值 | | |

## Observation & Conclusion
- 結果與預期是否相同?至少一個 model mismatch:______
  (提示:皮帶背隙 / 非線性剛度 / 摩擦,模型都當成線性彈簧。)
- 一階低通與 notch 的取捨:低通把所有高頻(含頻寬內)都衰減、增加相位落後;
  notch 只挖掉共振頻段、保留頻寬,但需準確知道共振頻率。______
- 下一章如何建立在本章結果上:______

## Exit Criteria
- [ ] 能不用背公式解釋 resonance / anti-resonance 的物理意義。
- [ ] `sim.py` 從乾淨環境可完整執行並通過 ✅。
- [ ] 圖表有單位、label 與可比較 baseline(含 rigid-body 趨勢線與 k sweep)。
- [ ] 至少做過一次 parameter sweep(改 k、Jl 或 b,觀察 peak 移動)。
- [ ] 若做硬體:保存原始 CSV(同時含 rpm_raw 與 rpm_filt)。
- [ ] 能指出至少一個 model mismatch。

## 練習(Exercises)
1. 用 `filter.h` 現有元件做不到理想抗共振:實作一個 **notch(二階帶阻)** 濾波器,
   中心頻率設在共振頻率,和 OnePoleLP 比較「頻寬保留 vs 相位落後」。
2. 改 sim.py 的 Jl 或 b,先寫下預測再驗證 peak/dip 移動方向。
3. 在 firmware 逐步提高 KP,量出激發共振(rpm 開始振盪)的臨界 gain,
   再開/關低通比較臨界 gain 差多少。

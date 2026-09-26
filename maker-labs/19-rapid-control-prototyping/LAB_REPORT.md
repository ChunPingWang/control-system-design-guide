# Ch19 Rapid Control Prototyping (RCP) — Lab Report

> 對應 maker-labs 講義 Chapter 19。本章是里程碑 **Project 5「Mini RCP Platform」** 的前導/平台:
> 建立 PC(Python supervisory)+ ESP32(deterministic control)的快速控制原型環境。
> 填完本表即完成本章 engineering notebook。

## Objective
建立一套 Rapid Control Prototyping 環境,縮短 `model → controller → experiment → diagnosis` 的迭代:
PC 端負責 setpoint / 參數設定 / logging / plot / 自動實驗;ESP32 端負責固定週期 PID 轉速迴路。
核心觀念:**即時控制迴路留在 ESP32,PC 只做 supervisory,不用一般 USB serial 扛硬即時。**

## System
```
          PC (Python / Jupyter)                       ESP32 (deterministic)
  ┌──────────────────────────────┐   Serial   ┌──────────────────────────────┐
  │ setpoint / Kp Ki Kd 設定      │ ─────────▶ │ 命令解析(非即時, tick 外)    │
  │ experiment runner (sweep)    │            │ 100 Hz 固定週期 PID 迴路      │
  │ CSV logging / metrics / plot │ ◀───────── │ encoder → RPM → PID → PWM     │
  │ report 產生                   │  CSV 遙測  │ safety: STOP 預設 + 看門狗    │
  └──────────────────────────────┘            └──────────────────────────────┘
                                                 Motor + Encoder
```
- Plant / Input / Output / Measurement:______(DC motor 轉速 / PWM duty / 轉速 rpm / encoder counts→rpm)
- 哪些假設屬於 LTI?真實硬體在哪裡違反?______(死區、摩擦、PWM 飽和、量化、序列延遲)

## Hypothesis
（執行 simulation / 硬體實驗前先寫下預測)
- 閉迴路(含積分)能消除穩態誤差、收斂到 setpoint:______
- 掃描 Kp 時,overshoot 與 RMSE 會如何變化:______

## Model and Assumptions
- 馬達近似一階:`τ·rpm' = -rpm + K_motor·pwm`,`τ ≈ 0.12 s`。
- PID 與韌體同一份語意(先積分、對量測微分、輸出飽和 ±255、條件積分 anti-windup、導數低通 20 Hz)。
- 忽略:序列傳輸抖動、encoder 量化雜訊、驅動器死區。

## Parameters
| 參數 | 值 |
|---|---|
| 控制週期 CONTROL_US | 10 ms（100 Hz) |
| τ (motor time constant) | 0.12 s |
| K_motor | 600/255 rpm per duty |
| Setpoint | 300 rpm |
| PID 預設 (Kp, Ki, Kd) | 0.4, 6.0, 0.01 |
| 導數低通 | 20 Hz |
| 輸出飽和 | ±255 |
| Command timeout | 2000 ms |
| Kp sweep | 0.05, 0.10, 0.20, 0.40, 0.80 |

## Simulation
- 指令:`python maker-labs/19-rapid-control-prototyping/sim.py`
- 產生圖:`out/ch19_rcp.png`(左:主迴路響應;右:Kp sweep 的 overshoot / RMSE 趨勢)
- 記錄:主迴路終值/step metrics、實驗自動化 summary table(每個 Kp 的 rise/OS/settle/RMSE/終值)。
- 驗證:閉迴路收斂到 setpoint;掃描 Kp 時 overshoot 與 RMSE 單調不增;PWM 被飽和在 ±255。

## Hardware Setup (ESP32)
- 韌體:`firmware/main.cpp`(共用 `pid.h` + `encoder.h`;固定週期 PID + serial 命令解析 + 安全停機)。
- BOM:ESP32 DevKit、DC motor + 驅動器(如 TB6612 / DRV8833)、正交 encoder、電源、麵包板。
- 接線:ENC_A→GPIO32、ENC_B→GPIO33、PWM→GPIO25、DIR→GPIO26。
- Serial 命令:`KP <v>` / `KI <v>` / `KD <v>` / `SP <v>` / `RUN` / `STOP`(每行以 `\n` 結尾)。
- 安全:開機預設 STOP;`RUN` 才輸出;2 秒沒收到命令自動停機;輸出永遠飽和在 ±255。

## Raw Data
- 收集 `run_*.csv`,欄位:`t_ms,state,setpoint,rpm,error,pwm,kp,ki,kd,saturated`。
- 保存原始 CSV,不要只存截圖(processed 與 raw 分開存)。

## Result / Metrics
| 指標 | Sim | Hardware |
|---|---|---|
| Rise time | | |
| Overshoot | | |
| Settling time | | |
| Steady-state error | | |
| RMSE | | |

## Simulation vs Hardware
（至少三個 model mismatch 來源)
- ______(序列延遲 / jitter)
- ______(摩擦、死區 → 低速追不上)
- ______(encoder 量化、雜訊 → 速度估測抖動)

## Diagnosis
- 哪一張圖最能證明改善(而非「感覺比較順」)？______
- 哪些參數改變會影響 stability / noise / performance？______

## Observation & Conclusion
- 結果與預期是否相同?至少一個 model mismatch:______
- RCP 迭代帶來的好處(改參數→重跑實驗的成本降了多少)：______

## Next Experiment
- 整合成 Project 5「Mini RCP Platform」：PC 一鍵 sweep、自動存 CSV、自動出報告。
- 下一步可換 plant(compliant load)或加 cascade / observer,沿用同一 RCP 流程。

## Exit Criteria
- [ ] 能不用背公式解釋 RCP 的價值與分層(即時留在 ESP32,supervisory 在 PC)。
- [ ] `sim.py` 從乾淨環境可完整執行並通過 ✅。
- [ ] 圖表有單位、label 與可比較 baseline。
- [ ] 至少做過一次 parameter sweep（本章掃描 Kp）。
- [ ] firmware 有安全預設(STOP)、command timeout、輸出飽和。
- [ ] 若做硬體:保存原始 CSV 與 firmware commit/hash。
- [ ] 能指出至少一個 model mismatch。

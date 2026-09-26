# Ch8 Feed-Forward — Lab Report

> 對應 maker-labs 講義 Chapter 8。主題:plant-based feed-forward、converter compensation、command delay、double-integrating plant。

## Objective
理解 feed-forward 如何利用已知 plant/command 預先提供 control effort:由 open-loop 建立 `target_rpm → baseline_pwm` 模型,讓 PID 只修正 residual,使命令追蹤更快。FF 不取代 feedback。

## System
```
                    ┌── Feed-Forward(target→baseline PWM)──┐
                    │                                        ▼
Target RPM → (+) → PID(修正 residual) ──────────(+)──→ PWM → Motor → Encoder → Actual RPM
               ↑                                                                    │
               └────────────────────────────────────────────────────────────────────┘
```
- Plant / Input / Output / Measurement:______
- 哪些假設屬於 LTI?真實硬體在哪裡違反(converter 非線性、dead-time)?______

## Hypothesis
（執行前先預測:FF+PID 相對 PID-only 的 rise time、overshoot、穩態誤差差異）

## Parameters
| 參數 | 值 |
|---|---|
| Plant（sim） | 1/(0.3s+1) |
| Controller（sim） | C(s)=(3s+6)/s = 3 + 6/s |
| Feed-forward | F = 1/DCgain(G) = 1（靜態） |
| FF 校準點（firmware） | (100rpm,40duty),(500rpm,200duty) |
| Kp / Ki / Kd（firmware，FF+PID） | 0.2 / 3.0 / 0.005 |
| Derivative LPF | 20 Hz |
| Control loop | 100 Hz |
| Encoder CPR | 1000 |

## Simulation
- 指令:`python maker-labs/08-feedforward/sim.py`
- 產生圖:`out/ch08_feedforward.png`（feedback vs feedback+FF step overlay）
- 記錄:兩者 DC 增益、rise time、overshoot、命令追蹤誤差積分(IAE)。
- 驗證:兩者穩態相同,FF+FB 的 rise 顯著變快、IAE 下降。

## Hardware (ESP32)
- 韌體:`firmware/main.cpp`（共用 `feedforward.h` + `pid.h` + `encoder.h`）
- CSV 欄位:`t,setpoint,rpm,error,pwm_ff,pwm_pid,pwm,saturated`
- 流程:先做 open-loop step test 量 `(target_rpm, steady_pwm)` 兩點 → 更新 `ff.calibrate()` → 再跑 FF+PID。
- 比較 PID-only 與 FF+PID:同一 setpoint 跳變,看 rise time 與積分器負擔(pwm_pid 大小)。

## Result / Metrics
| 指標 | Sim (FB) | Sim (FB+FF) | HW (PID-only) | HW (FF+PID) |
|---|---|---|---|---|
| Rise time | | | | |
| Overshoot | | | | |
| Settling time | | | | |
| Steady-state error | | | | |
| Command IAE | | | | |

## Observation & Conclusion
- FF 是否加快命令追蹤而不改變穩態?PID 積分器負擔是否下降?______
- Model mismatch 來源(至少三個):converter 非線性、command delay/dead-time、負載/摩擦變動、電池電壓下降。______
- FF 模型不準時會怎樣?（殘差變大,靠 PID 收尾,穩態仍對）______
- 下一章(Ch9 filters)如何延伸:______

## Exit Criteria
- [ ] 能不用背公式解釋 feed-forward 的物理意義(先給 baseline effort)。
- [ ] `sim.py` 從乾淨環境可完整執行並通過 ✅。
- [ ] 圖表有單位、label 與可比較 baseline(FB vs FB+FF)。
- [ ] 至少做過一次 parameter sweep（改 FF 增益或 PID 增益）。
- [ ] 若做硬體:保存 open-loop 校準與 FF+PID 的 `run_*.csv`。
- [ ] 能指出至少一個 model mismatch。
- [ ] 能說明為何 FF 不能取代 feedback。

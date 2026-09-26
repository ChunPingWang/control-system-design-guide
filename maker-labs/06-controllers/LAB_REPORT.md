# Ch6 Four Types of Controllers — Lab Report

> 對應 maker-labs 講義 Chapter 6。里程碑專案 1:DC Motor PID Speed Controller。

## Objective
比較 P / PI / PD / PID,理解 P（現在誤差）、I（累積誤差、消除穩態誤差）、D（誤差變化率、增加 damping）各自角色。

## System
```
Target RPM → (+) → PID → PWM → Motor → Encoder → Actual RPM
               ↑                                      │
               └──────────────────────────────────────┘
```

## Hypothesis
（先預測 P/PI/PD/PID 各自 overshoot、steady-state error）

## Parameters
| 參數 | 值 |
|---|---|
| Plant | 1/(0.4s+1)（sim） |
| Kp / Ki / Kd | 0.4 / 6.0 / 0.01（firmware 初值） |
| Derivative LPF | 20 Hz |
| Control loop | 100 Hz |
| Encoder CPR | 1000 |

## Simulation
- 指令:`python maker-labs/06-controllers/sim.py`
- 圖:`out/ch06_controllers.png`（P/PI/PD/PID 疊圖）
- 記錄各 controller 的 DC gain、sse、overshoot、rise time。

## Hardware (ESP32)
- 韌體:`firmware/main.cpp`（共用 `pid.h` + `encoder.h`）
- CSV 欄位:`t,setpoint,rpm,error,pwm,p,i,d,saturated`
- 每次只改一個 control action，建立因果直覺（勿用 auto-tune 跳過比較）。

## Result / Metrics
| Controller | Rise | Overshoot | Settling | SSE |
|---|---|---|---|---|
| P | | | | |
| PI | | | | |
| PD | | | | |
| PID | | | | |

## Observation & Conclusion
- I 是否消除 steady-state error?D 是否放大 noise?______
- 一個 model mismatch:______
- 下一章(Ch7 disturbance)如何延伸:______

## Exit Criteria
- [ ] `sim.py` 通過 ✅。
- [ ] 韌體 host 編譯通過（`verify_firmware.sh`）。
- [ ] 完成 P→PI→PD→PID 至少一輪比較。
- [ ] 硬體:保存 `run_*.csv` 與至少一張 response plot。
- [ ] 能說明 anti-windup 與 derivative filter 的必要性。

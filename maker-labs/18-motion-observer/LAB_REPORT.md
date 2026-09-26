# Ch18 Using the Luenberger Observer in Motion Control — Lab Report

> 對應 maker-labs 講義 Chapter 18。填完本表即完成本章 engineering notebook。

## Objective
把 Ch10 的 Luenberger observer 從「離線估計」推進到「真正進 feedback loop」:
用 observer 估的 velocity 當速度回授,取代直接對 noisy position 差分的
finite-difference velocity,理解 observer 如何在 noise 與 lag 之間取得更好折衷,
讓控制命令更平滑、迴路更穩、追蹤更準。

## System
```
                      ┌──────── Luenberger Observer ────────┐
                      │  xhat += dt(A xhat + B u) + L(y-Cxhat)│
                      │        → velocity estimate            │
                      └───────────────▲──────────┬───────────┘
                                      │ y=pos     │ v_hat
Setpoint(v) → (+) → PID → u → Plant(motion) ──────┘ (feedback)
                 ↑                     │
                 └──── observer v_hat ─┘        Measurement: encoder position(含噪)
```
- Plant / Input / Output / Measurement:motion(pos,vel)/ 命令 u / velocity / encoder position。
- 哪些假設屬於 LTI?真實硬體在哪裡違反?______(摩擦、backlash、PWM 非線性、量化)

## Hypothesis
（執行 simulation 前先寫下預測）
- observer velocity 對 true velocity 的 RMS 誤差 << finite-difference。
- observer 迴路的控制命令抖動 << 差分迴路。
- 兩迴路都穩定,但差分迴路因噪聲被命令飽和整流而追蹤偏差較大。

## Parameters
| 參數 | 值 |
|---|---|
| Plant A / B / C | [[0,1],[0,-2]] / [[0],[2]] / [1,0] |
| Observer poles | -20, -25 |
| Observer gain L | [43, 414]（`ct.place` 求得,firmware 直接帶入） |
| Velocity PID | Kp=0.6, Ki=6.0, Kd=0 |
| Command clamp | ±20 |
| Measurement noise (position) | σ=0.01 |
| Control loop freq | 100 Hz（sim dt=1 ms 積分） |
| Noise seed | 20180924（FIXED，可重現） |

## Simulation
- 指令:`MPLBACKEND=Agg .venv/bin/python maker-labs/18-motion-observer/sim.py`
- 產生圖:`out/ch18_motion_observer.png`
- 記錄:回授速度 RMS 誤差、控制命令抖動 std(Δu)、穩態 true velocity 平均。

## Hardware (ESP32)
- 韌體:`firmware/main.cpp`（encoder → observer → velocity PID → PWM，closed-loop）。
- BOM:ESP32 DevKit、帶 encoder 的 DC motor、motor driver、電源、麵包板。
- 接線:ENC_A→GPIO32、ENC_B→GPIO33、PWM→GPIO25。
- 收集 `run_*.csv`（欄位 `t_ms,pos,vel_sp,fd_vel,obs_vel,u`）。
- 建議:先只記錄(不驅動)比較 `fd_vel` 與 `obs_vel`,確認 observer 可信再閉迴路。

## Result / Metrics
| 指標 | Sim | Hardware |
|---|---|---|
| 回授速度 RMS 誤差(observer) | 0.014 | |
| 回授速度 RMS 誤差(finite-diff) | 13.9 | |
| 控制命令抖動 std(Δu)(observer) | 0.003 | |
| 控制命令抖動 std(Δu)(finite-diff) | 14.1 | |
| 穩態 velocity(observer / fd，sp=5) | 5.08 / 4.01 | |

## Observation & Conclusion
- observer 回授對 true velocity 幾乎無噪聲;差分回授把量測噪聲放大 1/dt 倍後直接灌進命令。
- 差分迴路穩態 velocity 偏低(4.0 vs 5.0):噪聲被命令飽和 + anti-windup 整流成偏差。
- 至少一個 model mismatch:______（sim 的 -2 極點 vs 真實馬達時間常數/摩擦）。
- 下一章如何建立在本章結果上:______（Ch19 RCP 把此迴路做快速原型化）。

## Exit Criteria
- [ ] 能不用背公式解釋本章物理意義(observer 在 noise 與 lag 間取折衷)。
- [ ] `sim.py` 從乾淨環境可完整執行並通過 ✅。
- [ ] 圖表有單位、label 與可比較 baseline(observer vs finite-diff)。
- [ ] 至少做過一次 parameter sweep(掃 observer poles:slow vs fast)。
- [ ] 若做硬體:保存原始 CSV。
- [ ] 能指出至少一個 model mismatch。

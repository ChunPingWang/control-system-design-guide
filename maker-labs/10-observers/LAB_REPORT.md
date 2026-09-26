# Ch10 Introduction to Observers — Lab Report

> 對應 maker-labs 講義 Chapter 10。填完本表即完成本章 engineering notebook。

## Objective
用 Luenberger observer,由「只有 position 的含噪量測」估計整個 state
(position + velocity),理解 observer poles(A-LC）如何決定收斂速度與抗噪能力。

## System
```
        u ─────────────┐
                        ▼
Setpoint      ┌──────────────┐   y = position (+ noise)
   (n/a)      │ Plant (A,B,C)│ ───────────────┐
              └──────────────┘                 │
                        ┌────────────────────┐ │
   xhat (est) ◄─────────│ Observer  xhat +=  │◄┘
   [pos, vel]           │ dt(A xhat + B u)   │
                        │  + L(y - C xhat)   │
                        └────────────────────┘
```
- Plant / Input / Output / Measurement:二階運動模型 / 力矩類輸入 u / [position, velocity] / 只量測 position。
- 哪些假設屬於 LTI?真實硬體在哪裡違反?______(摩擦、量化、backlash、量測噪聲）

## Hypothesis
（執行 simulation 前先寫下預測:observer poles 拉快後,誤差多久收斂?噪聲會怎樣?）

## Parameters
| 參數 | 值 |
|---|---|
| A | [[0, 1], [0, -2]] |
| B | [[0], [2]] |
| C | [1, 0]（只量測 position） |
| Desired observer poles | -6, -7 |
| Observer gain L | [11, 20]（`ct.place` 求得，firmware 帶入） |
| 量測噪聲 σ | 0.01 |
| Control / observer loop freq | 100 Hz |

## Simulation
- 指令:`python maker-labs/10-observers/sim.py`
- 產生圖:`out/ch10_observer.png`
- 記錄:eig(A-LC) 是否等於指定 poles、估計誤差 early→late 收斂、observer 速度 vs finite-difference 速度的 RMSE。

## Hardware (ESP32)
- 韌體:`firmware/main.cpp`(`observer.h` Observer2 + `encoder.h`,離線比對，不進控制迴路）。
- BOM:ESP32 DevKit、正交編碼器（或帶編碼器的 DC motor）、麵包板。
- 接線:ENC_A→GPIO32、ENC_B→GPIO33。
- 收集 `run_*.csv`(欄位 `t,pos,fd_vel,obs_vel`)。
- ⚠️ observer 未驗證前不要放進 safety-critical loop。

## Result / Metrics
| 指標 | Sim | Hardware |
|---|---|---|
| eig(A-LC) = poles? | | |
| 估計誤差 early → late | | |
| 速度估計 RMSE(observer） | | |
| 速度估計 RMSE(finite-diff） | | |

## Observation & Conclusion
- observer 速度是否比 finite-difference 更平滑/更準?為什麼?______
- 結果與預期是否相同?至少一個 model mismatch:______
- observer poles 太快會發生什麼(放大噪聲/model mismatch)?______
- 下一章如何建立在本章結果上:______

## Exit Criteria
- [ ] 能不用背公式解釋 observer 與 A-LC error dynamics 的物理意義。
- [ ] `sim.py` 從乾淨環境可完整執行並通過 ✅。
- [ ] 圖表有單位、label 與可比較 baseline（true / measured / estimated）。
- [ ] 至少做過一次 parameter sweep（改 observer poles 或量測噪聲）。
- [ ] 若做硬體:保存原始 CSV。
- [ ] 能指出至少一個 model mismatch。

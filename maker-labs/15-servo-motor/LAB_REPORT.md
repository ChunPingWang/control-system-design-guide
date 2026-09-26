# Ch15 Basics of the Electric Servomotor and Drive — Lab Report

> 對應 maker-labs 講義 Chapter 15。填完本表即完成本章 engineering notebook。

## Objective
理解電動伺服馬達不是單純 gain:同時具備電氣 dynamics(R, L, Ke, Kt)與機械 dynamics(J, B)。
掌握兩個關鍵概念:(1) 電氣時間常數 τe = L/R 遠小於機械時間常數 τm = J/B;
(2) 伺服驅動器的 **cascade(串級)** 架構 —— 內層電流/轉矩迴路頻寬遠大於外層速度迴路。

## System
```
              外層(慢)                     內層(快)
ω_ref → (+) → Velocity PID → i_ref → (+) → Current P → V → Motor → i, ω
          ↑                              ↑              (R,L,Ke) (Kt,J,B)
          └────── speed (encoder) ───────┘── current (sensor/model) ──┘
```
- Plant / Input / Output / Measurement:______
  - Plant:DC 伺服馬達(電氣 + 機械);Input:電壓 V(PWM);Output:角速度 ω;
    Measurement:encoder → RPM,電流 i(current sensor 或電氣模型估算)。
- 哪些假設屬於 LTI?真實硬體在哪裡違反?______
  - LTI 假設:R/L/Ke/Kt/J/B 為常數、線性;違反來源:磁飽和、摩擦非線性(靜/庫倫)、
    PWM 死區、供電壓降、溫升改變 R。

## Hypothesis
（執行 simulation 前先寫下預測）
- τm/τe 大約會差幾個數量級?內層頻寬會是外層的幾倍?

## Parameters
| 參數 | 值 |
|---|---|
| R (電樞電阻) | 2.0 Ω |
| L (電感) | 0.01 H |
| Ke (反電動勢常數) | 0.08 V·s/rad |
| Kt (轉矩常數) | 0.08 N·m/A |
| J (轉動慣量) | 0.002 kg·m² |
| B (黏滯摩擦) | 0.002 N·m·s/rad |
| 電氣時間常數 τe = L/R | 5 ms |
| 機械時間常數 τm = J/B | 1000 ms |
| 內層電流迴路更新率 | 1 kHz |
| 外層速度迴路更新率 | 100 Hz |

## Simulation
- 指令:`python maker-labs/15-servo-motor/sim.py`
- 產生圖:`out/ch15_servo_motor.png`
- 記錄:τe / τm 比值、開迴路電流峰值時間 vs 速度 63% 時間、內/外層閉迴路 -3dB 頻寬。

## Hardware (ESP32)
- 韌體:`firmware/main.cpp`(外層速度 PID → i_ref;內層電流 P → PWM 電壓,cascade）。
- BOM:ESP32 DevKit、低壓 DC gearmotor + motor driver、正交 encoder、
  (選配)current sensor、獨立低壓電源、麵包板。
- 接線:ENC_A→GPIO32、ENC_B→GPIO33、PWM→GPIO25、DIR→GPIO26、I_SENSE→GPIO34。
- 收集 `run_*.csv`(欄位 `t,sp_rpm,rpm,err_rpm,i_ref,i_meas,duty,sat`)。
- 安全:僅用低壓 DC,勿接觸高功率/市電;電流命令有 ±I_MAX 保護。

## Result / Metrics
| 指標 | Sim | Hardware |
|---|---|---|
| τe (電氣時間常數) | 5 ms | |
| τm (機械時間常數) | 1000 ms | |
| 內層電流迴路頻寬 | | |
| 外層速度迴路頻寬 | | |
| 內/外頻寬比 | | |

## Observation & Conclusion
- 結果與預期是否相同?內層是否確實遠快於外層?______
- 至少一個 model mismatch:______(例:摩擦非線性、電流估算 vs 真實量測差異)
- 下一章如何建立在本章結果上:______(位置迴路 / observer / RCP)

## Exit Criteria
- [ ] 能不用背公式解釋:為何電機不是單純 gain,電氣與機械 dynamics 各扮演什麼角色。
- [ ] `sim.py` 從乾淨環境可完整執行並通過 ✅。
- [ ] 圖表有單位、label 與可比較 baseline(電氣 vs 機械、內層 vs 外層)。
- [ ] 至少做過一次 parameter sweep（改 L、J 或內/外層增益,觀察頻寬變化）。
- [ ] 能說明 cascade 為何要求內層頻寬 >> 外層頻寬。
- [ ] 若做硬體:保存原始 CSV。
- [ ] 能指出至少一個 model mismatch。

# Roadmap

## 已完成(2026-09-24)
- [x] M1 基礎:回授/Bode/調機(Ch1–3),DSA 量測管線(common/dsa.py)
- [x] M2 數位控制:延遲/z 域/離散 PID(Ch4–5)
- [x] M3 迴路設計:控制器/擾動/前饋/濾波/觀測器(Ch6–10)
- [x] M4 建模:DC 馬達/非線性/模型驗證(Ch11–13)
- [x] M5 運動控制:編碼器/伺服/共振/位置/運動觀測器(Ch14–18)
- [x] M6 RCP 軟體側:辨識 → λ-tuning → 模擬驗證 → 韌體等價性(Ch19)
- [x] 韌體:pid.h + main.cpp(編碼器 ISR、TB6612、序列協定)+ host 端等價測試
- [x] 全量驗證:19/19 notebook 通過(tools/run_all_labs.sh)
- [x] C/C++ 版本:`cpp/` 19 章 C++17 程式 + header-only 函式庫,19/19 驗證通過(cpp/run_all.sh)

## 硬體階段(待硬體到位)
- [ ] H1 編碼器讀取與標定(counts/rev 實測)
- [ ] H2 PWM→轉速開迴路特性(Ch19 STEP 命令 + 擬合)
- [ ] H3 速度 PID 閉迴路 + 實機 zone-based 調機(Ch3 流程)
- [ ] H4 位置串級迴路(Ch17 流程)
- [ ] H5 實機 DSA:chirp 量測閉迴路 FRF,與模擬比對(Ch13 流程)
- [ ] Capstone:MPU6050 自平衡機器人(觀測器 + 串級)

## 選配
- [ ] ipywidgets 滑桿互動調參(Live Constant 體驗)
- [ ] CI:GitHub Actions 跑 run_all_labs.sh 與 cpp/run_all.sh 回歸

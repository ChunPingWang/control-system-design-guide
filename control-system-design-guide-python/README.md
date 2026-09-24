# Control System Design Guide — Python 實驗教材

George Ellis《Control System Design Guide》(4th ed.) 的現代化開源實驗環境:
以 **Python + Jupyter + python-control + SciPy** 取代原書的 Visual ModelQ,
以 **ESP32** 取代商用 RCP 硬體。19 章、每章一份可執行、可驗證的 lab notebook。

> 總覽、章節對照與驗證報告見上層目錄的
> [`control-system-design-guide-study-guide-v2.md`](../control-system-design-guide-study-guide-v2.md)。

## 安裝與執行

```bash
python -m venv .venv                # 建議 Python 3.11/3.12
source .venv/bin/activate
pip install -r requirements.txt
jupyter lab                         # 開任一章的 lab.ipynb
```

一鍵重跑全部驗證(19 份 notebook,任一章 assertion 失敗即報錯):

```bash
./tools/run_all_labs.sh
```

## 目錄結構

```
common/                  ModelQ-lite 共用函式庫
├── control_helpers.py     解析工具:plants、PID、margins、step metrics
├── sim.py                 逐樣本模擬:DiscretePID、MotorPlant、DCMotorPlant、
│                          TwoMassPlant、Backlash、EncoderModel、Delay…
├── dsa.py                 DSA:chirp/PRBS 激發 + Welch 交叉頻譜 FRF 量測
└── plots.py               Bode 疊圖(解析系統與量測 FRF 同圖)

01-introduction/  …  19-rapid-control-prototyping/    每章一份 lab.ipynb

hardware/esp32/          第 19 章 RCP 韌體(PlatformIO)
├── src/pid.h              PID 邏輯(純 C++,host 可測)
├── src/main.cpp            編碼器 ISR、TB6612 PWM、序列命令與遙測
└── test_host/              host 端 C/Python 逐樣本等價性測試
```

## 章節一覽

| 章 | 主題 | 章 | 主題 |
|---|---|---|---|
| 01 | 回授導論 | 11 | 建模入門(DC 馬達) |
| 02 | 頻域分析 + DSA | 12 | 非線性(windup/摩擦/背隙) |
| 03 | zone-based 調機 | 13 | 模型驗證(量測 vs 模型) |
| 04 | 取樣與延遲 | 14 | 編碼器與量化雜訊 |
| 05 | z 域與離散化 | 15 | 伺服馬達與電流迴路 |
| 06 | P/PI/PID/PID+ | 16 | 機構柔性與共振 |
| 07 | 擾動響應與解耦 | 17 | 位置迴路(串級 vs PID) |
| 08 | 前饋 | 18 | 運動控制觀測器 |
| 09 | 迴路內濾波器 | 19 | RCP(ESP32) |
| 10 | Luenberger 觀測器 | | |

每章結構:理論重點 → 實驗(圖 + 數值表)→ **✅ 驗證 cell(assertion)** → 練習。

本教材為原創內容,不含原書文字、圖表或 ModelQ 專有檔案;請搭配原書使用。

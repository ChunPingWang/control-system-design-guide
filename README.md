# Control System Design Guide — Python / C++ 實驗教材

George Ellis《Control System Design Guide》(4th ed.) 的現代化開源實驗環境:
以開源軟體取代原書的 Visual ModelQ,以 **ESP32** 取代商用 RCP 硬體。
全書 19 章,提供 **Python 與 C/C++ 兩個對稱版本**,章節目錄一一對應、
實驗參數與 ✅ 驗證條件相同。

| 版本 | 路徑 | 技術 | 執行 | 說明 |
|---|---|---|---|---|
| Python | [`python/`](python/README.md) | Jupyter + python-control + SciPy | `python/run_all.sh` | 每章一份 `lab.ipynb`,圖表內嵌,適合互動學習 |
| C/C++ | [`cpp/`](cpp/README.md) | C++17,header-only,零外部相依 | `cpp/run_all.sh` | 每章一支 `main.cpp`,輸出 CSV,貼近嵌入式 / 韌體 |

> 總覽、章節對照與驗證報告見
> [`control-system-design-guide-study-guide-v2.md`](control-system-design-guide-study-guide-v2.md)。

## 快速開始

```bash
# Python 版
python -m venv .venv && source .venv/bin/activate   # 建議 Python 3.11/3.12
pip install -r python/requirements.txt
jupyter lab python/                                 # 開任一章的 lab.ipynb

# C++ 版(只需 C++17 編譯器,CMake 選用)
./cpp/run_all.sh
```

一鍵重跑全部驗證(Python 19 章 + 韌體 host 測試 + C++ 19 章,任一失敗即回報):

```bash
./tools/run_all_labs.sh
```

## 目錄結構

```
python/                  Python 版
├── common/                ModelQ-lite 共用函式庫(control_helpers / sim / dsa / plots)
├── 01-introduction/ … 19-rapid-control-prototyping/   每章一份 lab.ipynb
├── requirements.txt
└── run_all.sh             執行 19 份 notebook 驗證

cpp/                     C/C++ 版(與 python/ 對稱)
├── common/                header-only 函式庫(lti / sim / dsa / linalg / util)
├── 01-introduction/ … 19-rapid-control-prototyping/   每章一支 main.cpp
├── tools/plot_csv.py      CSV → 圖(選用)
├── CMakeLists.txt
└── run_all.sh             建置 + 執行 19 章驗證

hardware/esp32/          第 19 章 RCP 韌體(PlatformIO,兩版共用)
├── src/pid.h              PID 邏輯(純 C++,host 可測)
├── src/main.cpp           編碼器 ISR、TB6612 PWM、序列命令與遙測
└── test_host/             host 端 C/Python 逐樣本等價性測試

tools/run_all_labs.sh    全部驗證(兩版 + 韌體)
```

## 章節一覽

每章資料夾內都有 `README.md`:Python 版為完整章節說明(理論、實驗設計、結果、驗證條件、練習),
C++ 版為程式導讀(程式結構、Python↔C++ 對照、CSV 輸出、實際執行結果)。

| 章 | 主題 | Python 說明 | C++ 說明 |
|---|---|---|---|
| 01 | 回授導論 | [README](python/01-introduction/README.md) · [notebook](python/01-introduction/lab.ipynb) | [README](cpp/01-introduction/README.md) · [main.cpp](cpp/01-introduction/main.cpp) |
| 02 | 頻域分析 + DSA | [README](python/02-frequency-domain/README.md) · [notebook](python/02-frequency-domain/lab.ipynb) | [README](cpp/02-frequency-domain/README.md) · [main.cpp](cpp/02-frequency-domain/main.cpp) |
| 03 | zone-based 調機 | [README](python/03-tuning/README.md) · [notebook](python/03-tuning/lab.ipynb) | [README](cpp/03-tuning/README.md) · [main.cpp](cpp/03-tuning/main.cpp) |
| 04 | 取樣與延遲 | [README](python/04-sampling-delay/README.md) · [notebook](python/04-sampling-delay/lab.ipynb) | [README](cpp/04-sampling-delay/README.md) · [main.cpp](cpp/04-sampling-delay/main.cpp) |
| 05 | z 域與離散化 | [README](python/05-z-domain/README.md) · [notebook](python/05-z-domain/lab.ipynb) | [README](cpp/05-z-domain/README.md) · [main.cpp](cpp/05-z-domain/main.cpp) |
| 06 | P/PI/PID/PID+ | [README](python/06-controllers/README.md) · [notebook](python/06-controllers/lab.ipynb) | [README](cpp/06-controllers/README.md) · [main.cpp](cpp/06-controllers/main.cpp) |
| 07 | 擾動響應與解耦 | [README](python/07-disturbance/README.md) · [notebook](python/07-disturbance/lab.ipynb) | [README](cpp/07-disturbance/README.md) · [main.cpp](cpp/07-disturbance/main.cpp) |
| 08 | 前饋 | [README](python/08-feedforward/README.md) · [notebook](python/08-feedforward/lab.ipynb) | [README](cpp/08-feedforward/README.md) · [main.cpp](cpp/08-feedforward/main.cpp) |
| 09 | 迴路內濾波器 | [README](python/09-filters/README.md) · [notebook](python/09-filters/lab.ipynb) | [README](cpp/09-filters/README.md) · [main.cpp](cpp/09-filters/main.cpp) |
| 10 | Luenberger 觀測器 | [README](python/10-observers/README.md) · [notebook](python/10-observers/lab.ipynb) | [README](cpp/10-observers/README.md) · [main.cpp](cpp/10-observers/main.cpp) |
| 11 | 建模入門(DC 馬達) | [README](python/11-modeling/README.md) · [notebook](python/11-modeling/lab.ipynb) | [README](cpp/11-modeling/README.md) · [main.cpp](cpp/11-modeling/main.cpp) |
| 12 | 非線性(windup/摩擦/背隙) | [README](python/12-nonlinear/README.md) · [notebook](python/12-nonlinear/lab.ipynb) | [README](cpp/12-nonlinear/README.md) · [main.cpp](cpp/12-nonlinear/main.cpp) |
| 13 | 模型驗證(量測 vs 模型) | [README](python/13-model-verification/README.md) · [notebook](python/13-model-verification/lab.ipynb) | [README](cpp/13-model-verification/README.md) · [main.cpp](cpp/13-model-verification/main.cpp) |
| 14 | 編碼器與量化雜訊 | [README](python/14-encoder/README.md) · [notebook](python/14-encoder/lab.ipynb) | [README](cpp/14-encoder/README.md) · [main.cpp](cpp/14-encoder/main.cpp) |
| 15 | 伺服馬達與電流迴路 | [README](python/15-servo-motor/README.md) · [notebook](python/15-servo-motor/lab.ipynb) | [README](cpp/15-servo-motor/README.md) · [main.cpp](cpp/15-servo-motor/main.cpp) |
| 16 | 機構柔性與共振 | [README](python/16-resonance/README.md) · [notebook](python/16-resonance/lab.ipynb) | [README](cpp/16-resonance/README.md) · [main.cpp](cpp/16-resonance/main.cpp) |
| 17 | 位置迴路(串級 vs PID) | [README](python/17-position-control/README.md) · [notebook](python/17-position-control/lab.ipynb) | [README](cpp/17-position-control/README.md) · [main.cpp](cpp/17-position-control/main.cpp) |
| 18 | 運動控制觀測器 | [README](python/18-motion-observer/README.md) · [notebook](python/18-motion-observer/lab.ipynb) | [README](cpp/18-motion-observer/README.md) · [main.cpp](cpp/18-motion-observer/main.cpp) |
| 19 | RCP(ESP32) | [README](python/19-rapid-control-prototyping/README.md) · [notebook](python/19-rapid-control-prototyping/lab.ipynb) | [README](cpp/19-rapid-control-prototyping/README.md) · [main.cpp](cpp/19-rapid-control-prototyping/main.cpp) |

- Python 版每章:理論重點 → 實驗(圖 + 數值表)→ **✅ 驗證 cell(assertion)** → 練習。
- C++ 版每章:實驗(數值表 + CSV)→ **✅ 驗證(CHECK)** → 以結束碼回報。

本教材為原創內容,不含原書文字、圖表或 ModelQ 專有檔案;請搭配原書使用。

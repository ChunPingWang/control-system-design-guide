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

> 若系統的 C++ 編譯器不叫 `c++`(例如只有 `g++-15`),用 `CXX` 指定即可;
> `PYTHON` 亦可覆寫(預設用 repo 根目錄的 `.venv`):
>
> ```bash
> CXX=g++-15 ./tools/run_all_labs.sh
> ```

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

- Python 版每章:理論重點 → 實驗(圖 + 數值表)→ **✅ 驗證 cell(assertion)** → 練習。
- C++ 版每章:實驗(數值表 + CSV)→ **✅ 驗證(CHECK)** → 以結束碼回報。

本教材為原創內容,不含原書文字、圖表或 ModelQ 專有檔案;請搭配原書使用。

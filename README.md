# Control System Design Guide — Python / C++ 實驗教材

George Ellis《Control System Design Guide》(4th ed.) 的現代化開源實驗環境:
以開源軟體取代原書的 Visual ModelQ,以 **ESP32** 取代商用 RCP 硬體。
全書 19 章,提供 **三個對稱版本**(Jupyter / 純 Python 腳本 / C++),章節目錄一一對應、
實驗參數與 ✅ 驗證條件相同。

| 版本 | 路徑 | 技術 | 執行 | 說明 |
|---|---|---|---|---|
| Jupyter | [`jupyter/`](jupyter/README.md) | Jupyter + python-control + SciPy | `jupyter/run_all.sh` | 每章一份 `lab.ipynb`,圖表內嵌,適合互動學習 |
| Python 腳本 | [`python/`](python/README.md) | 純 `.py` + python-control + SciPy | `python/run_all.sh` | 每章一支 `main.py`,可直接 `python main.py`,適合命令列 / CI |
| C/C++ | [`cpp/`](cpp/README.md) | C++17,header-only,零外部相依 | `cpp/run_all.sh` | 每章一支 `main.cpp`,輸出 CSV,貼近嵌入式 / 韌體 |

> Jupyter 版與 Python 腳本版是**同一套實驗的兩種呈現**:`python/*/main.py` 由
> `jupyter/*/lab.ipynb` 以 `nbconvert --to script` 轉出,程式碼與驗證條件完全相同。

> 總覽、章節對照與驗證報告見
> [`control-system-design-guide-study-guide-v2.md`](control-system-design-guide-study-guide-v2.md)。

## 環境需求

| 用途 | 需求 | 備註 |
|---|---|---|
| Jupyter 版 | Python ≥ 3.11 + `jupyter/requirements.txt` 套件 | 含 `jupyterlab`;已於 3.11–3.14 驗證 |
| Python 腳本版 | Python ≥ 3.11 + `python/requirements.txt` 套件 | 精簡版(不含 jupyterlab);見 [`python/requirements.txt`](python/requirements.txt) |
| C/C++ 版 | 任一 C++17 編譯器(g++ / clang++ / MSVC) | header-only,無 Eigen / Boost / FFTW 等外部相依 |
| C++ 一鍵驗證 | CMake ≥ 3.16(選用) | 無 CMake 時 `cpp/run_all.sh` 會退回直接以 `$CXX` 編譯 |
| 第 19 章韌體 | PlatformIO + ESP32(選用) | host 端 PID 等價性測試只需 C++ 編譯器,不需實機 |

> 三版彼此獨立,任選一版即可;只想跑 C++ 版時不需要安裝任何 Python 套件
> (畫圖用的 `tools/plot_csv.py` 除外)。

## 快速開始

```bash
# Python(Jupyter 或純腳本共用同一個 venv)
python -m venv .venv && source .venv/bin/activate   # Python ≥ 3.11(已於 3.11–3.14 驗證)

# 互動學習:Jupyter 版
pip install -r jupyter/requirements.txt
jupyter lab jupyter/                                 # 開任一章的 lab.ipynb

# 命令列 / CI:純 Python 腳本版
pip install -r python/requirements.txt
python python/03-tuning/main.py                      # 單跑一章(可從任意目錄)

# C++ 版(只需 C++17 編譯器,CMake 選用)
./cpp/run_all.sh
```

一鍵重跑全部驗證(Jupyter 19 章 + 純腳本 19 章 + 韌體 host 測試 + C++ 19 章,任一失敗即回報):

```bash
./tools/run_all_labs.sh
```

> 若系統的 C++ 編譯器不叫 `c++`(例如只有 `g++-15`),用 `CXX` 指定即可;
> `PYTHON` 亦可覆寫(預設用 repo 根目錄的 `.venv`):
>
> ```bash
> CXX=g++-15 ./tools/run_all_labs.sh
> ```

驗證涵蓋 **58 個程式單元**:19 章 Jupyter notebook(nbconvert 執行章末 ✅ assertion)、
19 章純 Python 腳本(直接執行同一組 assertion)、19 章 C++(ctest,`-Wall -Wextra` 零警告)、
以及 1 個韌體 host PID 等價性測試(`pid.h` 對比 Python 重現,逐樣本最大差異約 3e-5)。
任一失敗腳本即以非零結束碼回報。

## 疑難排解

- **`command not found: c++` 或略過 C++ / 韌體驗證** — 系統沒有名為 `c++` 的編譯器。
  用 `CXX=g++-15`(或你的編譯器名)覆寫,見上方範例。
- **`ModuleNotFoundError`(numpy / control 等)** — 未啟用 venv 或未裝套件;
  重跑 `python -m venv .venv && source .venv/bin/activate && pip install -r python/requirements.txt`。
- **`jupyter: command not found`** — `run_all.sh` 預設找 `.venv/bin/jupyter`,
  找不到才退回 PATH 上的 `jupyter`;確認已在 venv 內安裝(requirements 含 `jupyterlab`)。
- **CMake 快取指向舊編譯器** — 刪掉 `cpp/build/` 重新設定,或改用 `CXX` 直接編譯路徑。

## 目錄結構

```
jupyter/                 Jupyter 版
├── common/                ModelQ-lite 共用函式庫(control_helpers / sim / dsa / plots)
├── 01-introduction/ … 19-rapid-control-prototyping/   每章一份 lab.ipynb
├── requirements.txt
└── run_all.sh             執行 19 份 notebook 驗證

python/                  純 Python 腳本版(與 jupyter/ 同源,nbconvert 轉出)
├── common/                同 jupyter/common(純 .py)
├── 01-introduction/ … 19-rapid-control-prototyping/   每章一支 main.py
├── requirements.txt       精簡版(不含 jupyterlab)
└── run_all.sh             無頭(Agg)執行 19 支腳本驗證

cpp/                     C/C++ 版(與 python/ 對稱)
├── common/                header-only 函式庫(lti / sim / dsa / linalg / util)
├── 01-introduction/ … 19-rapid-control-prototyping/   每章一支 main.cpp
├── tools/plot_csv.py      CSV → 圖(選用)
├── CMakeLists.txt
└── run_all.sh             建置 + 執行 19 章驗證

hardware/esp32/          第 19 章 RCP 韌體(PlatformIO,各版共用)
├── src/pid.h              PID 邏輯(純 C++,host 可測)
├── src/main.cpp           編碼器 ISR、TB6612 PWM、序列命令與遙測
└── test_host/             host 端 C/Python 逐樣本等價性測試

tools/run_all_labs.sh    全部驗證(三版 + 韌體)
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

- Jupyter / Python 腳本版每章:理論重點 → 實驗(圖 + 數值表)→ **✅ 驗證(assertion)** → 練習。
- C++ 版每章:實驗(數值表 + CSV)→ **✅ 驗證(CHECK)** → 以結束碼回報。

本教材為原創內容,不含原書文字、圖表或 ModelQ 專有檔案;請搭配原書使用。

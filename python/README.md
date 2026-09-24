# Control System Design Guide — Python 實驗教材

以 **Python + Jupyter + python-control + SciPy** 取代原書的 Visual ModelQ。
19 章、每章一份可執行、可驗證的 lab notebook。C++ 對應版本見 [`../cpp/`](../cpp/README.md)。

## 安裝與執行

在 repo 根目錄:

```bash
python -m venv .venv                # 建議 Python 3.11/3.12
source .venv/bin/activate
pip install -r python/requirements.txt
jupyter lab python/                 # 開任一章的 lab.ipynb
```

一鍵重跑 19 份 notebook(任一章 assertion 失敗即報錯):

```bash
./python/run_all.sh                 # 只跑 Python 版
./tools/run_all_labs.sh             # Python + 韌體 host 測試 + C++ 版
```

## 目錄結構

```
python/
├── common/                ModelQ-lite 共用函式庫
│   ├── control_helpers.py   解析工具:plants、PID、margins、step metrics
│   ├── sim.py               逐樣本模擬:DiscretePID、MotorPlant、DCMotorPlant、
│   │                        TwoMassPlant、Backlash、EncoderModel、Delay…
│   ├── dsa.py               DSA:chirp/PRBS 激發 + Welch 交叉頻譜 FRF 量測
│   └── plots.py             Bode 疊圖(解析系統與量測 FRF 同圖)
├── 01-introduction/ … 19-rapid-control-prototyping/    每章 lab.ipynb + README.md(章節詳細說明)
├── requirements.txt
└── run_all.sh
```

各 notebook 以 `sys.path.append('..')` 載入 `python/common/`,
請從 notebook 所在目錄執行(Jupyter 預設即如此)。

每章結構:理論重點 → 實驗(圖 + 數值表)→ **✅ 驗證 cell(assertion)** → 練習。

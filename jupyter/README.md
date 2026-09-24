# Control System Design Guide — Jupyter 實驗教材

以 **Python + Jupyter + python-control + SciPy** 取代原書的 Visual ModelQ。
19 章、每章一份可執行、可驗證的 lab notebook。
不想開 Jupyter、想要純 `.py` 腳本版見 [`../python/`](../python/README.md);
C++ 對應版本見 [`../cpp/`](../cpp/README.md)。

## 安裝與執行

在 repo 根目錄:

```bash
python -m venv .venv                # 建議 Python ≥ 3.11
source .venv/bin/activate
pip install -r jupyter/requirements.txt
jupyter lab jupyter/                # 開任一章的 lab.ipynb
```

一鍵重跑 19 份 notebook(任一章 assertion 失敗即報錯):

```bash
./jupyter/run_all.sh                # 只跑 Jupyter 版
./tools/run_all_labs.sh             # Jupyter + 純 Python 腳本 + 韌體 host 測試 + C++ 版
```

## 目錄結構

```
jupyter/
├── common/                ModelQ-lite 共用函式庫
│   ├── control_helpers.py   解析工具:plants、PID、margins、step metrics
│   ├── sim.py               逐樣本模擬:DiscretePID、MotorPlant、DCMotorPlant、
│   │                        TwoMassPlant、Backlash、EncoderModel、Delay…
│   ├── dsa.py               DSA:chirp/PRBS 激發 + Welch 交叉頻譜 FRF 量測
│   └── plots.py             Bode 疊圖(解析系統與量測 FRF 同圖)
├── 01-introduction/ … 19-rapid-control-prototyping/    每章一份 lab.ipynb
├── requirements.txt
└── run_all.sh
```

各 notebook 以 `sys.path.append('..')` 載入 `jupyter/common/`,
請從 notebook 所在目錄執行(Jupyter 預設即如此)。

每章結構:理論重點 → 實驗(圖 + 數值表)→ **✅ 驗證 cell(assertion)** → 練習。

# Control System Design Guide — 純 Python 腳本版

與 [`../jupyter/`](../jupyter/README.md) 的 19 章 notebook **同一套實驗、同一組 ✅ 驗證條件**,
但改用**單純的 `.py` 檔**呈現:每章一支 `main.py`,可直接 `python main.py` 執行,
適合命令列、CI、或不想開 Jupyter 的場合。C++ 平行版本見 [`../cpp/`](../cpp/README.md)。

> 這些 `main.py` 由 `jupyter/` 的 `lab.ipynb` 以 `jupyter nbconvert --to script` 轉出:
> notebook 的 markdown 說明變成 `#` 註解、程式碼與章末 assertion 原樣保留。
> 想要圖表內嵌、逐段互動的學習體驗,請用 `jupyter/` 版;想要純腳本、批次驗證,用這裡。

## 安裝與執行

在 repo 根目錄:

```bash
python -m venv .venv                # Python ≥ 3.11
source .venv/bin/activate
pip install -r python/requirements.txt

# 單跑一章(從章目錄內執行,腳本會以 ../ 載入 common/)
cd python/03-tuning && python main.py
```

一鍵重跑全部 19 章(任一章 assertion 失敗即以非零結束碼回報):

```bash
./python/run_all.sh                 # 只跑純 Python 腳本版
./tools/run_all_labs.sh             # Jupyter + 純腳本 + 韌體 host 測試 + C++
```

`run_all.sh` 預設以 `MPLBACKEND=Agg` 無頭執行(不開圖形視窗,`plt.show()` 變 no-op);
互動執行時用一般後端即可跳出圖。

## 目錄結構

```
python/
├── common/                ModelQ-lite 共用函式庫(與 jupyter/common 內容相同)
│   ├── control_helpers.py   解析工具:plants、PID、margins、step metrics
│   ├── sim.py               逐樣本模擬:DiscretePID、MotorPlant、DCMotorPlant、
│   │                        TwoMassPlant、Backlash、EncoderModel、Delay…
│   ├── dsa.py               DSA:chirp/PRBS 激發 + Welch 交叉頻譜 FRF 量測
│   └── plots.py             Bode 疊圖(解析系統與量測 FRF 同圖)
├── 01-introduction/ … 19-rapid-control-prototyping/    每章一支 main.py
├── requirements.txt
└── run_all.sh
```

每支 `main.py` 以 `sys.path.append(str(pathlib.Path('..').resolve()))` 載入 `python/common/`,
故請**從該章目錄內**執行(`run_all.sh` 已代為 `cd`)。

每章結構:理論重點(註解)→ 實驗(印數值表 + 畫圖)→ **✅ 驗證(assertion)** → 練習。

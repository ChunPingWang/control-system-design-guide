# 《Control System Design Guide》Visual ModelQ 軟體替代計劃

| 項目 | 內容 |
|---|---|
| 書名 | *Control System Design Guide: Using Your Computer to Understand and Diagnose Feedback Controllers* |
| 作者 | George Ellis |
| 對應版本 | 第 4 版（2012） |
| 文件版本 | v1 |
| 狀態 | 草案，待核對章節目錄 |

> ⚠️ **待確認事項**：本計劃的章節對應依第 4 版整理，內容憑印象撰寫，**尚未逐頁核對原書**。第 3 版章節編排不同，使用前請以實際版本的目錄校正。

---

## 1. 背景與目標

### 1.1 背景

原書所有實驗皆依賴作者提供的 **Visual ModelQ** 模擬軟體，其限制如下：

- 僅支援 Windows
- 已長期未維護
- 模型檔（`.mqd`）為專有格式，無法轉換至其他工具

### 1.2 目標

- 以開源工具重建全書各章實驗模型。
- 可重現書中的**時域波形**與 **Bode 圖**。
- 產出可版本控管、可自動化測試、跨平台的實驗環境。

---

## 2. 功能對照與工具選型

| ModelQ 功能 | 主力替代 | 備援／補充 |
|---|---|---|
| 方塊圖建模 | Python：bdsim 或自寫離散模擬迴圈 | Scilab/Xcos（最接近拖拉式操作） |
| Live Scope（即時示波器） | matplotlib + Jupyter | Plotly |
| Live Constant（即時調參） | ipywidgets 滑桿 | Panel / Bokeh |
| DSA（動態訊號分析儀） | 自寫 chirp/PRBS 激發 + FFT 量測 Bode | `scipy.signal.csd` / `welch` |
| 解析 Bode／根軌跡 | python-control | Octave control 套件 |
| 物理模型（馬達、機構） | OpenModelica | Julia ModelingToolkit |
| RCP 實機 | STM32 / Teensy + SimpleFOC 或 ODrive | Raspberry Pi（控制頻寬較低） |

### 2.1 核心架構決策：自建「ModelQ-lite」

採**固定取樣時間的逐步模擬（sample-by-sample）**，而非只依賴 python-control 的 LTI 工具。原因是 LTI 工具不易處理書中大量出現的下列情境：

- 非線性元件：飽和、backlash、庫倫摩擦
- 多速率取樣
- 數位控制器的計算延遲
- 編碼器量化

---

## 3. 章節對應計劃

### Section I：控制原理（Ch1–10）

| 章節 | 主題 | 替代實作 |
|---|---|---|
| Ch2–3 | 頻域分析、調機 | python-control 解析 Bode，與 DSA 量測 Bode 交叉比對；PI/PID 滑桿調參介面 |
| Ch4–5 | 數位延遲、z 域 | `c2d` 比較 ZOH、Tustin 等離散化方法；延遲對相位邊限影響的掃描實驗 |
| Ch6–8 | 四種控制器、擾動響應、前饋 | 控制器方塊庫，提供擾動注入點與前饋路徑 |
| Ch9 | 控制系統中的濾波器 | `scipy.signal` 設計低通與 notch，評估迴路內相位損失 |
| Ch10 | 觀測器入門 | Luenberger 觀測器離散實作 |

### Section II：建模（Ch11–13）

| 章節 | 主題 | 替代實作 |
|---|---|---|
| Ch11–12 | 建模入門、非線性與時變 | 非線性方塊：飽和、backlash、庫倫摩擦、時變增益 |
| Ch13 | 模型開發與驗證 | 模擬 Bode 與量測 Bode（實機或 OpenModelica）比對流程 |

### Section III：運動控制（Ch14–19）

| 章節 | 主題 | 替代實作 |
|---|---|---|
| Ch14 | 編碼器與 resolver | 量化誤差與 resolver 誤差模型 |
| Ch15 | 伺服馬達與驅動器 | Python 簡化模型；細節以 OpenModelica 建模 |
| Ch16 | 柔性與共振 | **two-mass 共振模型**：全書最常用，列為優先開發項目與回歸測試基準 |
| Ch17 | 位置控制迴路 | P/PI 串級位置迴路 |
| Ch18 | 運動控制中的 Luenberger 觀測器 | 觀測器用於速度估測 |
| Ch19 | 快速控制原型（RCP） | STM32 + SimpleFOC 或 ODrive 低成本方案（見 §6 風險） |

---

## 4. 專案結構（建議）

```
modelq-lite/
├── core/          # 方塊庫、固定步長模擬器
├── dsa/           # 動態訊號分析儀（激發訊號 + FFT Bode）
├── models/        # 共用模型（two-mass、馬達、編碼器…）
├── notebooks/
│   ├── ch02/
│   ├── ch03/
│   └── …
├── cross-check/   # Octave / Xcos 交叉驗證腳本
├── reference/     # 書中圖表對應的 golden 數值
└── tests/         # 回歸測試
```

---

## 5. 執行階段

| 階段 | 範圍 | 預估時程 | 交付物 |
|---|---|---|---|
| P0 基礎建設 | repo 結構、工具鏈驗證 | 1–2 週 | 以 PI 速度迴路打通全流程 |
| P1 Section I | Ch2–10 | 3–4 週 | 每章一份 notebook |
| P2 Section II/III 模擬 | Ch11–18 | 4 週 | 以 two-mass 模型為核心擴充 |
| P3 RCP 實機 | Ch19 | 視硬體到位 | 實機 DSA 量測與模擬比對 |
| P4 驗收 | 全書 | — | 對照 golden reference 的驗收報告 |

> 時程為粗估值，未考慮人力配置與硬體採購週期。

---

## 6. 驗證策略與風險

### 6.1 驗證策略

- **驗收標準**：相位邊限、增益邊限、頻寬、overshoot 與書中數值誤差在 **±5%** 以內。此容差為建議值，可依需求調整。
- **Golden reference**：以書中圖表數值作為比對基準，納入自動化回歸測試。
- **雙軌交叉驗證**：關鍵模型另外用 Octave 或 Xcos 各跑一次，避免自寫模擬器的 bug 造成系統性偏差。

### 6.2 風險

| 風險 | 影響 | 對策 |
|---|---|---|
| ModelQ 內部積分器與取樣時序細節未公開 | 重現結果可能出現小幅差異 | 在文件中逐項記錄差異與原因 |
| `.mqd` 模型檔無法轉換 | 需人工重建，工作量大 | 對照書中截圖手動重建，並列出參數清單 |
| Ch19 原書使用特定商用硬體平台（廠牌待確認） | 無法完全等價替代 | 以低成本開源硬體重現核心概念，並標註差異 |
| 章節對應尚未核對原書 | 規劃範圍可能偏差 | 以實際版本目錄校正後發布 v2 |

---

## 7. 下一版（v2）候選項目

- [ ] 以實際版本目錄校正章節對應
- [ ] 展開 P0 repo 骨架與 ModelQ-lite 核心 API 設計
- [ ] 以 Ch16 two-mass 模型撰寫完整範例
- [ ] 確認 Ch19 原書硬體平台並評估替代方案

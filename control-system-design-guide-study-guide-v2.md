# 《Control System Design Guide》學習教材與軟體替代方案(v2)

| 項目 | 內容 |
|---|---|
| 原書 | George Ellis, *Control System Design Guide: Using Your Computer to Understand and Diagnose Feedback Controllers*, 4th ed. (2012) |
| 文件版本 | v2(2026-09-24) |
| 狀態 | **已實作並完成程式碼驗證** —— 19 章 Jupyter 實驗全數可執行,每章附自動化驗證(assertion) |
| 實作位置 | Jupyter 版 `jupyter/`(`01-…`~`19-…` 各章 `lab.ipynb`、`common/`);純 Python 腳本版 `python/`(各章 `main.py`,同源);C/C++ 版 `cpp/`(對稱結構);Maker 硬體版 `maker-labs/`(各章 `sim.py` + ESP32 `firmware/` + `LAB_REPORT.md`,對應兩份 Maker 教材);韌體 `hardware/` |
| 前版計劃 | `control-system-design-guide-software-replacement-plan-v1.md`(規劃草案,本文件取代之) |

---

## 1. 這份教材是什麼

原書所有實驗依賴作者的 **Visual ModelQ**(僅限 Windows、已停止維護、專有格式)。
本教材以開源工具重建全書 19 章的實驗環境:

- **每章一份 Jupyter notebook**(`jupyter/NN-主題/lab.ipynb`),含:理論重點、可執行實驗、
  **✅ 驗證 cell**(以 assertion 鎖住關鍵數值)、章末練習;另有同源的純 Python 腳本版
  (`python/NN-主題/main.py`,由 notebook 以 `nbconvert` 轉出)。
- **共用函式庫 `common/`**(「ModelQ-lite」,`jupyter/common/` 與 `python/common/` 內容相同):
  逐樣本模擬器 + DSA + 解析工具。
- **C/C++ 平行版本**(`cpp/`):19 章各一支 C++17 程式,header-only 函式庫、零外部相依,
  實驗與驗證條件與 notebook 一一對應(見 `cpp/README.md`)。
- **ESP32 韌體**(`hardware/esp32/`):第 19 章 RCP 實機平台,PID 邏輯已在 host 端
  與 Python 實作逐樣本比對驗證。

本教材為原創學習教材,不複製原書文字與圖表;需搭配原書閱讀。

## 2. 軟體替代對照(定案)

| Visual ModelQ 功能 | 本教材替代 | 位置 |
|---|---|---|
| 方塊圖建模(拖拉) | 逐樣本模擬類別(`DiscretePID`、`MotorPlant`、`TwoMassPlant`…) | `python/common/sim.py` |
| Live Scope 示波器 | Matplotlib(notebook 內嵌) | 各章 |
| Live Constant 即時調參 | 參數掃描 + 自動指標表(可另加 ipywidgets 滑桿) | 各章 |
| DSA 動態訊號分析儀 | chirp/PRBS 激發 + Welch 交叉頻譜 + coherence | `python/common/dsa.py` |
| 解析 Bode / 邊限 | python-control(`margins`、`bode_compare`) | `python/common/control_helpers.py`、`python/common/plots.py` |
| 馬達物理模型 | `DCMotorPlant`(含電氣動態)、`TwoMassPlant`(柔性) | `python/common/sim.py` |
| 商用 RCP 硬體 | ESP32 + TB6612 + 編碼器馬達;Python 負責辨識/設計/分析 | `hardware/esp32/` |

**核心架構決策(與 v1 相同,已實證有效)**:採固定步長逐樣本模擬,而非只靠 LTI 工具
—— 飽和、backlash、庫倫摩擦、量化、計算延遲、anti-windup 都只有逐樣本才做得出來。
LTI(python-control)用於解析 Bode/邊限,與逐樣本模擬**交叉驗證**(兩條獨立路徑)。

## 3. 章節教材總覽

### Section I:控制原理(Ch1–10)

| 章 | 主題 | 實驗內容 | 關鍵驗證結果 |
|---|---|---|---|
| 1 | 回授導論 | 開/閉迴路對受控體增益 ±50% 漂移與階躍擾動的比較 | 閉迴路穩態誤差 < 0.1%,開迴路誤差 50% |
| 2 | 頻域分析 | PI 速度迴路解析 Bode + **DSA chirp 量測**交叉比對 | PM≈89°、頻寬≈40 Hz,DSA 與解析誤差 <30% |
| 3 | 調機 | Ellis **zone-based tuning** 全自動重現(先 P 後 I) | kp=0.7(OS 4.5%)→ ki=30(OS 15.5%) |
| 4 | 數位延遲 | 相位損失公式 360·fc·1.5Ts vs Padé vs 逐樣本模擬三方對照 | 500 Hz 取樣 → PM 只剩 ~45°,振鈴可見 |
| 5 | z 域 | ZOH/Tustin/matched 離散化、極點映射 z=e^{sT}、混疊示範 | DC 增益不變;60 Hz@70 Hz 取樣 → 10 Hz 假象 |
| 6 | 四種控制器 | P/PI/PID/PID+(2-DOF)同場對決:命令 + 擾動 | PID+ overshoot 0%(PID 6.3%)且擾動抑制不變 |
| 7 | 擾動響應 | Gd=P/(1+CP) 頻域 + 時域 + **擾動解耦前饋** | ki×20 → 低頻擾動抑制 +26 dB;解耦改善 3.8 倍 |
| 8 | 前饋 | 梯形規劃 + 速度/加速度前饋,追隨誤差量化 | vel FF 改善 >4 倍,再加 acc FF 又 >2 倍 |
| 9 | 濾波器 | LPF/notch 的「衰減換相位」帳本;迴路內濾雜訊 | LPF@5×頻寬:PM -16°;@1.2×頻寬:PM 崩至 25° |
| 10 | 觀測器入門 | Luenberger 觀測器極點配置、收斂、量化位置重建速度 | 速度雜訊比差分低 3 倍以上 |

### Section II:建模(Ch11–13)

| 章 | 主題 | 實驗內容 | 關鍵驗證結果 |
|---|---|---|---|
| 11 | 建模入門 | DC 馬達物理建模、降階(忽略電感)、解析 vs 數值交叉驗證 | 兩實作誤差 <1%;降階在電氣極點以上失效 |
| 12 | 非線性 | windup/anti-windup、庫倫摩擦零速尖峰、backlash 遲滯 | naive 積分器 overshoot >2 倍;遲滯寬=背隙 |
| 13 | 模型驗證 | 「未知」受控體 DSA 量測 → 低頻擬合慣量 → 疊圖找未建模動態 | J 擬合誤差 <15%;共振處剛體模型差 >10 dB |

### Section III:運動控制(Ch14–19)

| 章 | 主題 | 實驗內容 | 關鍵驗證結果 |
|---|---|---|---|
| 14 | 編碼器 | 量化 → 差分速度雜訊 RMS≈q/(√6Ts) 理論對照;平均取捨 | 三種解析度均與理論吻合(±35%) |
| 15 | 伺服馬達 | 電流迴路極點對消設計、串級頻寬分離、電流/電壓極限 | 800 Hz 電流迴路上升 442 µs(理論 437);穩態電壓=反電動勢+IR |
| 16 | **共振** | two-mass Bode、**kp_max 一把尺量所有解方** | 共振砍增益 13 倍;notch 小補、LPF 反而更糟、慣量比 1:1 拉回 6 倍 |
| 17 | 位置迴路 | 串級 P/PI vs 單迴路 PID;速度極限;追隨誤差=v/kv | 串級速度精確貼極限;PID 速度衝到 9 倍 |
| 18 | 運動觀測器 | 觀測器速度回授 vs 編碼器差分回授(閉迴路) | 轉矩雜訊改善 **40 倍**,追蹤不犧牲 |
| 19 | RCP | 遙測解析 → 一階辨識 → λ-tuning → 模擬驗證 → **韌體等價性** | K 誤差<5%、τ<10%;C/Python PID 逐樣本等價 |

## 4. 建議學習路徑

```
基礎:Ch1 → Ch2 → Ch3(回授、頻域、調機 —— 全書的語言)
數位:Ch4 → Ch5(為什麼取樣率重要)
迴路設計:Ch6 → Ch7 → Ch8 → Ch9(控制器、擾動、前饋、濾波)
進階:Ch10(觀測器)
建模紀律:Ch11 → Ch12 → Ch13(模型是用量測驗證出來的)
運動控制:Ch14 → Ch15 → Ch16(共振!)→ Ch17 → Ch18
實機:Ch19 + hardware/(硬體到位後)
```

每章結構固定:**理論重點 → 實驗 → ✅ 驗證 → 練習**。
練習刻意不附解答,多數可用該章程式碼改幾行完成。

## 5. 程式碼驗證方法與結果

### 驗證方法(三層)

1. **每章 assertion**:每份 notebook 最後有「✅ 驗證」cell,把該章關鍵數值
   (PM、頻寬、overshoot、雜訊 RMS、擬合誤差…)鎖進 assert。任何一條失敗,
   `jupyter nbconvert --execute` 就會失敗。
2. **交叉驗證**:同一物理用兩條獨立路徑計算並比對 —— 解析 TF vs 逐樣本 Euler(Ch11 <1%)、
   理論公式 vs 模擬量測(Ch4 相位損失、Ch14 量化雜訊、Ch17 追隨誤差)、
   解析 Bode vs DSA 量測(Ch2、Ch13)。
3. **韌體等價性**:`hardware/esp32/src/pid.h` 以 host 端 clang++ 編譯,
   對 1000 筆決定性偽隨機誤差序列與 Python 重現實作逐樣本比對(最大差異 3.1e-5,容差 1e-3)。

### 驗證結果(2026-09-24)

| 項目 | 結果 |
|---|---|
| 環境 | Python 3.12.13 / numpy 2.5.3 / scipy 1.18.1 / control 0.10.2 / matplotlib 3.11.2(macOS) |
| 19 份 notebook 全量執行 | **19/19 通過**(`jupyter nbconvert --to notebook --execute`) |
| 每章 assertion | 全數通過(各章「✅ 驗證通過」輸出保留在 notebook 內) |
| 韌體 host 測試 | 通過(`hardware/esp32/test_host/`) |
| C++ 版 19 章(`cpp/`) | **19/19 通過**(g++ 13 / clang++,`cpp/run_all.sh`);確定性章節數值與 notebook 逐位相同 |
| `main.cpp`(Arduino 層) | 未編譯 —— 需 PlatformIO + ESP32 toolchain;控制邏輯(pid.h)已 host 驗證 |

重跑全部驗證:

```bash
python -m venv .venv && source .venv/bin/activate
pip install -r jupyter/requirements.txt   # 或 python/requirements.txt(純腳本版,精簡)
./tools/run_all_labs.sh          # 全部:Jupyter 19 章 + 純腳本 19 章 + 韌體 host 測試 + C++ 版
./jupyter/run_all.sh             # 只跑 Jupyter notebook 版
./python/run_all.sh              # 只跑純 Python 腳本版
./cpp/run_all.sh                 # 只跑 C++ 版(只需 C++17 編譯器)
```

## 6. 與原書的已知差異

| 差異 | 說明 |
|---|---|
| 數值不逐圖對齊 | 本教材用自訂參數組(J=0.002 速度迴路、two-mass 慣量比 9 等),驗證的是**機制與公式**,不是複刻原書圖表數值。 |
| ModelQ 積分器細節 | ModelQ 內部時序未公開;本教材固定採「量測 → 控制 → Euler 積分」順序 + 明確的 `Delay` 方塊,時序假設全部攤在程式碼裡。 |
| Ch19 硬體平台 | 原書用商用 RCP 平台;本教材用 ESP32(百元等級)重現核心流程:遙測、辨識、調機、下載、DSA。 |
| 章節對應 | 依第 4 版章節主題整理;第 3 版編排不同,對照時以主題為準。 |

## 7. 後續(硬體階段)

- [ ] 實機:ESP32 + TB6612 + 編碼器馬達接線(見 `hardware/README.md`),跑 Ch19 完整流程
- [ ] 實機 DSA:chirp 疊加於 target,量測實機閉迴路 FRF 與模擬比對(Ch13 流程)
- [ ] Capstone:自平衡機器人(MPU6050),串起 Ch10 觀測器 + Ch17 串級
- [ ] 選配:ipywidgets 滑桿版「Live Constant」互動調參介面

# Ch13 模型建立與驗證 — C++ 版

> **理論與完整說明**:[`python/13-model-verification/README.md`](../../python/13-model-verification/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch13`
> **上一章**:[Ch12 非線性行為](../12-nonlinear/README.md) · **下一章**:[Ch14 編碼器與解角器](../14-encoder/README.md)

## 本章做什麼

把 two-mass 受控體($J_m=J_l=10^{-3}$、$k_s=100$、$c_s=0.01$)當成「參數未知的實機」:
(1)以 2→300 Hz 對數 chirp(20 s、0.5 N·m、$f_s=10$ kHz)激發,量測含雜訊(σ=0.02)的馬達速度,用 Welch 交叉頻譜估 FRF,只留 coherence > 0.9 的點;
(2)從 2~10 Hz 以 $|H|\approx1/(J\omega)$ 擬合總慣量;
(3)在 30~150 Hz 自動找共振峰 / 反共振谷,對照理論,並量化剛體模型在共振處的誤差。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch13 && ./cpp/build/ch13
c++ -std=c++17 -O2 cpp/13-model-verification/main.cpp -o ch13 && ./ch13   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| 量測 | `TwoMassPlant rig(Jm, Jl, ks, cs, dt)`;`chirp_excitation(2, 300, 20, fs, 0.5)`;`Rng(7).normal(0, 0.02, N)`;`y = rig.wm + noise` | `TwoMassPlant`、`chirp_excitation`、`Rng` |
| FRF | `measure_frf(u, y, fs)`,再以 `select(...)` 篩出 coherence > 0.9、1.5 < f < 300 Hz | `measure_frf`、`FRF::select` |
| 慣量擬合 | 2~10 Hz 各點 `1 / (|H|·2πf)` 取 `mean` | `mean` |
| 模型 | `P_rigid = tf({1}, {J_est, 0})`;`P_2mass = tf({Jl, cs, ks}, {JmJl, (Jm+Jl)cs, (Jm+Jl)ks, 0})`;在量測頻點求值寫 CSV | `tf`、`frf_of_system`、`write_csv` |
| 讀圖 | 30~150 Hz 的 `argmax` / `argmin`;`resonance_hz()`、`antiresonance_hz()` 理論值;`mag_db_at(P_rigid, f_res)` | `argmax`、`argmin`、`mag_db_at` |
| ✅ 驗證 | 5 條 `CHECK`,與 notebook 的 assertion 一一對應 | `Checker`、`CHECK` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `TwoMassPlant(**SECRET, dt=dt)`;`wm, _ = rig.step(uk)` | `TwoMassPlant rig(Jm, Jl, ks, cs, dt)`;`rig.step(u); rig.wm` |
| `tt, u = chirp_excitation(...)` | `Excitation ex = chirp_excitation(...)`;`ex.u` |
| `np.random.default_rng(7).normal(0, 0.02, N)` | `Rng(7).normal(0, 0.02, N)`(mt19937_64) |
| `measure_frf(u, y, fs)` 回傳 dict | `measure_frf(u, y, fs)` 回傳 `FRF` 結構(Hann、50% 重疊,與 scipy 預設相同) |
| `{k: v[good] for k, v in meas.items()}` | `meas.select(lambda)` |
| `tm_check.resonance_hz`(property) | `tm_check.resonance_hz()`(成員函式) |
| `frf_of_system(P_rigid, [f])['mag_db'][0]` | `mag_db_at(P_rigid, f)` |
| `bode_compare` 疊圖 | `ch13_model_verification.csv`(下表) |

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch13_model_verification.csv` | `f_hz`, `meas_mag_db`, `meas_phase_deg`, `rigid_mag_db`, `twomass_mag_db` | 只含有效量測點;量測、剛體擬合、two-mass 模型三條幅值疊圖 |

```bash
python3 cpp/tools/plot_csv.py out/ch13_model_verification.csv
```

### 實際執行結果

```
有效量測點:110,頻率範圍 2.4 ~ 285.6 Hz
擬合 J_tot = 2.000 mkg·m²(真值 2.000,誤差 0.0%)
量測共振 75.7 Hz(理論 71.2 Hz)
量測反共振 46.4 Hz(理論 50.3 Hz)
共振處剛體模型誤差:19.8 dB

# ✅ 驗證
Ch13 驗證通過 ✅ (5/5)
```

**與 Python 的差異**:本章含亂數雜訊,C++ 的 `mt19937_64` 與 numpy PCG64 序列不同,因此有效量測點 C++ 為 110 點(2.4 ~ 285.6 Hz)、
Python 為 111 點(2.4 ~ 288.1 Hz)—— 高頻端最後一個有效點差了一個 $\Delta f$(288.1 vs 285.6 Hz),是 coherence 在 0.9 門檻附近隨雜訊起伏所致。
擬合慣量(2.000,誤差 0.0%)、共振 75.7 Hz、反共振 46.4 Hz、共振處誤差 19.8 dB 在印出的位數下兩版相同。

從 CSV 可以看出量測共振 / 反共振偏離理論 6~8% 的原因:48.8~53.7 Hz 與 65.9~73.2 Hz 的頻點因 coherence < 0.9 被剔除,
程式找到的極值是缺口邊緣的點(頻率解析度 $\Delta f = 10000/4096 = 2.44$ Hz)。

## ✅ 驗證條件(5 條)

| # | 條件 | 意義 |
|---|---|---|
| 1 | `|J_est − J_true| / J_true < 0.15` | 低頻擬合出正確的總慣量 |
| 2 | `|f_res_meas − resonance_hz()| / resonance_hz() < 0.1` | 找得到共振(理論 $\sqrt{k_s(J_m+J_l)/(J_mJ_l)}/2\pi$) |
| 3 | `|f_anti_meas − antiresonance_hz()| / antiresonance_hz() < 0.1` | 找得到反共振(理論 $\sqrt{k_s/J_l}/2\pi$) |
| 4 | `meas_at − rig_at > 10` dB | 剛體模型在共振處明顯失效 |
| 5 | 2~10 Hz two-mass 模型與量測最大差 `lo_dev < 2` dB | 正確模型在可信頻段與量測吻合 |

## 動手改改看

- 把 `rng.normal(0, 0.02, ...)` 的 σ 改成 0.2,看有效點數與頻率範圍縮到多少,擬合的 `J_est` 是否仍在 15% 內。
- 把 chirp 振幅 `0.5` 改成 `0.2`,或時長 `20` 改成 60 s(掃得更慢),觀察共振 / 反共振附近的 coherence 缺口是否變小。
- 把量測改成 `rig.wl`(負載端速度),重看 CSV:反共振谷消失,只剩共振峰。

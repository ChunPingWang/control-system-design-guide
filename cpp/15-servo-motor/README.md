# Ch15 伺服馬達與驅動器 — C++ 版

> **理論與完整說明**:[`python/15-servo-motor/README.md`](../../python/15-servo-motor/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch15`
> **上一章**:[Ch14 編碼器與解角器](../14-encoder/README.md) · **下一章**:[Ch16 機構柔性與共振](../16-resonance/README.md)

## 本章做什麼

以含電氣動態的直流馬達(`DCMotorPlant`,$R=1\,\Omega$、$L=1$ mH、$K_t=K_e=0.1$、$J=10^{-4}$)做兩件事:
(1)極點對消設計 800 Hz 電流迴路($k_p=L\omega_c$、$k_i=R\omega_c$),量 1 A 步階的上升時間;
(2)外包 80 Hz 速度迴路(電流極限 ±5 A),100 rad/s 大步階,看轉矩受限的等加速段與穩態反電動勢。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch15 && ./cpp/build/ch15
c++ -std=c++17 -O2 cpp/15-servo-motor/main.cpp -o ch15 && ./ch15   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。模擬步長 1 µs、共 20 萬步,-O2 下不到一秒。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| 參數 | 馬達常數、`dt=1e-6`;`wc_i=2π·800`、`kp_i=L·wc_i`、`ki_i=R·wc_i` | — |
| 實驗 1 | 5 ms 迴圈:`ci.step(1.0 - motor.i)` → `motor.step(v)`;以 `first_index` 找 10% / 90% 點 | `DCMotorPlant`、`DiscretePID`(±24 V)、`first_index` |
| 實驗 2 | 200 ms 串級迴圈:`i_cmd = cv.step(100 - w)`、`v = ci2.step(i_cmd - i)` | `DiscretePID`(±5 A 與 ±24 V) |
| CSV 抽點 | 電流步階每 10 µs、速度迴路每 100 µs 取一點輸出(避免百萬列) | `write_csv` |
| ✅ 驗證 | 6 條 `CHECK`,與 notebook 的 assertion 一一對應 | `Checker`、`CHECK`、`vmax` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `DCMotorPlant(R=R, L=L_ind, ..., dt=dt)` | `DCMotorPlant motor(R, L_ind, Kt, Ke, J, b, dt)` |
| `DiscretePID(kp_i, ki_i, dt=dt, out_min=-24, out_max=24)` | `DiscretePID ci(kp_i, ki_i, 0.0, dt, -24, 24)` |
| `t_i[np.nonzero(i_log >= 0.1)[0][0]]` | `first_index(n, [&](size_t k){ return i_log[k] >= 0.1; })` |
| `ia_log.max()` | `vmax(ia_log)` |
| `(ia_log > 4.5).sum() * dt` | 迴圈計數 `limited += ia > 4.5` |
| Matplotlib 兩張圖 | 兩份 CSV(下表,已抽點) |

兩邊用同一套前向 Euler 馬達模型與同一個 `DiscretePID`(含條件積分 anti-windup),
沒有亂數,因此數值與 notebook 一致。

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch15_current_step.csv` | `t`, `current`, `voltage` | 實驗 1:1 A 電流步階(每 10 µs 一點) |
| `ch15_speed_loop.csv` | `t`, `speed`, `current`, `voltage` | 實驗 2:串級速度步階(每 100 µs 一點) |

```bash
python3 cpp/tools/plot_csv.py out/ch15_speed_loop.csv
```

### 實際執行結果

```
電流迴路 PI:kp=5.027, ki=5027(零點 = ki/kp = 1000 rad/s = 電氣極點 R/L)
10–90% 上升時間 442 µs,理論 0.35/BW = 437 µs
速度迴路 PI:kp=0.5027, ki=25.266
穩態:轉速 100.00 rad/s,電壓 10.01 V(理論 Ke·w + R·i = 10.01 V)
峰值電流 4.92 A(極限 5 A),overshoot 0.65 rad/s

# ✅ 驗證
Ch15 驗證通過 ✅ (6/6)
```

與 Python notebook 的輸出逐位相同。

## ✅ 驗證條件(6 條)

| # | 條件 | 意義 |
|---|---|---|
| 1 | `|rise_us − theory_us| / theory_us < 0.4` | 電流迴路頻寬達標(對消設計有效) |
| 2 | `|i_log.back() − 1.0| < 0.05` | 電流無穩態誤差 |
| 3 | `vmax(ia_log) ≤ 5.0 + 1e-6` | 電流極限被尊重 |
| 4 | `|w_log.back() − 100| < 0.5` | 速度到位 |
| 5 | `|va_log.back() − (Ke·100 + R·i)| < 0.5` | 穩態電壓 = 反電動勢 + IR |
| 6 | `limited · dt > 0.01` | 有明顯的電流受限段(> 10 ms) |

## 動手改改看

- 把 `wc_i` 改成 `2 * PI * 160`(與速度迴路只差 2 倍),重新產生 `ch15_speed_loop.csv`,觀察速度振鈴。
- 把 `ci2` 的電壓極限從 ±24 改成 ±6:反電動勢 $K_e\omega$ 在 60 rad/s 就吃光 6 V,速度再也到不了 100。
- 在 `motor2.step(va_log[k])` 加上第二個參數 `t_load`(例如 `k*dt > 0.1 ? 0.05 : 0.0`),看速度迴路的擾動恢復。

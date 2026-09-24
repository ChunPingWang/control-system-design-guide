# Ch14 編碼器與解角器 — C++ 版

> **理論與完整說明**:[`python/14-encoder/README.md`](../../python/14-encoder/README.md)(學習目標、公式推導、練習)
> **本章檔案**:[`main.cpp`](main.cpp) · 執行檔:`ch14`
> **上一章**:[Ch13 模型建立與驗證](../13-model-verification/README.md) · **下一章**:[Ch15 伺服馬達與驅動器](../15-servo-motor/README.md)

## 本章做什麼

以 1 kHz 取樣、變速 $\omega = 1 + 0.5\sin(2\pi\cdot3t)$ 的運動:
(1)比較 500 / 2500 / 10000 線編碼器的差分速度雜訊與理論 $q/(\sqrt6 T_s)$;
(2)對 2500 線的差分速度做 1 / 4 / 16 點移動平均,量化「雜訊 ↓、延遲 ↑」的取捨。

## 建置與執行

```bash
cd cpp && ./run_all.sh                               # 全部章節(含本章)
cmake --build cpp/build --target ch14 && ./cpp/build/ch14
c++ -std=c++17 -O2 cpp/14-encoder/main.cpp -o ch14 && ./ch14   # 單檔編譯
```

CSV 預設寫到目前目錄的 `out/`,可用環境變數 `CSD_OUT` 指定。

## 程式結構(`main.cpp`)

| 段落 | 內容 | 使用的函式庫 API |
|---|---|---|
| 參數 | `fs=1000`、`n=2000`、`w_true`、`pos_rev = cumsum(w_true)·dt/2π` | `sample_times`、`cumsum` |
| `fd_velocity` lambda | 量化位置 `enc.read(pos_rev)·2π`,逐點差分(等同 `np.diff(..., prepend=0)/dt`) | `EncoderModel` |
| `err_rms` lambda | 捨去前 100 點,計算 $\hat\omega-\omega$ 的標準差 | `stdev` |
| 實驗 1 | `lines ∈ {500, 2500, 10000}`,記錄量測 RMS 與 `q/√6/dt` | `std::map` |
| 實驗 2 | 手寫因果卷積 `v_ma[k] += v_fd[k-j]/N`(等同 `np.convolve(..., 'full')[:n]`),延遲 `(N−1)/2·dt` | — |
| ✅ 驗證 | 7 條 `CHECK`,與 notebook 的 assertion 一一對應 | `Checker`、`CHECK` |

## Python ↔ C++ 對照

| Python(notebook) | C++(`main.cpp`) |
|---|---|
| `EncoderModel(lines=2500)` | `EncoderModel enc(2500)` |
| `enc.read(pos_rev)`(向量化) | `enc.read(pos_rev[k])`(逐點) |
| `np.diff(pos_q, prepend=0.0) / dt` | `fd_velocity` 內 `(pq - prev) / dt` |
| `np.cumsum(w_true) * dt / (2π)` | `cumsum(w_true)` 後逐點乘 `dt/(2π)` |
| `np.convolve(v_fd, np.ones(N)/N, 'full')[:n]` | 雙層迴圈因果移動平均 |
| `err.std()` | `stdev(e)`(母體標準差,同 numpy 預設) |
| Matplotlib 兩張圖 | 兩份 CSV(下表) |

本章沒有亂數,量化是確定性的 `floor`,因此 C++ 與 notebook 數值逐位相同。

## 輸出

| 檔案 | 欄位 | 用途 |
|---|---|---|
| `ch14_resolution.csv` | `t`, `w_true`, `v_fd_500`, `v_fd_2500`, `v_fd_10000` | 實驗 1:三種解析度的差分速度 |
| `ch14_moving_average.csv` | `t`, `w_true`, `ma1`, `ma4`, `ma16` | 實驗 2:移動平均後的速度 |

```bash
python3 cpp/tools/plot_csv.py out/ch14_moving_average.csv
```

### 實際執行結果

```
  lines 量測 RMS 理論 q/(√6·Ts)
    500     1.4130         1.2825
   2500     0.2465         0.2565
  10000     0.0659         0.0641
MA 1: 雜訊 RMS 0.2465,群延遲 0.0 ms
MA 4: 雜訊 RMS 0.0681,群延遲 1.5 ms
MA16: 雜訊 RMS 0.0523,群延遲 7.5 ms

# ✅ 驗證
Ch14 驗證通過 ✅ (7/7)
```

與 Python notebook 的輸出逐位相同。

## ✅ 驗證條件(7 條)

| # | 條件 | 意義 |
|---|---|---|
| 1–3 | `|meas − theory| / theory < 0.35`,lines = 500, 2500, 10000 | 差分雜訊符合 $q/(\sqrt6 T_s)$ |
| 4 | `RMS(500) > RMS(2500) > RMS(10000)` | 解析度越高雜訊越小 |
| 5 | `10 < RMS(500)/RMS(10000) < 40` | 解析度 20 倍 → 雜訊約 20 倍 |
| 6 | `ma[16].noise < ma[1].noise / 2` | 平均確實降雜訊 |
| 7 | `ma[16].lag_ms > 5` | 代價:延遲 |

## 動手改改看

- 把 `fs` 改成 `10000.0`(`n` 同步放大到 20000),看三種線數的 RMS 都放大約 10 倍 —— 驗證 $T_s$ 在分母。
- 在 `{1, 4, 16}` 加上 `64`,觀察雜訊不降反升:延遲誤差 $\dot\omega\cdot(N-1)T_s/2$ 已經主導。
- 把 `w_true` 改成常數 `1.0`,看 500 線的量測 RMS 如何偏離理論(量化誤差週期化,不再像白雜訊)。

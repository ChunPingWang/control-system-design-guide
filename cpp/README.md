# Control System Design Guide — C/C++ 實驗教材

Python 版([`../python/`](../python/README.md) 各章 `lab.ipynb`)的 **C++17 平行版本**:
19 章各一支可執行程式,重現相同實驗、輸出相同數值表,並以相同的 **✅ 驗證條件** 自我檢查。

- **零外部相依**:只用 C++17 標準函式庫(header-only 共用函式庫 `common/`),
  不需要 Eigen、Boost、FFTW。任何有 g++ / clang++ / MSVC 的機器都能跑。
- **與 Python 版逐章對應**:目錄名稱、參數、驗證 assertion 一一對照,可以兩邊交叉閱讀。
- **貼近韌體**:Ch19 直接 `#include` [`hardware/esp32/src/pid.h`](../hardware/esp32/src/pid.h),
  讓會燒進 ESP32 的那份程式碼本身參與驗證。
- **圖表以 CSV 輸出**:每章把波形 / Bode 資料寫成 CSV,可用 Excel、gnuplot,
  或附的 `tools/plot_csv.py` 畫圖。

## 建置與執行

```bash
cd cpp
./run_all.sh                      # 建置 + 執行全部 19 章(ctest),任一章驗證失敗即非零結束
```

手動使用 CMake:

```bash
cmake -S cpp -B cpp/build
cmake --build cpp/build -j
ctest --test-dir cpp/build --output-on-failure    # CSV 輸出到 cpp/build/out/
./cpp/build/ch16                                   # 單跑一章;CSV 預設寫到 ./out/,可用 CSD_OUT 指定
```

不用 CMake,單檔直接編譯也可以(header-only):

```bash
c++ -std=c++17 -O2 -o ch03 cpp/03-tuning/main.cpp && ./ch03
```

畫圖(選用,需 matplotlib):

```bash
python3 cpp/tools/plot_csv.py cpp/build/out/ch04_step_fs500.csv
python3 cpp/tools/plot_csv.py cpp/build/out/*.csv --save figs/     # 批次存 PNG
```

## 目錄結構

```
cpp/
├── common/                 C++ 版 ModelQ-lite(header-only)
│   ├── csd.hpp               總表頭(各章只 include 這一個)
│   ├── linalg.hpp            矩陣、expm、多項式、求根             ← numpy / scipy.linalg
│   ├── lti.hpp               轉移函數、feedback、離散化、邊限、頻寬、
│   │                         時域響應、Padé、Ackermann 極點配置   ← python-control
│   ├── sim.hpp               DiscretePID、MotorPlant、DCMotorPlant、TwoMassPlant、
│   │                         Backlash、EncoderModel、Delay、OnePole、Biquad、iirnotch
│   │                                                                ← python/common/sim.py
│   ├── dsa.hpp               FFT、chirp/PRBS、Welch 交叉頻譜 FRF    ← python/common/dsa.py
│   └── util.hpp              陣列、統計、step_metrics、CSV、Checker ← python/common/control_helpers.py
├── 01-introduction/ … 19-rapid-control-prototyping/   每章 main.cpp + README.md(程式導讀)
├── tools/plot_csv.py       CSV → 圖(選用)
├── CMakeLists.txt          每章一個執行檔 ch01…ch19,並註冊為 ctest
└── run_all.sh              一鍵建置 + 驗證(無 cmake 時退回直接編譯)
```

每章程式結構與 notebook 相同:**參數 → 實驗(印出數值表 + 寫 CSV)→ ✅ 驗證 → 結束碼**。

## Python ↔ C++ 對照

| Python(notebook) | C++ | 備註 |
|---|---|---|
| `ct.tf(num, den)`、`C * P`、`ct.feedback` | `tf(num, den)`、`C * P`、`feedback` | 係數同為降冪 |
| `pi(kp, ki)`、`pid(kp, ki, kd, n)` | `pi_ctrl`、`pid_ctrl` | 避開 `M_PI` 等巨集名稱衝突 |
| `margins(L)`、`bandwidth_hz(T)` | `margins(L)`、`bandwidth_hz(T)` | 密集頻率掃描 + 二分細化 |
| `ct.step_response`、`ct.forced_response` | `step_response`、`forced_response` | 精確離散化(expm),輸入樣本間線性內插 |
| `ct.sample_system(P, dt, method)` | `c2d(P, dt, "zoh"/"tustin"/"matched")` | matched 規則與 python-control 相同 |
| `ct.pade(T, 2)` | `pade2(T)` | |
| `ct.place(A.T, C.T, poles).T` | `place_siso(Aᵀ, Cᵀ, poles)` | Ackermann 公式 |
| `DiscretePID(..., dfilt_hz=None)` | `DiscretePID(..., dfilt_hz = 0)` | 0 表示導數不濾波 |
| `measure_frf(u, y, fs)` | `measure_frf(u, y, fs)` | Hann 窗、50% 重疊,與 scipy 預設相同 |
| `np.random.default_rng(seed)` | `Rng(seed)`(mt19937_64) | 見下方「數值差異」 |
| `scipy.optimize.curve_fit` | Ch19 內的兩參數 Levenberg–Marquardt | |
| Matplotlib 圖 | CSV + `tools/plot_csv.py` | |

## 數值差異說明

確定性的實驗(無亂數)與 Python 版**印出的每一位數字都相同**,例如:
Ch3 選出 kp=0.7 / ki=30(OS 4.5% → 15.5%)、Ch4 Padé PM 45.9°、Ch9 LPF@50 Hz PM 24.9°、
Ch14 雜訊表、Ch15、Ch16 kp_max 表、Ch17、Ch18 雜訊表。

含亂數雜訊的章節(Ch9、Ch13、Ch19)用的是 C++ 標準亂數產生器,無法與 numpy 的
PCG64 序列逐位相同,因此雜訊 RMS、有效量測點數、擬合值會有些微差異
(例:Ch13 共振 75.7 Hz / 反共振 46.4 Hz 相同,有效點 110 vs 111;Ch19 τ 248.6 vs 249.9 ms)。
這些章節的驗證條件本來就是統計性質(比例、上下限),兩邊都通過。

## 與 Python 版的額外驗證

- **Ch19** 除了 notebook 的「演算法等價性」外,另外把真正的韌體 `pid.h`(float32)
  與 double 版逐樣本比對(最大差異約 3e-5 pwm-unit),等於在 host 上驗證了
  韌體本身。
- `CMakeLists.txt` 同時建置 `hardware/esp32/test_host/test_pid.cpp`。

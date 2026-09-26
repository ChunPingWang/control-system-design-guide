# Control System Design Guide — Maker Labs(sim + ESP32 韌體 + lab report)

第 4 個與 `jupyter / python / cpp` 並排的 19 章平行實作,但走 **Maker/硬體路線**:
每章 = **Python simulation + ESP32 韌體 + 工程 lab report**,對應 George Ellis
《Control System Design Guide》19 章,並實踐兩份 Maker 教材
([`../maker-control-learning-roadmap.md`](../maker-control-learning-roadmap.md)、
[`../control-system-design-guide-19-chapter-maker-labs.md`](../control-system-design-guide-19-chapter-maker-labs.md))
的「理論 → 模擬 → 實作 → 量測 → 診斷」循環。

> 想要嚴謹、數值逐位對照的模擬,用 `../jupyter` / `../python` / `../cpp`;
> 想要「從模擬一路做到 ESP32 實機」的 Maker 學習體驗,用這裡。

## 每章三件套

```
maker-labs/NN-主題/
├── sim.py            Python 模擬(python-control/scipy);可執行、章末 assert 自我驗證
├── firmware/main.cpp ESP32 sketch(Arduino);用共用控制 lib,可 host 編譯驗證
└── LAB_REPORT.md     工程 lab report 模板(Objective→…→Exit Criteria)
```

## 共用韌體控制函式庫(`firmware/lib/`,純 C++、host 可測)

| header | 內容 | 主要章節 |
|---|---|---|
| `pid.h` | PID:輸出飽和、條件積分 anti-windup、導數低通、對量測微分 | 3,6,7,8,15,17,18,19 |
| `filter.h` | 一階 IIR 低通、移動平均 | 9,14,16 |
| `encoder.h` | 正交解碼、由 count 估速度、counts/sec→RPM | 2,3,5,14,17… |
| `feedforward.h` | 線性前饋 target→baseline output | 8 |
| `observer.h` | 二階 Luenberger 觀測器(position/velocity) | 10,18 |
| `util.h` | clamp / saturate / deadband / rate limit | 12 |
| `fusion.h` | IMU 互補濾波(傾角) | capstone 自平衡車 |

`firmware/arduino_shim.h` 在定義 `MAKERLAB_HOST` 時提供 Arduino/ESP32 API 的 host mock,
讓每支 `main.cpp` 不需實體板子即可用桌機 g++ 編譯驗證。

## 驗證方式(不依賴 CI 硬體)

因為多數實驗需要實體馬達/IMU,無法在 CI 執行,改採分層驗證:

```bash
# 1) 全部 sim.py:可執行 + 章末 assert(無頭 Agg)
./maker-labs/run_all.sh

# 2) 韌體:共用 lib 的 host 單元測試(真的比對數值)+ 每章 firmware host 編譯
CXX=g++-15 ./maker-labs/verify_firmware.sh
```

- **sim** → 真的跑、真的驗(和 `python/` 同套 assert 機制)。
- **韌體控制邏輯** → `firmware/test/test_lib.cpp` 在 host 上以合成訊號比對數值(沿用
  `hardware/esp32/` 的 `pid.h` + `test_host` 模式)。
- **韌體 Arduino 層** → 套 `arduino_shim.h` 用 g++ 編譯,證明「編得過、API 用法正確」。
- **上機驗證** → 每章 `LAB_REPORT.md` 的 Exit Criteria 作為有硬體時的人工清單。

> 誠實聲明:`verify_firmware.sh` 驗證的是**編譯正確性與控制邏輯數值**,不是真實電機行為。
> 真正的 rise time / overshoot / disturbance recovery 必須上機量測並填入 lab report。

## 硬體(BOM 摘要)

ESP32 DevKit、低壓 DC gear motor + quadrature encoder、TB6612FNG 類 H-bridge、
獨立低壓 DC 電源、麵包板/導線;後期加 MPU6050(自平衡車)。詳見
[`../hardware/README.md`](../hardware/README.md) 與各章 `LAB_REPORT.md`。

## 里程碑專案(對應 roadmap)

1. **DC Motor PID Speed Control**(Ch6)
2. **Digital PID Controller**(Ch4/5)
3. **Motor Model / Digital Twin Lite**(Ch11/13)
4. **Servo Position Control**(Ch17/18)
5. **Self-Balancing Robot / RCP**(Ch19 + capstone,用 `fusion.h`)

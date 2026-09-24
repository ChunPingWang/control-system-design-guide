# Hardware — 第 19 章 RCP 實機平台

商用 RCP 硬體的低成本開源替代:**ESP32 負責即時迴路,Python 負責其他一切**
(規劃、遙測記錄、系統辨識、增益設計、視覺化)。

## 料件

| 元件 | 說明 |
|---|---|
| ESP32 DevKit | 100 Hz 控制迴路 + 序列遙測 |
| TB6612FNG | 馬達驅動(PWM 20 kHz) |
| 直流減速馬達 + AB 相編碼器 | 例:JGA25-371 帶霍爾編碼器 |
| 馬達電源 | 6–12 V,與 ESP32 分開供電、共地 |
| MPU6050(選配) | Capstone 自平衡機器人 |

## 接線(預設腳位,見 `esp32/src/main.cpp` 頂部)

| ESP32 | 接往 |
|---|---|
| GPIO32 / GPIO33 | 編碼器 A / B |
| GPIO25 | TB6612 PWMA |
| GPIO26 / GPIO27 | TB6612 AIN1 / AIN2 |
| GPIO14 | TB6612 STBY |

> 記得依馬達實際規格修改 `COUNTS_PER_REV`(4×線數×減速比)。

## 使用流程(對應 Ch19 notebook)

```bash
cd esp32
pio run -t upload        # 燒錄(需 PlatformIO)
pio device monitor       # 遙測:millis,target,actual,pwm @115200
```

| 命令 | 作用 |
|---|---|
| `STEP,100` | 開迴路固定 PWM(系統辨識用) |
| `CLOSE` | 回到閉迴路 |
| `SET,KP,2.0` / `SET,KI,1.0` / `SET,KD,0.05` | 線上改增益 |
| `SET,TARGET,500` | 速度命令 [rpm] |

## 韌體驗證

控制邏輯集中在 `esp32/src/pid.h`(純 C++、無 Arduino 相依),
已在 host 端與 Python 版逐樣本比對(見 `esp32/test_host/`):

```bash
cd esp32/test_host
c++ -std=c++11 -O2 -o test_pid test_pid.cpp
./test_pid > pid_c_output.csv
python3 check_pid.py pid_c_output.csv     # PASS = 等價
```

`main.cpp`(Arduino 層)需 PlatformIO + ESP32 toolchain 才能編譯;
本 repo 未附編譯產物。

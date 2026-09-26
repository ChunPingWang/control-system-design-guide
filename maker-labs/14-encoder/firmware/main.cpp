// Ch14 Encoders and Resolvers — ESP32 正交編碼器速度估測 + 低通
//
// 本章 Maker Lab:用共用 encoder.h(QuadDecoder + VelocityEstimator)讀
// quadrature encoder,固定量測窗差分求速度;再用 filter.h 的 OnePoleLP
// 對 velocity 做一階低通,CSV 同時輸出 raw 與 filtered velocity 以便比較
// 「resolution vs latency(noise 換 lag)」的取捨。
//
// Serial CSV 欄位:t_ms,position,rpm_raw,rpm_filt
//
// host 編譯驗證(不需實體板子):
//   g++ -DMAKERLAB_HOST -I../../firmware -I../../firmware/lib -c main.cpp
#ifdef MAKERLAB_HOST
#include "arduino_shim.h"
#else
#include <Arduino.h>
#endif
#include "encoder.h"
#include "filter.h"

using namespace maker;

// ---- 接腳 ----
constexpr int PIN_ENC_A = 32;
constexpr int PIN_ENC_B = 33;

// ---- 編碼器 / 取樣參數 ----
constexpr float COUNTS_PER_REV = 400.0f;   // 4x 解碼後 CPR
constexpr uint32_t CONTROL_US  = 10000;    // 100 Hz 量測窗 (10 ms)
constexpr float DT = CONTROL_US / 1e6f;
constexpr float FILT_FC_HZ = 8.0f;         // velocity 低通截止頻率

QuadDecoder enc;             // 正交解碼狀態機(ISR 內累積 position)
VelocityEstimator vel;       // 固定窗差分求 counts/sec
OnePoleLP vfilt;             // velocity 一階低通
volatile long g_pos = 0;     // ISR 與主迴圈共享的最新 position
uint32_t nextTick;

#ifndef MAKERLAB_HOST
// 真實硬體:A/B 任一相變化都觸發 4x 解碼
void IRAM_ATTR onEncoder() {
    enc.update(digitalRead(PIN_ENC_A), digitalRead(PIN_ENC_B));
    g_pos = enc.position;
}
#endif

void setup() {
    Serial.begin(115200);
    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);
#ifndef MAKERLAB_HOST
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), onEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), onEncoder, CHANGE);
#endif
    vfilt.init(FILT_FC_HZ, DT);   // 依截止頻率與量測窗設定 alpha
    nextTick = micros();
    Serial.println("t_ms,position,rpm_raw,rpm_filt");
}

void loop() {
    uint32_t now = micros();
    if ((int32_t)(now - nextTick) >= 0) {
        nextTick += CONTROL_US;

        // 1. 讀取本窗最新 position(ISR 累積的量化 count)
        long pos = g_pos;
        // 2. 固定窗差分 → counts/sec → rpm(raw)
        float cps = vel.step(pos, DT);
        float rpm_raw = VelocityEstimator::counts_per_sec_to_rpm(cps, COUNTS_PER_REV);
        // 3. 一階低通:壓量化雜訊,代價是引入 lag
        float rpm_filt = vfilt.step(rpm_raw);
        // 4. CSV 遙測(raw vs filtered 供離線比較)
        Serial.printf("%lu,%ld,%.2f,%.2f\n",
                      (unsigned long)millis(), pos, rpm_raw, rpm_filt);
    }
}

#ifdef MAKERLAB_HOST
// host 驗證:餵入合成 encoder count 模擬等速旋轉。
// 每 10 ms 窗約前進 (rpm/60*CPR*DT) 個 count,對 120 rpm 為 8 counts/窗;
// 用整數量化(floor)重現「低速時每窗 count 少 → velocity 有量化抖動」。
int main() {
    setup();
    const double rpm_true = 120.0;
    const double cps_true = rpm_true / 60.0 * COUNTS_PER_REV;  // 真實 counts/sec
    double true_pos = 0.0;
    for (int i = 0; i < 100; i++) {
        g_micros += CONTROL_US;
        true_pos += cps_true * DT;          // 連續真實位置
        g_pos = (long)true_pos;             // 編碼器只回報整數 count(量化)
        loop();
    }
    return 0;
}
#endif

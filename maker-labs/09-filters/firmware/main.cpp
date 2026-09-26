// Ch9 Filters in Control Systems — ESP32 一階 IIR low-pass on RPM
//
// 本章 Maker Lab:用共用 filter.h 的 OnePoleLP 對 encoder 量得的 RPM 做低通,
// 同時記錄 raw RPM 與 filtered RPM,觀察數位濾波如何壓掉 sensor noise。
// 重點實驗:逐步降低 cutoff(FILTER_FC_HZ),噪聲更平滑,但 filter 引入的
// phase lag 會進到控制迴路裡 → 相位裕度下降、響應變鈍甚至震盪。
// 「越平滑越好」是錯的:filter cutoff 要和迴路頻寬一起看。
//
// Serial CSV 欄位:t,raw_rpm,filt_rpm
//
// host 編譯驗證:
//   g++ -DMAKERLAB_HOST -I../../firmware -I../../firmware/lib -c main.cpp
#ifdef MAKERLAB_HOST
#include "arduino_shim.h"
#else
#include <Arduino.h>
#endif
#include "filter.h"
#include "encoder.h"

using namespace maker;

// ---- 接腳 ----
constexpr int PIN_ENC_A = 32;
constexpr int PIN_ENC_B = 33;

constexpr float COUNTS_PER_REV = 1000.0f;
constexpr uint32_t CONTROL_US = 10000;   // 100 Hz 控制/取樣迴路
constexpr float DT = CONTROL_US / 1e6f;

// filter 截止頻率:實驗時逐步調低(如 30 → 15 → 5 Hz),
// 觀察 filt_rpm 更平滑,但相對 raw_rpm 的延遲變大。
constexpr float FILTER_FC_HZ = 15.0f;

QuadDecoder enc;
VelocityEstimator vel;
OnePoleLP rpm_filter;            // 一階 IIR:y += alpha*(x-y)
volatile long g_pos = 0;
uint32_t nextTick;

#ifndef MAKERLAB_HOST
// 正交編碼器中斷:僅在真板上編譯(host 端無中斷)。
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
    // 由截止頻率 + 取樣週期算 alpha(filter.h 內建)
    rpm_filter.init(FILTER_FC_HZ, DT);
    Serial.println("t,raw_rpm,filt_rpm");
    nextTick = micros();
}

void loop() {
    uint32_t now = micros();
    if ((int32_t)(now - nextTick) >= 0) {
        nextTick += CONTROL_US;

        // 1. 讀 encoder → raw RPM(含量測雜訊)
        float cps = vel.step(g_pos, DT);
        float raw_rpm = VelocityEstimator::counts_per_sec_to_rpm(cps, COUNTS_PER_REV);

        // 2. 一階 IIR 低通 → filtered RPM
        float filt_rpm = rpm_filter.step(raw_rpm);

        // 3. 遙測:同時輸出 raw 與 filtered 供離線比對
        //    註:降低 FILTER_FC_HZ 會讓 filt_rpm 落後 raw_rpm 更多,
        //        若把 filt_rpm 餵回 controller,這段 lag 會侵蝕穩定裕度。
        Serial.printf("%lu,%.1f,%.1f\n",
                      (unsigned long)millis(), raw_rpm, filt_rpm);
    }

    // 非即時的指令解析可另外處理
}

#ifdef MAKERLAB_HOST
int main() {
    setup();
    // 餵入「漸增位置 + 交替雜訊」的假 encoder 資料,確認濾波路徑可執行、
    // filtered 值介於雜訊範圍內、且不 crash。
    float last_filt = 0.0f;
    for (int i = 0; i < 100; i++) {
        g_micros += CONTROL_US;
        long jitter = (i % 2 == 0) ? +8 : -8;   // 模擬量測雜訊
        g_pos += 100 + jitter;
        loop();
        // 讀出目前 filter 狀態(host 端驗證用)
        last_filt = rpm_filter.y;
    }
    // 濾波器應收斂到雜訊平均附近(約 6000 RPM),而非 NaN/發散。
    if (!(last_filt > 100.0f && last_filt < 12000.0f)) return 1;
    return 0;
}
#endif

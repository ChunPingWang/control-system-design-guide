// Ch2 The Frequency Domain — ESP32 DC Motor step-response 資料收集
//
// 本章 Maker Lab:對 DC motor 施加「多組 PWM step」,用 encoder 讀 RPM。
// 先不做正式 frequency sweep;目的是從 step response 估 time constant τ,
// 為後續建立 motor transfer function(1/(τs+1))做準備 —— 這正是把
// 時域量測轉成頻域模型的第一步。
//
// Serial CSV 欄位:t,pwm_duty,rpm
// 之後在 Python 對每個 step 擬合一階響應,τ ≈ 到達 63.2% 終值所需時間。
//
// host 編譯驗證(不需實機):
//   g++ -DMAKERLAB_HOST -I../../firmware -I../../firmware/lib -c main.cpp
// 於 ESP32 上請用 PlatformIO / Arduino,移除 MAKERLAB_HOST。
#ifdef MAKERLAB_HOST
#include "arduino_shim.h"
#else
#include <Arduino.h>
#endif
#include "encoder.h"

using namespace maker;

// ---- 接腳 ----
constexpr int PIN_ENC_A = 32;
constexpr int PIN_ENC_B = 33;
constexpr int PIN_PWM   = 25;
constexpr int PIN_DIR   = 26;
constexpr int PWM_CH = 0, PWM_FREQ = 20000, PWM_RES = 8;   // 8-bit:0..255

constexpr float COUNTS_PER_REV = 1000.0f;
constexpr uint32_t CONTROL_US = 10000;   // 100 Hz 取樣
constexpr float DT = CONTROL_US / 1e6f;

// ---- Step 排程:每個 duty 維持固定時間,量測開迴路 step response ----
// 開迴路(不加 controller):直接把 PWM step 送進馬達,觀察 RPM 如何逼近終值。
constexpr int   STEP_LEVELS[] = {0, 80, 160, 255, 120, 0};   // 依序施加的 duty
constexpr int   N_STEPS = sizeof(STEP_LEVELS) / sizeof(STEP_LEVELS[0]);
constexpr uint32_t STEP_TICKS = 200;     // 每個 step 維持 200 tick = 2.0 s

QuadDecoder enc;
VelocityEstimator vel;
volatile long g_pos = 0;
uint32_t nextTick;
uint32_t tick_count = 0;      // 累積控制週期數
int      step_idx = 0;        // 目前第幾個 step

#ifndef MAKERLAB_HOST
void IRAM_ATTR onEncoder() {
    enc.update(digitalRead(PIN_ENC_A), digitalRead(PIN_ENC_B));
    g_pos = enc.position;
}
#endif

void setup() {
    Serial.begin(115200);
    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);
    pinMode(PIN_DIR, OUTPUT);
    ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_PWM, PWM_CH);
#ifndef MAKERLAB_HOST
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), onEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), onEncoder, CHANGE);
#endif
    digitalWrite(PIN_DIR, 1);            // 本實驗固定單方向,只看轉速上升/下降
    nextTick = micros();
}

void loop() {
    uint32_t now = micros();
    if ((int32_t)(now - nextTick) >= 0) {
        nextTick += CONTROL_US;

        // 1. 依排程決定目前 step 的 PWM duty(step_idx 隨時間前進)
        step_idx = (int)(tick_count / STEP_TICKS);
        if (step_idx >= N_STEPS) step_idx = N_STEPS - 1;   // 收尾停在最後一個 level
        int duty = STEP_LEVELS[step_idx];
        if (duty < 0) duty = 0;
        if (duty > 255) duty = 255;

        // 2. 讀 encoder → RPM(開迴路,不做控制計算)
        float cps = vel.step(g_pos, DT);
        float rpm = VelocityEstimator::counts_per_sec_to_rpm(cps, COUNTS_PER_REV);

        // 3. 更新 PWM(clamp 已完成)
        ledcWrite(PWM_CH, duty);

        // 4. 遙測 CSV:t,pwm_duty,rpm(供 Python 擬合 τ)
        Serial.printf("%lu,%d,%.1f\n", (unsigned long)millis(), duty, rpm);

        tick_count++;
    }

    // 非即時的指令解析可另外處理(本章不需要)
}

#ifdef MAKERLAB_HOST
int main() {
    setup();
    // 餵入合成的 encoder 位置:模擬一階馬達對 step 的近似反應,
    // 確認 step 排程、encoder→RPM、CSV 路徑都跑得起來且不 crash。
    long pos = 0;
    for (uint32_t i = 0; i < STEP_TICKS * (uint32_t)N_STEPS; i++) {
        int idx = (int)(i / STEP_TICKS);
        if (idx >= N_STEPS) idx = N_STEPS - 1;
        // 假設 counts/tick 與 duty 大致成比例(粗略一階近似)
        long dpos = STEP_LEVELS[idx];
        pos += dpos;
        g_micros += CONTROL_US;
        g_pos = pos;
        loop();
    }
    return 0;
}
#endif

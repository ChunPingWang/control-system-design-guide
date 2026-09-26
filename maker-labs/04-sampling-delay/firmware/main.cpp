// Ch4 Delay in Digital Controllers — ESP32 固定週期控制 loop 與取樣延遲量測
//
// 本章 Maker Lab 重點:
//   * 用 micros() 建「固定週期」排程(nextTick += CONTROL_US),
//     而不是用長時間 delay() 卡住 CPU —— delay() 會讓週期漂移且無法量 jitter。
//   * 可切換控制週期(1 / 5 / 10 / 50 ms),觀察同一 controller 在不同
//     sample time 下的 RPM 響應與穩定度。
//   * 每個 tick 量「實際週期」與「相對名目週期的 jitter」,證明排程精度。
//
// Serial CSV 欄位:
//   t_ms,control_us,actual_dt_us,jitter_us,setpoint,rpm,pwm
//
// host 編譯驗證(不需實體板子):
//   g++ -std=c++17 -DMAKERLAB_HOST -I../../firmware -I../../firmware/lib -c main.cpp
#ifdef MAKERLAB_HOST
#include "arduino_shim.h"
#else
#include <Arduino.h>
#endif
#include "encoder.h"
#include "pid.h"

using namespace maker;

// ---- 接腳 ----
constexpr int PIN_ENC_A = 32;
constexpr int PIN_ENC_B = 33;
constexpr int PIN_PWM   = 25;
constexpr int PIN_DIR   = 26;
constexpr int PWM_CH = 0, PWM_FREQ = 20000, PWM_RES = 8;

constexpr float COUNTS_PER_REV = 1000.0f;

// ---- 可切換的控制週期(本章實驗核心)----
// 依序試 1 / 5 / 10 / 50 ms,比較實際 jitter 與 RPM 響應。
// 真機可改成從 Serial 讀指令切換;此處以陣列 + 索引示範。
constexpr uint32_t PERIOD_TABLE_US[] = {1000, 5000, 10000, 50000};
constexpr int      PERIOD_COUNT      = 4;
int      g_period_idx = 2;                               // 預設 10 ms
uint32_t g_control_us = PERIOD_TABLE_US[2];

QuadDecoder enc;
VelocityEstimator vel;
Pid pid;
volatile long g_pos = 0;

uint32_t nextTick;       // 下一個控制 tick 的名目時間(micros)
uint32_t lastTickUs = 0; // 上一次實際進入控制區塊的時間(量 jitter 用)
bool     firstTick = true;
float    setpoint_rpm = 300.0f;

#ifndef MAKERLAB_HOST
void IRAM_ATTR onEncoder() {
    enc.update(digitalRead(PIN_ENC_A), digitalRead(PIN_ENC_B));
    g_pos = enc.position;
}
#endif

// 切換控制週期並重置 PID 與 scheduler(讓每段實驗乾淨開始)
void applyPeriod(int idx) {
    if (idx < 0) idx = 0;
    if (idx >= PERIOD_COUNT) idx = PERIOD_COUNT - 1;
    g_period_idx = idx;
    g_control_us = PERIOD_TABLE_US[idx];
    float dt = g_control_us / 1e6f;
    // 同一組增益,只改 dt —— 觀察 sample time 對穩定度的影響
    pid.init(0.4f, 6.0f, 0.01f, dt, -255.0f, 255.0f);
    pid.dfilt_hz = 20.0f;
    vel.reset();
    firstTick = true;
    nextTick = micros();
}

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
    applyPeriod(g_period_idx);
    Serial.println("t_ms,control_us,actual_dt_us,jitter_us,setpoint,rpm,pwm");
}

void loop() {
    uint32_t now = micros();

    // === 固定週期排程:到時才做,沒到時 CPU 可做別的事(不要用 delay 卡住) ===
    if ((int32_t)(now - nextTick) >= 0) {
        // 先量「實際週期」與「jitter = 實際 - 名目」,用來評估排程精度
        uint32_t actual_dt = firstTick ? g_control_us : (now - lastTickUs);
        int32_t  jitter    = (int32_t)actual_dt - (int32_t)g_control_us;
        lastTickUs = now;
        firstTick  = false;

        nextTick += g_control_us;   // 累加名目週期(避免週期漂移)
        // 若因負載超時落後多個週期,追上以免爆量補跑
        if ((int32_t)(micros() - nextTick) >= 0) {
            nextTick = micros() + g_control_us;
        }

        float dt = g_control_us / 1e6f;

        // 1. 讀 encoder → RPM
        float cps = vel.step(g_pos, dt);
        float rpm = VelocityEstimator::counts_per_sec_to_rpm(cps, COUNTS_PER_REV);
        // 2. PID(setpoint, measurement)
        float u = pid.step(setpoint_rpm, rpm);
        // 3. 方向 + duty(限幅)
        int dir = (u >= 0) ? 1 : 0;
        int duty = (int)(u >= 0 ? u : -u);
        if (duty > 255) duty = 255;
        digitalWrite(PIN_DIR, dir);
        ledcWrite(PWM_CH, duty);
        // 4. CSV 遙測(含實際週期與 jitter)
        Serial.printf("%lu,%lu,%lu,%ld,%.1f,%.1f,%d\n",
                      (unsigned long)millis(), (unsigned long)g_control_us,
                      (unsigned long)actual_dt, (long)jitter,
                      setpoint_rpm, rpm, duty);
    }

    // 非即時工作(如 Serial 指令切換週期)可放這裡,不影響控制排程。
}

#ifdef MAKERLAB_HOST
// host 驗證:對每種控制週期各跑一段,餵入假的漸增 encoder 位置,
// 確認排程/jitter 計算與控制路徑都能執行、不 crash。
int main() {
    setup();
    for (int p = 0; p < PERIOD_COUNT; p++) {
        applyPeriod(p);
        for (int i = 0; i < 40; i++) {
            // 模擬「時間到 + 些微 jitter」:名目週期再加一點抖動
            g_micros += g_control_us + (uint32_t)((i % 3) * 7);
            g_pos += 100;                 // 假裝馬達持續轉
            loop();
        }
    }
    return 0;
}
#endif

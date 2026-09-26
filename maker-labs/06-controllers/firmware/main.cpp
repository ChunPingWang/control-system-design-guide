// Ch6 Four Types of Controllers — ESP32 DC Motor Speed PID
//
// 本章 Maker Lab:用共用 pid.h + encoder.h 做馬達轉速閉迴路。
// Serial CSV 欄位:t,setpoint,rpm,error,pwm,p,i,d,saturated
// 含 output clamp、integral anti-windup(pid.h 內建)、derivative low-pass。
//
// host 編譯驗證:
//   g++ -DMAKERLAB_HOST -I../../firmware -I../../firmware/lib -c main.cpp
#ifdef MAKERLAB_HOST
#include "arduino_shim.h"
#else
#include <Arduino.h>
#endif
#include "pid.h"
#include "encoder.h"

using namespace maker;

// ---- 接腳 ----
constexpr int PIN_ENC_A = 32;
constexpr int PIN_ENC_B = 33;
constexpr int PIN_PWM   = 25;
constexpr int PIN_DIR   = 26;
constexpr int PWM_CH = 0, PWM_FREQ = 20000, PWM_RES = 8;

constexpr float COUNTS_PER_REV = 1000.0f;
constexpr uint32_t CONTROL_US = 10000;   // 100 Hz
constexpr float DT = CONTROL_US / 1e6f;

QuadDecoder enc;
VelocityEstimator vel;
Pid pid;
volatile long g_pos = 0;
uint32_t nextTick;
float setpoint_rpm = 300.0f;

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
    // 輸出範圍 -255..255(方向 + duty),導數低通 20 Hz
    pid.init(0.4f, 6.0f, 0.01f, DT, -255.0f, 255.0f);
    pid.dfilt_hz = 20.0f;
    nextTick = micros();
}

void loop() {
    uint32_t now = micros();
    if ((int32_t)(now - nextTick) >= 0) {
        nextTick += CONTROL_US;

        // 1. 讀 encoder → RPM
        float cps = vel.step(g_pos, DT);
        float rpm = VelocityEstimator::counts_per_sec_to_rpm(cps, COUNTS_PER_REV);
        // 2. PID(setpoint, measurement)
        float u = pid.step(setpoint_rpm, rpm);
        // 3. 方向 + duty
        int dir = (u >= 0) ? 1 : 0;
        int duty = (int)(u >= 0 ? u : -u);
        if (duty > 255) duty = 255;
        digitalWrite(PIN_DIR, dir);
        ledcWrite(PWM_CH, duty);
        // 4. 遙測
        float e = setpoint_rpm - rpm;
        int sat = (u <= -255.0f || u >= 255.0f) ? 1 : 0;
        Serial.printf("%lu,%.1f,%.1f,%.1f,%d,%.2f,%.2f,%.2f,%d\n",
                      (unsigned long)millis(), setpoint_rpm, rpm, e, duty,
                      pid.kp * e, pid.ki * pid.integ, 0.0f, sat);
    }
}

#ifdef MAKERLAB_HOST
int main() {
    setup();
    // 餵入假的漸增 encoder 位置,確認控制路徑可執行、pid 收斂邏輯不 crash
    for (int i = 0; i < 50; i++) { g_micros += CONTROL_US; g_pos += 100; loop(); }
    return 0;
}
#endif

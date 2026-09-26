// Ch8 Feed-Forward — ESP32 DC Motor Speed (FF + PID)
//
// 本章 Maker Lab:先由 open-loop step test 建立 target_rpm → baseline_pwm 的
// 線性模型(feedforward.h 的 LinearFeedforward),控制時把 baseline PWM 直接前饋,
// 再讓 PID(pid.h)只修正 residual error(model mismatch / disturbance / 負載變動)。
//
// 概念比較(PID-only vs FF+PID):
//   - PID-only:setpoint 一跳,積分器要從 0 慢慢累積出穩態 PWM → rise 慢、易 windup。
//   - FF+PID :FF 立刻給出接近正確的 baseline PWM → PID 只補小殘差 → rise 快、
//              積分器負擔小、overshoot/windup 都較低。FF 不取代 FB:模型不準時
//              仍靠 PID 收尾,兩者穩態相同。
//
// Serial CSV 欄位:t,setpoint,rpm,error,pwm_ff,pwm_pid,pwm,saturated
// host 編譯驗證:
//   g++ -DMAKERLAB_HOST -I../../firmware -I../../firmware/lib -c main.cpp
#ifdef MAKERLAB_HOST
#include "arduino_shim.h"
#else
#include <Arduino.h>
#endif
#include "feedforward.h"
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

// ---- Feed-forward 模型(由 open-loop step test 擬合)----
// 校準點:target 100 rpm → 需 ~40 duty;target 500 rpm → ~200 duty。
// 實驗時請以自己量到的 (target_rpm, steady_pwm) 兩點重新 calibrate()。
constexpr float FF_T1 = 100.0f, FF_U1 = 40.0f;
constexpr float FF_T2 = 500.0f, FF_U2 = 200.0f;

QuadDecoder enc;
VelocityEstimator vel;
LinearFeedforward ff;
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
    // Feed-forward:target_rpm → baseline_pwm 線性模型(兩點擬合)。
    ff.calibrate(FF_T1, FF_U1, FF_T2, FF_U2);
    // PID 只修正殘差,故增益比 PID-only 版本小;輸出留給 residual 修正範圍。
    // 導數對量測微分 + 20 Hz 低通(pid.h 內建),anti-windup 內建。
    pid.init(0.2f, 3.0f, 0.005f, DT, -120.0f, 120.0f);
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
        // 2. Feed-forward:由 target 直接算 baseline PWM(不看 error)
        float u_ff = ff.compute(setpoint_rpm);
        // 3. PID:只修正 residual error(feedback 收尾)
        float u_pid = pid.step(setpoint_rpm, rpm);
        // 4. 合成 + clamp(8-bit duty)
        float u = u_ff + u_pid;
        int dir = (u >= 0) ? 1 : 0;
        int duty = (int)(u >= 0 ? u : -u);
        if (duty > 255) duty = 255;
        digitalWrite(PIN_DIR, dir);
        ledcWrite(PWM_CH, duty);
        // 5. 遙測(CSV)
        float e = setpoint_rpm - rpm;
        int sat = (duty >= 255) ? 1 : 0;
        Serial.printf("%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d\n",
                      (unsigned long)millis(), setpoint_rpm, rpm, e,
                      u_ff, u_pid, duty, sat);
    }

    // 非即時的命令解析(改 setpoint 等)可在此處理。
}

#ifdef MAKERLAB_HOST
int main() {
    setup();
    // host 煙霧測試:餵入假的漸增 encoder 位置,確認 FF+PID 控制路徑可執行、不 crash。
    for (int i = 0; i < 50; i++) { g_micros += CONTROL_US; g_pos += 100; loop(); }
    return 0;
}
#endif

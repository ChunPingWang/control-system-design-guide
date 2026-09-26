// Ch15 Basics of the Electric Servomotor and Drive — ESP32 串級控制概念草稿
//
// 本章 Maker Lab:示範伺服驅動器的 cascade(串級)架構 ——
//   外層「速度迴路」PID 產生「電流/轉矩命令」i_ref,
//   內層「電流迴路」再快速把實際電流追到 i_ref(這裡以簡化 P 控制近似)。
// 內層取樣/更新遠快於外層,對應 sim.py 的頻寬分層概念。
//
// 真實伺服驅動器的電流量測需 current sensor;本草稿在無 sensor 時
// 以「馬達電氣一階模型」在 host/板上估算電流,讓學生先看懂控制結構。
// 所有馬達實驗請使用低壓 DC,勿接觸高功率/市電。
//
// Serial CSV 欄位:t,sp_rpm,rpm,err_rpm,i_ref,i_meas,duty,sat
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
constexpr int PIN_ISENSE = 34;   // 電流感測 ADC(選配;無 sensor 時走模型估算)
constexpr int PWM_CH = 0, PWM_FREQ = 20000, PWM_RES = 8;

constexpr float COUNTS_PER_REV = 1000.0f;

// ---- 串級時脈:內層電流迴路遠快於外層速度迴路 ----
constexpr uint32_t INNER_US = 1000;    // 內層 1 kHz(電流迴路)
constexpr uint32_t OUTER_US = 10000;   // 外層 100 Hz(速度迴路)
constexpr float DT_INNER = INNER_US / 1e6f;
constexpr float DT_OUTER = OUTER_US / 1e6f;

// ---- DC 馬達電氣參數(用於在無 current sensor 時估算電流)----
constexpr float R_MOT = 2.0f, L_MOT = 0.01f;   // Ω, H
constexpr float KE_MOT = 0.08f;                // V·s/rad(反電動勢)
constexpr float V_SUPPLY = 12.0f;              // 供電電壓(V)
constexpr float I_MAX = 3.0f;                  // 電流命令上限(A,保護)

QuadDecoder enc;
VelocityEstimator vel;
Pid pid_vel;      // 外層速度 PID(輸出 = 電流命令 i_ref)
Pid pid_cur;      // 內層電流 P 控制(輸出 = 電壓命令)
volatile long g_pos = 0;

uint32_t nextInner, nextOuter;
float setpoint_rpm = 300.0f;
float i_ref = 0.0f;       // 外層產生的電流命令(A)
float i_meas = 0.0f;      // 內層量測/估算電流(A)
float omega = 0.0f;       // 目前角速度(rad/s),供電氣模型用
float last_rpm = 0.0f;

#ifndef MAKERLAB_HOST
void IRAM_ATTR onEncoder() {
    enc.update(digitalRead(PIN_ENC_A), digitalRead(PIN_ENC_B));
    g_pos = enc.position;
}
#endif

// 由 PWM 電壓命令,以電氣一階模型更新估算電流(無 sensor 時的替代)。
// V = R i + L di/dt + Ke ω  →  di/dt = (V - R i - Ke ω)/L
static float estimate_current(float v_cmd, float dt) {
    float didt = (v_cmd - R_MOT * i_meas - KE_MOT * omega) / L_MOT;
    return i_meas + didt * dt;
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_ISENSE, INPUT);
    ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_PWM, PWM_CH);
#ifndef MAKERLAB_HOST
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), onEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), onEncoder, CHANGE);
#endif
    // 外層速度 PID:輸出限制在 ±I_MAX(電流命令),導數低通 20 Hz
    pid_vel.init(0.003f, 0.02f, 0.0f, DT_OUTER, -I_MAX, I_MAX);
    pid_vel.dfilt_hz = 20.0f;
    // 內層電流 P 控制:輸出電壓命令,限制在 ±V_SUPPLY
    pid_cur.init(20.0f, 0.0f, 0.0f, DT_INNER, -V_SUPPLY, V_SUPPLY);

    uint32_t now = micros();
    nextInner = now;
    nextOuter = now;
}

void loop() {
    uint32_t now = micros();

    // ---- 外層:速度迴路(慢)---- 產生電流命令 i_ref
    if ((int32_t)(now - nextOuter) >= 0) {
        nextOuter += OUTER_US;
        float cps = vel.step(g_pos, DT_OUTER);
        float rpm = VelocityEstimator::counts_per_sec_to_rpm(cps, COUNTS_PER_REV);
        last_rpm = rpm;
        omega = rpm * 2.0f * 3.14159265358979f / 60.0f;   // rad/s
        i_ref = pid_vel.step(setpoint_rpm, rpm);
    }

    // ---- 內層:電流迴路(快)---- 把實際電流追到 i_ref
    if ((int32_t)(now - nextInner) >= 0) {
        nextInner += INNER_US;

        // 電壓命令 = 內層電流 P 控制
        float v_cmd = pid_cur.step(i_ref, i_meas);

        // 無 current sensor:以電氣模型估算電流(有 sensor 可改讀 ADC)
        i_meas = estimate_current(v_cmd, DT_INNER);

        // 電壓命令 → 方向 + PWM duty(0..255)
        int dir = (v_cmd >= 0) ? 1 : 0;
        float duty_f = (v_cmd >= 0 ? v_cmd : -v_cmd) / V_SUPPLY * 255.0f;
        int duty = (int)duty_f;
        if (duty > 255) duty = 255;
        digitalWrite(PIN_DIR, dir);
        ledcWrite(PWM_CH, duty);

        // 遙測(以外層量測的 rpm 一起輸出)
        float e_rpm = setpoint_rpm - last_rpm;
        int sat = (i_ref <= -I_MAX || i_ref >= I_MAX) ? 1 : 0;
        Serial.printf("%lu,%.1f,%.1f,%.1f,%.3f,%.3f,%d,%d\n",
                      (unsigned long)millis(), setpoint_rpm, last_rpm, e_rpm,
                      i_ref, i_meas, duty, sat);
    }
}

#ifdef MAKERLAB_HOST
int main() {
    setup();
    // 餵入假的漸增 encoder 位置,確認外/內層串級控制路徑可執行、不 crash
    for (int i = 0; i < 500; i++) {
        g_micros += INNER_US;
        if (i % 10 == 0) g_pos += 100;   // 每 10 個內層 tick 動一次(對應外層節奏)
        loop();
    }
    return 0;
}
#endif

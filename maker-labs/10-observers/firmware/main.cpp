// Ch10 Introduction to Observers — ESP32 Luenberger Observer(離線驗證)
//
// 本章 Maker Lab:用共用 observer.h(Observer2)+ encoder.h。
//   - 量測(measurement):encoder position(唯一直接可量的 state)。
//   - 估計(estimate)   :用 2 狀態 Luenberger observer 估 velocity。
//   - 對照(baseline)   :finite-difference velocity(直接對 position 差分)。
// 目的是離線比較 observer-estimated velocity 與 finite-difference velocity,
// 看 observer 如何在含噪/量化的 position 上給出較平滑的速度估計。
//
// ⚠️ 安全提醒:observer 尚未在硬體上驗證前,「不要」把它接進 safety-critical
//    control loop。先用本 sketch 做離線 telemetry 比對,確認估計可信、
//    error dynamics 收斂、對 model mismatch 不發散後,才用於實際控制。
//
// Serial CSV 欄位:t,pos,fd_vel,obs_vel
//
// host 編譯驗證:
//   g++ -std=c++17 -DMAKERLAB_HOST -I../../firmware -I../../firmware/lib -c main.cpp
#ifdef MAKERLAB_HOST
#include "arduino_shim.h"
#else
#include <Arduino.h>
#endif
#include "observer.h"
#include "encoder.h"

using namespace maker;

// ---- 接腳 ----
constexpr int PIN_ENC_A = 32;
constexpr int PIN_ENC_B = 33;

constexpr uint32_t CONTROL_US = 10000;   // 100 Hz
constexpr float DT = CONTROL_US / 1e6f;

// ---- observer 模型(與 Python sim 一致)----
// 連續模型 state = [position, velocity]:
//   A = [[0, 1], [0, -2]], B = [[0], [2]], C = [1, 0]
// 由 pole placement(poles = -6, -7)得 observer gain L = [11, 20]
//   char(A-LC) = s^2 + (l0+2)s + (2 l0 + l1) = (s+6)(s+7) = s^2 + 13 s + 42
//   ⇒ l0 = 11, l1 = 20(Python 端用 ct.place 驗證,firmware 直接帶入)
float OBS_A[2][2] = {{0, 1}, {0, -2}};
float OBS_B[2]    = {0, 2};
float OBS_C[2]    = {1, 0};
float OBS_L[2]    = {11.0f, 20.0f};

QuadDecoder enc;
VelocityEstimator vel;      // finite-difference 速度(對照組)
Observer2 obs;              // Luenberger observer
volatile long g_pos = 0;
uint32_t nextTick;

// 本離線比對不施加已知控制輸入,observer 僅靠 measurement 修正(u = 0)。
constexpr float U_INPUT = 0.0f;

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
#ifndef MAKERLAB_HOST
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), onEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), onEncoder, CHANGE);
#endif
    obs.init(OBS_A, OBS_B, OBS_C, OBS_L, DT);
    nextTick = micros();
}

void loop() {
    uint32_t now = micros();
    if ((int32_t)(now - nextTick) >= 0) {
        nextTick += CONTROL_US;

        // 1. 讀 encoder position(唯一 measurement)
        float pos = (float)g_pos;
        // 2. 對照組:finite-difference velocity(直接差分,含量化噪聲)
        float fd_vel = vel.step(g_pos, DT);
        // 3. observer:用 position 修正,估計 velocity
        obs.step(U_INPUT, pos);
        float obs_vel = obs.velocity();
        // 4. 遙測(離線分析用;observer 尚未進入控制迴路)
        Serial.printf("%lu,%.1f,%.3f,%.3f\n",
                      (unsigned long)millis(), pos, fd_vel, obs_vel);
    }

    // 非即時的指令解析可另外處理
}

#ifdef MAKERLAB_HOST
int main() {
    setup();
    // 餵入假的漸增 encoder counts(等速運動),確認估計路徑可執行、
    // observer 速度會收斂到差分速度附近而不發散。
    for (int i = 0; i < 50; i++) {
        g_micros += CONTROL_US;
        g_pos += 5;               // 每步 +5 counts → 5/DT counts/s
        loop();
    }
    return 0;
}
#endif

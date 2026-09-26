// Ch18 Using the Luenberger Observer in Motion Control — ESP32(host 可編譯)
//
// 本章 Maker Lab:把 Luenberger observer 真正接進 velocity 控制迴路。
//   - 量測(measurement):encoder position(唯一直接可量的 state,含量化噪聲)。
//   - 估計(estimate)   :用 2 狀態 Luenberger observer(observer.h / Observer2)
//                        由 position 估 velocity,當成迴路的速度回授。
//   - 控制(control)    :velocity PID(pid.h / Pid)追 velocity setpoint。
//   - 對照(baseline)   :同步算 finite-difference velocity(encoder.h)只做遙測,
//                        方便離線比較「差分回授 vs observer 回授」的噪聲差異。
//
// 為什麼要用 observer:直接對 noisy position 差分,噪聲被 1/dt 放大後灌進控制命令,
// 造成馬達命令抖動;observer 由 plant model 估速度,回授平滑、命令乾淨。
//
// ⚠️ 安全提醒:observer 進 feedback loop 前,務必先確認離線 telemetry 中 observer
//    velocity 收斂、對 model mismatch 不發散;上機時保留命令飽和(clamp)與急停。
//
// Serial CSV 欄位:t_ms,pos,vel_sp,fd_vel,obs_vel,u
//
// host 編譯驗證:
//   g++ -std=c++17 -DMAKERLAB_HOST -I../../firmware -I../../firmware/lib -c main.cpp
#ifdef MAKERLAB_HOST
#include "arduino_shim.h"
#else
#include <Arduino.h>
#endif
#include "observer.h"
#include "pid.h"
#include "encoder.h"

using namespace maker;

// ---- 接腳 ----
constexpr int PIN_ENC_A = 32;
constexpr int PIN_ENC_B = 33;
constexpr int PIN_PWM   = 25;   // 馬達 driver PWM
constexpr int PWM_CH    = 0;

constexpr uint32_t CONTROL_US = 10000;   // 100 Hz
constexpr float DT = CONTROL_US / 1e6f;

// ---- observer 模型(與 Python sim 完全一致)----
// 連續模型 state = [position, velocity]:
//   A = [[0, 1], [0, -2]], B = [[0], [2]], C = [1, 0]
// 由 pole placement(poles = -20, -25)得 observer gain L = [43, 414]
//   char(A-LC) = s^2 + (l0+2)s + (2 l0 + l1) = (s+20)(s+25) = s^2 + 45 s + 500
//   ⇒ l0 = 43, l1 = 414(Python 端用 ct.place 驗證,firmware 直接帶入)
float OBS_A[2][2] = {{0, 1}, {0, -2}};
float OBS_B[2]    = {0, 2};
float OBS_C[2]    = {1, 0};
float OBS_L[2]    = {43.0f, 414.0f};

// ---- velocity PID(回授訊號用 observer velocity)----
constexpr float KP = 0.6f, KI = 6.0f, KD = 0.0f;
constexpr float U_MIN = -20.0f, U_MAX = 20.0f;   // 命令飽和(driver 限制)

QuadDecoder enc;
VelocityEstimator vel;      // finite-difference 速度(只做遙測對照)
Observer2 obs;              // Luenberger observer(提供控制用的速度回授)
Pid pid;                    // velocity 迴路控制器

volatile long g_pos = 0;
uint32_t nextTick;
float g_u = 0.0f;           // 上一步的控制命令(餵回 observer 的 u)
float g_vel_sp = 5.0f;      // velocity setpoint

// 把控制命令映射成 PWM duty(0..255);此處僅示意,實作依 driver 調整。
static int command_to_duty(float u) {
    float mag = u < 0 ? -u : u;
    int duty = (int)(mag / U_MAX * 255.0f + 0.5f);
    return constrain(duty, 0, 255);
}

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
    ledcSetup(PWM_CH, 20000, 8);
    ledcAttachPin(PIN_PWM, PWM_CH);
#endif
    obs.init(OBS_A, OBS_B, OBS_C, OBS_L, DT);
    pid.init(KP, KI, KD, DT, U_MIN, U_MAX);
    nextTick = micros();
}

void loop() {
    uint32_t now = micros();
    if ((int32_t)(now - nextTick) >= 0) {
        nextTick += CONTROL_US;

        // 1. 讀 encoder position(唯一 measurement)
        float pos = (float)g_pos;
        // 2. 對照組:finite-difference velocity(只做遙測,含放大的量化噪聲)
        float fd_vel = vel.step(g_pos, DT);
        // 3. observer:用「上一步命令 u」與 position 更新,估 velocity
        obs.step(g_u, pos);
        float obs_vel = obs.velocity();
        // 4. 控制器:velocity PID,回授用 observer velocity(平滑)
        float u = pid.step(g_vel_sp, obs_vel);
        g_u = u;                       // 存起來給下一步 observer 用
        // 5. 輸出 + 安全:命令已在 PID 內飽和,再映射成 PWM duty
        int duty = command_to_duty(u);
#ifndef MAKERLAB_HOST
        ledcWrite(PWM_CH, duty);
#else
        (void)duty;
#endif
        // 6. 遙測 CSV
        Serial.printf("%lu,%.1f,%.3f,%.3f,%.3f,%.3f\n",
                      (unsigned long)millis(), pos, g_vel_sp, fd_vel, obs_vel, u);
    }

    // 非即時的指令解析(改 setpoint 等)可另外處理
}

#ifdef MAKERLAB_HOST
int main() {
    setup();
    // 餵入假的漸增 encoder counts(近似等速運動),確認整條路徑可執行:
    //   encoder → observer → velocity PID → command,且不發散。
    for (int i = 0; i < 100; i++) {
        g_micros += CONTROL_US;
        g_pos += 5;               // 每步 +5 counts → 對照 fd_vel = 5/DT
        loop();
    }
    return 0;
}
#endif

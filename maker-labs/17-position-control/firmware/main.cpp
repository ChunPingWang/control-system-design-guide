// Ch17 Position-Control Loops — ESP32 位置閉迴路
//
// 本章 Maker Lab:用共用 pid.h + encoder.h 做「角度位置」閉迴路。
//   目標角度(以 encoder count 表示) → 位置 PID → PWM(方向 + duty) → 馬達。
//   位置量測直接用 encoder 累積 count(QuadDecoder.position),
//   不需再積分——編碼器本身就給出位置狀態。
//
// 和轉速迴路(Ch6)的差別:
//   - 受控體「位置 = 速度積分」內建一個積分器,所以純 P 對定位命令
//     即可零穩態誤差;I 項主要用來抵抗靜摩擦/負載造成的殘差。
//   - 導數項(對量測微分)提供阻尼,抑制接近目標時的 overshoot。
//
// Serial CSV 欄位:t,target_cnt,pos_cnt,error,pwm,p,i,d,saturated
//
// host 編譯驗證(不需實體板):
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

// ---- 常數 ----
constexpr float COUNTS_PER_REV = 1000.0f;                 // 編碼器每圈 count
constexpr float COUNTS_PER_DEG = COUNTS_PER_REV / 360.0f; // 每度 count
constexpr uint32_t CONTROL_US = 10000;                    // 100 Hz 控制迴路
constexpr float DT = CONTROL_US / 1e6f;

QuadDecoder enc;
Pid pid;
volatile long g_pos = 0;             // 目前位置(encoder count),由 ISR 更新
uint32_t nextTick;

// 目標角度 → 目標 count(此例定位到 +90 度)
float target_deg = 90.0f;
long  target_cnt = (long)(90.0f * COUNTS_PER_DEG);

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
    // 位置 PID:單位為 count。輸出 -255..255(方向 + duty)。
    // Kp 主導、小 Ki 抵抗殘差、Kd 提供阻尼避免定位 overshoot。
    pid.init(0.8f, 0.05f, 6.0f, DT, -255.0f, 255.0f);
    pid.dfilt_hz = 20.0f;        // 導數低通,抑制 count 量化噪聲
    pid.deriv_on_meas = true;    // 對量測微分,避免 setpoint 跳變的 derivative kick
    nextTick = micros();
}

void loop() {
    uint32_t now = micros();
    if ((int32_t)(now - nextTick) >= 0) {
        nextTick += CONTROL_US;

        // 1. 讀 encoder 位置(count)
        long pos = g_pos;
        // 2. 位置 PID(目標 count, 目前 count)
        float u = pid.step((float)target_cnt, (float)pos);
        // 3. 方向 + duty
        int dir = (u >= 0) ? 1 : 0;
        int duty = (int)(u >= 0 ? u : -u);
        if (duty > 255) duty = 255;
        digitalWrite(PIN_DIR, dir);
        ledcWrite(PWM_CH, duty);
        // 4. 遙測
        float e = (float)target_cnt - (float)pos;
        int sat = (u <= -255.0f || u >= 255.0f) ? 1 : 0;
        Serial.printf("%lu,%ld,%ld,%.1f,%d,%.2f,%.2f,%.2f,%d\n",
                      (unsigned long)millis(), target_cnt, pos, e, duty,
                      pid.kp * e, pid.ki * pid.integ, 0.0f, sat);
    }

    // 非即時的命令解析(改 target_deg / target_cnt)可放這裡。
}

#ifdef MAKERLAB_HOST
int main() {
    setup();
    // 主機端合成測試:用簡化「位置 = 速度積分」模型,
    // 讓 encoder count 依 PWM 命令朝目標收斂,確認控制路徑可跑、PID 不 crash。
    float vel = 0.0f;               // 目前速度(count/step)
    float pos_model = 0.0f;         // 連續位置(count)
    const float K_MOTOR = 0.10f;    // PWM → 加速度增益(簡化)
    const float DAMP = 0.85f;       // 速度阻尼(慣性/摩擦)
    for (int i = 0; i < 400; i++) {
        g_micros += CONTROL_US;
        loop();                     // 依目前 g_pos 算出 PWM,寫入模型

        // 依 P 主導的近似輸出推進簡化馬達模型(速度 → 位置積分)
        long e = target_cnt - g_pos;
        float cmd = (float)e * 0.8f;                 // 對應 P 主導的近似輸出
        if (cmd > 255.0f) cmd = 255.0f;
        if (cmd < -255.0f) cmd = -255.0f;
        vel = DAMP * vel + K_MOTOR * cmd;
        pos_model += vel;
        g_pos = (long)pos_model;                     // 回饋為新的 encoder 位置
    }
    return (labs(target_cnt - g_pos) < 5) ? 0 : 1;   // 收斂到目標視為通過
}
#endif

// Ch11 Introduction to Modeling — ESP32 DC Motor Open-Loop Step Test
//
// 本章 Maker Lab:open-loop step test。固定供電、施加不同的 PWM step,
// 用 encoder 量 RPM,由 step 響應估 DC gain(K)與 time constant(τ),
// 建立第一個可辨識的一階 motor model:G(s)=K/(τs+1)。
//   - K ≈ 穩態 RPM / duty(單位輸入的穩態輸出)
//   - τ ≈ RPM 上升到穩態 63.2% 所需時間
// Serial CSV 欄位:t_ms,duty,pos,rpm
// 分析時對每個 duty step 各取一段,離線用 python 擬合 K、τ。
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
constexpr uint32_t CONTROL_US = 10000;   // 100 Hz 量測窗
constexpr float DT = CONTROL_US / 1e6f;

// ---- open-loop step 排程:固定供電下,依序施加不同 PWM duty ----
// 每個 step 停留 STEP_HOLD_MS,確保 RPM 走到穩態才換下一個 duty。
constexpr uint32_t STEP_HOLD_MS = 1500;
constexpr int STEP_DUTIES[] = {0, 64, 128, 192, 255};
constexpr int N_STEPS = sizeof(STEP_DUTIES) / sizeof(STEP_DUTIES[0]);

QuadDecoder enc;
VelocityEstimator vel;
volatile long g_pos = 0;
uint32_t nextTick;

#ifndef MAKERLAB_HOST
// 實機:encoder A/B 邊緣觸發中斷,更新正交解碼位置。
void IRAM_ATTR onEncoder() {
    enc.update(digitalRead(PIN_ENC_A), digitalRead(PIN_ENC_B));
    g_pos = enc.position;
}
#endif

// 由當前時間決定要施加的 duty(open-loop step 排程)。
int scheduled_duty(uint32_t now_ms) {
    uint32_t idx = now_ms / STEP_HOLD_MS;
    if (idx >= (uint32_t)N_STEPS) idx = N_STEPS - 1;   // 停在最後一個 step
    return STEP_DUTIES[idx];
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
    digitalWrite(PIN_DIR, 1);          // 固定單一轉向(open-loop)
    vel.reset();
    nextTick = micros();
    Serial.println("t_ms,duty,pos,rpm");   // CSV 表頭
}

void loop() {
    uint32_t now = micros();
    if ((int32_t)(now - nextTick) >= 0) {
        nextTick += CONTROL_US;

        // 1. 依排程施加 open-loop PWM step(不做 feedback)
        int duty = scheduled_duty(millis());
        ledcWrite(PWM_CH, duty);
        // 2. 讀 encoder → RPM(量測窗 = DT)
        long pos = g_pos;
        float cps = vel.step(pos, DT);
        float rpm = VelocityEstimator::counts_per_sec_to_rpm(cps, COUNTS_PER_REV);
        // 3. 遙測 CSV:t_ms,duty,pos,rpm(離線擬合 K、τ 用)
        Serial.printf("%lu,%d,%ld,%.1f\n",
                      (unsigned long)millis(), duty, pos, rpm);
    }

    // 非即時的指令解析可在此另行處理(本章不需要)。
}

#ifdef MAKERLAB_HOST
// host 端:用一階馬達差分模型合成 encoder counts,確認整條量測/遙測路徑
// 可執行且 open-loop step 排程正確。模型:RPM 以 τ 趨向 K·duty。
int main() {
    setup();
    const float K_MODEL = 1.5f;         // 每單位 duty 的穩態 RPM(合成用)
    const float TAU_MODEL = 0.20f;      // 合成 time constant(s)
    float rpm_state = 0.0f;
    double pos_accum = 0.0;             // 以浮點累積,再取整成 encoder count
    // 模擬約 N_STEPS 段、每段 STEP_HOLD_MS,涵蓋整個 step 排程。
    const int total_ticks =
        (int)((uint32_t)N_STEPS * STEP_HOLD_MS * 1000u / CONTROL_US) + 10;
    for (int i = 0; i < total_ticks; i++) {
        g_micros += CONTROL_US;
        int duty = scheduled_duty(millis());
        float rpm_cmd = K_MODEL * (float)duty;
        // 一階離散更新:rpm += (rpm_cmd - rpm)·DT/τ
        rpm_state += (rpm_cmd - rpm_state) * (DT / TAU_MODEL);
        // RPM → counts/s → 本窗 counts,累積成位置
        float cps = rpm_state / 60.0f * COUNTS_PER_REV;
        pos_accum += (double)cps * DT;
        g_pos = (long)pos_accum;
        loop();
    }
    return 0;
}
#endif

// Ch13 Model Development and Verification — ESP32 System-ID 資料收集
//
// 本章 Maker Lab:對馬達施加 PWM step,記錄 (t, pwm, rpm) 成 CSV,
// 之後匯回 Python 用 curve_fit 估一階 K/τ 並驗證(見 sim.py)。
// 為了「訓練/驗證」分離,韌體會自動跑兩段 step:
//   RUN A(高 duty)→ 用來 fitting;RUN B(不同 duty)→ 用來 validation。
// Serial CSV 欄位:run,t_ms,pwm,rpm
//
// host 編譯驗證:
//   g++ -DMAKERLAB_HOST -I../../firmware -I../../firmware/lib -c main.cpp
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
constexpr int PWM_CH = 0, PWM_FREQ = 20000, PWM_RES = 8;

constexpr float COUNTS_PER_REV = 1000.0f;
constexpr uint32_t CONTROL_US = 10000;   // 100 Hz 取樣
constexpr float DT = CONTROL_US / 1e6f;

// ---- System-ID 實驗排程 ----
constexpr uint32_t STEP_MS   = 2000;     // 每段 step 持續 2 s
constexpr int      PWM_RUN_A = 200;      // Run A(train)PWM duty
constexpr int      PWM_RUN_B = 140;      // Run B(validation)PWM duty

QuadDecoder enc;
VelocityEstimator vel;
volatile long g_pos = 0;
uint32_t nextTick;
uint32_t run_start_ms = 0;   // 目前 step 的起始時間
int      run_id = 0;         // 0=Run A, 1=Run B, 2=結束
int      cur_pwm = 0;

#ifndef MAKERLAB_HOST
void IRAM_ATTR onEncoder() {
    enc.update(digitalRead(PIN_ENC_A), digitalRead(PIN_ENC_B));
    g_pos = enc.position;
}
#endif

// 啟動一段新的 step 實驗:設定 duty、清零速度估測與計時。
void begin_run(int id, int pwm) {
    run_id = id;
    cur_pwm = pwm;
    vel.reset();
    run_start_ms = millis();
    digitalWrite(PIN_DIR, 1);        // 固定正轉,單向 system-ID
    ledcWrite(PWM_CH, pwm);
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
    Serial.println("run,t_ms,pwm,rpm");   // CSV 表頭
    nextTick = micros();
    begin_run(0, PWM_RUN_A);               // 先跑 Run A
}

void loop() {
    uint32_t now = micros();
    if ((int32_t)(now - nextTick) >= 0) {
        nextTick += CONTROL_US;

        if (run_id >= 2) {                 // 兩段都跑完:停馬達
            ledcWrite(PWM_CH, 0);
            return;
        }

        // 1. 讀 encoder → RPM
        float cps = vel.step(g_pos, DT);
        float rpm = VelocityEstimator::counts_per_sec_to_rpm(cps, COUNTS_PER_REV);

        // 2. step 內的相對時間
        uint32_t t_ms = millis() - run_start_ms;

        // 3. 輸出 CSV(run,t_ms,pwm,rpm)供 Python 事後 fitting
        Serial.printf("%d,%lu,%d,%.1f\n", run_id, (unsigned long)t_ms, cur_pwm, rpm);

        // 4. 這段 step 結束 → 切到下一段(A→B→停)
        if (t_ms >= STEP_MS) {
            if (run_id == 0) begin_run(1, PWM_RUN_B);
            else             run_id = 2;
        }
    }

    // 非即時的指令解析可另外處理(此範例省略)。
}

#ifdef MAKERLAB_HOST
int main() {
    setup();
    // 餵入假的漸增 encoder 位置,確認資料收集路徑與 A→B→停 狀態機不 crash。
    for (int i = 0; i < 500; i++) { g_micros += CONTROL_US; g_pos += 20; loop(); }
    return 0;
}
#endif

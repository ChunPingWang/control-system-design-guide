// Ch7 Disturbance Response — ESP32 DC Motor Speed Loop under Load Step
//
// 本章 Maker Lab:馬達穩定轉速後施加可重複負載(例如小摩擦輪/固定機構),
// 量測 RPM dip(轉速下陷)、recovery time(回復時間)、max error(最大誤差)。
// 重點是分開 command response(setpoint step)與 disturbance response(load step)。
//
// 用共用 pid.h + encoder.h。Serial CSV 欄位:
//   t,setpoint,rpm,error,pwm,load,dip,recovered
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

// ---- 負載擾動狀態(可重複的 load step)----
// 硬體上由摩擦機構造成;此旗標僅用於遙測標記與 host 模擬。
volatile bool load_on = false;

// ---- disturbance 指標(執行中即時估算)----
float max_dip_rpm = 0.0f;      // 施加負載後的最大轉速下陷(相對 setpoint)
float max_error   = 0.0f;      // 施加負載後的最大絕對誤差
bool  recovered   = true;      // 是否已回復到 ±2% band 內

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
    // 輸出範圍 -255..255(方向 + duty),導數低通 20 Hz。
    // PI 為主(kd 小):擾動抑制靠積分器把穩態誤差拉回 0。
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
        // 4. disturbance 指標:負載開啟後追蹤 dip / max error / 回復狀態
        float e = setpoint_rpm - rpm;
        if (load_on) {
            float dip = setpoint_rpm - rpm;          // 正值代表轉速下陷
            if (dip > max_dip_rpm) max_dip_rpm = dip;
            if (fabsf(e) > max_error) max_error = fabsf(e);
            // 回復判定:誤差回到 setpoint 的 ±2% 內視為 recovered
            recovered = (fabsf(e) <= 0.02f * setpoint_rpm);
        }
        // 5. 遙測(load 與 recovered 便於離線切出 disturbance 響應段)
        Serial.printf("%lu,%.1f,%.1f,%.1f,%d,%d,%.1f,%d\n",
                      (unsigned long)millis(), setpoint_rpm, rpm, e, duty,
                      load_on ? 1 : 0, max_dip_rpm, recovered ? 1 : 0);
    }

    // non-real-time:實際硬體可在此解析 Serial 指令切換 load(如接繼電器/舵機夾具)
}

#ifdef MAKERLAB_HOST
// host 測試:先讓速度閉迴路穩定,再施加「可重複 load step」
// (以降低每步 encoder count 增量來模擬負載使轉速掉落),
// 觀察 PID 是否把轉速拉回、dip/max_error 是否被記錄、控制路徑不 crash。
int main() {
    setup();
    // 名目每步位置增量:對應 setpoint_rpm 的穩態(僅為 host 合成訊號)。
    // 300 rpm × 1000 cpr / 60 = 5000 cps × DT(0.01s) = 50 counts/step。
    const long nominal_inc = 50;
    for (int i = 0; i < 200; i++) {
        g_micros += CONTROL_US;
        long inc = nominal_inc;
        if (i >= 100 && i < 160) {
            // load step:負載讓實際轉速掉約 30%(count 增量下降)
            load_on = true;
            inc = (long)(nominal_inc * 0.7);
        } else if (i >= 160) {
            load_on = false;   // 移除負載
        }
        g_pos += inc;
        loop();
    }
    // host 端印出彙整(真實板子由離線 Python 分析 CSV 得到相同指標)
    Serial.printf("SUMMARY,max_dip_rpm=%.1f,max_error=%.1f,recovered=%d\n",
                  max_dip_rpm, max_error, recovered ? 1 : 0);
    return 0;
}
#endif

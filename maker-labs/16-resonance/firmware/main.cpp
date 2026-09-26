// Ch16 Compliance and Resonance — ESP32 柔性負載速度回授 + 抗共振低通
//
// 本章 Maker Lab:馬達透過彈性 coupling / 皮帶帶動負載,形成兩慣量系統。
// 提高 gain 時容易激發機械共振,速度回授訊號會出現高頻抖動(resonance)。
// 這裡示範最基本的緩解手法:對「速度回授」套一個一階低通 OnePoleLP(filter.h),
// 把 gain crossover 以上、落在共振頻段的高頻量測衰減掉,避免共振被回授放大。
//
// 注意:低通會犧牲相位/頻寬,對「特定」共振頻率而言,notch(帶阻)濾波器
//   才是更精準的解(只挖掉共振頻段、保留其餘頻寬)。共用 lib 目前只有
//   OnePoleLP,沒有 notch → 實作 notch 並比較留作練習(見 LAB_REPORT.md)。
//
// Serial CSV 欄位:t,setpoint,rpm_raw,rpm_filt,error,pwm,saturated
//
// host 編譯驗證(不需實機):
//   g++ -std=c++17 -DMAKERLAB_HOST -I../../firmware -I../../firmware/lib -c main.cpp
// 於 ESP32 上請用 PlatformIO / Arduino,移除 MAKERLAB_HOST。
#ifdef MAKERLAB_HOST
#include "arduino_shim.h"
#else
#include <Arduino.h>
#endif
#include "encoder.h"
#include "filter.h"

using namespace maker;

// ---- 接腳 ----
constexpr int PIN_ENC_A = 32;
constexpr int PIN_ENC_B = 33;
constexpr int PIN_PWM   = 25;
constexpr int PIN_DIR   = 26;
constexpr int PWM_CH = 0, PWM_FREQ = 20000, PWM_RES = 8;

constexpr float COUNTS_PER_REV = 1000.0f;
constexpr uint32_t CONTROL_US = 5000;         // 200 Hz(要能看到 ~18 Hz 共振)
constexpr float DT = CONTROL_US / 1e6f;

// 抗共振低通截止:設在共振頻率(~18 Hz,見 sim.py)之下,衰減共振頻段回授。
// fc 太高→擋不住共振;太低→相位落後、頻寬掉太多。此值需依實機掃頻調整。
constexpr float VEL_LP_HZ = 8.0f;

// 簡單 P 速度控制(本章重點在「共振與濾波」,不在整定 PID)。
constexpr float KP = 0.6f;
constexpr float PWM_MAX = 255.0f;

QuadDecoder enc;
VelocityEstimator vel;
OnePoleLP vel_lp;               // 速度回授低通(抗共振)
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
    vel_lp.init(VEL_LP_HZ, DT);   // 一階低通:alpha 由截止頻率與 DT 決定
    nextTick = micros();
}

void loop() {
    uint32_t now = micros();
    if ((int32_t)(now - nextTick) >= 0) {
        nextTick += CONTROL_US;

        // 1. 讀 encoder → 原始 RPM(含共振/量測雜訊)
        float cps = vel.step(g_pos, DT);
        float rpm_raw = VelocityEstimator::counts_per_sec_to_rpm(cps, COUNTS_PER_REV);
        // 2. 抗共振:對速度回授做一階低通(notch 更佳,留作練習)
        float rpm_filt = vel_lp.step(rpm_raw);
        // 3. 用「濾波後」的速度做閉迴路,避免共振頻段被回授放大
        float e = setpoint_rpm - rpm_filt;
        float u = KP * e;
        // 4. clamp + 方向
        int dir = (u >= 0) ? 1 : 0;
        float umag = (u >= 0) ? u : -u;
        if (umag > PWM_MAX) umag = PWM_MAX;
        int duty = (int)umag;
        int sat = (umag >= PWM_MAX) ? 1 : 0;
        digitalWrite(PIN_DIR, dir);
        ledcWrite(PWM_CH, duty);
        // 5. 遙測 CSV(同時輸出 raw 與 filt,方便分析共振被衰減多少)
        Serial.printf("%lu,%.1f,%.1f,%.1f,%.1f,%d,%d\n",
                      (unsigned long)millis(), setpoint_rpm,
                      rpm_raw, rpm_filt, e, duty, sat);
    }
}

#ifdef MAKERLAB_HOST
int main() {
    setup();
    // 餵入「漸增位置 + 高頻擾動」模擬共振抖動,確認低通/控制路徑可執行不 crash
    for (int i = 0; i < 50; i++) {
        g_micros += CONTROL_US;
        long ripple = (long)(3.0 * ((i % 2) ? 1 : -1));   // 模擬共振造成的位置抖動
        g_pos += 100 + ripple;
        loop();
    }
    return 0;
}
#endif

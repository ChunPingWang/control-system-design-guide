// Ch1 Introduction to Controls — ESP32 sketch(open-loop:pot → ADC → PWM → LED）
//
// 這是本章的 Maker Lab 韌體:電位器當 setpoint,ESP32 讀 ADC、輸出 PWM 到 LED/低壓馬達。
// 第一週只做 open-loop(命令直接驅動輸出),用來對比後續章節加入 feedback 的差異。
//
// host 編譯驗證(不需實機):
//   g++ -DMAKERLAB_HOST -I../../firmware -c main.cpp
// 於 ESP32 上請用 PlatformIO / Arduino,移除 MAKERLAB_HOST。
#ifdef MAKERLAB_HOST
#include "arduino_shim.h"
#else
#include <Arduino.h>
#endif

// ---- 接腳 ----
constexpr int PIN_POT = 34;    // ADC1_CH6
constexpr int PIN_PWM = 25;    // LEDC 輸出到 LED 或 motor driver
constexpr int PWM_CH = 0;
constexpr int PWM_FREQ = 20000;
constexpr int PWM_RES = 8;     // 8-bit:0..255

constexpr uint32_t CONTROL_US = 10000;  // 100 Hz
uint32_t nextTick;

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);           // ESP32 ADC 12-bit:0..4095
    ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_PWM, PWM_CH);
    nextTick = micros();
}

void loop() {
    uint32_t now = micros();
    if ((int32_t)(now - nextTick) >= 0) {
        nextTick += CONTROL_US;

        // 1. 讀 setpoint(電位器)
        int raw = analogRead(PIN_POT);          // 0..4095
        // 2. open-loop:直接把命令映射成 PWM(無 feedback)
        int duty = raw * 255 / 4095;            // → 0..255
        if (duty < 0) duty = 0;
        if (duty > 255) duty = 255;
        // 3. 輸出
        ledcWrite(PWM_CH, duty);
        // 4. 遙測 CSV:t,setpoint_raw,duty
        Serial.printf("%lu,%d,%d\n", (unsigned long)millis(), raw, duty);
    }
}

#ifdef MAKERLAB_HOST
int main() { setup(); for (int i = 0; i < 5; i++) { g_micros += CONTROL_US; loop(); } return 0; }
#endif

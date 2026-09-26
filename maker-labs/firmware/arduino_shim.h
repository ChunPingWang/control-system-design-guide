// arduino_shim.h — host 端 Arduino/ESP32 API mock
//
// 目的:讓每章的 ESP32 sketch(main.cpp)能在沒有實體板子的情況下,
//       用桌機 g++ 編譯通過(verify_firmware.sh 用 -fsyntax-only / -c)。
//       這不是模擬器,只驗證「編得過、API 用法正確、控制邏輯接得起來」。
//
// 只有在定義 MAKERLAB_HOST 時才會被 sketch include(見各 main.cpp 開頭)。
#pragma once
#ifdef MAKERLAB_HOST

#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstdarg>

// ---- 常數 ----
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 3
#define RISING 1
#define FALLING 2
#define LED_BUILTIN 2

typedef uint8_t byte;

// ---- 時間 ----
inline uint32_t g_micros = 0;
inline uint32_t micros() { return g_micros; }
inline uint32_t millis() { return g_micros / 1000; }
inline void delay(uint32_t) {}
inline void delayMicroseconds(uint32_t) {}

// ---- GPIO / PWM / ADC ----
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int  digitalRead(int) { return 0; }
inline int  analogRead(int) { return 0; }
inline void analogWrite(int, int) {}
inline void analogReadResolution(int) {}

// ESP32 LEDC PWM
inline void ledcSetup(int, double, int) {}
inline void ledcAttachPin(int, int) {}
inline void ledcWrite(int, int) {}
inline double ledcSetupR(int, double, int) { return 0; }

// ---- 中斷 ----
inline int  digitalPinToInterrupt(int p) { return p; }
inline void attachInterrupt(int, void (*)(), int) {}
inline void detachInterrupt(int) {}
#define IRAM_ATTR

// ---- Serial ----
struct SerialShim {
    void begin(long) {}
    void print(const char*) {}
    void print(float) {}
    void print(int) {}
    void print(long) {}
    void println(const char* = "") {}
    void println(float) {}
    void println(int) {}
    void println(long) {}
    int  printf(const char* fmt, ...) {
        va_list ap; va_start(ap, fmt);
        int r = 0; (void)fmt; va_end(ap); return r;
    }
    int  available() { return 0; }
    int  read() { return -1; }
    char peek() { return -1; }
    void flush() {}
    operator bool() const { return true; }
};
inline SerialShim Serial;

// ---- min/max/constrain 巨集(Arduino 慣例)----
#ifndef constrain
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#endif

#endif  // MAKERLAB_HOST

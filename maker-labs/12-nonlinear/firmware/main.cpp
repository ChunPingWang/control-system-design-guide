// Ch12 Nonlinear Behavior and Time Variation — ESP32 Motor Nonlinearity Lab
//
// 本章 Maker Lab:示範用共用 util.h 的非線性補償把「非 LTI 致動器」拉回
// 接近線性的行為:
//   1) deadband compensation:PWM 有起轉死區(低 duty 馬達不轉),
//      用 util.h 的 deadband 反向平移補回,讓小指令也能有效驅動。
//   2) saturate / clampf:輸出安全夾位(保留 safety clamp,不因補償而超限)。
//   3) RateLimiter:限制指令變化率,抑制 backlash 撞擊與電流突波。
// 另讀 encoder.h 取得 RPM 遙測(觀察 stiction / deadband 造成的低速死區)。
//
// Serial CSV 欄位:t,setpoint,rpm,u_raw,u_comp,u_rl,duty,dir,saturated
//
// host 編譯驗證:
//   g++ -DMAKERLAB_HOST -I../../firmware -I../../firmware/lib -c main.cpp
#ifdef MAKERLAB_HOST
#include "arduino_shim.h"
#else
#include <Arduino.h>
#endif
#include "util.h"
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

// ---- 非線性參數(以實測校準:量起轉 PWM、正反轉 deadband、最大 RPM)----
constexpr float PWM_MAX      = 255.0f;   // 8-bit duty 上限(safety clamp)
constexpr float PWM_DEADBAND = 30.0f;    // 起轉死區:duty < 30 馬達幾乎不轉
constexpr float MAX_RATE     = 20.0f;    // 每步(10ms)duty 變化上限 → 抑制突波

QuadDecoder enc;
VelocityEstimator vel;
RateLimiter cmd_rl;                       // util.h:指令變化率限制器
volatile long g_pos = 0;
uint32_t nextTick;
float setpoint_rpm = 200.0f;

// 簡單比例控制器(本章重點在非線性,不在調 PID)
constexpr float KP = 0.5f;

#ifndef MAKERLAB_HOST
void IRAM_ATTR onEncoder() {
    enc.update(digitalRead(PIN_ENC_A), digitalRead(PIN_ENC_B));
    g_pos = enc.position;
}
#endif

// 死區補償:把控制器輸出「加回」死區量,使有效驅動接近線性。
// 這是 util.h::deadband 的反運算:非零指令平移 +PWM_DEADBAND(依方向)。
static float deadband_compensate(float u) {
    if (u > 0.5f)  return u + PWM_DEADBAND;
    if (u < -0.5f) return u - PWM_DEADBAND;
    return 0.0f;                          // 指令太小 → 不驅動(避免抖動)
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
    cmd_rl.reset();
    nextTick = micros();
}

void loop() {
    uint32_t now = micros();
    if ((int32_t)(now - nextTick) >= 0) {
        nextTick += CONTROL_US;

        // 1. 讀 encoder → RPM
        float cps = vel.step(g_pos, DT);
        float rpm = VelocityEstimator::counts_per_sec_to_rpm(cps, COUNTS_PER_REV);

        // 2. 比例控制器輸出(raw command)
        float e = setpoint_rpm - rpm;
        float u_raw = KP * e;

        // 3. 死區補償(把 PWM 起轉死區加回)
        float u_comp = deadband_compensate(u_raw);

        // 4. 變化率限制(util.h::RateLimiter):抑制 backlash 撞擊與電流突波
        float u_rl = cmd_rl.step(u_comp, MAX_RATE);

        // 5. 輸出安全夾位(util.h::saturate):補償後仍不得超過 PWM 上限
        float u_out = saturate(u_rl, PWM_MAX);

        // 6. 方向 + duty
        int dir = (u_out >= 0) ? 1 : 0;
        float mag = clampf(u_out >= 0 ? u_out : -u_out, 0.0f, PWM_MAX);
        int duty = (int)mag;
        digitalWrite(PIN_DIR, dir);
        ledcWrite(PWM_CH, duty);

        // 7. 遙測
        int sat = (mag >= PWM_MAX) ? 1 : 0;
        Serial.printf("%lu,%.1f,%.1f,%.2f,%.2f,%.2f,%d,%d,%d\n",
                      (unsigned long)millis(), setpoint_rpm, rpm,
                      u_raw, u_comp, u_rl, duty, dir, sat);
    }
}

#ifdef MAKERLAB_HOST
int main() {
    setup();
    // 餵入假的漸增 encoder 位置,確認非線性補償路徑可執行、不 crash。
    for (int i = 0; i < 50; i++) { g_micros += CONTROL_US; g_pos += 20; loop(); }
    // 直接驗證 util.h 非線性 helper 的邊界行為(host 端 smoke test)。
    float s = saturate(1000.0f, PWM_MAX);
    float d = deadband(0.05f, 0.15f);
    if (s != PWM_MAX || d != 0.0f) return 1;
    return 0;
}
#endif

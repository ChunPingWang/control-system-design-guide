// Ch3 Tuning a Control System — ESP32 DC Motor Speed Loop 調參實驗
//
// 本章 Maker Lab:用共用 pid.h + encoder.h 做馬達轉速閉迴路,
// 逐步提高 Kp(P-only)並在每個 setpoint step 記錄 overshoot / settling /
// 是否 oscillate。PWM 有 clamp,飽和狀態一併回傳,避免 saturation 掩蓋
// 真實 loop dynamics(對應講義「調參時要避免 actuator saturation」)。
//
// Serial CSV 欄位:t,kp,setpoint,rpm,error,pwm,saturated
//
// host 編譯驗證:
//   g++ -std=c++17 -DMAKERLAB_HOST -I../../firmware -I../../firmware/lib -c main.cpp
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
constexpr uint32_t CONTROL_US = 10000;      // 100 Hz 控制迴圈
constexpr float DT = CONTROL_US / 1e6f;

constexpr float PWM_MAX = 255.0f;           // 8-bit PWM 上限(actuator 飽和邊界)
constexpr float SETPOINT_RPM = 300.0f;      // 目標轉速

// 調參序列:每 KP_STEP_MS 換一個 Kp,重跑同一個 setpoint step,
// 觀察 overshoot / oscillation 如何隨 loop gain 上升而惡化。
constexpr float KP_SEQUENCE[] = {0.2f, 0.6f, 1.5f, 3.0f};
constexpr int   KP_COUNT = sizeof(KP_SEQUENCE) / sizeof(KP_SEQUENCE[0]);
constexpr uint32_t KP_STEP_MS = 2000;       // 每個 Kp 停留時間

QuadDecoder enc;
VelocityEstimator vel;
Pid pid;
volatile long g_pos = 0;
uint32_t nextTick;
int kp_idx = 0;
uint32_t kp_started_ms = 0;

#ifndef MAKERLAB_HOST
// 正交編碼器中斷:只在真實硬體上掛;host 端由 main() 直接餵 g_pos。
void IRAM_ATTR onEncoder() {
    enc.update(digitalRead(PIN_ENC_A), digitalRead(PIN_ENC_B));
    g_pos = enc.position;
}
#endif

// 套用目前的 Kp(P-only:Ki=Kd=0,單純觀察 loop gain 的影響)。
void apply_kp(float kp) {
    pid.init(kp, 0.0f, 0.0f, DT, -PWM_MAX, PWM_MAX);
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
    kp_idx = 0;
    apply_kp(KP_SEQUENCE[kp_idx]);
    kp_started_ms = millis();
    nextTick = micros();
    Serial.println("t,kp,setpoint,rpm,error,pwm,saturated");
}

void loop() {
    uint32_t now = micros();
    if ((int32_t)(now - nextTick) >= 0) {
        nextTick += CONTROL_US;

        // 0. 到時間就切下一個 Kp(重跑同一 step,做 loop-gain sweep)
        uint32_t t_ms = millis();
        if (kp_idx < KP_COUNT - 1 && (t_ms - kp_started_ms) >= KP_STEP_MS) {
            kp_idx++;
            apply_kp(KP_SEQUENCE[kp_idx]);
            kp_started_ms = t_ms;
        }

        // 1. 讀 encoder → RPM
        float cps = vel.step(g_pos, DT);
        float rpm = VelocityEstimator::counts_per_sec_to_rpm(cps, COUNTS_PER_REV);

        // 2. P 控制器(setpoint, measurement)→ 控制量 u(已在 pid 內 clamp)
        float u = pid.step(SETPOINT_RPM, rpm);

        // 3. 方向 + duty,並二次 clamp(硬體安全:actuator saturation 邊界)
        int dir = (u >= 0) ? 1 : 0;
        float mag = (u >= 0) ? u : -u;
        int duty = (int)mag;
        if (duty > (int)PWM_MAX) duty = (int)PWM_MAX;
        digitalWrite(PIN_DIR, dir);
        ledcWrite(PWM_CH, duty);

        // 4. 遙測:把 saturation 狀態送回 PC(判斷 overshoot 是真動態還是被削頂)
        float e = SETPOINT_RPM - rpm;
        int sat = (u <= -PWM_MAX || u >= PWM_MAX) ? 1 : 0;
        Serial.printf("%lu,%.2f,%.1f,%.1f,%.1f,%d,%d\n",
                      (unsigned long)t_ms, pid.kp, SETPOINT_RPM, rpm, e, duty, sat);
    }

    // 非即時的指令解析可放在這裡(本章不需要)
}

#ifdef MAKERLAB_HOST
// host 測試:餵入假的漸增 encoder 位置,確認控制路徑、Kp 切換與飽和判斷
// 都能執行、不會 crash。這不是動態模擬,只驗證程式邏輯接得起來。
int main() {
    setup();
    long step_counts = 40;   // 每個控制週期假裝轉了 40 counts
    for (int i = 0; i < 1000; i++) {
        g_micros += CONTROL_US;
        g_pos += step_counts;
        loop();
    }
    return 0;
}
#endif

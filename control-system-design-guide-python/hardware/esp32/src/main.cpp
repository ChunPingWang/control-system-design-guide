// ESP32 速度伺服 RCP 韌體
// 硬體:ESP32 + TB6612FNG + 直流減速馬達(AB 相編碼器)
//
// 迴路:100 Hz 速度 PID(rpm),PWM 輸出 ±255
// 遙測:115200 baud,每個迴路週期一行 CSV:millis,target,actual,pwm
// 命令(以換行結尾):
//   SET,KP,2.0   SET,KI,1.0   SET,KD,0.05   SET,TARGET,500
//   STEP,100     → 切開迴路,直接輸出固定 PWM(系統辨識用,見 Ch19)
//   CLOSE        → 回到閉迴路
#include <Arduino.h>
#include "pid.h"

// ---- 腳位(依實際接線調整)----
static const int PIN_ENC_A = 32;
static const int PIN_ENC_B = 33;
static const int PIN_PWM   = 25;   // TB6612 PWMA
static const int PIN_AIN1  = 26;
static const int PIN_AIN2  = 27;
static const int PIN_STBY  = 14;

// ---- 編碼器 ----
static const float COUNTS_PER_REV = 4.0f * 11.0f * 21.3f; // 4x 解碼 × 線數 × 減速比,依馬達修改
volatile long enc_count = 0;

void IRAM_ATTR isr_enc_a() {
    bool a = digitalRead(PIN_ENC_A), b = digitalRead(PIN_ENC_B);
    enc_count += (a == b) ? 1 : -1;
}
void IRAM_ATTR isr_enc_b() {
    bool a = digitalRead(PIN_ENC_A), b = digitalRead(PIN_ENC_B);
    enc_count += (a != b) ? 1 : -1;
}

// ---- 控制狀態 ----
static const float LOOP_DT = 0.01f;          // 100 Hz
pid_t_ pid;
float target_rpm = 0.0f;
bool open_loop = false;
float open_loop_pwm = 0.0f;
long prev_count = 0;
uint32_t prev_us = 0;

float readEncoderRPM(float dt) {
    long c;
    noInterrupts();
    c = enc_count;
    interrupts();
    long d = c - prev_count;
    prev_count = c;
    return (d / COUNTS_PER_REV) / dt * 60.0f;
}

void setMotorPWM(float u) {
    int duty = (int)constrain(u, -255.0f, 255.0f);
    if (duty >= 0) {
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, LOW);
    } else {
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, HIGH);
        duty = -duty;
    }
    ledcWrite(0, duty);
}

void handleCommand(String line) {
    line.trim();
    if (line.startsWith("SET,")) {
        int c2 = line.indexOf(',', 4);
        if (c2 < 0) return;
        String key = line.substring(4, c2);
        float val = line.substring(c2 + 1).toFloat();
        if      (key == "KP") pid.kp = val;
        else if (key == "KI") pid.ki = val;
        else if (key == "KD") pid.kd = val;
        else if (key == "TARGET") { target_rpm = val; open_loop = false; }
        Serial.printf("# %s=%.4f\n", key.c_str(), val);
    } else if (line.startsWith("STEP,")) {
        open_loop_pwm = line.substring(5).toFloat();
        open_loop = true;
        pid_init(&pid, pid.kp, pid.ki, pid.kd, LOOP_DT, -255, 255);  // 清積分
        Serial.printf("# open-loop PWM=%.1f\n", open_loop_pwm);
    } else if (line == "CLOSE") {
        open_loop = false;
        Serial.println("# closed loop");
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, HIGH);
    ledcSetup(0, 20000, 8);                  // 20 kHz PWM, 8-bit
    ledcAttachPin(PIN_PWM, 0);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isr_enc_a, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), isr_enc_b, CHANGE);
    pid_init(&pid, 2.0f, 1.0f, 0.05f, LOOP_DT, -255.0f, 255.0f);
    prev_us = micros();
}

void loop() {
    while (Serial.available()) {
        static String buf;
        char ch = (char)Serial.read();
        if (ch == '\n') { handleCommand(buf); buf = ""; }
        else buf += ch;
    }

    uint32_t now = micros();
    if (now - prev_us < (uint32_t)(LOOP_DT * 1e6f)) return;
    float dt = (now - prev_us) / 1e6f;
    prev_us = now;

    float actual = readEncoderRPM(dt);
    float u;
    if (open_loop) {
        u = open_loop_pwm;
    } else {
        u = pid_step(&pid, target_rpm - actual);
    }
    setMotorPWM(u);
    Serial.printf("%lu,%.3f,%.3f,%.3f\n", (unsigned long)millis(), target_rpm, actual, u);
}

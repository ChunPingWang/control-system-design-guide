// Ch19 Rapid Control Prototyping (RCP) — ESP32 Motion System firmware
//
// 本章 Maker Lab(里程碑 Project 5「Mini RCP Platform」的板端):
//   ESP32 = deterministic 固定週期 PID 轉速迴路(硬即時,不受 PC 影響)
//   PC     = 透過 serial 下命令設定 Kp/Ki/Kd/setpoint、start/stop、擷取 CSV
//
// 分層原則(對齊講義):
//   - 控制計算只在固定週期 tick 內做(CONTROL_US = 10 ms → 100 Hz)。
//   - serial 命令解析是「非即時」工作,放在 loop() 的 tick 之外處理,
//     不阻塞控制節拍(bumpless:改參數不重置積分器)。
//   - 安全預設:開機為 STOP(輸出 0);另有 command timeout 看門狗與輸出飽和。
//
// Serial 命令(每行一條,以 '\n' 結尾):
//   KP <v>   設定比例增益      KI <v>  設定積分增益      KD <v>  設定微分增益
//   SP <v>   設定 setpoint(rpm)
//   RUN      進入 RUN(開始輸出)  STOP  進入 STOP(安全停機,輸出 0)
//   例:  "KP 0.4\n"  "SP 300\n"  "RUN\n"
//
// CSV 遙測欄位:t_ms,state,setpoint,rpm,error,pwm,kp,ki,kd,saturated
//
// host 編譯驗證(不需實體板子):
//   g++ -std=c++17 -DMAKERLAB_HOST -I../../firmware -I../../firmware/lib -c main.cpp
#ifdef MAKERLAB_HOST
#include "arduino_shim.h"
#include <cstdlib>   // atof(host);Arduino 端由 Arduino.h 提供
#endif
#ifndef MAKERLAB_HOST
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

// ---- 迴路常數 ----
constexpr float    COUNTS_PER_REV = 1000.0f;
constexpr uint32_t CONTROL_US = 10000;         // 100 Hz 固定週期
constexpr float    DT = CONTROL_US / 1e6f;
constexpr float    PWM_MAX = 255.0f;
constexpr uint32_t CMD_TIMEOUT_MS = 2000;      // 命令看門狗:逾時未收命令即安全停機

// ---- 執行狀態機:安全預設為 STOP ----
enum RunState { ST_STOP = 0, ST_RUN = 1 };
RunState g_state = ST_STOP;

QuadDecoder enc;
VelocityEstimator vel;
Pid pid;
volatile long g_pos = 0;
uint32_t nextTick;
uint32_t lastCmdMs = 0;
float setpoint_rpm = 0.0f;   // 安全預設:靜止

// ---- serial 逐字元行緩衝(非即時)----
char cmdBuf[48];
uint8_t cmdLen = 0;

#ifndef MAKERLAB_HOST
void IRAM_ATTR onEncoder() {
    enc.update(digitalRead(PIN_ENC_A), digitalRead(PIN_ENC_B));
    g_pos = enc.position;
}
#endif

// 把 duty/方向實際寫到馬達驅動(STOP 時強制 0,確保安全)。
void driveMotor(float u) {
    if (g_state != ST_RUN) {
        ledcWrite(PWM_CH, 0);
        return;
    }
    int dir = (u >= 0) ? 1 : 0;
    float mag = (u >= 0) ? u : -u;
    int duty = (int)mag;
    if (duty > (int)PWM_MAX) duty = (int)PWM_MAX;
    digitalWrite(PIN_DIR, dir);
    ledcWrite(PWM_CH, duty);
}

// 解析一條完整命令(以 '\0' 結尾的 cmdBuf)。
// 這是非即時工作:只改參數/狀態,不做控制計算,也不重置積分器(bumpless)。
void handleCommand(const char* s) {
    // 跳過前導空白
    while (*s == ' ') s++;
    if (s[0] == '\0') return;

    // 兩字母指令 + 選用數值
    if (s[0] == 'R' && s[1] == 'U' && s[2] == 'N') {          // RUN
        g_state = ST_RUN;
    } else if (s[0] == 'S' && s[1] == 'T' && s[2] == 'O') {   // STOP
        g_state = ST_STOP;
        pid.reset();                                          // 停機清積分器
    } else if (s[0] == 'K' && s[1] == 'P') {                  // KP <v>
        pid.kp = (float)atof(s + 2);
    } else if (s[0] == 'K' && s[1] == 'I') {                  // KI <v>
        pid.ki = (float)atof(s + 2);
    } else if (s[0] == 'K' && s[1] == 'D') {                  // KD <v>
        pid.kd = (float)atof(s + 2);
    } else if (s[0] == 'S' && s[1] == 'P') {                  // SP <v>
        setpoint_rpm = (float)atof(s + 2);
    }
    // 收到任何合法命令就餵狗
    lastCmdMs = millis();
}

// 非即時:把 serial 進來的位元組組成整行,遇 '\n' 才解析。
void pollSerial() {
    while (Serial.available() > 0) {
        int c = Serial.read();
        if (c < 0) break;
        if (c == '\n' || c == '\r') {
            if (cmdLen > 0) {
                cmdBuf[cmdLen] = '\0';
                handleCommand(cmdBuf);
                cmdLen = 0;
            }
        } else if (cmdLen < sizeof(cmdBuf) - 1) {
            cmdBuf[cmdLen++] = (char)c;
        } else {
            cmdLen = 0;  // 過長 → 丟棄整行,避免溢位
        }
    }
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
    // 輸出範圍 -255..255(方向 + duty),導數低通 20 Hz
    pid.init(0.4f, 6.0f, 0.01f, DT, -PWM_MAX, PWM_MAX);
    pid.dfilt_hz = 20.0f;
    g_state = ST_STOP;          // 安全預設
    setpoint_rpm = 0.0f;
    lastCmdMs = millis();
    nextTick = micros();
}

void loop() {
    // (A) 非即時:命令解析(tick 之外)
    pollSerial();

    // (B) 固定週期控制 tick(硬即時)
    uint32_t now = micros();
    if ((int32_t)(now - nextTick) >= 0) {
        nextTick += CONTROL_US;

        // 命令看門狗:太久沒收到 PC 命令 → 安全停機
        if ((uint32_t)(millis() - lastCmdMs) > CMD_TIMEOUT_MS) {
            g_state = ST_STOP;
        }

        // 1. 讀 encoder → RPM
        float cps = vel.step(g_pos, DT);
        float rpm = VelocityEstimator::counts_per_sec_to_rpm(cps, COUNTS_PER_REV);

        // 2. 控制計算(STOP 時把 setpoint 視為 0,積分不亂跑)
        float sp = (g_state == ST_RUN) ? setpoint_rpm : 0.0f;
        float u = pid.step(sp, rpm);

        // 3. 輸出(STOP → driveMotor 強制 0)+ 飽和旗標
        driveMotor(u);
        int sat = (u <= -PWM_MAX || u >= PWM_MAX) ? 1 : 0;

        // 4. CSV 遙測
        float e = sp - rpm;
        int duty = (g_state == ST_RUN) ? (int)(u >= 0 ? u : -u) : 0;
        if (duty > (int)PWM_MAX) duty = (int)PWM_MAX;
        Serial.printf("%lu,%d,%.1f,%.1f,%.1f,%d,%.4f,%.4f,%.4f,%d\n",
                      (unsigned long)millis(), (int)g_state, sp, rpm, e,
                      duty, pid.kp, pid.ki, pid.kd, sat);
    }
}

#ifdef MAKERLAB_HOST
#include <cassert>
int main() {
    setup();
    assert(g_state == ST_STOP);            // 安全預設:開機必為 STOP

    // 模擬 PC 下命令:設 gains / setpoint / RUN(bumpless,不重置積分)
    handleCommand("KP 0.5");
    handleCommand("KI 4.0");
    handleCommand("KD 0.02");
    handleCommand("SP 300");
    handleCommand("RUN");
    assert(g_state == ST_RUN);
    assert(setpoint_rpm > 299.0f && setpoint_rpm < 301.0f);

    // 餵入假的漸增 encoder 位置,跑控制路徑,確認不 crash
    for (int i = 0; i < 50; i++) {
        g_micros += CONTROL_US;
        g_pos += 100;
        loop();
    }

    // STOP 命令 → 立刻安全停機
    handleCommand("STOP");
    assert(g_state == ST_STOP);

    // 命令看門狗:讓時間跳超過 timeout,下一拍應自動回到 STOP
    handleCommand("RUN");
    assert(g_state == ST_RUN);
    g_micros += (CMD_TIMEOUT_MS + 100) * 1000;   // 前進超過 timeout
    g_pos += 100;
    loop();
    assert(g_state == ST_STOP);

    return 0;
}
#endif

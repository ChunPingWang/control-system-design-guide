// Ch5 The z-Domain — ESP32 Encoder Quantization / Measurement Window Lab
//
// 本章 Maker Lab:用 encoder pulse count 示範「低速量測 quantization」。
// 低速時,一個量測窗內只累積到少數幾個 count,量測解析度受量化限制;
// 把量測窗 (window) 拉長 → 解析度變好(counts 變多)、但 latency 變差。
// 這就是 sampling / quantization 的核心 trade-off,直接對應 z-domain 章。
//
// 量測解析度(counts/sec)≈ 1 count / window_sec,例如:
//   window=10ms  → 每 count 值 100.0 counts/sec(粗糙,但延遲低)
//   window=100ms → 每 count 值  10.0 counts/sec(細緻,但延遲高)
//
// Serial CSV 欄位:t_ms,window_ms,raw_counts,cps,rpm,resolution_cps,latency_ms
//
// host 編譯驗證(不需實體板):
//   g++ -std=c++17 -DMAKERLAB_HOST -I../../firmware -I../../firmware/lib -c main.cpp
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

// ---- 量測參數 ----
constexpr float COUNTS_PER_REV = 1000.0f;   // 編碼器 4x 解碼後每轉 count 數

// 掃描的量測窗(毫秒):示範 resolution vs latency trade-off。
// 窗越短延遲越低但解析度越粗;窗越長解析度越細但延遲越高。
constexpr uint32_t WINDOWS_MS[] = {10, 20, 50, 100};
constexpr int N_WINDOWS = sizeof(WINDOWS_MS) / sizeof(WINDOWS_MS[0]);

QuadDecoder enc;
volatile long g_pos = 0;      // ISR 累積的 encoder 位置(count)

// 目前量測窗索引與窗起點
int win_idx = 0;
uint32_t window_start_us = 0;
long window_start_pos = 0;

#ifndef MAKERLAB_HOST
// 實機:A/B 兩相任一邊緣觸發,更新正交解碼器
void IRAM_ATTR onEncoder() {
    enc.update(digitalRead(PIN_ENC_A), digitalRead(PIN_ENC_B));
    g_pos = enc.position;
}
#endif

void setup() {
    Serial.begin(115200);
    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);
#ifndef MAKERLAB_HOST
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), onEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), onEncoder, CHANGE);
#endif
    window_start_us = micros();
    window_start_pos = g_pos;
    // CSV 表頭(方便直接匯入分析)
    Serial.println("t_ms,window_ms,raw_counts,cps,rpm,resolution_cps,latency_ms");
}

void loop() {
    uint32_t now = micros();
    uint32_t window_us = WINDOWS_MS[win_idx] * 1000UL;

    // 一個量測窗結束:計算這段時間內累積的 count → 速度估測
    if ((int32_t)(now - (window_start_us + window_us)) >= 0) {
        long raw_counts = g_pos - window_start_pos;   // 本窗累積 count(可能為 0)
        float dt = window_us / 1e6f;                   // 窗長(秒)
        float cps = (float)raw_counts / dt;            // counts/sec(量化後速度)
        float rpm = VelocityEstimator::counts_per_sec_to_rpm(cps, COUNTS_PER_REV);

        // 量化解析度:1 count 對應多少 counts/sec(越小越細緻)
        float resolution_cps = 1.0f / dt;
        // latency:量測至少落後半個窗(平均),此處以整窗長度表示上界
        float latency_ms = (float)WINDOWS_MS[win_idx];

        Serial.printf("%lu,%lu,%ld,%.2f,%.2f,%.2f,%.1f\n",
                      (unsigned long)millis(),
                      (unsigned long)WINDOWS_MS[win_idx],
                      raw_counts, cps, rpm, resolution_cps, latency_ms);

        // 進入下一個量測窗;掃完所有窗後回到第一個(方便反覆比較)
        win_idx = (win_idx + 1) % N_WINDOWS;
        window_start_us = now;
        window_start_pos = g_pos;
    }

    // 非即時的指令解析可放這裡(本 lab 未使用)
}

#ifdef MAKERLAB_HOST
// ---- host 測試 harness ----
// 餵入固定「低速」的合成 encoder 訊號,確認量測窗邏輯可執行、
// 並驗證「窗越長 → 累積 count 越多、解析度越細」的 trade-off 方向。
int main() {
    setup();
    // 模擬一個固定低速:每毫秒約 0.3 count(≈ 300 counts/sec,低速)。
    // 用整數累積避免浮點漂移:每 10 步(10ms)加 3 count。
    const long counts_per_10ms = 3;
    long acc10 = 0;
    for (int step = 0; step < 400; step++) {
        g_micros += 1000;               // 前進 1 ms
        acc10++;
        if (acc10 == 10) {              // 每 10 ms 注入 3 count(低速)
            g_pos += counts_per_10ms;
            acc10 = 0;
        }
        loop();
    }
    return 0;
}
#endif

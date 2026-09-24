// pid.h — 韌體 PID(純 C++,無 Arduino 相依,可在 host 端編譯測試)
//
// 與 Python 版 common/sim.py 的 DiscretePID 的已知差異(見 Ch19 notebook):
//   1. 積分器「先更新、再算輸出」(差一拍積分量 ki*e*Ts)
//   2. 導數不濾波(dfilt 由上層決定要不要加)
// 除此之外行為一致,test_host/test_pid.cpp 逐樣本驗證。
#pragma once

typedef struct {
    float kp, ki, kd;
    float dt;
    float out_min, out_max;
    float integ;
    float prev_e;
    int first;
} pid_t_;

static inline void pid_init(pid_t_ *p, float kp, float ki, float kd, float dt,
                            float out_min, float out_max) {
    p->kp = kp; p->ki = ki; p->kd = kd; p->dt = dt;
    p->out_min = out_min; p->out_max = out_max;
    p->integ = 0.0f; p->prev_e = 0.0f; p->first = 1;
}

static inline float pid_step(pid_t_ *p, float e) {
    p->integ += e * p->dt;
    float d = 0.0f;
    if (!p->first) d = (e - p->prev_e) / p->dt;
    p->first = 0;
    p->prev_e = e;
    float u = p->kp * e + p->ki * p->integ + p->kd * d;
    if (u > p->out_max) u = p->out_max;
    if (u < p->out_min) u = p->out_min;
    return u;
}

// test_lib.cpp — Maker Labs 共用韌體邏輯的 host 單元測試
// 編譯:g++ -std=c++17 -I../lib test_lib.cpp -o test_lib && ./test_lib
// 全部通過回傳 0,任一失敗回傳非零(供 verify_firmware.sh 判定)。
#include "pid.h"
#include "filter.h"
#include "encoder.h"
#include "feedforward.h"
#include "observer.h"
#include "util.h"
#include "fusion.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

static int g_fail = 0;
#define CHECK(cond, msg)                                                    \
    do {                                                                    \
        if (!(cond)) { std::printf("  FAIL: %s\n", msg); g_fail++; }       \
        else         { std::printf("  ok:   %s\n", msg); }                 \
    } while (0)

static bool close(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

// 一階受控體:G(s)=K/(tau s+1),前向 Euler 離散模擬
struct FirstOrder {
    float K, tau, y = 0;
    float step(float u, float dt) { y += dt * (K * u - y) / tau; return y; }
};

int main() {
    using namespace maker;

    std::printf("[PID]\n");
    {
        // 純 P:u = kp*e,飽和內
        Pid p; p.init(2.0f, 0, 0, 0.01f, -100, 100);
        float u = p.step(/*r=*/10, /*meas=*/0);
        CHECK(close(u, 20.0f, 1e-4f), "P term: kp*e");

        // 閉迴路穩態:PI 應消除 steady-state error
        Pid pi; pi.init(1.0f, 5.0f, 0, 0.001f, -1e6f, 1e6f);
        FirstOrder plant{1.0f, 0.2f, 0};
        float y = 0;
        for (int k = 0; k < 20000; k++) {
            float uu = pi.step(1.0f, y);
            y = plant.step(uu, 0.001f);
        }
        CHECK(close(y, 1.0f, 1e-2f), "PI 消除 steady-state error → y≈1");

        // anti-windup:輸出飽和時積分不應無限累積
        Pid ps; ps.init(0.5f, 10.0f, 0, 0.01f, -1.0f, 1.0f);
        FirstOrder slow{0.1f, 1.0f, 0};  // DC gain 小 → 容易飽和
        float ys = 0, umax = 0;
        for (int k = 0; k < 500; k++) {
            float uu = ps.step(5.0f, ys);
            if (std::fabs(uu) > umax) umax = std::fabs(uu);
            ys = slow.step(uu, 0.01f);
        }
        CHECK(umax <= 1.0f + 1e-5f, "輸出保持在飽和界內");
        // 目標無法到達,但積分不爆走:切回可達目標應快速穩定
        for (int k = 0; k < 3000; k++) { float uu = ps.step(0.05f, ys); ys = slow.step(uu, 0.01f); }
        CHECK(close(ys, 0.05f, 5e-3f), "anti-windup:切回可達目標仍能穩定");

        // 導數對量測:setpoint 階躍不應造成 derivative kick
        Pid pd; pd.init(0, 0, 1.0f, 0.01f, -1e6f, 1e6f);
        pd.step(0, 0);                 // 建立 prev
        float uk = pd.step(100.0f, 0); // r 跳變、meas 不變
        CHECK(close(uk, 0.0f, 1e-4f), "deriv-on-measurement:無 setpoint kick");
    }

    std::printf("[Filter]\n");
    {
        // 低通:DC 增益 = 1(常數輸入 → 輸出收斂到同值)
        OnePoleLP lp; lp.init(5.0f, 0.001f);
        float y = 0; for (int k = 0; k < 20000; k++) y = lp.step(3.0f);
        CHECK(close(y, 3.0f, 1e-3f), "OnePoleLP DC 增益=1");

        // 低通應衰減高頻:比較高頻正弦輸入/輸出振幅
        OnePoleLP lp2; lp2.init(2.0f, 0.001f);
        float amax = 0;
        for (int k = 0; k < 20000; k++) {
            float x = std::sin(2 * 3.14159265f * 50.0f * k * 0.001f);  // 50 Hz
            float yy = lp2.step(x);
            if (k > 10000 && std::fabs(yy) > amax) amax = std::fabs(yy);
        }
        CHECK(amax < 0.2f, "OnePoleLP 衰減 50Hz(截止 2Hz)");

        // 移動平均:常數 → 同值;DC 增益 1
        MovingAverage<8> ma;
        float m = 0; for (int k = 0; k < 100; k++) m = ma.step(4.0f);
        CHECK(close(m, 4.0f, 1e-4f), "MovingAverage DC 增益=1");
    }

    std::printf("[Encoder]\n");
    {
        // 正交解碼:A/B 走一個完整正向序列應累加
        QuadDecoder q;
        // 正向序列 (A,B): 00→10→11→01→00 (每步 +1,共 4 count)
        int seq[5][2] = {{0,0},{1,0},{1,1},{0,1},{0,0}};
        for (int i = 0; i < 5; i++) q.update(seq[i][0], seq[i][1]);
        CHECK(q.position == 4 || q.position == -4, "正交解碼:一圈序列累積 ±4 count");
        long fwd = q.position;

        // 反向序列應反號
        QuadDecoder q2;
        int rseq[5][2] = {{0,0},{0,1},{1,1},{1,0},{0,0}};
        for (int i = 0; i < 5; i++) q2.update(rseq[i][0], rseq[i][1]);
        CHECK((q2.position > 0) != (fwd > 0), "反向序列方向相反");

        // 速度估測:每窗 +100 count、窗長 0.01s → 10000 counts/sec
        VelocityEstimator ve;
        ve.step(0, 0.01f);
        float cps = ve.step(100, 0.01f);
        CHECK(close(cps, 10000.0f, 1e-3f), "速度估測 counts/sec");
        float rpm = VelocityEstimator::counts_per_sec_to_rpm(cps, 1000.0f);
        CHECK(close(rpm, 600.0f, 1e-2f), "counts/sec → RPM(1000 CPR)");
    }

    std::printf("[Feedforward]\n");
    {
        LinearFeedforward ff;
        ff.calibrate(/*t1*/100, /*u1*/40, /*t2*/300, /*u2*/120);  // 斜率 0.4、截距 0
        CHECK(close(ff.gain, 0.4f, 1e-5f), "線性擬合斜率");
        CHECK(close(ff.compute(200.0f), 80.0f, 1e-4f), "前饋輸出 = gain*target+offset");
    }

    std::printf("[Observer]\n");
    {
        // state=[pos,vel];真實系統 xdot=[v; -2v+2u],量測 pos。
        float A[2][2] = {{0, 1}, {0, -2}};
        float B[2] = {0, 2};
        float C[2] = {1, 0};
        // 觀測器極點 -6,-7 → L=place(...)。此處直接給合理增益驗證收斂性。
        float L[2] = {11.0f, 30.0f};
        Observer2 obs; obs.init(A, B, C, L, 0.001f);

        // 真實 plant 模擬
        float x0 = 0, x1 = 0;  // pos,vel
        obs.xhat[0] = 0.5f; obs.xhat[1] = 0.0f;  // 故意給錯初值
        float u = 1.0f, dt = 0.001f;
        for (int k = 0; k < 5000; k++) {
            // 真實系統
            float dx0 = x1;
            float dx1 = -2 * x1 + 2 * u;
            x0 += dt * dx0; x1 += dt * dx1;
            // 觀測器只拿到 position
            obs.step(u, x0);
        }
        CHECK(close(obs.position(), x0, 1e-2f), "觀測器 position 收斂到真值");
        CHECK(close(obs.velocity(), x1, 5e-2f), "觀測器 velocity 收斂到真值");
    }

    std::printf("[Util]\n");
    {
        CHECK(close(clampf(5, -1, 1), 1.0f, 0), "clamp 上界");
        CHECK(close(clampf(-5, -1, 1), -1.0f, 0), "clamp 下界");
        CHECK(close(saturate(3, 2), 2.0f, 0), "saturate");
        CHECK(close(deadband(0.1f, 0.15f), 0.0f, 0), "deadband 內 → 0");
        CHECK(close(deadband(0.5f, 0.15f), 0.35f, 1e-6f), "deadband 外 → 平移連續");
        RateLimiter rl;
        rl.step(0, 1.0f);
        float r = rl.step(10.0f, 1.0f);  // 一步最多 +1
        CHECK(close(r, 1.0f, 1e-6f), "rate limit 每步上限");
    }

    std::printf("[Fusion]\n");
    {
        // 純加速度傾角
        CHECK(close(accel_tilt(0, 1), 0.0f, 1e-4f), "accel_tilt 0°");
        CHECK(close(accel_tilt(1, 0), 3.14159265f / 2, 1e-4f), "accel_tilt 90°");
        // 互補濾波:acc 恆為 0.2rad、gyro=0 → 應收斂到 0.2
        ComplementaryFilter cf; cf.init(0.98f);
        float ang = 0;
        for (int k = 0; k < 5000; k++) ang = cf.step(0.2f, 0.0f, 0.01f);
        CHECK(close(ang, 0.2f, 1e-2f), "互補濾波收斂到加速度傾角");
    }

    std::printf("\n%s (fail=%d)\n", g_fail ? "TEST FAILED" : "ALL TESTS PASSED", g_fail);
    return g_fail ? 1 : 0;
}

// observer.h — Maker Labs 二階 Luenberger 觀測器(純 C++,host 可測)
// 章節用途:Ch10 observer intro、Ch18 observer in motion control。
//
// 連續模型 xdot = A x + B u,量測 y = C x;離散化以前向 Euler(dt 小時足夠)。
// 估測更新:xhat += dt*(A xhat + B u) + L*(y - C xhat)
// 針對 motion:state = [position, velocity],量測 position。
#pragma once

namespace maker {

// 2 狀態、單輸入、單輸出觀測器。
struct Observer2 {
    // 連續系統矩陣(2x2 A、2x1 B、1x2 C)
    float A[2][2] = {{0, 1}, {0, 0}};
    float B[2] = {0, 0};
    float C[2] = {1, 0};
    float L[2] = {0, 0};   // 觀測器增益
    float dt = 0.01f;

    float xhat[2] = {0, 0};

    void init(float A_[2][2], float B_[2], float C_[2], float L_[2], float dt_) {
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) A[i][j] = A_[i][j];
            B[i] = B_[i]; C[i] = C_[i]; L[i] = L_[i];
        }
        dt = dt_;
        xhat[0] = xhat[1] = 0;
    }

    void set_gain(float l0, float l1) { L[0] = l0; L[1] = l1; }
    void reset() { xhat[0] = xhat[1] = 0; }

    // u = 控制輸入,y = 量測輸出;回傳更新後的估測 state 指標
    void step(float u, float y) {
        float yhat = C[0] * xhat[0] + C[1] * xhat[1];
        float err = y - yhat;
        float ax0 = A[0][0] * xhat[0] + A[0][1] * xhat[1] + B[0] * u + L[0] * err;
        float ax1 = A[1][0] * xhat[0] + A[1][1] * xhat[1] + B[1] * u + L[1] * err;
        xhat[0] += dt * ax0;
        xhat[1] += dt * ax1;
    }

    float position() const { return xhat[0]; }
    float velocity() const { return xhat[1]; }
};

}  // namespace maker

// Host 端 PID 等價性測試(不需要任何 Arduino 環境)
//
//   c++ -std=c++11 -O2 -o test_pid test_pid.cpp && ./test_pid > pid_c_output.csv
//
// 之後執行 check_pid.py,把 C 輸出與 Python 的 CStylePID 逐樣本比對。
#include <cstdio>
#include "../src/pid.h"

int main() {
    pid_t_ p;
    pid_init(&p, 2.0f, 1.0f, 0.05f, 0.01f, -255.0f, 255.0f);
    // 決定性的偽隨機誤差序列(LCG),Python 端產生同一序列
    unsigned long long s = 12345;
    printf("k,e,u\n");
    for (int k = 0; k < 1000; ++k) {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        float e = (float)((double)(s >> 33) / 2147483648.0 - 0.5) * 200.0f; // ±100
        float u = pid_step(&p, e);
        printf("%d,%.9g,%.9g\n", k, e, u);
    }
    return 0;
}

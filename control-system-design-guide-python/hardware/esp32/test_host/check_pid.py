#!/usr/bin/env python3
"""比對韌體 pid.h(host 編譯)與 Python 重現實作的輸出。

用法:
    c++ -std=c++11 -O2 -o test_pid test_pid.cpp && ./test_pid > pid_c_output.csv
    python3 check_pid.py pid_c_output.csv
"""
import csv
import sys


class CStylePID:
    """與 pid.h 逐行對應(float 運算在 Python 以 double 進行,誤差以容差吸收)。"""

    def __init__(self, kp, ki, kd, dt, out_min, out_max):
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self.out_min, self.out_max = out_min, out_max
        self.integ, self.prev_e, self.first = 0.0, 0.0, True

    def step(self, e):
        self.integ += e * self.dt
        d = 0.0 if self.first else (e - self.prev_e) / self.dt
        self.first = False
        self.prev_e = e
        u = self.kp * e + self.ki * self.integ + self.kd * d
        return min(max(u, self.out_min), self.out_max)


def main(path):
    pid = CStylePID(2.0, 1.0, 0.05, 0.01, -255.0, 255.0)
    worst = 0.0
    n = 0
    with open(path) as f:
        for row in csv.DictReader(f):
            u_py = pid.step(float(row["e"]))
            worst = max(worst, abs(u_py - float(row["u"])))
            n += 1
    tol = 1e-3  # float32 vs float64 的累積差
    print(f"樣本 {n},最大差異 {worst:.6f}(容差 {tol})")
    assert n == 1000 and worst < tol, "C/Python PID 不等價!"
    print("PASS:韌體 PID 與 Python 實作逐樣本等價 ✅")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "pid_c_output.csv")

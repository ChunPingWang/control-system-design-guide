#!/usr/bin/env python
"""Ch19 Rapid Control Prototyping (RCP) for a Motion System — Python simulation lab.

對應 maker-labs 講義 Chapter 19。本章是里程碑 Project 5「Mini RCP Platform」的前導:
    PC(Python)= supervisory / 實驗自動化 / logging / 分析
    ESP32      = deterministic 固定週期 PID motor speed loop

本 sim.py 做兩件事:
  1. 用「和 ESP32 韌體同一份離散 PID」(逐拍複刻 firmware/lib/pid.h 的語意:
     先積分、對量測微分、輸出飽和、條件積分 anti-windup、導數低通)驅動一個
     一階馬達轉速 plant,示範閉迴路能收斂到 setpoint。
  2. 示範「Experiment Automation」概念:掃描一個參數(Kp),每一組跑一次閉迴路、
     計算指標(rise time / overshoot / settling / RMSE / steady-state),彙整成
     一張 summary table,並驗證掃描出來的趨勢符合物理預期(單調)。

可從任意目錄執行:python maker-labs/19-rapid-control-prototyping/sim.py
章末以 assert 自我驗證(供 run_all.sh 判定綠/紅)。
"""
import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent))  # → maker-labs/

import numpy as np
import matplotlib.pyplot as plt
from common.makerlab import step_metrics, show_or_save


# ---------------------------------------------------------------------------
# 1. 與 ESP32 韌體同語意的離散 PID(逐拍複刻 firmware/lib/pid.h::Pid::step)
#    這是 RCP 的核心精神:PC 模擬用的控制器 = 板子上跑的控制器,
#    才能讓「模擬 → 硬體」的參數直接搬過去而不是重寫一份。
# ---------------------------------------------------------------------------
class Pid:
    def __init__(self, kp, ki, kd, dt, out_min, out_max,
                 dfilt_hz=0.0, deriv_on_meas=True):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.out_min, self.out_max = out_min, out_max
        self.dfilt_hz = dfilt_hz
        self.deriv_on_meas = deriv_on_meas
        self.reset()

    def reset(self):
        self.integ = 0.0
        self.prev_meas = 0.0
        self.prev_e = 0.0
        self.d_filt = 0.0
        self.first = True

    def step(self, r, meas):
        e = r - meas
        # 積分(先更新)——與 pid.h 一致
        self.integ += e * self.dt

        # 導數項(對量測微分,避免 setpoint 跳變的 derivative kick)
        d_raw = 0.0
        if not self.first:
            if self.deriv_on_meas:
                d_raw = -(meas - self.prev_meas) / self.dt
            else:
                d_raw = (e - self.prev_e) / self.dt
        self.first = False
        self.prev_meas = meas
        self.prev_e = e

        # 導數一階低通
        if self.dfilt_hz > 0:
            alpha = self.dt / (self.dt + 1.0 / (2.0 * np.pi * self.dfilt_hz))
            self.d_filt += alpha * (d_raw - self.d_filt)
            d_term = self.d_filt
        else:
            d_term = d_raw

        u_unsat = self.kp * e + self.ki * self.integ + self.kd * d_term

        # 輸出飽和
        u = min(max(u_unsat, self.out_min), self.out_max)

        # 條件積分 anti-windup:飽和且積分讓輸出更飽和時,回退這一拍積分
        if u != u_unsat and (e * u_unsat) > 0:
            self.integ -= e * self.dt
        return u


# ---------------------------------------------------------------------------
# 2. 一階馬達轉速 plant(離散化):tau * rpm' = -rpm + Kmotor * pwm
#    PWM 範圍 -PWM_MAX..PWM_MAX(對應 ESP32 8-bit duty + 方向)。
# ---------------------------------------------------------------------------
DT = 0.01                # 100 Hz 控制週期(與韌體 CONTROL_US=10000 一致)
PWM_MAX = 255.0
TAU_MOTOR = 0.12         # 馬達機械時間常數 (s)
K_MOTOR = 600.0 / PWM_MAX  # 滿 duty ≈ 600 rpm
SETPOINT = 300.0         # 目標轉速 (rpm)


def simulate_loop(kp, ki, kd, setpoint=SETPOINT, t_end=2.0,
                  dfilt_hz=20.0, disturbance=None):
    """跑一次閉迴路,回傳 (t, rpm, pwm)。disturbance(t)->rpm 為外加擾動(選用)。"""
    pid = Pid(kp, ki, kd, DT, -PWM_MAX, PWM_MAX, dfilt_hz=dfilt_hz)
    n = int(round(t_end / DT))
    t = np.arange(n) * DT
    rpm = np.zeros(n)
    pwm = np.zeros(n)
    x = 0.0  # plant 狀態:目前轉速
    a = np.exp(-DT / TAU_MOTOR)          # 一階離散化
    b = K_MOTOR * (1.0 - a)
    for k in range(n):
        meas = x
        if disturbance is not None:
            meas += disturbance(t[k])
        u = pid.step(setpoint, meas)
        x = a * x + b * u
        rpm[k] = meas
        pwm[k] = u
    return t, rpm, pwm


# ---------------------------------------------------------------------------
# 3. 主閉迴路示範(PI(D) 收斂到 setpoint)——對應板子預設 gains。
# ---------------------------------------------------------------------------
print("=== Ch19 Rapid Control Prototyping (RCP) ===")
KP0, KI0, KD0 = 0.4, 6.0, 0.01
t, rpm, pwm = simulate_loop(KP0, KI0, KD0)
m0 = step_metrics(t, rpm, setpoint=SETPOINT)
final_rpm = rpm[-1]
print(f"[主迴路] Kp={KP0} Ki={KI0} Kd={KD0} → 終值 {final_rpm:.1f} rpm "
      f"(目標 {SETPOINT:.0f})")
print(f"[主迴路] step metrics: rise={m0['rise_time']:.3f}s "
      f"OS={m0['overshoot_pct']:.1f}% settle={m0['settling_time']:.3f}s "
      f"sse={m0['steady_state_error']:.2f} rpm")


# ---------------------------------------------------------------------------
# 4. Experiment Automation:掃描 Kp,自動跑實驗、算指標、彙整 summary table。
#    這正是 Week 22 的自動實驗流程:set gains → run → collect → metrics → report。
# ---------------------------------------------------------------------------
KP_SWEEP = [0.05, 0.10, 0.20, 0.40, 0.80]
print("\n[實驗自動化] 掃描 Kp(固定 Ki={}, Kd={}):".format(KI0, KD0))
header = f"{'Kp':>6} | {'rise(s)':>8} | {'OS(%)':>7} | {'settle(s)':>9} | {'RMSE':>7} | {'final rpm':>9}"
print(header)
print("-" * len(header))

rows = []
for kp in KP_SWEEP:
    tt, rr, _ = simulate_loop(kp, KI0, KD0)
    mm = step_metrics(tt, rr, setpoint=SETPOINT)
    err = SETPOINT - rr
    rmse = float(np.sqrt(np.mean(err ** 2)))
    rows.append({
        "kp": kp,
        "rise": mm["rise_time"],
        "os": mm["overshoot_pct"],
        "settle": mm["settling_time"],
        "rmse": rmse,
        "final": rr[-1],
    })
    print(f"{kp:>6.2f} | {mm['rise_time']:>8.3f} | {mm['overshoot_pct']:>7.1f} | "
          f"{mm['settling_time']:>9.3f} | {rmse:>7.1f} | {rr[-1]:>9.1f}")

rise_times = [r["rise"] for r in rows]
overshoots = [r["os"] for r in rows]
rmses = [r["rmse"] for r in rows]


# ---------------------------------------------------------------------------
# 5. 圖:主迴路響應 + 掃描的 rise time 趨勢。
# ---------------------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))

ax1.plot(t, rpm, label="rpm (measurement)")
ax1.axhline(SETPOINT, ls=":", c="gray", label="setpoint")
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Speed (rpm)")
ax1.set_title(f"Ch19 RCP closed loop (Kp={KP0}, Ki={KI0})")
ax1.grid(True)
ax1.legend()

ax2.plot(KP_SWEEP, overshoots, "o-", label="overshoot (%)")
ax2.plot(KP_SWEEP, rmses, "s--", label="RMSE (rpm)")
ax2.set_xlabel("Kp")
ax2.set_ylabel("metric")
ax2.set_title("Experiment automation: Kp sweep")
ax2.grid(True)
ax2.legend()

show_or_save(plt, "ch19_rcp.png")


# ---------------------------------------------------------------------------
# 6. ✅ 驗證
# ---------------------------------------------------------------------------
# (a) 閉迴路(含積分)應收斂到 setpoint。
assert abs(final_rpm - SETPOINT) < 0.02 * SETPOINT, \
    f"閉迴路應收斂到 setpoint,實際終值 {final_rpm:.1f} rpm"
assert abs(m0["steady_state_error"]) < 0.02 * SETPOINT, \
    "積分作用應消除穩態誤差"

# (b) 實驗自動化掃描:此積分主導迴路中,提高 Kp(相對削弱積分的比重)
#     會降低 overshoot → overshoot 單調不增。
for i in range(1, len(overshoots)):
    assert overshoots[i] <= overshoots[i - 1] + 1e-6, \
        f"overshoot 應隨 Kp 增大而單調不增: {overshoots}"
assert overshoots[-1] < overshoots[0], \
    f"最大 Kp 的 overshoot 應明顯小於最小 Kp: {overshoots}"

# (c) 追蹤誤差(RMSE)也應隨 Kp 增大而單調不增(收斂更快 → 累積誤差更小)。
for i in range(1, len(rmses)):
    assert rmses[i] <= rmses[i - 1] + 1e-6, \
        f"RMSE 應隨 Kp 增大而單調不增: {rmses}"
assert rmses[-1] < rmses[0], \
    f"最大 Kp 的 RMSE 應明顯小於最小 Kp: {rmses}"

# (d) PID 與韌體一致性 sanity:飽和輸出不得超出 ±PWM_MAX。
_, _, pwm_chk = simulate_loop(0.8, KI0, KD0)
assert np.all(np.abs(pwm_chk) <= PWM_MAX + 1e-6), "PWM 輸出必須被飽和在 ±255"

print("\nCh19 驗證通過 ✅")

"""Continuous-domain helpers built on python-control.

Analytic counterparts of the Visual ModelQ blocks: plants, controllers,
loop-shaping metrics and step-response metrics.
"""
import control as ct
import numpy as np

__all__ = [
    "first_order", "second_order", "integrator", "inertia",
    "pid", "pi", "closed_loop", "margins", "step_metrics",
    "bandwidth_hz", "metrics",
]


def first_order(tau=1.0, gain=1.0):
    """gain / (tau*s + 1)"""
    return ct.tf([gain], [tau, 1])


def second_order(wn=10.0, zeta=0.7, gain=1.0):
    """gain * wn^2 / (s^2 + 2*zeta*wn*s + wn^2)"""
    return ct.tf([gain * wn**2], [1, 2 * zeta * wn, wn**2])


def integrator(gain=1.0):
    """gain / s"""
    return ct.tf([gain], [1, 0])


def inertia(J=1.0, b=0.0):
    """Torque -> speed of a rigid inertia: 1 / (J*s + b)."""
    return ct.tf([1], [J, b])


def pi(kp=1.0, ki=0.0):
    """PI controller kp + ki/s."""
    s = ct.tf('s')
    return kp + ki / s


def pid(kp=1.0, ki=0.0, kd=0.0, n=100.0):
    """PID with first-order derivative filter: kp + ki/s + kd*n*s/(s+n)."""
    s = ct.tf('s')
    out = kp + kd * (n * s) / (s + n)
    if ki:
        out = out + ki / s
    return out


def closed_loop(c, p):
    """Unity-feedback closed loop T = C*P / (1 + C*P)."""
    return ct.feedback(c * p, 1)


def margins(loop):
    """Gain margin [dB], phase margin [deg] and their crossover freqs [Hz].

    Returns dict(gm_db, pm_deg, f_gm_hz, f_pm_hz). inf gain margin -> gm_db=inf.
    """
    gm, pm, _, wpc, wgc, _ = ct.stability_margins(loop)
    gm_db = np.inf if np.isinf(gm) else 20 * np.log10(gm)
    return {
        "gm_db": float(gm_db),
        "pm_deg": float(pm),
        "f_gm_hz": float(wpc / (2 * np.pi)) if np.isfinite(wpc) else np.nan,
        "f_pm_hz": float(wgc / (2 * np.pi)) if np.isfinite(wgc) else np.nan,
    }


def bandwidth_hz(tsys):
    """-3 dB closed-loop bandwidth in Hz."""
    return float(ct.bandwidth(tsys) / (2 * np.pi))


def step_metrics(t, y, target=1.0, settle_band=0.02):
    """Rise time (10-90%), overshoot %, 2% settling time, steady-state error."""
    t = np.asarray(t)
    y = np.asarray(y).squeeze()
    yf = target
    # rise time 10% -> 90%
    try:
        t10 = t[np.nonzero(y >= 0.1 * yf)[0][0]]
        t90 = t[np.nonzero(y >= 0.9 * yf)[0][0]]
        rise = t90 - t10
    except IndexError:
        rise = np.nan
    overshoot = max(0.0, (y.max() - yf) / abs(yf) * 100)
    outside = np.abs(y - yf) > settle_band * abs(yf)
    settle = t[np.nonzero(outside)[0][-1] + 1] if outside.any() and outside[-1] == False else (np.nan if outside.any() else t[0])
    return {
        "rise_time": float(rise),
        "overshoot_pct": float(overshoot),
        "settling_time": float(settle) if settle == settle else np.nan,
        "steady_state_error": float(yf - y[-1]),
    }


# Backwards-compatible alias used by early notebooks.
def metrics(t, y, target=1.0):
    return step_metrics(t, y, target)

"""Fixed-step, sample-by-sample simulation blocks (the ModelQ-lite core).

Visual ModelQ executes every block once per sample tick.  The classes here
do the same, which makes it easy to model what LTI tools cannot: saturation,
backlash, Coulomb friction, quantized encoders, calculation delay and
multi-rate loops.  All blocks expose ``step(...)`` and ``reset()``.
"""
import numpy as np

__all__ = [
    "DiscretePID", "MotorPlant", "TwoMassPlant", "DCMotorPlant",
    "Backlash", "EncoderModel", "Delay",
    "saturate", "quantize", "coulomb",
]


def saturate(u, lo, hi):
    return min(max(u, lo), hi)


def quantize(x, q):
    """Quantize x to a grid of size q (encoder counts, DAC steps...)."""
    return np.floor(x / q) * q


def coulomb(v, fc, eps=1e-9):
    """Coulomb friction force opposing velocity v (0 at rest)."""
    return fc * np.sign(v) if abs(v) > eps else 0.0


class DiscretePID:
    """Textbook discrete PID with derivative low-pass and anti-windup clamp.

    u = kp*e + ki*integ(e) + kd*d(e_f)/dt,  derivative filtered by a
    first-order low-pass with bandwidth ``dfilt_hz``.  The integrator is
    clamped when the output saturates (conditional integration).
    """

    def __init__(self, kp, ki=0.0, kd=0.0, dt=0.001, out_min=-np.inf,
                 out_max=np.inf, dfilt_hz=None, anti_windup=True):
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self.out_min, self.out_max = out_min, out_max
        self.anti_windup = anti_windup
        if dfilt_hz is None:
            self.alpha = 1.0                      # unfiltered derivative
        else:
            a = 2 * np.pi * dfilt_hz * dt
            self.alpha = a / (1.0 + a)            # backward-Euler LPF pole
        self.reset()

    def reset(self):
        self.integ = 0.0
        self.prev_ef = 0.0
        self.ef = 0.0
        self._first = True

    def step(self, e):
        self.ef += self.alpha * (e - self.ef)
        if self._first:
            self.prev_ef = self.ef
            self._first = False
        deriv = (self.ef - self.prev_ef) / self.dt
        self.prev_ef = self.ef
        u_unsat = self.kp * e + self.ki * self.integ + self.kd * deriv
        u = saturate(u_unsat, self.out_min, self.out_max)
        # conditional integration: freeze integrator when pushing deeper into saturation
        if not (self.anti_windup and u != u_unsat and e * u_unsat > 0):
            self.integ += e * self.dt
        return u


class MotorPlant:
    """Rigid inertia: torque -> speed -> position.  J*dw/dt = T - b*w - T_dist."""

    def __init__(self, J=1.0, b=0.0, dt=0.001):
        self.J, self.b, self.dt = J, b, dt
        self.reset()

    def reset(self):
        self.w = 0.0
        self.pos = 0.0

    def step(self, torque, t_dist=0.0):
        dw = (torque - self.b * self.w - t_dist) / self.J
        self.w += dw * self.dt
        self.pos += self.w * self.dt
        return self.w


class DCMotorPlant:
    """Brushed DC motor with electrical dynamics.

    L*di/dt = V - R*i - Ke*w ;  J*dw/dt = Kt*i - b*w - T_load
    """

    def __init__(self, R=1.0, L=1e-3, Kt=0.1, Ke=0.1, J=1e-4, b=1e-5, dt=1e-5):
        self.R, self.L, self.Kt, self.Ke = R, L, Kt, Ke
        self.J, self.b, self.dt = J, b, dt
        self.reset()

    def reset(self):
        self.i = 0.0
        self.w = 0.0
        self.pos = 0.0

    def step(self, v, t_load=0.0):
        di = (v - self.R * self.i - self.Ke * self.w) / self.L
        self.i += di * self.dt
        dw = (self.Kt * self.i - self.b * self.w - t_load) / self.J
        self.w += dw * self.dt
        self.pos += self.w * self.dt
        return self.i, self.w


class TwoMassPlant:
    """Motor inertia coupled to load inertia through a compliant shaft.

    Jm*dwm/dt = T - ks*(thm-thl) - cs*(wm-wl)
    Jl*dwl/dt =      ks*(thm-thl) + cs*(wm-wl)
    The classic resonant plant of Chapter 16.
    """

    def __init__(self, Jm=1e-3, Jl=1e-3, ks=100.0, cs=0.01, dt=1e-4):
        self.Jm, self.Jl, self.ks, self.cs, self.dt = Jm, Jl, ks, cs, dt
        self.reset()

    def reset(self):
        self.wm = self.wl = self.thm = self.thl = 0.0

    @property
    def resonance_hz(self):
        """Resonant frequency seen by the motor (locked-load formula)."""
        return float(np.sqrt(self.ks * (self.Jm + self.Jl) /
                             (self.Jm * self.Jl)) / (2 * np.pi))

    @property
    def antiresonance_hz(self):
        return float(np.sqrt(self.ks / self.Jl) / (2 * np.pi))

    def step(self, torque):
        tw = self.ks * (self.thm - self.thl) + self.cs * (self.wm - self.wl)
        self.wm += (torque - tw) / self.Jm * self.dt
        self.wl += tw / self.Jl * self.dt
        self.thm += self.wm * self.dt
        self.thl += self.wl * self.dt
        return self.wm, self.wl


class Backlash:
    """Backlash (lost motion) of total width ``width`` between input and output."""

    def __init__(self, width):
        self.half = width / 2.0
        self.out = 0.0

    def reset(self):
        self.out = 0.0

    def step(self, x):
        if x - self.out > self.half:
            self.out = x - self.half
        elif x - self.out < -self.half:
            self.out = x + self.half
        return self.out


class EncoderModel:
    """Incremental encoder: quantizes position to counts.

    lines * 4 counts/rev (quadrature).  Positions in revolutions.
    """

    def __init__(self, lines=1000):
        self.counts_per_rev = lines * 4

    def read(self, pos_rev):
        return np.floor(np.asarray(pos_rev) * self.counts_per_rev) / self.counts_per_rev


class Delay:
    """Pure transport delay of n samples (models calculation/communication delay)."""

    def __init__(self, n, initial=0.0):
        self.buf = [initial] * max(1, int(n))
        self.n = int(n)

    def reset(self, initial=0.0):
        self.buf = [initial] * max(1, self.n)

    def step(self, x):
        if self.n == 0:
            return x
        self.buf.append(x)
        return self.buf.pop(0)

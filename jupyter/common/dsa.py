"""Dynamic Signal Analyzer (DSA) replacement.

Visual ModelQ ships a DSA that injects an excitation into the loop and
measures the frequency response.  Here we do the same with SciPy: excite
with a chirp (or PRBS), then estimate the FRF with Welch cross-spectra:

    H(f) = Pxy(f) / Pxx(f),   coherence tells you where to trust it.
"""
import numpy as np
from scipy import signal

__all__ = ["chirp_excitation", "prbs_excitation", "measure_frf", "frf_of_system"]


def chirp_excitation(f0, f1, duration, fs, amplitude=1.0):
    """Logarithmic chirp sweeping f0 -> f1 Hz.  Returns (t, u)."""
    t = np.arange(0, duration, 1.0 / fs)
    u = amplitude * signal.chirp(t, f0=f0, t1=duration, f1=f1, method="logarithmic")
    return t, u


def prbs_excitation(n_samples, amplitude=1.0, seed=0):
    """Pseudo-random binary sequence (+/- amplitude)."""
    rng = np.random.default_rng(seed)
    return amplitude * (2.0 * rng.integers(0, 2, n_samples) - 1.0)


def measure_frf(u, y, fs, nperseg=None):
    """Estimate frequency response y/u from time-domain records.

    Returns dict(f_hz, mag_db, phase_deg, coherence, H).
    """
    u = np.asarray(u, float)
    y = np.asarray(y, float)
    if nperseg is None:
        nperseg = min(len(u) // 8, 4096)
    f, pxx = signal.welch(u, fs=fs, nperseg=nperseg)
    _, pxy = signal.csd(u, y, fs=fs, nperseg=nperseg)
    _, coh = signal.coherence(u, y, fs=fs, nperseg=nperseg)
    H = pxy / pxx
    mask = f > 0
    f, H, coh = f[mask], H[mask], coh[mask]
    return {
        "f_hz": f,
        "mag_db": 20 * np.log10(np.abs(H)),
        "phase_deg": np.degrees(np.unwrap(np.angle(H))),
        "coherence": coh,
        "H": H,
    }


def frf_of_system(sys_tf, f_hz):
    """Analytic frequency response of a python-control LTI system at f_hz."""
    import control as ct
    f_hz = np.atleast_1d(np.asarray(f_hz, float))
    w = 2 * np.pi * f_hz
    resp = ct.frequency_response(sys_tf, w)
    H = np.atleast_1d(np.asarray(resp.frdata).squeeze())
    return {
        "f_hz": f_hz,
        "mag_db": 20 * np.log10(np.abs(H)),
        "phase_deg": np.degrees(np.unwrap(np.angle(H))),
        "H": H,
    }

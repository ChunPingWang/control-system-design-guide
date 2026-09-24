"""Shared plotting helpers: uniform Bode plots for analytic and measured FRFs."""
import numpy as np
import matplotlib.pyplot as plt

from .dsa import frf_of_system

__all__ = ["bode_compare"]


def bode_compare(items, f_hz=None, title=None):
    """Overlay Bode plots.

    items: list of (label, obj) where obj is a python-control LTI system or a
    measured-FRF dict with keys f_hz / mag_db / phase_deg (from measure_frf).
    Returns (fig, (ax_mag, ax_phase)).
    """
    f_default = np.logspace(-1, 3, 600) if f_hz is None else np.asarray(f_hz)
    fig, (axm, axp) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for item in items:
        label, obj = item[0], item[1]
        style = item[2] if len(item) > 2 else "-"
        d = obj if isinstance(obj, dict) else frf_of_system(obj, f_default)
        axm.semilogx(d["f_hz"], d["mag_db"], style, label=label)
        axp.semilogx(d["f_hz"], d["phase_deg"], style, label=label)
    axm.set_ylabel("Magnitude [dB]")
    axp.set_ylabel("Phase [deg]")
    axp.set_xlabel("Frequency [Hz]")
    for ax in (axm, axp):
        ax.grid(True, which="both", alpha=0.3)
    axm.legend(fontsize=9)
    if title:
        axm.set_title(title)
    fig.tight_layout()
    return fig, (axm, axp)

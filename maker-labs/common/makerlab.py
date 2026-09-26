"""makerlab — Maker Labs 共用的模擬小工具。

刻意精簡:只放各章 sim.py 會重複用到的
「step 響應指標」與「無頭安全存圖」兩件事。
控制計算本身直接用 python-control / numpy / scipy,和 maker-labs 講義一致。
"""
from __future__ import annotations
import os
import pathlib
import numpy as np

# 無頭環境(run_all.sh 設 MPLBACKEND=Agg)不開視窗;有 GUI 時正常顯示。
import matplotlib
if os.environ.get("MPLBACKEND", "").lower() == "agg":
    matplotlib.use("Agg")


def step_metrics(t, y, setpoint=1.0):
    """由 step 響應算 rise time / overshoot / settling time / steady-state error。

    回傳 dict;定義與 python/common 一致(2%~98% rise、2% settling band)。
    """
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    yf = y[-1]
    sp = float(setpoint)

    # rise time: 10%→90% of setpoint
    lo, hi = 0.1 * sp, 0.9 * sp
    def _cross(level):
        idx = np.where(y >= level)[0]
        return t[idx[0]] if len(idx) else np.nan
    rise = _cross(hi) - _cross(lo)

    # overshoot
    peak = y.max() if sp >= 0 else y.min()
    overshoot = (peak - sp) / sp * 100.0 if sp != 0 else 0.0
    overshoot = max(overshoot, 0.0)

    # settling time: 進入 ±2% 且不再離開
    band = 0.02 * abs(sp) if sp != 0 else 0.02
    settling = np.nan
    for i in range(len(t)):
        if np.all(np.abs(y[i:] - sp) <= band):
            settling = t[i]
            break

    sse = sp - yf
    return {
        "rise_time": float(rise),
        "overshoot_pct": float(overshoot),
        "settling_time": float(settling),
        "steady_state_error": float(sse),
    }


def savefig(fig, name):
    """把圖存到本章目錄旁的 out/(供檢視);回傳存檔路徑。

    無頭模式一定存檔;互動模式也會存,方便 lab report 附圖。
    """
    caller_dir = pathlib.Path.cwd()
    out = caller_dir / "out"
    out.mkdir(exist_ok=True)
    path = out / name
    fig.savefig(path, dpi=110, bbox_inches="tight")
    return str(path)


def show_or_save(plt, name):
    """無頭:存檔;有 GUI:顯示。各章 sim.py 收尾呼叫。"""
    fig = plt.gcf()
    p = savefig(fig, name)
    if matplotlib.get_backend().lower() != "agg":
        plt.show()
    plt.close(fig)
    return p

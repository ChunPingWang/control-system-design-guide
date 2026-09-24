#!/usr/bin/env python3
"""把 C++ lab 輸出的 CSV 畫成圖(選用;C++ 程式本身不依賴 Python)。

用法:
    python3 tools/plot_csv.py build/out/ch04_step_fs500.csv
    python3 tools/plot_csv.py build/out/ch02_bode_open_closed.csv --logx
    python3 tools/plot_csv.py build/out/*.csv --save figs/

第一欄當 x 軸,其餘欄各畫一條線;欄名含 phase 的另開下方子圖(Bode 版型)。
檔名以 f_hz 開頭或加 --logx 時 x 軸取對數。
"""
import argparse
import csv
import math
import pathlib

import matplotlib.pyplot as plt


def load(path):
    with open(path) as f:
        rows = list(csv.reader(f))
    header, data = rows[0], rows[1:]
    cols = {h: [] for h in header}
    for r in data:
        for h, v in zip(header, r):
            cols[h].append(float(v) if v not in ("", "inf", "-inf") else math.nan)
    return header, cols


def plot(path, logx=False, save_dir=None):
    header, cols = load(path)
    x = header[0]
    ys = header[1:]
    phase = [h for h in ys if "phase" in h]
    mag = [h for h in ys if h not in phase]
    logx = logx or x == "f_hz"
    fig, axes = plt.subplots(2 if phase else 1, 1, figsize=(9, 6 if phase else 4), sharex=True, squeeze=False)
    for ax, names in zip(axes[:, 0], [mag, phase] if phase else [mag]):
        for h in names:
            ax.plot(cols[x], cols[h], label=h, lw=1)
        if logx:
            ax.set_xscale("log")
        ax.grid(alpha=0.3, which="both")
        ax.legend(fontsize=8)
    axes[-1, 0].set_xlabel(x)
    axes[0, 0].set_title(pathlib.Path(path).name)
    fig.tight_layout()
    if save_dir:
        out = pathlib.Path(save_dir) / (pathlib.Path(path).stem + ".png")
        out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out, dpi=120)
        plt.close(fig)
        print(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--logx", action="store_true")
    ap.add_argument("--save", metavar="DIR", help="存成 PNG 而不是開視窗")
    a = ap.parse_args()
    for p in a.csv:
        plot(p, a.logx, a.save)
    if not a.save:
        plt.show()


if __name__ == "__main__":
    main()

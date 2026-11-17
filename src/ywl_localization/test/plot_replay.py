#!/usr/bin/env python3
"""Plot common-frame trajectories, their separation, and YWL scan processing time."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from compare_replay import read_csv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--paired", type=Path, required=True)
    parser.add_argument("--diagnostics", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--scipilot-scripts", type=Path, help="Optional SciPilot styles and layout audit")
    args = parser.parse_args()
    if args.scipilot_scripts:
        sys.path.insert(0, str(args.scipilot_scripts))
        from setup_style import setup_style
        setup_style(journal="general", lang="en")
        plt.rcParams["savefig.bbox"] = None

    poses = read_csv(args.paired)
    report = [json.loads(s) for s in args.diagnostics.read_text().splitlines()]
    frames = sorted({d["stamp_ns"]: float(d["processing_ms"]) for d in report
                     if d["state"] == "TRACKING"}.items())
    stamp = poses["stamp_ns"]
    t0 = stamp[0]
    axis_t = (stamp - t0) / 1e9
    base = np.column_stack([poses[f"baseline_{axis}"] for axis in "xyz"])
    ywl = np.column_stack([poses[f"ywl_{axis}"] for axis in "xyz"])
    delta = np.linalg.norm(base - ywl, axis=1) * 100
    work_s = (np.array([p[0] for p in frames], dtype=np.int64) - t0) / 1e9
    work_ms = np.array([p[1] for p in frames])

    blue, orange = "#0072B2", "#D55E00"
    fig = plt.figure(figsize=(8.2, 4.1), dpi=100, layout="constrained")
    grid = fig.add_gridspec(2, 2, width_ratios=(1, 1.5))
    xy, separation, timing = fig.add_subplot(grid[:, 0]), fig.add_subplot(grid[0, 1]), fig.add_subplot(grid[1, 1])
    xy.plot(*base[:, :2].T, color=blue, lw=1.25, label="FAST-LIO2")
    xy.plot(*ywl[:, :2].T, color=orange, lw=1.0, ls="--", label="YWL")
    xy.scatter(*ywl[0, :2], marker="o", c="black", s=12, zorder=8)
    xy.set(xlabel="Map x (m)", ylabel="Map y (m)", title="A   Map trajectory")
    xy.set_aspect("equal", adjustable="datalim")
    xy.legend(loc="upper right", frameon=True)

    separation.plot(axis_t, delta, color="#21666E", lw=0.9)
    separation.set(title="B   Separation between estimates", ylabel="3D distance (cm)", ylim=(0, None))
    separation.text(0.98, 0.93, f"{len(stamp):,} paired scans", transform=separation.transAxes,
                    ha="right", va="top", fontsize=8,
                    bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.9})

    timing.plot(work_s, work_ms, color=orange, lw=0.65)
    timing.scatter(work_s[0], work_ms[0], c=orange, s=10, zorder=5)
    timing.annotate("Prior map switch", (work_s[0], work_ms[0]), xytext=(14, -1),
                    textcoords="offset points", ha="left", va="center", fontsize=7)
    timing.axhline(100, ls="--", color="#4D4D4D", lw=0.75, label="100 ms scan period")
    timing.set(title="C   YWL scan processing", xlabel="Seconds since first paired scan",
               ylabel="Time (ms)", ylim=(0, max(110, max(work_ms) * 1.08)))
    timing.legend(loc="upper right", frameon=True)
    for ax in (separation, timing):
        ax.set_xlim(-3, max(axis_t[-1], work_s[-1]) + 2)
        ax.grid(axis="y", alpha=0.25, lw=0.5)
    if args.scipilot_scripts:
        from visual_qa import audit_layout, render_preview
        from export_figure import export_figure
        fig.canvas.draw()
        issues = audit_layout(fig)
        if issues:
            raise RuntimeError(issues)
        render_preview(fig, str(args.out) + "_preview.png")
        export_figure(fig, str(args.out), formats=("png", "svg"), dpi=300, tight=False,
                      grayscale_preview=False)
        from PIL import Image
        with Image.open(str(args.out) + ".png") as png:
            png.convert("L").save(str(args.out) + "_grayscale.png")
    else:
        fig.savefig(str(args.out) + ".png", dpi=300)
        fig.savefig(str(args.out) + ".svg")


if __name__ == "__main__":
    main()

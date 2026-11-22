#!/usr/bin/env python3
"""Generate the figures used in the IEEE paper.

Inputs
------
docs/results/results.csv   produced by proyecto_rm_control/trajectory_tracker
docs/results/summary.txt

Outputs
-------
docs/figures/fig_geometry.pdf
docs/figures/fig_trajectory.pdf
docs/figures/fig_errors.pdf
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "docs" / "results"
FIGURES = ROOT / "docs" / "figures"

# IEEE single-column width is 3.5 in; two-column is 7.16 in.
COL_W = 3.5
FULL_W = 7.16

plt.rcParams.update(
    {
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
        "font.size": 8,
        "axes.labelsize": 8,
        "legend.fontsize": 7,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "axes.grid": True,
        "grid.alpha": 0.35,
        "grid.linewidth": 0.4,
        "lines.linewidth": 1.0,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.02,
    }
)


def load_results() -> dict[str, np.ndarray]:
    path = RESULTS / "results.csv"
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    data: dict[str, np.ndarray] = {}
    for key in rows[0]:
        data[key] = np.array([float(r[key]) for r in rows])
    return data


# --------------------------------------------------------------------- geometry
def fig_geometry() -> None:
    L = 0.160
    t = 0.202
    r = 0.037
    a = 0.120  # body half length
    b = 0.090  # body half width

    fig, ax = plt.subplots(figsize=(COL_W, 2.35))

    # chassis
    ax.add_patch(
        plt.Rectangle((-a, -b), 2 * a, 2 * b, fill=False, lw=1.1, ec="0.25")
    )

    # wheels (rectangles, not rotated for the rear axle)
    def wheel(x, y, ang=0.0, color="0.15"):
        w, h = 2 * r, 0.022
        rect = plt.Rectangle(
            (x - w / 2, y - h / 2), w, h, angle=ang, fill=True,
            fc=color, ec="0.1", lw=0.6,
        )
        ax.add_patch(rect)

    wheel(-L / 2, -t / 2)
    wheel(-L / 2, t / 2)
    wheel(L / 2, -t / 2)
    wheel(L / 2, t / 2)

    # steering angles
    delta = math.radians(25)
    for y in (-t / 2, t / 2):
        ax.plot(
            [-L / 2, -L / 2 + 2 * r * math.cos(delta)],
            [y, y + 2 * r * math.sin(delta)],
            color="tab:blue", lw=0.8, ls="--",
        )

    # axles
    ax.plot([-L / 2, -L / 2], [-t / 2, t / 2], color="0.4", lw=0.8, ls=":")
    ax.plot([L / 2, L / 2], [-t / 2, t / 2], color="0.4", lw=0.8, ls=":")

    # dimensions
    def dim(x0, y0, x1, y1, text, off=0.0, rot=0):
        ax.annotate(
            "", xy=(x1, y1), xytext=(x0, y0),
            arrowprops=dict(arrowstyle="<->", lw=0.6, color="0.2"),
        )
        ax.text(
            (x0 + x1) / 2, (y0 + y1) / 2 + off, text,
            ha="center", va="center", rotation=rot, fontsize=7,
            bbox=dict(fc="white", ec="none", pad=0.5),
        )

    dim(-L / 2, -b - 0.045, L / 2, -b - 0.045, r"$L=160$ mm")
    dim(a + 0.045, -t / 2, a + 0.045, t / 2, r"$t=202$ mm", rot=90, off=0.0)
    dim(-a - 0.02, -b, -a - 0.02, b, "", off=0)

    # wheel radius callout
    ax.annotate(
        r"$r=37$ mm", xy=(-L / 2 + r * 0.7, t / 2 + r * 0.7),
        xytext=(-0.155, 0.155), fontsize=7,
        arrowprops=dict(arrowstyle="-", lw=0.5, color="0.3"),
    )

    # steering angle
    ax.annotate(
        r"$\delta$", xy=(-L / 2 + 0.022, t / 2 + 0.012), fontsize=8,
        color="tab:blue",
    )

    # ICR and turning radius
    R = 0.30
    ax.plot([0], [R], marker="+", color="tab:red", ms=7, mew=1.0)
    ax.text(0.004, R + 0.012, "ICR", color="tab:red", fontsize=7)
    ax.plot([0, 0], [0, R], color="tab:red", lw=0.7, ls="-.")
    ax.text(-0.03, R * 0.55, r"$R$", color="tab:red", fontsize=8)
    ax.plot([0], [0], marker="o", color="tab:red", ms=2.5)

    ax.set_aspect("equal")
    ax.set_xlim(-0.20, 0.20)
    ax.set_ylim(-0.19, 0.38)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Ackermann geometry (top view)", fontsize=8)
    fig.savefig(FIGURES / "fig_geometry.pdf")
    plt.close(fig)


# ------------------------------------------------------------------- trajectory
def fig_trajectory(d: dict[str, np.ndarray]) -> None:
    fig, ax = plt.subplots(figsize=(COL_W, 2.6))
    ax.plot(d["ref_x"], d["ref_y"], color="0.35", lw=0.8, ls="--", label="Reference")
    ax.plot(d["gt_x"], d["gt_y"], color="tab:blue", lw=1.0, label="Ground truth")
    ax.plot(
        d["odom_x"], d["odom_y"], color="tab:orange", lw=0.7, ls=":",
        label="Wheel+IMU odometry",
    )
    ax.plot(d["gt_x"][0], d["gt_y"][0], "o", color="tab:green", ms=3, label="Start")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper right", frameon=False)
    ax.set_title("Figure-eight tracking", fontsize=8)
    fig.savefig(FIGURES / "fig_trajectory.pdf")
    plt.close(fig)


# ----------------------------------------------------------------------- errors
def fig_errors(d: dict[str, np.ndarray]) -> None:
    t = d["time"] - d["time"][0]
    fig, axes = plt.subplots(3, 1, figsize=(COL_W, 3.4), sharex=True)

    axes[0].plot(t, 100 * d["cross_track_error"], color="tab:orange", lw=0.8,
                 label="odometry")
    axes[0].plot(t, 100 * d["cross_track_error_gt"], color="tab:blue", lw=0.8,
                 label="ground truth")
    axes[0].set_ylabel("CTE [cm]")
    axes[0].legend(loc="upper right", frameon=False, ncol=2)

    axes[1].plot(t, d["v_cmd"], color="tab:green", lw=0.8)
    axes[1].set_ylabel("v [m/s]")

    axes[2].plot(t, np.degrees(d["delta_left"]), color="tab:red", lw=0.8,
                 label=r"$\delta_L$")
    axes[2].plot(t, np.degrees(d["delta_right"]), color="tab:purple", lw=0.8,
                 ls="--", label=r"$\delta_R$")
    axes[2].set_ylabel(r"$\delta$ [deg]")
    axes[2].set_xlabel("time [s]")
    axes[2].legend(loc="upper right", frameon=False, ncol=2)

    for ax in axes:
        ax.set_xlim(0, t[-1])
    fig.align_ylabels(axes)
    fig.savefig(FIGURES / "fig_errors.pdf")
    plt.close(fig)


def main() -> None:
    FIGURES.mkdir(parents=True, exist_ok=True)
    fig_geometry()
    d = load_results()
    fig_trajectory(d)
    fig_errors(d)
    print("Figures written to", FIGURES)


if __name__ == "__main__":
    main()

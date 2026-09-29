#!/usr/bin/env python3
"""Visualize the installed left-arm RealMan/CuRobo base-axis relationship."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


# p_curobo = R_curobo_from_realman @ p_realman
R_curobo_from_realman = np.array(
    [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]],
    dtype=float,
)
R_realman_from_curobo = R_curobo_from_realman.T


def draw_frame(ax, origin, rotation, length, name, colors, linestyle="-"):
    labels = ("X", "Y", "Z")
    for index, (label, color) in enumerate(zip(labels, colors)):
        direction = rotation[:, index]
        end = origin + length * direction
        ax.quiver(
            *origin,
            *direction,
            length=length,
            color=color,
            arrow_length_ratio=0.12,
            linestyle=linestyle,
            linewidth=2.5,
        )
        ax.text(*end, f"{name} +{label}", color=color, fontsize=11, weight="bold")


def set_equal_axes(ax, limit):
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_xlabel("RealMan +X")
    ax.set_ylabel("RealMan +Y")
    ax.set_zlabel("RealMan +Z")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        default="left_realman_curobo_axes.png",
        help="PNG output path.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Also open an interactive 3D window.",
    )
    args = parser.parse_args()

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    realman_colors = ("#d62728", "#2ca02c", "#1f77b4")
    curobo_colors = ("#ff9896", "#98df8a", "#aec7e8")

    # Draw both frames at different origins so coincident axes remain visible.
    realman_origin = np.array([-0.45, 0.0, 0.0])
    curobo_origin = np.array([0.45, 0.0, 0.0])
    draw_frame(ax, realman_origin, np.eye(3), 0.34, "RealMan", realman_colors)
    draw_frame(
        ax,
        curobo_origin,
        R_realman_from_curobo,
        0.34,
        "CuRobo",
        curobo_colors,
        linestyle="--",
    )

    ax.scatter(*realman_origin, color="black", s=35)
    ax.scatter(*curobo_origin, color="black", s=35)
    ax.text(*realman_origin, "  RealMan left_base", fontsize=10)
    ax.text(*curobo_origin, "  CuRobo driver_base", fontsize=10)
    ax.set_title(
        "Left arm base-frame relationship\n"
        "CuRobo X = -RealMan Z,  CuRobo Y = RealMan Y,  CuRobo Z = RealMan X"
    )
    set_equal_axes(ax, 0.95)
    ax.view_init(elev=24, azim=-58)
    fig.tight_layout()
    output = Path(args.output).resolve()
    fig.savefig(output, dpi=160)
    print(f"Saved: {output}")
    print("RealMan -> CuRobo position: [x, y, z] -> [-z, y, x]")
    print("CuRobo axes expressed in RealMan: Xc=-Zr, Yc=Yr, Zc=Xr")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()

"""Plot the sim experiments in the layout of the papers' figures.

    pixi run python ft_tilt_bringup/scripts/plot_paper_figures.py [bags/] [docs/figures/]

Reads the bags written by `pixi run experiment` (names: <config>_gamma<γ>_<gains>_<battery>_<allocation>)
and writes one PNG per paper figure. Angles are in the sim's FLU frame (+pitch = nose down,
+yaw = counter-clockwise from above); the papers don't state theirs.
"""
import math
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

BAGS = sys.argv[1] if len(sys.argv) > 1 else "bags"
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join("docs", "figures")

# Battery constants, as in ft_tilt.sdf.xacro: (k_f N/%, F_M N).
BATTERY = {"3s": (0.0776, 0.644 * 9.80665), "4s": (0.083, 8.3)}

# Reference categorical palette (dataviz skill), validated for 6 adjacent series on #fcfcfb.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": INK2, "axes.titlecolor": INK, "axes.titlesize": 10,
    "axes.labelsize": 9, "xtick.color": INK2, "ytick.color": INK2, "xtick.labelsize": 8,
    "ytick.labelsize": 8, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 1.5,
    "legend.fontsize": 8, "legend.frameon": False, "font.size": 9,
})


def load(name):
    """Bag -> dict of numpy arrays, time in s from experiment start (first motor_health)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=os.path.join(BAGS, name)), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    raw = {}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        key = topic.split("/")[-1]
        if key in ("attitude", "attitude_ref", "torque_cmd", "disturbance_torque"):
            row = [msg.x, msg.y, msg.z]
        elif key in ("pwm", "motor_health"):
            row = list(msg.normalized)
        elif key == "odometry":
            a = msg.twist.twist.angular
            row = [a.x, a.y, a.z]
        else:
            continue
        raw.setdefault(key, []).append([stamp * 1e-9] + row)
    d = {k: np.array(v) for k, v in raw.items()}
    t0 = d["motor_health"][0, 0]
    for v in d.values():
        v[:, 0] -= t0
    return d


def window(a, t0, t1, shift=0.0):
    m = (a[:, 0] >= t0) & (a[:, 0] <= t1)
    return a[m, 0] - shift, a[m, 1:]


def linear_pwm(pwm, battery):
    """Quadratic PWM (as published) -> the papers' linear f/k_f, for their simulated figures."""
    kf, fm = BATTERY[battery]
    return fm * (pwm * kf / fm) ** 2 / kf


def label_ends(ax, t, ys, names):
    """Direct labels at the right end of each line (relief for low-contrast hues)."""
    lo, hi = ax.get_ylim()
    gap = 0.045 * (hi - lo)
    ends = sorted((y[-1], n) for y, n in zip(ys, names))
    placed = []
    for y, n in ends:  # push labels up until they clear the one below
        placed.append(max(y, placed[-1] + gap) if placed else y)
        ax.annotate(n, (t[-1], y), xytext=(t[-1], placed[-1]), textcoords="data", va="center",
                    fontsize=7, color=INK2, annotation_clip=False)


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("wrote", path)


GAINS = [  # (bag, label, colour, line width, dash) — the paper's thick / thin / dashed
    ("icris18_pulse_gamma107_mild_4s_null_space", "Mild gains", SERIES[0], 2.2, "-"),
    ("icris18_pulse_gamma107_aggressive_4s_null_space", "Aggressive gains", SERIES[1], 1.2, "-"),
    ("icris18_pulse_nofault_gamma107_aggressive_4s_null_space", "No failure", SERIES[2], 1.5, "--"),
]


def icris18_fig6_7():
    # The paper's sim pulse runs 5-15 s; ours runs 20-30 s, so shift by 15 s.
    runs = [(load(b), lab, c, lw, ls) for b, lab, c, lw, ls in GAINS]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(6.5, 5), sharex=True)
    for d, lab, c, lw, ls in runs:
        t, att = window(d["attitude"], 15, 40, 15)
        a1.plot(t, np.degrees(att[:, 1]), color=c, lw=lw, ls=ls, label=lab)
        t, w = window(d["odometry"], 15, 40, 15)
        a2.plot(t, np.degrees(w[:, 1]), color=c, lw=lw, ls=ls, label=lab)
    for ax in (a1, a2):
        ax.axvspan(5, 15, color=GRID, alpha=0.5, lw=0)
    a1.set_title("Pitch angle (ICRIS18 Fig. 6), −1 N·m pitch disturbance 5–15 s, motor 2 off")
    a1.set_ylabel("θ [deg]")
    a2.set_title("Pitch rate")
    a2.set_ylabel("q [deg/s]")
    a2.set_xlabel("t [s]")
    a1.legend(loc="upper right")
    save(fig, "icris18_fig6_pitch_pulse.png")

    fig, axes = plt.subplots(3, 2, figsize=(7.5, 7), sharex=True)
    for i, ax in enumerate(axes.flat):
        for d, lab, c, lw, ls in runs:
            t, p = window(d["pwm"], 15, 40, 15)
            ax.plot(t, linear_pwm(p[:, i], "4s"), color=c, lw=lw, ls=ls, label=lab)
        ax.set_title(f"Rotor {i + 1}")
        ax.set_ylabel("PWM [%]")
        ax.set_ylim(bottom=-5)
    for ax in axes[-1]:
        ax.set_xlabel("t [s]")
    axes[0, 0].legend(loc="upper left")
    fig.suptitle("Rotor PWM (ICRIS18 Fig. 7), linear f/k_f as in the paper's simulation", fontsize=10, color=INK)
    fig.tight_layout()
    save(fig, "icris18_fig7_pwm_pulse.png")


def icris18_fig11_12_13():
    d = load("icris18_yaw_gamma107_mild_4s_null_space")
    fig, ax = plt.subplots(figsize=(6.5, 3.5))
    t, p = window(d["pwm"], 2, 19.9)  # stop before the yaw step at 20 s
    names = [f"M{i + 1}" for i in range(6)]
    for i in range(6):
        ax.plot(t, p[:, i], color=SERIES[i], label=names[i])
    ax.set_ylim(-5, 100)
    label_ends(ax, t, p.T, names)
    ax.axvline(10, color=INK2, lw=0.8)
    ax.annotate("motor 2 off", (10, 5), xytext=(4, 0), textcoords="offset points", fontsize=7, color=INK2)
    ax.set_title("PWM signals in hover (ICRIS18 Fig. 11), thrust ~ PWM²")
    ax.set_xlim(right=21.5)
    ax.set_ylabel("PWM [%]")
    ax.set_xlabel("t [s]")
    ax.legend(loc="lower left", ncol=6)
    save(fig, "icris18_fig11_pwm_hover.png")

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(8, 3.5))
    for ax, (t0, t1), title in ((a1, (18, 30), "Relaxed direction, +yaw (ICRIS18 Fig. 12)"),
                                (a2, (30, 45), "Stressed direction, −yaw (ICRIS18 Fig. 13)")):
        t, att = window(d["attitude"], t0, t1)
        tr, ref = window(d["attitude_ref"], t0, t1)
        ax.plot(tr, np.degrees(ref[:, 2]), color=INK2, lw=1.2, label="Reference")
        ax.plot(t, np.degrees(att[:, 2]), color=SERIES[0], ls="--", label="Measured")
        ax.set_title(title)
        ax.set_xlabel("t [s]")
    a1.set_ylabel("ψ [deg]")
    a1.legend(loc="lower right")
    save(fig, "icris18_fig12_13_yaw.png")


def rpic17_fig7_8():
    runs = [("rpic17_inflight_fault_gamma90_rpic17_3s_pinv_clamp", "Tilting angle = 90° (pinv + clamp)"),
            ("rpic17_inflight_fault_gamma107_rpic17_3s_null_space", "Tilting angle = 107° (null-space)")]
    fig, axes = plt.subplots(2, 2, figsize=(10, 6))
    for row, (bag, title) in enumerate(runs):
        d = load(bag)
        t, att = window(d["attitude"], 5, 40)
        tr, ref = window(d["attitude_ref"], 5, 40)
        ax = axes[row, 0]
        ax.plot(t, np.degrees(att[:, 1]), color=SERIES[0], label="Pitch angle")
        ax.plot(tr, np.degrees(ref[:, 1]), color=SERIES[0], lw=1, ls="--", label="Pitch reference")
        ax.plot(t, np.degrees(att[:, 0]), color=SERIES[1], label="Roll angle")
        ax.plot(tr, np.degrees(ref[:, 0]), color=SERIES[1], lw=1, ls="--", label="Roll reference")
        ax.set_ylim(-90, 40)
        over = np.nonzero(np.abs(att[:, 0]) > math.pi / 2)[0]
        if len(over):
            ax.annotate(f"rolled over at {t[over[0]]:.1f} s", (t[over[0]], -60), xytext=(8, 0),
                        textcoords="offset points", fontsize=8, color=INK)
        ax.set_ylabel("Angle [deg]")
        ax.set_title(title + " — angles (RPIC17 Fig. 7)")
        ax = axes[row, 1]
        t, p = window(d["pwm"], 5, 40)
        names = [f"M{i + 1}" for i in range(6)]
        for i in range(6):
            ax.plot(t, p[:, i], color=SERIES[i], lw=1.2, label=names[i])
        ax.axhline(40, color=INK2, lw=0.8, ls=":")
        ax.set_ylim(0, 100)
        ax.set_xlim(right=43)
        label_ends(ax, t, p.T, names)
        ax.set_ylabel("PWM [%]")
        ax.set_title(title + " — PWM (RPIC17 Fig. 8)")
        for a in axes[row]:
            a.axvspan(15, 30, color=GRID, alpha=0.5, lw=0)
    axes[0, 0].legend(loc="lower left", ncol=2)
    axes[0, 1].legend(loc="lower left", ncol=6)
    for a in axes[-1]:
        a.set_xlabel("t [s]  (motor 2 capped at 40 % PWM from 15 to 30 s)")
    fig.tight_layout()
    save(fig, "rpic17_fig7_8_inflight_fault.png")


if __name__ == "__main__":
    icris18_fig6_7()
    icris18_fig11_12_13()
    rpic17_fig7_8()

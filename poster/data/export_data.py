#!/usr/bin/env python3
"""Export the runs the poster draws, as JSON.

    pixi run python poster/data/export_data.py > poster/data/data.json
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.argv = sys.argv[:1]  # plot_paper_figures reads its bag folder from argv
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ft_tilt_bringup" / "scripts"))
import plot_paper_figures as pf  # noqa: E402

pf.BAGS = str(Path(__file__).resolve().parents[2] / "bags")


def resample(a, t):
    """Columns of a (time in column 0) at times t."""
    return np.stack([np.interp(t, a[:, 0], a[:, j]) for j in range(1, a.shape[1])], axis=1)


def r(x, n=1):
    return np.round(x, n).tolist()


def fault():
    """RPIC17 in-flight partial fault: standard + pinv vs tilted + null space, 10 Hz, 10-35 s."""
    t = np.arange(10.0, 35.0 + 1e-9, 0.1)
    out = {"t": r(t)}
    for key, bag in (("std", "rpic17_inflight_fault_gamma90_rpic17_3s_pinv_clamp"),
                     ("tilt", "rpic17_inflight_fault_gamma107_rpic17_3s_null_space")):
        d = pf.load(bag)
        att = np.degrees(resample(d["attitude"], t))
        out[key] = {"roll": r(att[:, 0]), "pitch": r(att[:, 1]), "pwm": r(resample(d["pwm"], t), 0)}
    return out


def pulse():
    """ICRIS18 Fig. 6: pitch for a -1 N·m pulse, motor 2 off, 20 Hz, paper time (pulse 5-15 s)."""
    t = np.arange(0.0, 25.0 + 1e-9, 0.05)
    out = {"t": r(t, 2)}
    for key, (bag, *_rest) in zip(("mild", "aggr", "nofault"), pf.GAINS):
        d = pf.load(bag)
        out[key] = r(np.degrees(resample(d["attitude"], t + 15))[:, 1])
    return out


def hover():
    """ICRIS18 Fig. 11: mean PWM per motor, healthy hover vs motor 2 off."""
    d = pf.load("icris18_yaw_gamma107_mild_4s_null_space")
    p = d["pwm"]
    mean = lambda t0, t1: p[(p[:, 0] >= t0) & (p[:, 0] <= t1), 1:].mean(axis=0)
    return {"before": r(mean(6, 9.5)), "after": r(mean(15, 19.5))}


json.dump({"fault": fault(), "pulse": pulse(), "hover": hover()}, sys.stdout, separators=(",", ":"))

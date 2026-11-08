#!/usr/bin/env python3
"""Turn the original poster's data.json (from data/export_data.py) into the CSVs the charts read.

    python3 data/from-json.py
"""
import csv
import json
import sys
from pathlib import Path

here = Path(__file__).parent
d = json.load(open(sys.argv[1] if len(sys.argv) > 1 else here / "data.json"))

# Fault: one row per vehicle and 0.1 s step; t is seconds since the clip starts
# (10 s into the run), so motor 2 is capped at t = 5 s (15 s in the run).
f = d["fault"]
with open(here / "fault.csv", "w", newline="") as out:
    w = csv.writer(out)
    w.writerow(["vehicle", "t", "roll", "pitch", *(f"pwm{m}" for m in range(1, 7))])
    for v in ("std", "tilt"):
        for j, t in enumerate(f["t"]):
            w.writerow([v, round(t - 10, 1), f[v]["roll"][j], f[v]["pitch"][j], *(int(x) for x in f[v]["pwm"][j])])

# Pulse: pitch (deg) per gain set, 20 Hz, paper time.
p = d["pulse"]
with open(here / "pulse.csv", "w", newline="") as out:
    w = csv.writer(out)
    w.writerow(["t", "nofault", "aggr", "mild"])
    for j, t in enumerate(p["t"]):
        w.writerow([t, p["nofault"][j], p["aggr"][j], p["mild"][j]])

# Hover: mean PWM (%) per motor, healthy vs motor 2 off.
h = d["hover"]
with open(here / "hover.csv", "w", newline="") as out:
    w = csv.writer(out)
    w.writerow(["motor", "state", "pwm"])
    for m in range(6):
        w.writerow([m + 1, "healthy", h["before"][m]])
        w.writerow([m + 1, "motor 2 off", h["after"][m]])

#!/usr/bin/env python3
"""Fly the Aerostack2 mission over a matrix of cases, each in a fresh headless sim, and print a table.

    pixi run test-missions [--gui] [case ...]      # all cases, or only the named ones

Each case runs on its own ROS domain / gz partition, so a GUI sim you have open is not touched.
Logs: bags/missions/<case>.log. A case passes when the mission's RESULT matches `expect`.
"""
import os
import re
import signal
import subprocess
import sys
import time

# name: (launch args, mission args, expected result)
CASES = {
    "nominal":        ([], [], "PASS"),
    **{f"fault{m}":   ([], ["--fault-motor", str(m)], "PASS") for m in range(1, 7)},
    "goto_fault2":    ([], ["--fault-motor", "2", "--goto", "3", "2"], "PASS"),
    # RPIC17's partial fault: 40 % PWM cap = 0.241·F_M (AGENTS.md); 3s is for partial faults only.
    "3s_partial2":    (["battery:=3s"], ["--fault-motor", "2", "--fault-health", "0.241"], "PASS"),
    "gamma73_fault2": (["gamma_deg:=73"], ["--fault-motor", "2"], "PASS"),
    # Standard vehicle: losing a motor removes one yaw direction (AGENTS.md), so it can't hold yaw.
    "gamma90_fault2": (["gamma_deg:=90"], ["--fault-motor", "2"], "FAIL"),
}
LOGS = os.path.normpath(os.path.join(os.getcwd(), "bags", "missions"))


def run(name, launch_args, mission_args, gui):
    env = dict(os.environ, ROS_DOMAIN_ID="77", GZ_PARTITION=f"ft_tilt_test_{os.getpid()}")
    log = open(os.path.join(LOGS, f"{name}.log"), "w")
    sim = subprocess.Popen(["ros2", "launch", "ft_tilt_as2", "as2.launch.py", f"headless:={str(not gui).lower()}", *launch_args],
                           stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    try:
        deadline = time.time() + 120  # controller + take-off, land and go-to behaviours
        while open(log.name).read().count("PLUGIN LOADED") < 4:
            if time.time() > deadline or sim.poll() is not None:
                return "FAIL sim did not start"
            time.sleep(1)
        time.sleep(3)
        try:
            out = subprocess.run(["python3", "-u", os.path.join(os.path.dirname(__file__), "mission.py"),
                                  *mission_args], env=env, capture_output=True, text=True, timeout=300).stdout
        except subprocess.TimeoutExpired as e:
            out = (e.stdout or b"").decode() + "\nRESULT FAIL timeout"
        log.write(out)
        m = re.search(r"RESULT (.*)", out)
        return m.group(1) if m else "FAIL no result"
    finally:  # stop only this case's launch (its own process group)
        for sig, wait in ((signal.SIGINT, 20), (signal.SIGKILL, 5)):
            try:
                os.killpg(sim.pid, sig)
                sim.wait(wait)
                break
            except subprocess.TimeoutExpired:
                continue
            except ProcessLookupError:
                break
        log.close()


def main():
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(1))  # still stop the running case's sim
    gui = "--gui" in sys.argv
    names = [a for a in sys.argv[1:] if a != "--gui"] or list(CASES)
    unknown = set(names) - set(CASES)
    if unknown:
        sys.exit(f"unknown cases: {', '.join(sorted(unknown))}; known: {', '.join(CASES)}")
    os.makedirs(LOGS, exist_ok=True)
    rows = []
    for name in names:
        launch_args, mission_args, expect = CASES[name]
        result = run(name, launch_args, mission_args, gui)
        rows.append((name, expect, result, result.split()[0] == expect))
        print(f"{'ok ' if rows[-1][3] else 'BAD'} {name:15} expect {expect}: {result}", flush=True)
    bad = [r[0] for r in rows if not r[3]]
    print(f"\n{len(rows) - len(bad)}/{len(rows)} as expected" + (f"; unexpected: {', '.join(bad)}" if bad else ""))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()

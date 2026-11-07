#!/usr/bin/env python3
"""Aerostack2 mission on the ft_tilt vehicle: arm, take off, hover, (fault), hover, (go to), land.

    pixi run mission [--fault-motor 2 [--fault-health 0.241]] [--goto 3 2] [--height 1.5] [--hover 10]

With --fault-motor the motor is switched off mid-hover (motor_health = 0, or --fault-health for a
partial fault), as the papers did with an RC switch; the fault-aware allocation and gains in the
gz plugin take over. All motors are set healthy at the start, so a previous run's fault is cleared.

Ends with a RESULT line. PASS = every step succeeded, tilt ≤ 15°, yaw within 0.2 rad of the
start and back on the ground (assumed limits: hover is within ~2° when the vehicle copes).
Past 60° of tilt the vehicle is flipping: the motors are cut (kill switch) and the mission stops.
"""
import argparse
import math
import threading
import time

import rclpy
from actuator_msgs.msg import Actuators
from as2_msgs.msg import AlertEvent
from as2_python_api.drone_interface import DroneInterface

MAX_TILT_DEG, MAX_YAW_ERR, CRASH_TILT_DEG = 15.0, 0.2, 60.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--height", type=float, default=1.5)
    ap.add_argument("--hover", type=float, default=10.0, help="seconds of hover before and after the fault")
    ap.add_argument("--fault-motor", type=int, choices=range(1, 7), help="motor (1-6) to fail mid-hover")
    ap.add_argument("--fault-health", type=float, default=0.0, help="0 = off, 0 < h < 1 caps thrust at h·F_M")
    ap.add_argument("--goto", type=float, nargs=2, metavar=("DX", "DY"),
                    help="after the fault, fly this far (m, earth frame) at 0.5 m/s, then land there")
    args = ap.parse_args()

    rclpy.init()
    drone = DroneInterface(drone_id="ft_tilt", use_sim_time=True)
    ok = False
    stats = {"tilt": 0.0, "yaw_err": 0.0, "yaw0": None, "z0": None, "crashed": False}
    try:
        ok = fly(drone, args, stats)
    finally:  # the API's spin threads keep the process alive if it isn't shut down
        landed = stats["z0"] is not None and drone.position[2] - stats["z0"] < 0.1
        passed = ok and landed and stats["tilt"] <= MAX_TILT_DEG and stats["yaw_err"] <= MAX_YAW_ERR
        print(f"RESULT {'PASS' if passed else 'FAIL'} steps={'ok' if ok else 'failed'} "
              f"max_tilt={stats['tilt']:.1f}deg max_yaw_err={stats['yaw_err']:.3f}rad "
              f"landed={'yes' if landed else 'no'} crashed={'yes' if stats['crashed'] else 'no'}", flush=True)
        drone.shutdown()
        rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


def fly(drone, args, stats):
    health_pub = drone.create_publisher(Actuators, "/ft_tilt/motor_health", 10)

    def step(name, ok):
        print(f"{name}: {'ok' if ok else 'FAILED'}", flush=True)
        return ok

    def set_health(motor=None, h=1.0):
        msg = Actuators()
        msg.normalized = [h if i + 1 == motor else 1.0 for i in range(6)]
        health_pub.publish(msg)

    def watch():  # runs in the API's executor thread
        roll, pitch, yaw = drone.orientation
        tilt = math.degrees(math.acos(math.cos(roll) * math.cos(pitch)))
        if drone.position[2] - stats["z0"] > 0.1:  # airborne
            stats["tilt"] = max(stats["tilt"], tilt)
            err = abs(math.remainder(yaw - stats["yaw0"], math.tau))
            stats["yaw_err"] = max(stats["yaw_err"], err)
        if tilt > CRASH_TILT_DEG and not stats["crashed"]:  # flipping: cut the motors
            stats["crashed"] = True
            print(f"crash: tilt {tilt:.0f}deg, kill switch", flush=True)
            # A blocking go_to/land would wait forever for a flipped vehicle. stop() is a blocking
            # service call, so not from this executor thread.
            threading.Thread(target=lambda: [b.stop() for b in (drone.takeoff, drone.go_to, drone.land)],
                             daemon=True).start()
        if stats["crashed"]:
            # The API's send_emergency_killswitch_to_aircraft() loops forever; it would freeze this thread.
            drone.alert_pub.publish(AlertEvent(alert=AlertEvent.KILL_SWITCH))

    def hold(seconds):  # False once crashed
        end = time.time() + seconds
        while time.time() < end and not stats["crashed"]:
            time.sleep(0.05)
        return not stats["crashed"]

    deadline = time.time() + 10.0  # the bridge must see us, or the reset is lost; and a first pose
    while (health_pub.get_subscription_count() == 0 or not any(drone.position)) and time.time() < deadline:
        time.sleep(0.1)
    set_health()
    stats["z0"], stats["yaw0"] = drone.position[2], drone.orientation[2]
    drone.create_timer(0.05, watch)

    # Aerostack2 answers a repeated arm/offboard request with a failure, so only ask when needed.
    if not drone.info["armed"] and not step("arm", drone.arm()):
        return False
    if not drone.info["offboard"] and not step("offboard", drone.offboard()):
        return False
    target = stats["z0"] + args.height
    if not step("takeoff", drone.takeoff(height=args.height, speed=0.5)):
        return False
    # takeoff_plugin_position reports success when |height − z| < 0.2 m with absolute z, ~0.4 m
    # early for our base 0.215 m up. It leaves the takeoff target set, so wait until we're there.
    deadline = time.time() + 15.0
    while abs(drone.position[2] - target) > 0.05 and time.time() < deadline:
        time.sleep(0.1)
    if not hold(args.hover):
        return False
    if args.fault_motor:
        set_health(args.fault_motor, args.fault_health)
        print(f"motor {args.fault_motor} health {args.fault_health}", flush=True)
        if not hold(args.hover):
            return False
    if args.goto:
        x, y, _ = drone.position
        if not step("go_to", drone.go_to(x + args.goto[0], y + args.goto[1], target, 0.5)) or not hold(3.0):
            return False
    return step("land", drone.land(speed=0.5))  # the land behaviour disarms the platform itself


if __name__ == "__main__":
    main()

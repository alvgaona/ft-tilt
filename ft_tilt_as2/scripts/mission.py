#!/usr/bin/env python3
"""Aerostack2 mission on the ft_tilt vehicle: arm, take off, hover, (fault), hover, land.

    pixi run mission [--fault-motor 2] [--height 1.5] [--hover 10]

With --fault-motor the motor is switched off mid-hover (motor_health = 0), as the papers did
with an RC switch; the fault-aware allocation and gains in the gz plugin take over.
"""
import argparse
import time

import rclpy
from actuator_msgs.msg import Actuators
from as2_python_api.drone_interface import DroneInterface


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--height", type=float, default=1.5)
    ap.add_argument("--hover", type=float, default=10.0, help="seconds of hover before and after the fault")
    ap.add_argument("--fault-motor", type=int, choices=range(1, 7), help="motor (1-6) to switch off mid-hover")
    args = ap.parse_args()

    rclpy.init()
    drone = DroneInterface(drone_id="ft_tilt", use_sim_time=True)
    drone.load_module("motion_reference_handler_module")  # hover(); not loaded by default
    try:
        fly(drone, args)
    finally:  # the API's spin threads keep the process alive if it isn't shut down
        drone.shutdown()
        rclpy.shutdown()


def fly(drone, args):
    health_pub = drone.create_publisher(Actuators, "/ft_tilt/motor_health", 10)

    def step(name, ok):
        print(f"{name}: {'ok' if ok else 'FAILED'}", flush=True)
        if not ok:
            raise SystemExit(1)

    # Aerostack2 answers a repeated arm/offboard request with a failure, so only ask when needed.
    if not drone.info["armed"]:
        step("arm", drone.arm())
    if not drone.info["offboard"]:  # stays on after a previous mission
        step("offboard", drone.offboard())
    step("takeoff", drone.takeoff(height=args.height, speed=0.5))
    # Aerostack2's takeoff_plugin_position reports success when |height − z| < 0.2 m (absolute z),
    # ~0.4 m early for a vehicle whose origin starts 0.215 m up. Our controller keeps the takeoff
    # target, so let it get there before hover() freezes the reference.
    time.sleep(3.0)
    # Hover works, but the API logs "Timeout waiting for mode 1": the controller reports the
    # plugin's hover mode (POSITION), not HOVER.
    drone.motion_ref_handler.hover()
    time.sleep(args.hover)
    if args.fault_motor:
        health = Actuators()
        health.normalized = [0.0 if i + 1 == args.fault_motor else 1.0 for i in range(6)]
        health_pub.publish(health)
        print(f"motor {args.fault_motor} off", flush=True)
        time.sleep(args.hover)
    step("land", drone.land(speed=0.5))  # the land behaviour disarms the platform itself


if __name__ == "__main__":
    main()

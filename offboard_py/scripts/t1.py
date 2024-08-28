#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
from pymavlink import mavutil

# Establish connection to the autopilot
master = mavutil.mavlink_connection('udpin:192.168.144.25:15001')
master.wait_heartbeat()

def send_control(pitch, roll, throttle, yaw):
    """Send manual control commands to the autopilot."""
    master.mav.manual_control_send(
        master.target_system,
        pitch,    # Pitch forward/back
        roll,     # Roll left/right
        throttle, # Throttle up/down
        yaw,      # Yaw left/right
        0)        # No buttons

def maneuver():
    print("Moving forward...")
    send_control(500, 0, 700, 0)
    time.sleep(5)

    print("Stopping...")
    send_control(0, 0, 500, 0)  # Neutral all controls to stop
    time.sleep(2)

    print("Hard turn right...")
    send_control(0, 1000, 700, 0)
    time.sleep(5)

    print("Stopping...")
    send_control(0, 0, 500, 0)
    time.sleep(2)

    print("Hard turn left...")
    send_control(0, -1000, 700, 0)
    time.sleep(5)

    print("Stopping...")
    send_control(0, 0, 500, 0)
    time.sleep(2)

    print("Spinning in a circle to the right...")
    send_control(0, 0, 700, 1000)
    time.sleep(5)

    print("Stopping...")
    send_control(0, 0, 500, 0)
    time.sleep(2)

    print("Spinning in a circle to the left...")
    send_control(0, 0, 700, -1000)
    time.sleep(5)

    print("Stopping motors completely...")
    send_control(0, 0, 500, 0)  # Neutral throttle to stop motors
    time.sleep(2)

if __name__ == "__main__":
    maneuver()
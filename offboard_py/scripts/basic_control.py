#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MAVLink Control Script for Pixhawk

This script demonstrates basic communication with a Pixhawk flight controller
using the MAVLink protocol via pymavlink.

It includes functions to connect to the Pixhawk, send RC channel override
commands, and receive telemetry data.

Usage:
    python3 mavlink_control.py

Make sure to install the required libraries:
    pip install pymavlink
"""

import sys
import time
from pymavlink import mavutil

def set_rc_channel_pwm(master, channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        master: MAVLink connection object
        channel_id (int): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return
    
    # Mavlink 2 supports up to 18 channels:
    rc_channel_values = [65535] * 18
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_channel_values)

def main():
    # Connect to the Pixhawk
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)  # Adjust port as needed

    # Wait for the first heartbeat 
    master.wait_heartbeat()
    print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

    # Example usage
    print("Setting throttle to 75%")
    set_rc_channel_pwm(master, 3, 1750)  # Channel 3 (throttle) to 75% forward

    time.sleep(2)  # Wait for 2 seconds

    print("Returning to neutral")
    set_rc_channel_pwm(master, 3, 1500)  # Return to neutral

    # To receive and print messages:
    print("Listening for messages (press Ctrl+C to exit)...")
    try:
        while True:
            msg = master.recv_match(blocking=True)
            if not msg:
                continue
            if msg.get_type() == 'SERVO_OUTPUT_RAW':
                print(f"Servo outputs: {msg.servo1_raw}, {msg.servo2_raw}, {msg.servo3_raw}, {msg.servo4_raw}")
            # Add more message types as needed
    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == '__main__':
    main()

if __name__ == '__main__':
    main()

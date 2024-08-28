#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from pymavlink import mavutil
import time

target_system_id = 1  # Replace with your system ID
target_component_id = 1  # Replace with your component ID

# Define min and max PWM values
PWM_NEUTRAL = 1500
PWM_MIN = 1100
PWM_MAX = 1900

# Define min and max cmd_vel values
CMD_VEL_MIN = -0.5
CMD_VEL_MAX = 0.5

def set_rc_channel_pwm(master, channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        master: MAVLink connection object
        channel_id (int): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 8:
        print("Channel does not exist.")
        return
   
    rc_channel_values = [65535] * 8
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_channel_values)

def cmd_vel_callback(msg, master):
    """ Callback function for cmd_vel topic
    Args:
        msg: Twist message containing velocity data
        master: MAVLink connection object
    """
    linear_vel = msg.linear.x
    angular_vel = msg.angular.z

    # Calculate motor speeds based on linear and angular velocities
    left_motor_speed = linear_vel - angular_vel
    right_motor_speed = linear_vel + angular_vel

    # Map motor speeds to PWM
    left_pwm = int((left_motor_speed - CMD_VEL_MIN) / (CMD_VEL_MAX - CMD_VEL_MIN) * (PWM_MAX - PWM_MIN) + PWM_MIN)
    right_pwm = int((right_motor_speed - CMD_VEL_MIN) / (CMD_VEL_MAX - CMD_VEL_MIN) * (PWM_MAX - PWM_MIN) + PWM_MIN)

    # Clamp PWM values to ensure they're within bounds
    left_pwm = max(PWM_MIN, min(PWM_MAX, left_pwm))
    right_pwm = max(PWM_MIN, min(PWM_MAX, right_pwm))

    print(f"Received cmd_vel: linear={linear_vel}, angular={angular_vel}")
    print(f"Setting left motor PWM: {left_pwm}, right motor PWM: {right_pwm}")

    # Set PWM values to the motors
    set_rc_channel_pwm(master, 1, left_pwm)  # Left motor on channel 1
    set_rc_channel_pwm(master, 3, right_pwm)  # Right motor on channel 3

def main():
    # Initialize the ROS node
    rospy.init_node('mavlink_cmd_vel_listener')

    # Connect to the Pixhawk
    master = mavutil.mavlink_connection('udpin:192.168.144.25:15001')
    master.target_system = target_system_id
    master.target_component = target_component_id

    # Wait for the first heartbeat
    master.wait_heartbeat()
    print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

    # Set throttle to neutral to prevent any initial spin
    print("Setting throttle to neutral (1500) to prevent initial motor spinning")
    set_rc_channel_pwm(master, 1, PWM_NEUTRAL)
    set_rc_channel_pwm(master, 3, PWM_NEUTRAL)
    time.sleep(1)  # Allow time for throttle to stabilize

    # Subscribe to cmd_vel topic
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback, master)

    # Keep the script running
    rospy.spin()

if __name__ == '__main__':
    main()

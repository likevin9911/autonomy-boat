#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from pymavlink import mavutil

# Start a connection listening on a UDP port
target_system_id = 1  # Replace with your system ID
target_component_id = 1  # Replace with your component ID



connection = mavutil.mavlink_connection('udpin:192.168.144.25:15001')
connection.target_system = target_system_id
connection.target_component = target_component_id
# Wait for the first heartbeat to set the system and component ID of remote system for the link
connection.wait_heartbeat(timeout=5)
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
print("hi")
# Define command_long_encode message to send MAV_CMD_SET_MESSAGE_INTERVAL command
# param1: MAVLINK_MSG_ID_BATTERY_STATUS (message to stream)
# param2: 1000000 (Stream interval in microseconds)
message = connection.mav.command_long_encode(
        connection.target_system,  # Target system ID
        connection.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
        0,  # Confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,  # param1: Message ID to be streamed
        1000000, # param2: Interval in microseconds
        0,       # param3 (unused)
        0,       # param4 (unused)
        0,       # param5 (unused)
        0,       # param5 (unused)
        0        # param6 (unused)
        )

# Send the COMMAND_LONG
connection.mav.send(message)

# Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result
response = connection.recv_match(type='COMMAND_ACK', blocking=True)
if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("Command accepted")
else:
    print("Command failed")
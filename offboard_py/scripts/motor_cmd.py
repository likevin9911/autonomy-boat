
#!/usr/bin/env python3

import rospy
from pymavlink import mavutil

target_system_id = 1  # Replace with your system ID
target_component_id = 1  # Replace with your component ID

# Motor PWM values for testing
PWM_TEST = 1600

class ThrusterTest:
    def __init__(self):
        # Initialize MAVLink connection
        self.master = mavutil.mavlink_connection('udpin:192.168.144.25:15001')
        self.master.target_system = target_system_id
        self.master.target_component = target_component_id

        # Wait for the first heartbeat
        self.master.wait_heartbeat()
        rospy.loginfo(f"Heartbeat from system (system {self.master.target_system} component {self.master.target_component})")

        # Start testing thrusters
        self.test_thrusters()

    def set_rc_channel_pwm(self, channel_id, pwm=1500):
        if channel_id < 1 or channel_id > 18:
            rospy.logwarn("Channel does not exist.")
            return
       
        rc_channel_values = [65535] * 18
        rc_channel_values[channel_id - 1] = pwm
        rospy.loginfo(f"Setting PWM on channel {channel_id} to {pwm}")
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_channel_values[:8])

    def test_thrusters(self):
        rospy.loginfo("Testing thrusters with constant PWM")
        self.set_rc_channel_pwm(3, PWM_TEST)  # Left motor
        self.set_rc_channel_pwm(1, PWM_TEST)  # Right motor
        print(f"Thrusters are running with PWM: {PWM_TEST} (Left: Channel 3, Right: Channel 1)")
if __name__ == '__main__':
    try:
        rospy.init_node('thruster_test', anonymous=True)
        ThrusterTest()
    except rospy.ROSInterruptException:
        pass

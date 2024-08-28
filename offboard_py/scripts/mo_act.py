#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import ActuatorControl, OverrideRCIn

def map_channel_value(value, min_input, max_input, min_output, max_output):
    # Map the channel value to the range expected by the actuator control
    return min_output + (float(value - min_input) * (max_output - min_output) / float(max_input - min_input))

def talker():
    rospy.init_node('actuator_controller', anonymous=True)

    # Publisher for actuator control
    pub_actuator = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=10)
   
    # Publisher for RC override
    pub_rc_override = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

    rate = rospy.Rate(100)

    # Initialize the ActuatorControl message
    msg_out = ActuatorControl()
    msg_out.group_mix = 2  # Use group 2 (auxiliary controls)
    msg_out.header.frame_id = 'base_link'
    msg_out.controls = [0.0] * 8  # Initialize all channels to 0.0

    # Initialize the OverrideRCIn message with 18 channels
    rc_msg = OverrideRCIn()
    rc_msg.channels = [0] * 18  # Initialize all 18 RC channels to 0

    while not rospy.is_shutdown():
        # Assuming you want to run both motors at full power (+100% or 2006)
        channel_1_value = 2006  # +100% for channel 1
        channel_3_value = 2006  # +100% for channel 3

        # Map the channel values to the actuator control range (-1.0 to 1.0)
        msg_out.controls[0] = map_channel_value(channel_1_value, 870, 2006, -1.0, 1.0)
        msg_out.controls[2] = map_channel_value(channel_3_value, 870, 2006, -1.0, 1.0)

        # Set specific channels in RC override
        rc_msg.channels[0] = 2000  # Setting channel 1 to a specific value (e.g., 2000)

        rospy.loginfo("Publishing actuator control and RC override")

        # Publish the messages
        msg_out.header.stamp = rospy.Time.now()
        pub_actuator.publish(msg_out)
        pub_rc_override.publish(rc_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

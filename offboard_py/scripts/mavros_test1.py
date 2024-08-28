#!/usr/bin/env python

import rospy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode

## USE THIS FOR ARMING AND DISARMING/ SET MODE

def arm_drone():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        arm_service(True)
        rospy.loginfo("Drone armed")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to arm drone: {e}")

def set_mode(mode):
    rospy.wait_for_service('/mavros/set_mode')
    try:
        set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        set_mode_service(0, mode)
        rospy.loginfo(f"Mode set to {mode}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to set mode: {e}")

def send_motor_commands(throttle):
    rc_override = OverrideRCIn()
    rc_override.channels[0] = throttle    # Channel 1: Roll
    rc_override.channels[1] = 0   # Channel 2: Pitch
    rc_override.channels[2] = throttle  # Channel 3: Throttle
    rc_override.channels[3] = 0     # Channel 4: Yaw
    rc_override.channels[4] = 0       # Channel 5
    rc_override.channels[5] = 0       # Channel 6
    rc_override.channels[6] = 0       # Channel 7
    rc_override.channels[7] = 0       # Channel 8
    
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
    rospy.loginfo(f"Sending RC commands: Throttle={throttle}")
    pub.publish(rc_override)

def main():
    rospy.init_node('motor_control', anonymous=True)

    # Arming the drone and setting mode
    arm_drone()
    set_mode("MANUAL")
    

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Example: send some test motor commands
        throttle = 2006
        

        send_motor_commands(throttle)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

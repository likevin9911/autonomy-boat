#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from cv_bridge import CvBridge
from std_msgs.msg import Bool

class BuoyNavigator:
    def __init__(self):
        rospy.init_node('buoy_navigator', anonymous=True)
        self.bridge = CvBridge()
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.calibration_sub = rospy.Subscriber('/calibrate_color', Bool, self.calibration_callback)

        # Color range for buoy detection (initial values, will be calibrated)
        self.lower_color = np.array([0, 0, 0])
        self.upper_color = np.array([255, 255, 255])
        
        self.is_calibrating = False
        self.current_state = None

        # PID controller parameters
        self.Kp = 0.005  # Proportional gain
        self.Ki = 0.0001  # Integral gain
        self.Kd = 0.001  # Derivative gain
        self.integral = 0
        self.prev_error = 0

        # Navigation parameters
        self.max_angular_speed = 0.5  # Maximum angular speed (rad/s)
        self.forward_speed = 0.5  # Forward speed (m/s)

        rospy.loginfo("Buoy Navigator initialized. Waiting for image data...")

    def calibration_callback(self, msg):
        self.is_calibrating = msg.data
        rospy.loginfo("Calibration mode: %s", "ON" if self.is_calibrating else "OFF")

    def state_callback(self, msg):
        self.current_state = msg

    def image_callback(self, data):
        if not self.current_state or not self.current_state.armed:
            return  # Don't process if not armed

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        if self.is_calibrating:
            self.calibrate_color(cv_image)
        else:
            self.process_image(cv_image)

    def calibrate_color(self, image):
        h, w = image.shape[:2]
        roi = image[h//3:2*h//3, w//3:2*w//3]
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        self.lower_color = np.percentile(hsv_roi, 5, axis=(0, 1))
        self.upper_color = np.percentile(hsv_roi, 95, axis=(0, 1))
        rospy.loginfo("Color calibration complete. Lower: %s, Upper: %s", self.lower_color, self.upper_color)
        self.is_calibrating = False

    def process_image(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)
        
        # Apply morphological operations to reduce noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                self.navigate_to_buoy(cx, image.shape[1])
        else:
            rospy.logwarn("No buoy detected")
            self.stop_boat()

    def navigate_to_buoy(self, buoy_x, frame_width):
        center = frame_width // 2
        error = buoy_x - center

        # PID control
        self.integral += error
        derivative = error - self.prev_error
        angular_z = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error

        # Limit the angular speed
        angular_z = max(min(angular_z, self.max_angular_speed), -self.max_angular_speed)

        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.header.frame_id = "base_link"
        twist.twist.angular.z = angular_z
        twist.twist.linear.x = self.forward_speed

        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Navigating: linear_x=%.2f, angular_z=%.2f", self.forward_speed, angular_z)

    def stop_boat(self):
        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.header.frame_id = "base_link"
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Stopping boat")

if __name__ == '__main__':
    try:
        BuoyNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
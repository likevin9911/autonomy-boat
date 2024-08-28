#!/usr/bin/env python3
import sys
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from pymavlink import mavutil

# Define the color range for a wider orange in HSV
lower_bound = np.array([5, 150, 150])
upper_bound = np.array([30, 255, 255])

# Reduced minimum area for a contour to be considered a buoy
min_contour_area = 200  # Adjusted to detect smaller, distant objects

target_system_id = 1  # Replace with your system ID
target_component_id = 1  # Replace with your component ID

# Motor PWM range
PWM_MIN = 1100
PWM_NEUTRAL = 1500
PWM_MAX = 1900

class BuoyDetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('buoy_detector', anonymous=True)
       
        # Set up the subscriber to the camera topic
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
       
        # Initialize CV bridge
        self.bridge = CvBridge()
       
        # Initialize MAVLink connection
        self.master = mavutil.mavlink_connection('udpin:192.168.144.25:15001')
        self.master.target_system = target_system_id
        self.master.target_component = target_component_id

        # Wait for the first heartbeat
        self.master.wait_heartbeat(timeout=5)
        rospy.loginfo(f"Heartbeat from system (system {self.master.target_system} component {self.master.target_component})")

        # Start the ROS loop
        self.print_motor_values_thread()
        rospy.spin()

    def set_rc_channel_pwm(self, channel_id, pwm=1500):
        """ Set RC channel PWM value
        Args:
            channel_id (int): Channel ID
            pwm (int, optional): Channel PWM value 1100-1900
        """
        if channel_id < 1 or channel_id > 18:
            rospy.logwarn("Channel does not exist.")
            return
       
        # Mavlink 2 supports up to 18 channels:
        rc_channel_values = [65535] * 18
        rc_channel_values[channel_id - 1] = pwm
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_channel_values[:8])

        # Print PWM values to terminal
        print(f"Channel {channel_id} set to PWM {pwm}")

    def detect_color(self, image, lower_bound, upper_bound):
        """Detects the specified color in an image and returns a binary mask and the centroid of the detected area."""
        # Convert the image to HSV
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
       
        # Create a mask for the specified color range
        mask = cv2.inRange(image_hsv, lower_bound, upper_bound)
       
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
       
        # Filter contours by area only
        filtered_contours = [contour for contour in contours if cv2.contourArea(contour) > min_contour_area]
       
        if filtered_contours:
            # Find the largest contour, which is likely to be the buoy
            largest_contour = max(filtered_contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                # Compute the centroid of the contour
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                return True, mask, (cX, cY)
       
        return False, mask, None

    def image_callback(self, data):
        """Callback function for image topic."""
        try:
            # Convert the ROS Image message to a CV2 image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Check if the specified color is detected with significant features
            detected, mask, centroid = self.detect_color(cv_image, lower_bound, upper_bound)

            if detected and cv2.countNonZero(mask) >= min_contour_area:
                # Significant buoy feature detected, adjust motor values
                print("Significant buoy feature detected! Adjusting motor values.")
               
                # Get the width of the frame to calculate the position of the buoy
                frame_height, frame_width = cv_image.shape[:2]
                cX, cY = centroid
               
                # Calculate the offset of the buoy from the center of the frame
                offset = cX - (frame_width // 2)
               
                # Determine motor speeds based on the offset
                if abs(offset) < frame_width * 0.1:
                    # If the buoy is near the center, move forward
                    left_motor_pwm = PWM_NEUTRAL
                    right_motor_pwm = PWM_NEUTRAL
                elif offset > 0:
                    # If the buoy is to the right, slow down the right motor
                    left_motor_pwm = PWM_NEUTRAL + 100  # Adjust as needed
                    right_motor_pwm = PWM_NEUTRAL - 100  # Adjust as needed
                else:
                    # If the buoy is to the left, slow down the left motor
                    left_motor_pwm = PWM_NEUTRAL - 100  # Adjust as needed
                    right_motor_pwm = PWM_NEUTRAL + 100  # Adjust as needed
               
                # Apply the motor values using MAVLink
                self.set_rc_channel_pwm(3, left_motor_pwm)  # Left motor
                self.set_rc_channel_pwm(1, right_motor_pwm)  # Right motor
               
                # Print the simulated motor values
                print(f"Left Motor PWM: {left_motor_pwm}, Right Motor PWM: {right_motor_pwm}")

            else:
                # No significant features detected or mask too weak
                print("No significant features detected or mask too weak.")
               
                # Stop the motors
                self.set_rc_channel_pwm(3, PWM_NEUTRAL)
                self.set_rc_channel_pwm(1, PWM_NEUTRAL)

            # Display the camera frame and the mask for visualization
            cv2.imshow("Camera Frame", cv_image)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def print_motor_values_thread(self):
        """ Continuously print motor values """
        def print_motor_values():
            """ Function to print motor values from MAVLink messages """
            print("Listening for messages (press Ctrl+C to exit)...")
            try:
                while not rospy.is_shutdown():
                    msg = self.master.recv_match(blocking=True)
                    if not msg:
                        continue
                    if msg.get_type() == 'SERVO_OUTPUT_RAW':
                        print(f"Servo outputs: {msg.servo1_raw}, {msg.servo2_raw}, {msg.servo3_raw}, {msg.servo4_raw}")
                    # Add more message types as needed
            except KeyboardInterrupt:
                print("\nExiting...")

        # Run the motor values print in a separate thread
        import threading
        thread = threading.Thread(target=print_motor_values)
        thread.daemon = True
        thread.start()

if __name__ == '__main__':
    try:
        BuoyDetector()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

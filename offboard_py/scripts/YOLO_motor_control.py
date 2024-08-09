#!/usr/bin/env python3
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from pymavlink import mavutil

def set_rc_channel_pwm(master, channel_id, pwm=1500):
    """ Set RC channel pwm value """
    rc_channel_values = [65535] * 18
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component, *rc_channel_values)

def main():
    # Initialize RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    # Initialize YOLO model
    model = YOLO('yolov8n.pt')  # or use a custom trained model

    # Connect to the Pixhawk
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)  # Adjust as needed
    master.wait_heartbeat()
    print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

    try:
        while True:
            # Wait for a coherent pair of frames
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            # Perform YOLO detection
            results = model(color_image)

            # Process results
            for result in results:
                boxes = result.boxes.cpu().numpy()
                for box in boxes:
                    r = box.xyxy[0].astype(int)
                    cv2.rectangle(color_image, (r[0], r[1]), (r[2], r[3]), (0, 255, 0), 2)
                    cv2.putText(color_image, f"{result.names[int(box.cls[0])]} {box.conf[0]:.2f}",
                                (r[0], r[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Simple motor control based on object position
                    center_x = (r[0] + r[2]) / 2
                    if center_x < 213:  # Left third of the image
                        print("Object on the left, turning left")
                        #set_rc_channel_pwm(master, 1, 1300)  # Turn left
                    elif center_x > 426:  # Right third of the image
                        print("Object on the right, turning right")
                        #set_rc_channel_pwm(master, 1, 1700)  # Turn right
                    else:
                        print("Object in the center, moving forward")
                        #set_rc_channel_pwm(master, 1, 1500)  # Center steering
                        #set_rc_channel_pwm(master, 3, 1600)  # Slight forward throttle

            # Display the image
            cv2.imshow('RealSense', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        # Reset all RC channels to neutral
        for i in range(1, 19):
            set_rc_channel_pwm(master, i, 1500)

if __name__ == "__main__":
    main()


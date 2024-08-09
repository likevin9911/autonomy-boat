#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2

def main():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    # Enable streams
    config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
    config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)

    # Start streaming
    pipeline.start(config)

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            infrared_frame_left = frames.get_infrared_frame(1)
            infrared_frame_right = frames.get_infrared_frame(2)
            if not infrared_frame_left or not infrared_frame_right:
                continue

            # Convert images to numpy arrays
            infrared_image_left = np.asanyarray(infrared_frame_left.get_data())
            infrared_image_right = np.asanyarray(infrared_frame_right.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            infrared_colormap_left = cv2.applyColorMap(cv2.convertScaleAbs(infrared_image_left, alpha=1), cv2.COLORMAP_JET)
            infrared_colormap_right = cv2.applyColorMap(cv2.convertScaleAbs(infrared_image_right, alpha=1), cv2.COLORMAP_JET)

            # Stack both images horizontally
            images = np.hstack((infrared_colormap_left, infrared_colormap_right))

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            cv2.waitKey(1)

    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

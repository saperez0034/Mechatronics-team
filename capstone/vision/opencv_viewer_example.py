import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)


# Color Range for Grape
lower_grape = np.array([150, 50, 50])
upper_grape = np.array([180, 255, 255])

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        mask = cv2.inRange(color_image, lower_grape, upper_grape)
        cv2.namedWindow('RealSense')
        cv2.imshow('RealSense', mask)
        cv2.waitKey(1)

finally:

    # Stop streaming
    pipeline.stop()

        # mask = cv2.inRange(color_image, lower_grape, upper_grape)
        # edges = cv2.Canny(mask, 100, 200)
        # edges = cv2.dilate(edges, None, iterations=2)
        # combined = cv2.bitwise_and(mask, mask, mask=edges)
        # contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # for contour in contours:
        #     area = cv2.contourArea(contour)
        #     if area > 500:  # Threshold area based on grape size
        # # Get the bounding rectangle for the grape
        #         x, y, w, h = cv2.boundingRect(contour)
        # # Draw a rectangle around the detected grape
        #         cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # # hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # # Show images
        
        # # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # # for contour in contours:
        # #     area = cv2.contourArea(contour)
        # #     if area > 100:  # Threshold area for grape size
        # #     # Draw a bounding box around the grape
        # #         x, y, w, h = cv2.boundingRect(contour)
        # #         cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # # for contour in contours:
        # #     x, y, w, h = cv2.boundingRect(contour)
        # #     if w*h > 10000 and w*h < 15000:  # Threshold area based on the grape size
        # #     # Get the bounding rectangle for the grape
        # #         cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

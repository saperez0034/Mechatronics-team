import cv2
import pyrealsense2 as rs
import numpy as np
from math import pi


def draw_target(img, x, y, w, h):
    cv2.line(img, ((x + w) / 2, (y + h) / 4), ((x + w) / 2, 3 * (y + h) / 4))
    cv2.line(img, ((x + w) / 4, (y + h) / 2), (3 * (x + w) / 4, (y + h) / 2))


def detect_and_log_grape_properties(img):
    mid_point = (None, None)
    original = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Preprocessing
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    edges = cv2.Canny(blurred, 30, 150)

    # Find contours
    contours, _ = cv2.findContours(
        edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Color range storage
    all_h = []
    all_s = []
    all_v = []

    # Shape detection parameters
    min_area = 500

    circularity_threshold = 0.7
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        # Calculate circularity
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue
        circularity = 4 * pi * area / (perimeter**2)

        if circularity > circularity_threshold:
            # Draw contour
            cv2.drawContours(original, [cnt], -1, (255, 0, 0), 2)

            # Draw Target
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.line(
                original,
                (x + int(w / 2), y + int(h / 4)),
                (x + int(w / 2), y + int(3 * h / 4)),
                255,
            )  # Vertical Line
            cv2.line(
                original,
                (x + int(w / 4), y + int(h / 2)),
                (x + int(3 * w / 4), y + int(h / 2)),
                255,
            )  # Horizontal Line
            mid_point = (x + int(w / 2), y + int(h / 2))

            # Create mask for the grape
            mask = np.zeros_like(gray)
            cv2.drawContours(mask, [cnt], -1, 255, -1)

            # Get color values within the contour

            mean_val = cv2.mean(hsv, mask=mask)
            min_val = np.min(hsv[mask == 255], axis=0)
            max_val = np.max(hsv[mask == 255], axis=0)

            # Store color values

            all_h.extend([min_val[0], max_val[0], mean_val[0]])
            all_s.extend([min_val[1], max_val[1], mean_val[1]])
            all_v.extend([min_val[2], max_val[2], mean_val[2]])
            # Calculate overall color range
            if len(all_h) > 0:
                h_range = (np.min(all_h), np.max(all_h))
                s_range = (np.min(all_s), np.max(all_s))
                v_range = (np.min(all_v), np.max(all_v))
                cv2.imshow("Detected Grapes", original)
                cv2.imshow("Edge Detection", edges)
                cv2.waitKey(1)
                return [h_range, s_range, v_range]
            else:
                return None
    # Display results
    # cv2.imshow("Detected Grapes", original)
    # cv2.imshow("Edge Detection", edges)
    # cv2.waitKey(1)
    # return mid_point


def vision_setup():
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
        if s.get_info(rs.camera_info.name) == "RGB Camera":
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)
    return pipeline


def get_color_image(pipeline):
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())
    return color_image


if __name__ == "__main__":
    pipeline = vision_setup()
    try:
        while True:
            color_image = get_color_image(pipeline)
            mid_point = detect_and_log_grape_properties(color_image)
            print(mid_point)

    finally:
        pipeline.stop()

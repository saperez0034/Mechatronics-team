import cv2
try:
    import pyrealsense2 as rs
    realSense = True
except ImportError:
    realSense = False
    print("pyrealsense2 not found. Make sure to install the RealSense SDK.")
import numpy as np
import time
from math import pi


def draw_target(img, x, y, w, h):
    cv2.line(
        img,
        (x + int(w / 2), y + int(h / 4)),
        (x + int(w / 2), y + int(3 * h / 4)),
        255,
    )  # Vertical Line
    cv2.line(
        img,
        (x + int(w / 4), y + int(h / 2)),
        (x + int(3 * w / 4), y + int(h / 2)),
        255,
    )  # Horizontal Line


def contour_check(img, contours, min_area, circularity_threshold):
    result = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue
        circularity = 4 * pi * area / (perimeter**2)
        if circularity > circularity_threshold:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.drawContours(img, [cnt], -1, (0, 255, 0), 2)
            draw_target(img, x, y, w, h)
            result.append(cnt)
    return result


def same_location(location1, location2):
    TOLERANCE = 15

    return abs(location2[0] - location1[0]) <= TOLERANCE and abs(location2[1] - location1[1]) <= TOLERANCE


def detect_stable_location(pipeline):
    img1 = get_color_image(pipeline)
    location1 = detect(img1)
    location2 = (-1, -1)
    if location1 != (-1, -1):
        time.sleep(0.3)
        img2 = get_color_image(pipeline)
        location2 = detect(img2)
    if same_location(location1, location2):
        return location1
    else:
        return (-1, -1)


def detect(img):
    original = img.copy()

    blurred = cv2.GaussianBlur(img, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)

    # lower_strawberry1 = np.array([0, 100, 80])
    # upper_strawberry1 = np.array([10, 255, 255])
    # lower_strawberry2 = np.array([160, 100, 80])
    # upper_strawberry2 = np.array([179, 255, 255])

    lower_blackberry_hsv = np.array([110, 30, 0])
    upper_blackberry_hsv = np.array([180, 255, 80])
    lower_blackberry_lab = np.array([20, 120, 120])
    upper_blackberry_lab = np.array([255, 150, 140])

    # mask_strawberry1 = cv2.inRange(hsv, lower_strawberry1, upper_strawberry1)
    # mask_strawberry2 = cv2.inRange(hsv, lower_strawberry2, upper_strawberry2)
    # mask_strawberry = cv2.bitwise_or(mask_strawberry1, mask_strawberry2)

    mask_blackberry_hsv = cv2.inRange(
        hsv, lower_blackberry_hsv, upper_blackberry_hsv)
    mask_blackberry_lab = cv2.inRange(
        hsv, lower_blackberry_lab, upper_blackberry_lab)
    mask_blackberry = cv2.bitwise_or(mask_blackberry_hsv, mask_blackberry_lab)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    # mask_strawberry = cv2.morphologyEx(
    #     mask_strawberry, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask_blackberry = cv2.morphologyEx(
        mask_blackberry, cv2.MORPH_CLOSE, kernel, iterations=2)

    # contours_strawberry, _ = cv2.findContours(
    #     mask_strawberry, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_blackberry, _ = cv2.findContours(
        mask_blackberry, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # valid_strawberry_contours = contour_check(
    #     original, contours_strawberry, 650, 0.5)
    valid_blackberry_contours = contour_check(
        original, contours_blackberry, 175, 0.5)

    largest_area = 0
    largest_point = (-1, -1)
    # mid_points.extend(midpoints_strawberry)
    # mid_points.extend(midpoints_blackberry)
    for cnt in valid_blackberry_contours:
        area = cv2.contourArea(cnt)
        if area > largest_area:
            largest_area = area
            x, y, w, h = cv2.boundingRect(cnt)
            largest_point = (x + w // 2, y + h // 2)

    # darkest_intensity = 255
    # darkest_point = (-1, -1)
    # for cnt in valid_blackberry_contours:
    #     mask = np.zeros(original.shape[:2], dtype = np.uint8)
    #     cv2.drawContours(mask, [cnt], -1, -255, -1)
    #     mean_val = cv2.mean(blurred, mask=mask)
    #     intensity = mean_val[0]

    #     if intensity < darkest_intensity:
    #         darkest_intensity = intensity
    #         x, y, w, h = cv2.boundingRect(cnt)
    #         darkest_point = (x + w // 2, y + h // 2)

    # Display the results
    try:
        cv2.imshow("Detected Fruits", original)
        cv2.imshow("Blackberry Mask", mask_blackberry)
        cv2.waitKey(1)
    except Exception as e:
        # if we run without monitors
        cv2.destroyAllWindows()

    return largest_point
    # return darkest_point
    # return mid_points


def vision_setup():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()

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
    depth_frame = frames.get_depth_frame()
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_mask = (depth_image >= 350) & (depth_image <= 407)
    filtered_color_image = np.zeros_like(color_image)
    filtered_color_image[depth_mask] = color_image[depth_mask]
    return filtered_color_image


if __name__ == "__main__":
    pipeline = vision_setup()
    try:
        while True:
            color_image = get_color_image(pipeline)
            mid_point = detect(color_image)
            print(mid_point)

    finally:
        pipeline.stop()
    image = cv2.imread(
        '/Users/perrintong/Documents/18-578/Mechatronics-team/capstone/vision/rs.png')
    # image = cv2.imread('/Users/perrintong/Desktop/fruitEw.png')
    mid_point = detect(image)
    while True:
        pass

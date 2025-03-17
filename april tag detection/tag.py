import cv2
from pupil_apriltags import Detector
import numpy as np
from scipy.spatial.transform import Rotation

detector = Detector(
    families='tag36h11',
    nthreads=4,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

mtx = np.loadtxt('april tag detection/cam_intrinsics.txt')
camera_params = (mtx[0, 0], mtx[1, 1], mtx[0, 2], mtx[1, 2])
tag_size = 0.135

needle_length = 0.05  # meters, placeholder value
gantry_depth = 0.1  # meters, placeholder value

test_img = cv2.imread('april tag detection/tag.jpg')


def main():
    img = cv2.cvtColor(test_img, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(img, estimate_tag_pose=True,
                           camera_params=camera_params, tag_size=tag_size)
    if (tags != []):
        if (tags[0].tag_id == 1):
            tvec = tags[0].pose_t
            rvec = tags[0].pose_R
            r = Rotation.from_matrix(rvec)
            euler_angles = r.as_euler('xyz', degrees=True)

            distance = np.linalg.norm(tvec)

            print(f"Translation Vector: {tvec}")
            print(f"Rotation Matrix: {rvec}")
            print(f"Distance to tag: {distance:.4f} meters")
            print(f"Euler angles (degrees): {euler_angles}")

            center = np.mean(tags[0].corners, axis=0)
            center = tuple(center.astype(int))

            # Compute the center of the bottom edge (assuming corners are ordered clockwise:
            # [top-left, top-right, bottom-right, bottom-left])
            bottom_edge_center = np.mean(tags[0].corners[2:4], axis=0)
            bottom_edge_center = tuple(bottom_edge_center.astype(int))

            # Calculate the direction vector from the center toward the bottom edge
            direction = (bottom_edge_center[0] - center[0],
                         bottom_edge_center[1] - center[1])
            norm = np.hypot(*direction)
            if norm == 0:
                norm = 1
            direction = (direction[0] / norm, direction[1] / norm)

            # Ensure the line points downward (image y increases downward)
            if direction[1] < 0:
                direction = (-direction[0], -direction[1])

            focal_length = mtx[0, 0]
            line_length = int((needle_length + tag_size/2)
                              * focal_length / (distance + gantry_depth))
            endpoint = (int(center[0] + direction[0] * line_length),
                        int(center[1] + direction[1] * line_length))
            cv2.line(test_img, center, endpoint, (0, 255, 0), 2)

            cv2.imshow('Detected Needle', test_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

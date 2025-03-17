import cv2
import numpy as np
import os

calibration_images_path = 'april tag detection/calibration_images/'


def calibrate():
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((5*7, 3), np.float32)
    objp[:, :2] = np.mgrid[0:5, 0:7].T.reshape(-1, 2)
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.
    for fname in os.listdir(calibration_images_path):
        if fname.endswith('.jpg'):
            print(fname)
            img = cv2.imread(
                calibration_images_path+fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, (5, 7), None)
            if ret is True:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)
                img = cv2.drawChessboardCorners(img, (5, 7), corners2, ret)
                cv2.imwrite(calibration_images_path + 'drawn/'+fname, img)
    _, mtx, _, _, _ = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)
    np.savetxt('april tag detection/cam_intrinsics.txt', mtx)


if __name__ == '__main__':
    calibrate()

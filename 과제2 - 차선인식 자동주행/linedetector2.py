import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


IMG_W = 640
scan_width, scan_height = 300, 80
roi_vertical_pos = 250
area_width = 5
area_height = 5
row_begin = (scan_height - area_height) // 2 - 10
row_end = row_begin + area_height
lmid, rmid = scan_width, IMG_W - scan_width
pixel_cnt_threshold = 0.6 * area_width * area_height


class LineDetector:

    def __init__(self, topic):
        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.result = np.zeros(shape=(scan_height, IMG_W, 3), dtype=np.uint8)
        self.bridge = CvBridge()
        self.direction_info = [-40, 680]
        self.img_debug = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        rospy.Subscriber(topic, Image, self.conv_image)

    def imageApplyRoi(self, image):
        return image[roi_vertical_pos:roi_vertical_pos + scan_height, :]

    def imageApplyCvt(self, image):
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    def imageApplyGaussianBlur(self, image, kernelSize):
        return cv2.GaussianBlur(image, (kernelSize, kernelSize), 0)

    def imageApplyCanny(self, image):
        return cv2.Canny(image, 50, 80)

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        img_roi = self.imageApplyRoi(self.cam_img)

        img_gray = self.imageApplyCvt(img_roi)

        img_blur = self.imageApplyGaussianBlur(img_gray, 7)

        img_canny = self.imageApplyCanny(img_blur)


        lines = cv2.HoughLines(img_canny, 1, np.pi / 180, 80, None, 0, 0)

        left = [0, 0, 0, 0]
        left_count = 0
        right = [0, 0, 0, 0]
        right_count = 0

        result = np.zeros((scan_height, IMG_W, 3), dtype=np.uint8)
        test = np.zeros((scan_height, IMG_W, 3), dtype=np.uint8)

        if lines is not None:
            for line in lines:
                for rho, theta in line:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho

                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))

                    if float(y2 - y1) / float(x2 - x1) <= -0.2:
                        left[0] += x1
                        left[1] += y1
                        left[2] += x2
                        left[3] += y2
                        left_count += 1
                        cv2.line(test, (x1, y1), (x2, y2), (255, 0, 0), 3, cv2.LINE_AA)
                    elif float(y2 - y1) / float(x2 - x1) >= 0.2:
                        right[0] += x1
                        right[1] += y1
                        right[2] += x2
                        right[3] += y2
                        right_count += 1
                        cv2.line(test, (x1, y1), (x2, y2), (0, 255, 0), 3, cv2.LINE_AA)
                    else:
                        cv2.line(test, (x1, y1), (x2, y2), (0, 0, 255), 3, cv2.LINE_AA)

        if left_count > 0:
            for i, value in enumerate(left):
                left[i] = value // left_count
        if right_count > 0:
            for i, value in enumerate(right):
                right[i] = value // right_count

        cv2.line(result, (left[0], left[1]), (left[2], left[3]), (255, 0, 0), 3, cv2.LINE_AA)
        cv2.line(result, (right[0], right[1]), (right[2], right[3]), (255, 0, 0), 3, cv2.LINE_AA)

        hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)

        lbound = np.array([0, 0, 60], dtype=np.uint8)
        ubound = np.array([131, 255, 255], dtype=np.uint8)

        bin = cv2.inRange(hsv, lbound, ubound)
        view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)

        # self.img_debug = result

        left, right = -40, 680

        for l in range(area_width, lmid):
            area = bin[row_begin:row_end, l - area_width:l]
            if cv2.countNonZero(area) > pixel_cnt_threshold:
                left = l
                break

        for r in range(IMG_W - area_width, rmid, -1):
            area = bin[row_begin:row_end, r:r + area_width]
            if cv2.countNonZero(area) > pixel_cnt_threshold:
                right = r
                break

        if left != -40:
            lsquare = cv2.rectangle(view,
                                    (left - area_width, row_begin),
                                    (left, row_end),
                                    (0, 255, 0), 3)
        else:
            pass
            # print("Lost left line")

        if right != 680:
            rsquare = cv2.rectangle(view,
                                    (right, row_begin),
                                    (right + area_width, row_end),
                                    (0, 255, 0), 3)
        else:
            pass
            # print("Lost right line")
        self.direction_info = [left, right]
        # print(left, right)

    def show_image(self):
        cv2.waitKey(1)

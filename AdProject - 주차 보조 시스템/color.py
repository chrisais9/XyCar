import numpy as np
import cv2

lower_blue = np.array([110, 50, 50])
upper_blue = np.array([130, 255, 255])

lower_red = np.array([0, 50, 50])
upper_red = np.array([10, 255, 255])


def detectRed(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    kernel = np.ones((5, 5), np.uint8)

    mask = cv2.inRange(hsv, lower_red, upper_red)

    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask, kernel, iterations=1)

    cnts, heir = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if radius > 20:
            return True

    return False

def detectBlue(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    kernel = np.ones((5, 5), np.uint8)

    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask, kernel, iterations=1)

    cnts, heir = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if radius > 15:
            return True

    return False

#!/usr/bin/env python
import cv2
from color import *
import numpy as np
import rospy, time
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

capture = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
bridge = CvBridge()


def drive(Angle, Speed):
    global ack_publisher
    drive_info = [Angle, Speed]
    drive_info = Int32MultiArray(data=drive_info)
    ack_publisher.publish(drive_info)


def park_lines(frame):
    # green
    frame = cv2.line(frame, (23, 355), (645, 355), (0, 255, 0), 3)
    frame = cv2.line(frame, (60, 331), (615, 331), (0, 255, 0), 3)
    frame = cv2.line(frame, (60, 331), (23, 355), (0, 255, 0), 3)
    frame = cv2.line(frame, (645, 355), (615, 331), (0, 255, 0), 3)
    # yellow
    frame = cv2.line(frame, (60, 331), (615, 331), (0, 255, 255), 3)
    frame = cv2.line(frame, (86, 319), (596, 319), (0, 255, 255), 3)
    frame = cv2.line(frame, (60, 331), (86, 319), (0, 255, 255), 3)
    frame = cv2.line(frame, (596, 319), (615, 331), (0, 255, 255), 3)
    # red
    frame = cv2.line(frame, (86, 319), (596, 319), (0, 0, 255), 3)
    frame = cv2.line(frame, (122, 302), (550, 302), (0, 0, 255), 3)
    frame = cv2.line(frame, (86, 319), (122, 302), (0, 0, 255), 3)
    frame = cv2.line(frame, (550, 302), (596, 319), (0, 0, 255), 3)

    return frame


def conv_image(data):
    global capture
    capture = bridge.imgmsg_to_cv2(data, "bgr8")

def wheel_reset():
    for i in range(2):
        drive(90, 90)
        time.sleep(0.1)
        drive(90, 60)
        time.sleep(0.1)


rospy.init_node("ad")
ack_publisher = rospy.Publisher("xycar_motor_msg", Int32MultiArray, queue_size=1)
rospy.Subscriber('/usb_cam/image_raw', Image, conv_image)

angle = 90
speed = 90
park = False
before = False
starttime = 0
nowtime = 0
prev_key =0

flag = False
while True:
    frame = capture
    if park:
        frame = park_lines(frame)
    cv2.imshow('img', frame)
    if detectRed(frame) and not park:
        park = True

    blue = detectBlue(frame)
    if nowtime != 0:
        print("{} Won".format(int(nowtime - starttime) * 10))
        flag = True
    if blue != before:
        if starttime == 0:
            starttime = time.time()
        else:
            nowtime = time.time()
        before = blue
    key = cv2.waitKey(1)
    if key & 0xff == 27:
        break
    if 81 <= key <= 84:
        if key == 82:
            speed = 120
        if key == 81:
            if angle - 10 >= 40:
                angle -= 10
        if key == 83:
            if angle + 10 <= 150:
                angle += 10
        if key == 84:
            #if prev_key != 84:
                # wheel_reset()
            speed = 65
    else:
        speed = 90
    drive(angle, speed)
    if key == 112:
        park = False
    prev_key = key


cv2.destroyAllWindows()
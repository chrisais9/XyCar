#!/usr/bin/env python

import rospy, time

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver

class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')

    def trace(self):
        obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        left, right = self.line_detector.direction_info
        # self.line_detector.show_image()

        angle = self.steer(left, right)
        speed = self.accelerate(angle, obs_l, obs_m, obs_r)
        self.driver.drive(angle + 90, speed + 90)

    def steer(self, left, right):
        mid = (left + right) // 2
        angle = (mid - 320) // 1.8
        angle = max(-30, angle) if angle > 0 else min(30, angle)
        return angle

    def accelerate(self, angle, left, mid, right):
        # if min(left, mid, right) < 50:
        #     speed = 0

        #turn
        if angle < -20 or angle > 20:
            speed = 20
        else:
            speed = 20
        return speed

    def exit(self):
        print('finished')

if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(4)
    rate = rospy.Rate(8)
    while not rospy.is_shutdown():
        car.trace()
    rospy.on_shutdown(car.exit)
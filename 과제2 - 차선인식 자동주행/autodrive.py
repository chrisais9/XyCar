#!/usr/bin/env python

import rospy, time
import math
import time

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver


class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')
        self.slow_time = time.time()
        self.prev_dis = 100
        self.prev_l = []
        self.prev_m = []
        self.prev_r = []
        self.MAX_SIZE = 3

    def average(self, L):
        if len(L) == 0:
            return 100
        return sum(L) / len(L)

    def trace(self):
        obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        line_theta, left, right = self.line_detector.detect_lines()
        angle = self.steer(line_theta)
        speed = self.accelerate(line_theta)
        print(line_theta)
        print("left: {} mid: {} right: {}".format(obs_l, obs_m, obs_r))
        cnt = 0
        s = 0
        cos_theta = 0.9
        dis = self.prev_dis
        obs_l *= cos_theta
        obs_r *= cos_theta
        if obs_l != 0:
            if len(self.prev_l) + 1 >= self.MAX_SIZE:
                self.prev_l.pop(0)
            self.prev_l.append(obs_l)
            if obs_l <= 70:
                cnt += 1
            s += obs_l
        if obs_m != 0:
            if len(self.prev_m) + 1 >= self.MAX_SIZE:
                self.prev_m.pop(0)
            self.prev_m.append(obs_m)
            if obs_m <= 70:
                cnt += 2
            s += obs_m
        if obs_r != 0:
            if len(self.prev_r) + 1 >= self.MAX_SIZE:
                self.prev_r.pop(0)
            self.prev_r.append(obs_r)
            if obs_r <= 70:
                cnt += 1
            s += obs_r
        if cnt >= 2:
            dis = s / cnt
        start_time = time.time()
        if ((cnt >= 3) or (obs_m != 0 and obs_m <= 70 and self.average(self.prev_m) <= 75)) or (
                self.average(self.prev_l) <= 75 and self.average(self.prev_m) <= 75 and self.average(
                self.prev_r) <= 75):
            if time.time() - start_time > 50:
                for i in range(2):
                    self.driver.drive(90, 90)
                    time.sleep(0.1)
                    self.driver.drive(90, 60)
                    time.sleep(0.1)
                self.driver.drive(90, 90)
                time.sleep(5)
        else:
            self.driver.drive(angle + 90 + 5.7, speed)

        if cnt >= 2:
            self.prev_dis = dis

    def steer(self, theta):
        threshold = 2.0
        if -0.8 <= theta <= 0.8:
            threshold = 8.0
        elif -5 <= theta <= 5:
            threshold = 3.0
        elif -10 <= theta <= 10:
            threshold = 2.3
        elif -15 <= theta <= 15:
            if theta < 0:
                threshold = 2.0
            else:
                threshold = 2.5
        else:
            if theta < 0:
                threshold = 2.5
            else:
                threshold = 3.5
        angle = theta * threshold

        return angle

    def accelerate(self, theta):
        K = 135

        if abs(theta) > 4:
            self.slow_time = time.time() + 2

        if time.time() < self.slow_time:
            K = 130

        speed = K  # - min(abs(theta)/2, 10)

        return speed

    def exit(self):
        print('finished')


if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)

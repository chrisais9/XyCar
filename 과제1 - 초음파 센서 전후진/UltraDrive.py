#!/usr/bin/env python

import rospy, time
from std_msgs.msg import Int32MultiArray

motor_pub = None
usonic_data = None


def init_node():
    global motor_pub
    rospy.init_node('sample')
    rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
    motor_pub = rospy.Publisher('xycar_motor_msg',
                                Int32MultiArray, queue_size=1)


def exit_node():
    print('finished')


def drive(angle, speed):
    print("drive {} {}".format(angle, speed))
    global motor_pub
    drive_info = [angle, speed]
    pub_data = Int32MultiArray(data=drive_info)
    motor_pub.publish(pub_data)


def callback(data):
    global usonic_data
    usonic_data = data.data


def wheel_fix():
    for i in range(0, 2):
        drive(master_angle, 90)
        time.sleep(0.1)
        drive(master_angle, 60)
        time.sleep(0.1)


if __name__ == '__main__':
    master_angle = 88
    init_node()
    time.sleep(3)
    x = 0
    level = 1
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print("front: {}    back: {}".format(usonic_data[1], usonic_data[4]))
        if level == 1:
            print("level 1")
            if x == 0:
                print("front")
                if usonic_data[1] <= 40.0:
                    print("1")
                    drive(master_angle, 90)
                    x = 1
                    time.sleep(3)
                    wheel_fix()

                else:
                    print("2")
                    drive(85, 110)
            elif x == 1:
                print("back")
                if usonic_data[4] <= 40.0:
                    print("3")
                    drive(master_angle, 90)
                    x = 0
                    time.sleep(3)
                    level += 1
                else:
                    print("4")
                    drive(master_angle, 75)
        if level == 2:
            print("level 2")
            if x == 0:
                print("front")
                if usonic_data[1] <= 45.0:
                    print("1")
                    drive(master_angle, 90)
                    x = 1
                    time.sleep(3)
                    wheel_fix()
                elif usonic_data[1] <= 65.0:
                    drive(master_angle, 110)
                else:
                    print("2")
                    drive(master_angle, 118)
            elif x == 1:
                print("back")
                if usonic_data[4] <= 40.0:
                    print("3")
                    drive(master_angle, 90)
                    x = 0
                    time.sleep(3)
                    level += 1
                else:
                    print("4")
                    drive(master_angle, 75)
        if level == 3:
            print("level 3")
            if x == 0:
                print("front")
                if usonic_data[1] <= 40.0:
                    print("5")
                    drive(master_angle, 90)
                    x = 1
                    time.sleep(3)
                    wheel_fix()
                elif usonic_data[1] <= 60.0:
                    print("4")
                    drive(master_angle, 105)
                elif usonic_data[1] <= 70.0:
                    print("3")
                    drive(master_angle, 110)
                elif usonic_data[1] <= 80.0:
                    print("2")
                    drive(master_angle, 115)

                else:
                    print("1")
                    drive(master_angle, 125)
            elif x == 1:
                print("back")
                if usonic_data[4] <= 40.0:
                    print("3")
                    drive(master_angle, 90)
                    x = 0
                    time.sleep(3)
                    level += 1
                else:
                    print("4")
                    drive(master_angle, 75)

        rate.sleep()
    rospy.on_shutdown(exit_node)


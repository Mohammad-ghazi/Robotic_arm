#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16


def main():
    rospy.init_node("servo_node")

    servo_pubs = []
    for i in range(1, 7):
        servo_pub = rospy.Publisher("servo{}_angle".format(i), UInt16, queue_size=1000)
        servo_pubs.append(servo_pub)

    rate = rospy.Rate(10)

    while True:
        servo_num = input("Enter servo number (1-6) or press 'Q' to quit: ")

        if servo_num.upper() == 'Q':
            break

        try:
            servo_num = int(servo_num)
            if servo_num < 1 or servo_num > 6:
                raise ValueError
        except ValueError:
            print("Invalid input. Servo number must be between 1 and 6.")
            continue

        angle_input = input("Enter the angle for servo {} (0-180) or press 'Q' to quit: ".format(servo_num))

        if angle_input.upper() == 'Q':
            break

        try:
            angle = int(angle_input)
            if angle < 0 or angle > 180:
                raise ValueError
        except ValueError:
            print("Invalid input. Angle must be between 0 and 180.")
            continue

        servo_pub = servo_pubs[servo_num - 1]
        servo_pub.publish(angle)
        rospy.loginfo("Angle %d published for servo %d.", angle, servo_num)

        rate.sleep()

    rospy.loginfo("Servo control completed.")


if __name__ == '__main__':
    main()

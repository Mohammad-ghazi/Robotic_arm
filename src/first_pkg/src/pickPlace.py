#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16


def main():
    rospy.init_node("pick_and_place_robot")

    servo_pubs = {}
    for i in range(1, 7):
        topic_name = "servo{}_angle".format(i)
        servo_pub = rospy.Publisher(topic_name, UInt16, queue_size=1000)
        servo_pubs[i] = servo_pub

    rate = rospy.Rate(10)

    move_to_point_A(servo_pubs)  # Move to Point A
    rospy.sleep(5)  # Wait for 5 seconds
    move_to_point_B(servo_pubs)  # Move to Point B
    rospy.sleep(5)  # Wait for 5 seconds
    move_to_point_A(servo_pubs)  # Move back to Point A

    rospy.loginfo("Pick and Place Robot Control Completed.")


def move_to_point_A(servo_pubs):
    angles = [0, 180, 180, 0, 0, 180]  # Angles for Point A position
    # move_servos(servo_pubs, angles)
    rospy.loginfo("Moved to Point A.")


def move_to_point_B(servo_pubs):
    angles = [90, 180, 40, 90, 180, 60]  # Angles for Point B position
    # move_servos(servo_pubs, angles)
    rospy.loginfo("Moved to Point B.")


# def move_servos(servo_pubs, angles):
#     if len(angles) != 6:
#         rospy.logwarn("Invalid angles. There should be 6 angles for 6 servos.")
#         return

#     for i, angle in enumerate(angles, start=1):
#         if i in servo_pubs:
#             servo_pub = servo_pubs[i]
#             servo_pub.publish(angle)
#             rospy.loginfo("Angle %d published for servo %d.", angle, i)
#         else:
#             rospy.logwarn("Invalid servo number: %d", i)


if __name__ == '__main__':
    main()

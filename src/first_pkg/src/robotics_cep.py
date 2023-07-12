#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16


def main():
    rospy.init_node("servo_node")

    servo1_pub = rospy.Publisher("servo1_angle", UInt16, queue_size=1000)
    servo2_pub = rospy.Publisher("servo2_angle", UInt16, queue_size=1000)
    servo3_pub = rospy.Publisher("servo3_angle", UInt16, queue_size=1000)
    servo4_pub = rospy.Publisher("servo4_angle", UInt16, queue_size=1000)
    servo5_pub = rospy.Publisher("servo5_angle", UInt16, queue_size=1000)
    servo6_pub = rospy.Publisher("servo6_angle", UInt16, queue_size=1000)

    rate = rospy.Rate(10)

    # initial position
    angles = [0, 180, 180, 0, 0, 180]
    servo1_pub.publish(angles[0])
    rospy.sleep(1)
    servo2_pub.publish(angles[1])
    rospy.sleep(1)
    servo3_pub.publish(angles[2])
    rospy.sleep(1)
    servo4_pub.publish(angles[3])
    rospy.sleep(1)
    servo5_pub.publish(angles[4])
    rospy.sleep(1)
    servo6_pub.publish(angles[5])
    rospy.sleep(1)

    # pick
    # [90, 180, 180, 90, 180, 60]
    pick = [90, 90, 180, 60]
    servo1_pub.publish(pick[0])
    rospy.sleep(1)
    servo4_pub.publish(pick[1])
    rospy.sleep(1)
    servo5_pub.publish(pick[2])
    rospy.sleep(1)
    servo6_pub.publish(pick[3])
    rospy.sleep(1)

    # lift
    servo4_pub.publish(angles[3])
    rospy.sleep(1)
    servo6_pub.publish(angles[5])
    rospy.sleep(1)

    # place
    place = [180, 90, 180, 180]
    servo1_pub.publish(place[0])
    rospy.sleep(1)
    servo4_pub.publish(place[1])
    rospy.sleep(1)
    servo5_pub.publish(place[2])
    rospy.sleep(1)
    servo6_pub.publish(place[3])

    rospy.loginfo("Servo control completed.")

if __name__ == '__main__':
    main()

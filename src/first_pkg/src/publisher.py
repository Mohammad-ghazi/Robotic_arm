import math
import time
import cvzone
from cvzone.FaceMeshModule import FaceMeshDetector
import mediapipe as mp
import cv2
import serial
import struct
import sys

# Set up serial communication
#ser1 = serial.Serial('com13', 115200)# for shoulder control
ser2 = serial.Serial('com5', 115200)# for arm control
# import numpy as np
# def inv_kin(le, wd):
#     pi = math.pi
#     value = le/wd
#     point = math.asin(value)*(180/pi)
#     print(point)
# Define parallel lengths, perpendicular height, and diagonal length of the trapezium
a = 30  # short side (length of forearm)
# d = 50 # long side ( distance between the camera and the chest of the patient)
h = 30  # height  ( length of the patient's chest)
d = 30  # diagonal (length of the upper arm bicep)


def midpoint(r1, r2, lx1, lx2):
    mdx = ((r1 + lx1) / 2)
    mdy = ((r2 + lx2) / 2)
    keypoint_r = [r1, r2]
    keypoint_l = [lx1, lx2]
    dist = math.sqrt(((keypoint_r[0] - keypoint_l[0]) ** 2) + ((keypoint_r[1] - keypoint_l[1]) ** 2))
    # print(dist)
    return dist, int(mdx), int(mdy)


def put_point(m1, m2):
    point1 = (m1, m2)
    cv2.putText(frame, str(point1), point1, cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)
    cv2.circle(frame, point1, 5, (0, 255, 0), -1)
    # print(point1)
    return point1


detector = FaceMeshDetector(maxFaces=1)
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
cap = cv2.VideoCapture(0)
pose = mp_pose.Pose()
send_p1 = True
send_p2 = True
send_p3 = True
send_p4 = True
send_p5 = True
send_p6 = True
# Declare a flag variable
yesSent = False
# Initialize variables for sending angleX and p
send_angleX = True
start_time = time.time()

while True:
    success, img = cap.read()
    img, faces = detector.findFaceMesh(img, draw=False)
    if faces:
        face = faces[0]
        pointLeft = face[145]
        pointRight = face[374]
        # Drawing
        # cv2.line(img, pointLeft, pointRight, (0, 200, 0), 3)
        # cv2.circle(img, pointLeft, 5, (255, 0, 255), cv2.FILLED)
        # cv2.circle(img, pointRight, 5, (255, 0, 255), cv2.FILLED)
        w, _ = detector.findDistance(pointLeft, pointRight)
        W = 6.3

        # # # Finding the Focal Length
        # d = 60
        # f = (w*d)/W
        # print(f)

        # Finding distance
        f = 690
        b = (W * f) / w
        # print(b)

        cvzone.putTextRect(img, f'Depth: {int(b)}cm',
                           (face[10][0] - 100, face[10][1] - 50),
                           scale=2)

        x = b - a
        # Calculate the angles of the trapezium
        angle1 = math.degrees(math.atan(h / x))
        angle2 = 180 - (90.0 + angle1)
        angle3 = 90.0
        angle4 = 90.0
        angleX = int(((90 - angle1)*10)/2)  # this angle is the angle needed to move the bicep

        min_angle = 0  # Minimum angle in the original range
        max_angle = 90  # Maximum angle in the original range



        # Print the angles in degrees
        # print(f"Angle 1: {angle1:.2f} degrees")
        # print(f"Angle 2: {angle2:.2f} degrees")
        # print(f"Angle 3: {angle3:.2f} degrees")
        # print(f"Angle 4: {angle4:.2f} degrees")
        # print(f"Angle X: {angleX:.2f} degrees")
        # Send the angleX value to the Arduino using serial communication
        # if send_angleX:
        #
        #     angleX_str = str(float(angleX * 10))
        #     mapped_value = int((angleX - min_angle) * (480 - 0) / (max_angle - min_angle) + 0)
        #     print(mapped_value)
        #     ser1.write(angleX_str.encode())
        #     ser1.write("\n".encode())
        #     print(angleX_str)
        #     if time.time() - start_time >= 50:
        #         send_angleX = False  # Stop sending angleX after 10 seconds
        #         ser1.write(b'stop')
        #
        #         # Close the serial connection
        #         ser1.close()

    ret, frame = cap.read()
    h, w, c = frame.shape
    # cv2.resize = (frame, (int(h*0.5), int(w*0.5)))
    frame = cv2.flip(frame, 1)
    # image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(frame)
    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark
        Right_shoulder = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
        Left_shoulder = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value]
        mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        for me in Left_shoulder, Right_shoulder:
            lx, ly = int(Left_shoulder.x * w), int(Left_shoulder.y * h)
            # print(lx, ly)
            rx, ry = int(Right_shoulder.x * w), int(Right_shoulder.y * h)
            # print(rx, ry)
            # x = int(me.x * frame.shape[1]+50)
            # y = int(me.y * frame.shape[0]+50)
            # keypoint1.append((rx, ry))
            cv2.circle(frame, (rx, ry), 5, (0, 255, 0), -1)
            # keypoint2.append((lx, ly))
            cv2.circle(frame, (lx, ly), 5, (0, 255, 0), -1)

        distance, mid_point_x, mid_point_y = midpoint(rx, ry, lx, ly)
        p = []
        # i = 0
        # x = [2]
        # print(rx)
        # print(ry)

        if 300 <= distance <= 400:
            w1, l1 = put_point(mid_point_x + 30, mid_point_y + 30)
            p1 = (w1, l1)
            p.append((w1, l1))
            # print("P1")
            # print(p1)
            # print(width, length)
            # inv_kin(width, length)

            w2, l2 = put_point(mid_point_x - 70, mid_point_y + 150)
            p2 = (w2, l2)
            p.append((w2, l2))
            # print("P2")
            # print(p2)
            # print(width, length)
            # inv_kin(width, length)
            w3, l3 = put_point(mid_point_x - 40, mid_point_y + 30)
            p3 = (w3, l3)
            p.append((w3, l3))
            # print("P3")
            # print(p3)
            # print(width, length)
            # inv_kin(width, length)
            w4, l4 = put_point(mid_point_x - 40, mid_point_y + 70)
            p4 = (w4, l4)
            p.append((w4, l4))
            # print("P4")
            # print(p4)
            # print(width, length)
            # inv_kin(width, length)
            w5, l5 = put_point(mid_point_x - 40, mid_point_y + 110)
            p5 = (w5, l5)
            p.append((w5, l5))
            # print("P5")
            # print(p5)
            # print(width, length)
            # inv_kin(width, length)
            w6, l6 = put_point(mid_point_x - 40, mid_point_y + 150)
            p6 = (w6, l6)
            p.append((w6, l6))
            # print("P6")
            # print(p6)
            # print(width, length)
            # inv_kin(width, length)

        elif 200 <= distance <= 300:
            w1, l1 = put_point(mid_point_x + 25, mid_point_y + 25)
            p1 = (w1, l1)
            p.append((w1, l1))
            # print("P1")
            # print(p1)
            # print(width, length)
            # inv_kin(width, length)

            w2, l2 = put_point(mid_point_x - 50, mid_point_y + 115)
            p2 = (w2, l2)
            p.append((w2, l2))
            # print("P2")
            # print(p2)
            # print(width, length)
            # inv_kin(width, length)
            w3, l3 = put_point(mid_point_x - 25, mid_point_y + 25)
            p3 = (w3, l3)
            p.append((w3, l3))
            # print("P3")
            # print(p3)
            # print(width, length)
            # inv_kin(width, length)
            w4, l4 = put_point(mid_point_x - 25, mid_point_y + 55)
            p4 = (w4, l4)
            p.append((w4, l4))
            # print("P4")
            # print(p4)
            # print(width, length)
            # inv_kin(width, length)
            w5, l5 = put_point(mid_point_x - 25, mid_point_y + 85)
            p5 = (w5, l5)
            p.append((w5, l5))
            # print("P5")
            # print(p5)
            # print(width, length)
            # inv_kin(width, length)
            w6, l6 = put_point(mid_point_x - 25, mid_point_y + 115)
            p6 = (w6, l6)
            p.append((w6, l6))
            # print("P6")
            # print(p6)
            # print(width, length)
            # inv_kin(width, length)

        if send_p1:
            print("performing P1")
            print(p1)
            print(angleX)
            ser2.write(str(angleX).encode() + ",".encode() + str(p1[1]).encode() + ",".encode() + str(p1[0]).encode() + "\n".encode())
        if 10 <= time.time() - start_time < 12: # 2secs
            print("performing auscultations")
            ser2.write("stop\n".encode())
            send_p1 = False
            ser2.write("yes\n".encode())
            yesSent = True
            print("yes")

        if 32 <= time.time() - start_time < 42: # 10secs
            print("send next point in")
            yesSent = False
            ser2.flush()

            if send_p2:
                print("performing P2")
                print(p2)
                print(angleX)
                ser2.write(str(angleX).encode() + ",".encode() + str(p2[1]).encode() + ",".encode() + str(
                    p2[0]).encode() + "\n".encode())
            if 42 <= time.time() - start_time < 44:  # 2secs
                print("performing P2 auscultations")
                ser2.write("stop\n".encode())
                send_p2 = False
                ser2.write("yes\n".encode())
                yesSent = True
                print("yes")

            if 62 <= time.time() - start_time < 64:  # 10secs
                print("send next point in")
                ser2.flush()
                yesSent = False

    cv2.imshow("live feed", frame)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
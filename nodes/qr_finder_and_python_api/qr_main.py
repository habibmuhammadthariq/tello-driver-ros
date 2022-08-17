#!/usr/bin/env python2
import traceback
import av
import cv2
import numpy as np
import math
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import CompressedImage

from video_stream_modul import StandaloneVideoStream

import update_find_contours as qr_finder

# import simple_find_contours as qr_finder

stream = StandaloneVideoStream()

# pid value
pid_yaw = [0.005, 0, 0.08]
pid_pitch = [0.005, 0, 0.05]
pid_roll = [0.003, 0, 0.02]

"""
# for image record
fps = 6
resolution = (640, 480)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
# name of this file output should be changed every time this program launched
save = cv2.VideoWriter('videos/test.mp4', fourcc, fps, resolution)
"""


def set_command(direction, distance, error):
    fb, lr, ud = 0, 0, 0
    #
    # speed = pid_yaw[0] * error + pid_yaw[2] * (error - pError)
    # speed = np.clip(speed, -1.0, 1.0)

    if direction == 'hover':
        fb = 0
        lr = 0
    elif direction == 'left':
        lr = pid_roll[0] * error + pid_roll[2] * (error - pError_roll)
        lr = np.clip(lr, -0.2, -0.25)
    elif direction == 'right':
        lr = pid_roll[0] * error + pid_roll[2] * (error - pError_roll)
        lr = np.clip(lr, 0.2, 0.25)
    elif direction == 'up':
        ud = 0.8
    elif direction == 'down':
        ud = -0.4
    elif direction == 'forward':
        fb = pid_pitch[0] * error + pid_pitch[2] * (error - pError_pitch)
        fb = np.clip(fb, 0.2, 0.4)
    elif direction == 'backward':
        fb = pid_pitch[0] * error + pid_pitch[2] * (error - pError_pitch)
        fb = np.clip(fb, -0.2, -0.4)

    # rospy.loginfo('{} : {} || {}'.format(direction, fb, lr))

    if pDistance != 0 and (math.fabs(distance - pDistance) > 30):
        print('Detection failed -> Hover')  # temporary
        move()
    elif direction == 'left' or direction == 'right' or direction == 'forward' or direction == 'backward' or direction == "up" or direction == "down":
        move(lr=lr, fb=fb, ud=ud)
    else:
        move()

    print('Distance : {}, y : {}, x : {}, z : {}'.format(distance, fb, lr, ud))


def callback(msg):
    stream.add_frame(msg.data)


def takeoff():
    rospy.logwarn('Taking Off')

    pub_takeoff.publish(takeoff_msg)
    rospy.sleep(1)

    # move(ud=1.1)
    # rospy.sleep(1)

    move()


def move(lr=0, fb=0, ud=0):
    cmd_vel.linear.y = fb
    cmd_vel.linear.x = lr
    cmd_vel.linear.z = ud
    pub_vel.publish(cmd_vel)


def find_missing_qrcode(count_direction, first, seconds, pDirection='hover', lr=0, fb=0, ud=0):
    if pDirection == 'up' or pDirection == 'down':
        if count_direction < 40:
            move(lr * first, fb * first, ud * first)
        else:  # if 80 <= count_direction < 160:
            move(lr * seconds, fb * seconds, ud * seconds)
    else:
        if count_direction < 100:
            move(lr * first, fb * first, ud * first)
        else:  # if 80 <= count_direction < 160:
            move(lr * seconds, fb * seconds, ud * seconds)


def main():
    # previous
    global pError_roll, pError_pitch, pDistance, pDirection, count_direction
    pDirection, pDistance = '', 0.0
    pError_roll, pError_pitch = 0.0, 0.0
    #
    container = av.open(stream)
    rospy.loginfo('main: opened')

    i = 0  # ,  j = 0, 0
    count_direction = 0
    # up, down = False, False
    hasTakenoff, ever_found, just_take_off, hover_for_a_while = False, False, False, False
    start = 0
    for frame in container.decode(video=0):
        # image = cv2.resize(image, (360, 240))

        # update i value
        i += 1
        if i % 50 == 0 and i <= 300:  # 400:
            print('Iterasi ke : {}'.format(i))
        if 401 < i <= 402:
            takeoff()  # ====== takeoff command =====
            hasTakenoff, just_take_off = True, True

        # image with detected qr code, the qr_code detected or not
        if hasTakenoff:
            # if j < 10:
            #     j += 1
            #     continue
            # else:
            image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
            image = cv2.resize(image, (640, 480))
            frm, found = qr_finder.extract(image, True)
        else:
            continue

        if found:
            ever_found, just_take_off, hover_for_a_while = True, False, False
            # getting error between qr center with frame center
            # error = qr_finder.get_error()

            # direction for the drone to come into qr code.
            direction, distance, error = qr_finder.get_direction()
            print(direction)  # temporary

            # print the direction and distance on the frame
            cv2.putText(frm, "distance : {}".format(distance), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 255), 1,
                        cv2.LINE_AA)
            cv2.putText(frm, "direction : {}".format(direction), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 255), 1,
                        cv2.LINE_AA)

            if direction == 'hover':
                rospy.loginfo('Drone going to hover and decode the qr code')
                move()

                # decoding qr code
                qr_data = qr_finder.qr_code_decoder(image)
                rospy.loginfo('QR Data is : {}'.format(qr_data))

                # stop looping
                rospy.loginfo('Mission Completed!')
                break
            else:
                # set command
                set_command(direction, distance, error)

            # update pError
            if direction == 'left' or direction == 'right':
                pError_roll = error
            elif direction == 'forward' or direction == 'backward':
                pError_pitch = error

            # update previous distance - direction and count_direction
            pDistance = distance
            pDirection = direction
            count_direction = 0

            # print('p error roll : {}, p error pitch : {}, p error distance : {}, p error direction : {}'.
            #       format(pError_roll, pError_pitch, pDistance, pDirection))
        else:
            rospy.logwarn("Where's qr code")  # temporary
            if hover_for_a_while:
                rospy.logwarn("Time to hover")  # temporary
                # let the drone hover while finding qr code till 15 secs.
                # If not found. then land
                move()
                now = rospy.get_rostime().secs
                if (now - start) / 60.0 == 0.25:  # 15 seconds
                    rospy.loginfo("Mission Failed!")
                    break
            elif ever_found:  # qr code lost
                rospy.logwarn("Find missing qr code")  # temporary
                if pDirection == 'up' or pDirection == 'down' and count_direction == 80:
                    hover_for_a_while = True
                    start = rospy.get_rostime().secs
                elif count_direction == 200:
                    hover_for_a_while = True
                    start = rospy.get_rostime().secs

                if pDirection == 'left':
                    find_missing_qrcode(count_direction, -1, 1, lr=0.25)  # left 20 times then right 20 times
                elif pDirection == 'right':
                    find_missing_qrcode(count_direction, 1, 1, lr=0.25)

                elif pDirection == 'up':  # need to adjust up and down value, because it's different
                    find_missing_qrcode(count_direction, 1, -1, pDirection, ud=0.8)  # up 20 times then down 20 times
                elif pDirection == 'down':
                    find_missing_qrcode(count_direction, -1, 1, pDirection, ud=0.4)

                elif pDirection == 'forward':  # backward 20 times then forward 20 times
                    find_missing_qrcode(count_direction, -1, 1, fb=0.3)
                elif pDirection == 'backward':
                    find_missing_qrcode(count_direction, 1, -1, fb=0.3)

                count_direction += 1
            elif just_take_off:
                rospy.logwarn("Start to find qr code")  # temporary

                if count_direction < 400:       # right 200 times then left 400 times
                    find_missing_qrcode(count_direction, 1, -1, lr=0.25)
                elif count_direction < 480:     # up            80 times   - final
                    move(ud=0.8)
                elif count_direction < 680:     # move right    200 times
                    move(lr=0.25)
                elif count_direction < 840:     # down          160 times       down should 3 times from up
                    move(ud=-0.4)
                # elif count_direction < 1040:    # move left     200 times
                #     move(lr=-0.25)
                # elif count_direction < 1120:    # up            80 times
                #     move(ud=0.8)
                elif count_direction == 1120:
                    hover_for_a_while = True
                    start = rospy.get_rostime().secs

                count_direction += 1
            else:  # follow instruction by qr code      ---     next job
                pass

        # j = 0

        cv2.imshow('Output', frm)
        # cv2.imshow('Edged', edged)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.loginfo("Mission Failed!")
            break


if __name__ == '__main__':
    #
    rospy.init_node('qr_code_detector')
    rospy.Subscriber("/tello/image_raw/h264", CompressedImage, callback)
    pub_takeoff = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
    pub_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
    pub_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)

    takeoff_msg = Empty()
    land_msg = Empty()
    cmd_vel = Twist()

    try:
        main()
    except BaseException:
        traceback.print_exc()
    finally:
        # hover then land
        move()
        rospy.logwarn('Landing')
        pub_land.publish(land_msg)
        stream.close()
        cv2.destroyAllWindows()

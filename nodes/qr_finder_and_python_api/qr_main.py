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

from datetime import  datetime

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
        # print('Detection failed -> Hover')  # temporary
        move()
    elif direction == 'left' or direction == 'right' or direction == 'forward' or direction == 'backward' or direction == "up" or direction == "down":
        move(lr=lr, fb=fb, ud=ud)
    else:
        move()

    # print('Distance : {}, y : {}, x : {}, z : {}'.format(distance, fb, lr, ud))


def callback(msg):
    stream.add_frame(msg.data)


def takeoff():
    rospy.logwarn('Taking Off')

    pub_takeoff.publish(takeoff_msg)
    rospy.sleep(1)

    # move(ud=1.1)
    # rospy.sleep(1)

    move()


def move(lr=0, fb=0, ud=0, rotate=0):
    cmd_vel.linear.y = fb
    cmd_vel.linear.x = lr
    cmd_vel.linear.z = ud
    cmd_vel.angular.z = rotate
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


def missing_function(pDirection, count_direction, direction):
    if pDirection == 'left':
        find_missing_qrcode(count_direction, -1, 1, lr=0.25)  # left 20 times then right 20 times
        direction = 'right'
    elif pDirection == 'right':
        find_missing_qrcode(count_direction, 1, -1, lr=0.25)
        direction = 'left'
    elif pDirection == 'up':  # need to adjust up and down value, because it's different
        find_missing_qrcode(count_direction, 1, -1, pDirection, ud=0.8)  # up 20 times then down 20 times
        direction = 'down'
    elif pDirection == 'down':
        find_missing_qrcode(count_direction, -1, 1, pDirection, ud=0.4)
        direction = 'up'
    elif pDirection == 'forward':  # backward 20 times then forward 20 times
        find_missing_qrcode(count_direction, -1, 1, fb=0.3)
        direction = 'backward'
    elif pDirection == 'backward':
        find_missing_qrcode(count_direction, 1, -1, fb=0.3)
        direction = 'forward'
    return direction


# def find_function():
#     return ''


def main():
    # previous
    global pError_roll, pError_pitch, pDistance, pDirection, count_direction
    pDirection, pDistance = '', 0.0
    pError_roll, pError_pitch = 0.0, 0.0
    #
    container = av.open(stream)
    rospy.loginfo('main: opened')

    i = 0  # ,  j = 0, 0
    count_direction, count_missing_qr_code, count_rotate = 0, 0, 0
    direction = ''
    # up, down = False, False
    hasTakenoff, ever_found, just_take_off, hover_for_a_while = False, False, False, False
    start = 0
    for frame in container.decode(video=0):
        # image = cv2.resize(image, (360, 240))

        # update i value
        i += 1
        # if i % 50 == 0 and i <= 300:  # 400:
        #     print('Iterasi ke : {}'.format(i))
        if 401 < i <= 402:
            takeoff()                  # ====== takeoff command =====
            hasTakenoff, just_take_off = True, True

        # image with detected qr code, the qr_code detected or not
        if hasTakenoff:
            # if j < 10:
            #     j += 1
            #     continue
            # else:
            start_image_processing = datetime.now()
            image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
            # rospy.loginfo("actual dimension of this image are {}".format(image.shape))
            # image = cv2.resize(image, (640, 480))
            frm, found = qr_finder.extract(image, True)
            # temp
            diff = (datetime.now() - start_image_processing).microseconds / 1000
            # if found:     # calculate duration
            #     rospy.loginfo("duration : {} milliseconds".format(diff))
        else:
            continue

        if found:
            ever_found, just_take_off, hover_for_a_while = True, False, False

            # direction for the drone to come into qr code.
            qr_position, direction, distance, error = qr_finder.get_direction()
            # print(direction)  # temporary

            # print the direction and distance on the frame
            cv2.putText(frm, "distance : {}".format(distance), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 255), 1,
                        cv2.LINE_AA)
            # cv2.putText(frm, "qr position : {}".format(qr_position), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
            #             (0, 255, 255), 1,
            #             cv2.LINE_AA)
            cv2.putText(frm, "direction : {}".format(direction), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 255), 1,
                        cv2.LINE_AA)

            rospy.loginfo("qr position : {}, direction : {}".format(qr_position, direction))

            if direction == 'hover':
                # rospy.loginfo('Drone going to hover and decode the qr code')
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
            count_missing_qr_code = 0
            count_rotate = 0

            # print('p error roll : {}, p error pitch : {}, p error distance : {}, p error direction : {}'.
            #       format(pError_roll, pError_pitch, pDistance, pDirection))
        else:
            if count_missing_qr_code < 5:
                move()
                count_missing_qr_code += 1
                continue

            # """
            # rospy.logwarn("Where's qr code")  # temporary
            if hover_for_a_while:
                # let the drone hover while finding qr code till 15 secs. If not found. then land
                move()
                direction = 'hover'
                now = rospy.get_rostime().secs
                if (now - start) / 60.0 == 0.25:  # 15 seconds
                    rospy.loginfo("Mission Failed!")
                    break

            elif ever_found:  # qr code lost
                # rospy.logwarn("Find missing qr code")  # temporary
                if pDirection == 'up' or pDirection == 'down' and count_direction == 80:
                    hover_for_a_while = True
                    start = rospy.get_rostime().secs
                elif count_direction == 200:
                    hover_for_a_while = True
                    start = rospy.get_rostime().secs

                direction = missing_function(pDirection, count_direction, direction)
                count_direction += 1

            elif just_take_off:
                # rospy.logwarn("Start to find qr code {}".format(count_direction))  # temporary
                if count_rotate == 0:
                    rospy.sleep(3)

                count_rotate += 1
                if count_rotate < 150:
                    move()
                else :
                    direction = 'kanan'
                    move(rotate=0.4)  # important 640x480=0.6
                    # need to change to be (-0.6) if the qr code position was in the left side of the drone

                if count_rotate == 1430: # 640x480 = 790
                    hover_for_a_while = True
                    start = rospy.get_rostime().secs
                    count_direction = 0
                    count_rotate = 0

                count_direction += 1
            else:  # follow instruction by qr code      ---     next job
                pass
            # """
            rospy.loginfo("qr position : {}, direction : {}".format('lost', direction))

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

"""
old find qr-code code
if count_direction < 60:
    move(ud=0.8)
    direction = 'up'
elif count_direction < 260:
    move(fb=0.25)
    direction = 'forward'
elif count_direction < 800:  # right 200 times then left 400 times
    find_missing_qrcode(count_direction, 1, -1, lr=0.25)
    direction = 'right'
elif count_direction < 880:  # up            80 times   - final
    move(ud=0.8)
    direction = 'up'
elif count_direction < 1080:  # move right    200 times
    move(lr=0.25)
    direction = 'right'
elif count_direction < 1480:  # move left     400 times
    move(lr=-0.25)
    direction = 'left'
elif count_direction < 1640:  # down          160 times       down should 3 times higher from up
    move(ud=-0.4)
    direction = 'down'
# elif count_direction < 1520:    # up            80 times
#     move(ud=0.8)
elif count_direction == 1640:
    hover_for_a_while = True
    start = rospy.get_rostime().secs
    count_direction = 0
"""

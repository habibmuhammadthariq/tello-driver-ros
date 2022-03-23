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

# import modul as qr_finder

stream = StandaloneVideoStream()

pid_yaw = [0.005, 0, 0.08]
pError, pDistance = 0.0, 0.0


def get_command(direction, distance, error):
    fb, ud, hover = 0, False, False

    speed = pid_yaw[0] * error + pid_yaw[2] * (error - pError)
    speed = np.clip(speed, -1.0, 1.0)

    if direction == 'up' or direction == 'down':
        ud = True
    elif direction == 'hover':
        fb = 0                  # previously
        # fb, speed = 0, 0
        hover = True
    elif direction == 'forward':
        fb = 0.1
    elif direction == 'backward':
        fb = -0.1
    print(fb, speed)

    if pDistance != 0 and (math.fabs(distance - pDistance) > 30):
        print('Detection failed -> Hover')  # temporary
        fb, speed = 0, 0

    return [fb, speed, hover]


def callback(msg):
    # rospy.loginfo('frame: %d bytes' % len(msg.data))
    stream.add_frame(msg.data)


def takeoff():
    rospy.logwarn('Taking Off')
    # cmd_vel.linear.z = 0.75
    # pub_vel.publish(cmd_vel)
    # rospy.sleep(3)
    #
    # cmd_vel.linear.z = 0
    # pub_vel.publish(cmd_vel)

    # pub_takeoff.publish(takeoff_msg)
    # rospy.sleep(1)

    cmd_vel.linear.y = 0
    cmd_vel.angular.z = 0
    cmd_vel.linear.z = 0.9
    pub_vel.publish(cmd_vel)
    rospy.sleep(1)

    cmd_vel.linear.y = 0
    cmd_vel.angular.z = 0
    cmd_vel.linear.z = 0
    pub_vel.publish(cmd_vel)


def main():
    global pError, pDistance
    pError, pDistance = 0, 0
    #
    container = av.open(stream)
    rospy.loginfo('main: opened')

    i = 0
    last_hover = 0
    isTakingOff = False
    counter, pDirection = 0, ''
    for frame in container.decode(video=0):
        image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)     # height 720, width 960
        image = cv2.resize(image, (640, 480))

        # update i value
        i += 1
        if i % 100 == 0 and i <= 1000:
            print('Iterasi ke : {}'.format(i))
        if 901 < i <= 902:
            takeoff()
            isTakingOff = True

        # image with detected qr code, the qr_code detected or not
        # frm, edged, status = qr_finder.extract(image, True)  # qr code
        if isTakingOff:
            frm, status = qr_finder.extract(image)        # blue circle
        else:
            continue

        if status:
            # getting error between qr center with frame center
            error = qr_finder.get_error()
            # direction for the drone to come into qr code.
            direction, distance = qr_finder.get_direction()
            print(direction)  # temporary

            if len(pDirection) == 0:
                pDirection = direction
                counter += 1
                continue
            else:
                if direction == pDirection:
                    counter += 1

            if counter == 2:
                # send command to the drone
                fb, speed, isHover = get_command(direction, distance, error)
                cmd_vel.linear.y = fb
                cmd_vel.angular.z = speed
                pub_vel.publish(cmd_vel)

                # decode qr code
                if isHover:
                    current_time = rospy.get_rostime().secs
                    if last_hover == 0:
                        last_hover = current_time

                    if (current_time-last_hover)/60.0 > 0.16:   # 1 seconds. 0.05 = 3 seconds, 0.1 = 6 seconds
                        qr_data = qr_finder.qr_code_decoder(image)
                        rospy.loginfo(qr_data)
                        # stop the iterations then land
                        break
                    # temp
                    print('yey akhirnya hover')
                else:
                    last_hover = 0

                counter = 0
                print('Maju woy')
            else:
                cmd_vel.linear.y = 0
                cmd_vel.angular.z = 0
                pub_vel.publish(cmd_vel)

            # update pError
            pError = error
            pDistance = distance
        else:
            cmd_vel.linear.y = 0
            cmd_vel.angular.z = 0
            pub_vel.publish(cmd_vel)
            # pass  # 3

        cv2.imshow('Output', frm)
        # cv2.imshow('Edged', edged)
        if cv2.waitKey(1) & 0xFF == ord('q'):
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
        rospy.loginfo('Mission Completed!')
        # hover for a 1/2 s then land
        cmd_vel.linear.y = 0
        cmd_vel.angular.z = 0
        pub_vel.publish(cmd_vel)
        # rospy.sleep(1)
        rospy.logwarn('Landing')
        pub_land.publish(land_msg)
        # rospy.sleep(1)
        stream.close()
        cv2.destroyAllWindows()

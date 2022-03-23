import math
import cv2
import numpy as np
import simple_find_contours as qr_finder
from djitellopy import tello
import time

# cap = cv2.VideoCapture(0)     # 1

drone = tello.Tello()
drone.connect()
print(f'Battery : {drone.get_battery()}')

# start stream
drone.streamon()
frame_read = drone.get_frame_read()

# global pError
# global pDistance
pid_yaw = [0.2, 0, 0.4]
pid_pitch = [0.4, 0, 0.8]
pid_roll = [0.06, 0, 0.9]

pError_pitch = 0.0
pError_roll = 0.0

pDistance = 0.0
pDirection = ''
count_directionF = 0
count_directionB = 0 #
count_directionL = 0
count_directionR = 0 #

print('Taking off')                          # b - 3 rows below     -> '
drone.takeoff()                              #
# time.sleep(1)
drone.send_rc_control(0, 0, 30, 0)
time.sleep(2)


def trackQrCode(direction, distance, error):
    fb, lr, ud = 0, 0, False
    fb_p, lr_p = 0, 0   # temporary
    if direction == 'hover':
        fb = 0
        lr = 0
    elif direction == 'left' or direction == 'right':
        lr_p = pid_roll[0] * error + pid_roll[2] * (error - pError_roll)
        lr = int(np.clip(lr_p, -10, 10))
    elif direction == 'forward':
        fb_p = pid_pitch[0] * error + pid_pitch[2] * (error - pError_pitch)
        fb = int(np.clip(fb_p, -10, 10))
    elif direction == 'backward':
        fb_p = pid_pitch[0] * error + pid_pitch[2] * (error - pError_pitch)
        fb = int(np.clip(fb_p, -10, 10))

    print('{} : {} || {}'.format(direction, fb, lr))

    # send_rc_control was command based on velocity. so it's cm/s.
    # the time determine by the sleep. time.sleep(1) mean 1 seconds.
    # example : 20 -> 20 cm in 1 seconds.
    # So the drone will move forward 20 cm on 1 seconds
    if pDistance != 0 and (math.fabs(distance - pDistance) > 30):
        print('Detection failed -> Hover')  # temporary
        drone.send_rc_control(0, 0, 0, 0)               # d - 7 rows below    -> '
    elif direction == 'left' or direction == 'right':
        drone.send_rc_control(lr, fb, 0, 0)
    elif direction == 'forward' or direction == 'backward':
        drone.send_rc_control(lr, fb, 0, 0)
    else:
        drone.send_rc_control(0, 0, 0, 0)


def land():
    print('Mission Completed!')
    drone.send_rc_control(0, 0, 0, 0)               # g - 2 row below   -> '
    # time.sleep(0.2)
    drone.land()
    drone.streamoff()
    cv2.destroyAllWindows()


while True:
    # _, img = cap.read()               # 2
    img = frame_read.frame  # e
    img = cv2.resize(img, (640, 480))  # (360, 240))

    # image with detected qr code, the qr_code detected or not
    # frame, status, edged = qr_finder.extract(img, True)     # with edged detected
    frame, status = qr_finder.extract(img, True)

    if status:
        # getting error between qr center with frame center
        # error = qr_finder.get_error()

        # direction for the drone to come into qr code.
        direction, distance, error = qr_finder.get_direction()

        # get command for the drone
        if direction == 'hover':
            drone.send_rc_control(0, 0, 0, 0)
            qr_data = qr_finder.qr_code_decoder(img)
            print('QR Data is : {}'.format(qr_data))

            # mission accomplish then land
            land()
            break
        else:
            # send command to the drone
            trackQrCode(direction, distance, error)
            # print('Direction : {}, Distance : {} '.format(direction, distance))  # temporary

        # update pError, pDistance and pDirection
        if direction == 'left' or direction == 'right':
            pError_roll = error
        elif direction == 'forward' or direction == 'backward':
            pError_pitch = error
        # else:
        #     pError_roll = 0.0
        #     pError_pitch = 0.0
        pDistance = distance
        pDirection = direction

        # update count_direction for L, R and F
        count_directionL = 0
        count_directionR = 0
        count_directionF = 0
    else:
        if pDirection == 'forward':                 # f (if and else)        -> '
            if count_directionF < 5:
                print('Move Forward')
                drone.send_rc_control(0, -10, 0, 0)
                time.sleep(0.1)
                count_directionF += 1
            else:
                print('Move Backward')
                drone.send_rc_control(0, 10, 0, 0)
                time.sleep(0.1)
                if count_directionF == 9:
                    count_directionF = 0
        # elif pDirection == 'left':
        #     if count_directionL == 0:
        #         print('Move Right')
        #         drone.send_rc_control(10, 0, 0, 0)
        #         time.sleep(0.5)
        #         count_directionL += 1
        #     else:
        #         print('Move Left')
        #         drone.send_rc_control(-10, 0, 0, 0)
        #         time.sleep(1)
        #         count_directionL = 0
        # elif pDirection == 'right':
        #     if count_directionR == 0:
        #         print('Move Left')
        #         drone.send_rc_control(-10, 0, 0, 0)
        #         time.sleep(0.5)
        #         count_directionR += 1
        #     else:
        #         print('Move Right')
        #         drone.send_rc_control(10, 0, 0, 0)
        #         time.sleep(1)
        #         count_directionR = 0
        else:
            drone.send_rc_control(0, 0, 0, 0)
        # pass  # 3

    # cv2.imshow('Edged', edged)
    cv2.imshow('Output', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        # cv2.imwrite("../img/qr_code.jpg", img)
        land()
        break

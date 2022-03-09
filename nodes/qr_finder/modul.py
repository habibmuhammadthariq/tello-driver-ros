#!/usr/bin/env python2
from json.tool import main
import math
import cv2
import numpy as np
from pyzbar import pyzbar

# error tolerance
Y_ERROR_TOLERANCE = 50


def distance_to_camera():
    # distance = 30 cm, known width = 19.6 cm, marker width in frame = 424  -> laptop
    # distance = 30 cm, known width = 19.6 cm, marker width in frame = 215  -> tello
    # focal length = (marker_width_in_the_frame * known_distance) / image_width

    # initialize focal length
    # focal_length = 670.8860759493671  # laptop camera
    focal_length = 329.0816326530612  # tello camera in frame size of 360,240

    # initialize the real known width
    known_width = 19.6  # cm

    return (known_width * focal_length) // contour_width


def get_direction():  # this function need both image_center and qr_center
    direction = ""
    # get distance between drone with the qr code
    distance = distance_to_camera()
    # print("distance : %s" % distance)

    # # up and down
    # if circle_center[1] < (image_center[1] - Y_ERROR_TOLERANCE):
    #     direction = "up"
    # elif circle_center[1] > (image_center[1] + Y_ERROR_TOLERANCE):
    #     direction = "down"
    # forward and backward
    # elif 55 > distance > 45:  # with up and down
    if 60 > distance > 50:
        direction = "hover"
    elif distance > 60:
        direction = "forward"
    elif distance < 50:
        direction = "backward"
    # temporary
    # print(f'Image center : {image_center[1]}, circle center : {circle_center[1]}')
    """
    else:
        # if image_center[0] - 50 < qr_center[0] < image_center[0] + 50 \
        #         and image_center[1] - 50 < qr_center[1] < image_center[1] + 50:
            # direction = "hover" # or forward
        if image_center[0] > qr_center[0] + 50:
            direction = "left"
        elif image_center[0] < qr_center[0] - 50:
            direction = "right"
        elif image_center[1] > qr_center[1] + 50:
            direction = "up"
        elif image_center[1] < qr_center[1] - 50:
            direction = "down"
        else:
            direction = "hover"
    """
    # print the direction and distance on the frame
    cv2.putText(output, "distance : {}".format(distance), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 1,
                cv2.LINE_AA)
    cv2.putText(output, "direction : {}".format(direction), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 1,
                cv2.LINE_AA)
    return direction, distance


def get_error():
    # error = 0
    direction = get_direction()
    # getting error between contour center with frame center
    if direction == 'up':
        error = (image_center[1]+10) - circle_center[1]
    elif direction == 'down':
        error = (image_center[1]-10) - circle_center[1]
    else:
        # direction is equals to hover, forward and backward
        error = circle_center[0] - image_center[0]

    # temporary
    # error = qr_center[0] - image_center[0]
    return error



def extract(frame):
    global output
    output = frame.copy()
    # get center image and draw solid circle at it
    global image_center
    height, width, channel = output.shape
    image_center = width // 2, height // 2
    cv2.circle(output, image_center, 5, (0, 255, 255), cv2.FILLED)

    # RGB to HSV
    img_hsv = cv2.cvtColor(output, cv2.COLOR_BGR2HSV)

    ##range value of blue color
    # lower = np.array([90, 60, 120])           # for laptop
    # upper = np.array([155, 255, 255])
    # lower = np.array([0, 90, 0])  # for tello
    # upper = np.array([80, 255, 255])
    lower = np.array([110, 50, 0])  # for tello
    upper = np.array([140, 255, 255])
    # masking image
    mask = cv2.inRange(img_hsv, lower, upper)
    # print out detail of masked image
    # print("[Mask Image] dimension : {}, number of channel : {}".format(mask.shape, mask.ndim))

    # find contours
    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # print("Number of contours found : {}".format(len(contours)))

    # Initializie variable for the centroid circle and is the circle detected or not
    global circle_center
    detected = False

    if len(contours) > 0:
        # get the biggest contours
        cnt = max(contours, key=cv2.contourArea)
        # draw contours
        cv2.drawContours(output, cnt, -1, (0, 255, 0), 3)

        # find x and y coordinate, width and height of that contour
        x, y, w, h = cv2.boundingRect(cnt)
        global contour_width, contour_height
        contour_width, contour_height = w, h
        # temporary
        # print(f'Width : {w}, Height : {h}')
        # draw rectangle
        cv2.rectangle(output, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # calculate centroid of the contour with cv2.moments
        M = cv2.moments(cnt)
        # if m["m00"] == 0: m["m00", "m01"] = 1
        if M["m00"] != 0 and cv2.contourArea(cnt) > 500:  # minimum area 5 is equals to approximately 2.5 m
            # switch the detected variable into true.
            detected = True
            # get circle center
            circle_center = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
        else:
            circle_center = 0, 0
        # draw solid circle on that centroid
        cv2.circle(output, circle_center, 7, (0, 255, 0), -1)
        # put text above those solid circle
        cv2.putText(output, "object centroid", (circle_center[0] - 20, circle_center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

        # temporary
        # area = cv2.contourArea(cnt)
        # print(area)
        # show up Error y value
        # cv2.putText(output, f'error y : {math.fabs(image_center[1] - circle_center[1])}', (20, height - 20),
        # cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return output, detected

from json.tool import main
import math
import cv2
import numpy as np
from pyzbar import pyzbar

BLUR_VALUE = 3
SQUARE_TOLERANCE = 0.15
AREA_TOLERANCE = 0.3    # previously 0.15
DISTANCE_TOLERANCE = 0.25
WARP_DIM = 300
SMALL_DIM = 29

# global variable
# global qr_center, image_center


def count_children(hierarchy, parent, inner=False):
    if parent == -1:
        return 0
    elif not inner:
        return count_children(hierarchy, hierarchy[parent][2], True)
    return 1 + count_children(hierarchy, hierarchy[parent][0], True) + count_children(hierarchy, hierarchy[parent][2],
                                                                                      True)


def has_square_parent(hierarchy, squares, parent):
    if hierarchy[parent][3] == -1:
        return False
    if hierarchy[parent][3] in squares:
        return True
    return has_square_parent(hierarchy, squares, hierarchy[parent][3])


def get_center(c):
    m = cv2.moments(c)
    # if m["m00"] == 0: m["m00", "m01"] = 1
    return [int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])]


def get_angle(p1, p2):
    x_diff = p2[0] - p1[0]
    y_diff = p2[1] - p1[1]
    return math.degrees(math.atan2(y_diff, x_diff))


def extend(a, b, length, int_represent=False):
    length_ab = math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
    if length_ab * length <= 0:
        return b
    result = [b[0] + (b[0] - a[0]) / length_ab * length, b[1] + (b[1] - a[1]) / length_ab * length]
    if int_represent:
        return [int(result[0]), int(result[1])]
    else:
        return result


def get_box(center1, center2, contour1, contour2):      # east_box, south_box, east, south
    peri1 = int(cv2.arcLength(contour1, True) / 4 / 2)      # jari-jari
    peri2 = int(cv2.arcLength(contour2, True) / 4 / 2)      # jari-jari
    center1 = int(center1[0]), int(center1[1])
    center2 = int(center2[0]), int(center2[1])

    # if center1[0] < center2[0] and center1[1] > center2[1]:
    #     left = center1[0] - peri1, center1[1] + peri1
    #     right = center2[0] + peri2, center2[1] - peri2
    # else:
    #     left = center2[0] - peri2, center2[1] + peri2
    #     right = center1[0] + peri1, center1[1] - peri1
    left = center2[0]-peri2, center2[1]+peri2
    right = center1[0]+peri1, center1[1]-peri1

    qr_cntr = int(left[0] + math.fabs(right[0] - left[0]) / 2), int(left[1] - math.fabs(left[1] - right[1]) / 2)

    return [left, right, qr_cntr]


def distance_to_camera():
    # distance = 40 cm, known width = 15.8 cm, marker width in frame = 132 -> focal length : 334.1772151898734

    # distance = 50 cm, known width = 19.8 cm, marker width in frame = 138/139/140 -> focal length : 334.1772151898734 (320, 240)
    # distance = 50 cm, known width = 19.8 cm, marker width in frame = 239/240 -> focal length : 603.5353535353535 (640, 480)
    # focal length = (marker_width_in_the_frame * known_distance) / known_width

    # initialize focal length
    # focal_length = 670.8860759493671  # laptop camera
    focal_length = 631.3131313131313  # tello camera

    # initialize the real known width
    known_width = 19.8  # cm

    return (known_width * focal_length) // contour_width


def get_direction(): # this function need both image_center and qr_center
    direction = ""
    error = 0
    # get distance between drone with the qr code
    distance = distance_to_camera()
    # print("distance : %s" % distance)

    # up and down
    # if image_center[1] > (qr_center[1] + 50):
    #     direction = "up"
    # elif image_center[1] < (qr_center[1] - 50):
    #     direction = "down"

    # left and right
    if qr_center[0] < image_width//3:
        direction = 'left'
        error = qr_center[0] - image_center[0]
    elif qr_center[0] > image_width*2/3:
        direction = 'right'
        error = qr_center[0] - image_center[0]
    # forward and backward
    # if 60 > distance > 50:        # without up and down
    elif 60 > distance > 50:
        direction = 'hover'
        error = 0
    elif distance > 60:
        direction = 'forward'
        error = distance - 50
    elif distance < 50:
        direction = 'backward'
        error = distance - 60
    """
    # forward and backward
    if 45 > distance > 35:        # without up and down
        direction = "hover"
    elif distance > 45:
        direction = "forward"
    elif distance < 35:
        direction = "backward"
    """
    # print the direction and distance on the frame
    cv2.putText(output, "distance : {}".format(distance), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 1,
                cv2.LINE_AA)
    cv2.putText(output, "direction : {}".format(direction), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 1,
                cv2.LINE_AA)
    return direction, distance, error


def get_error():
    error = 0
    direction = get_direction()
    # getting error between contour center with frame center
    if direction == 'up' or direction == 'down':
        error = image_center[1] - qr_center[1]
    else:
        if qr_center[0] < image_width//3 or qr_center[0] > image_width*2/3:
            # direction is equals to hover, forward and backward
            error = qr_center[0] - image_center[0]

    # temporary
    # error = qr_center[0] - image_center[0]

    return error


def qr_code_decoder(img):
    qr_data = ''
    qr_codes = pyzbar.decode(img)

    for qr_code in qr_codes:
        qr_data = qr_code.data.decode('utf-8')

    return qr_data


def extract(frame, debug=False):
    global output
    output = frame.copy()
    # get center image and draw solid circle at it
    global image_center
    global image_height, image_width
    image_height, image_width, channel = output.shape
    image_center = image_width // 2, image_height // 2
    cv2.circle(output, image_center, 5, (0, 255, 255), cv2.FILLED)

    # Remove noise and unnecessary contours from frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.bilateralFilter(gray, 11, 17, 17)
    gray = cv2.GaussianBlur(gray, (BLUR_VALUE, BLUR_VALUE), 0)
    edged = cv2.Canny(gray, 30, 200)

    _, contours, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    squares = []
    square_indices = []

    # count = 0
    i = 0
    detected = False # true if the contour get caught
    for c in contours:
        # Approximate the contour
        peri = cv2.arcLength(c, True)
        area = cv2.contourArea(c)
        approx = cv2.approxPolyDP(c, 0.03 * peri, True)

        # Find all quadrilateral contours
        if len(approx) == 4:
            # Determine if quadrilateral is a square to within SQUARE_TOLERANCE
            if area > 25 and\
                    1 - SQUARE_TOLERANCE < math.fabs((peri / 4) ** 2) / area < 1 + SQUARE_TOLERANCE and\
                    count_children(hierarchy[0], i) >= 2 and\
                    has_square_parent(hierarchy[0], square_indices, i) is False:
                squares.append(approx)
                square_indices.append(i)

        i += 1
    # print('Jumlah contour yang ditemukan : {}'.format(len(squares)))
    main_corners = []
    east_corners = []
    south_corners = []
    # rectangles = []
    # Determine if squares are QR codes
    for square in squares:
        area = cv2.contourArea(square)
        center = get_center(square)
        # peri = cv2.arcLength(square, True)  # keliling objek

        similar = []
        # tiny =
        # temporary
        # count = 1
        # print('area ke {} : {}'.format(count, area))
        # bagian perulangan ini bisa di buang sebenarnya. karena hanya ngecek luas squarenya lebih dari area toleransi ga nih.
        for other in squares:
            if square[0][0][0] != other[0][0][0]:  # bentuk lain dari indeks di dalam kontur
                # temporary
                # count += 1
                # print('area ke {} : {}'.format(count, cv2.contourArea(other)))
                # Determine if square is similar to other square within AREA_TOLERANCE
                if math.fabs(area - cv2.contourArea(other)) / max(area, cv2.contourArea(other)) <= AREA_TOLERANCE:
                    similar.append(other)


        if len(similar) >= 2:
            distances = []
            distances_to_contours = {}
            for sim in similar:
                sim_center = get_center(sim)
                d = math.hypot(sim_center[0] - center[0], sim_center[1] - center[1])
                distances.append(d)
                distances_to_contours[d] = sim
                # math.hypot -> jarak dari dua titik koordinat
            distances = sorted(distances)
            closest_a = distances[-1]  # indeks terakhir
            closest_b = distances[-2]  # indeks ke dua dari akhir

            temp = math.fabs(closest_a - closest_b)                 # temporary
            # print('Selisish jarak antara FIP east and south adalah : {}'.format(temp))
            # Determine if this square is the top left QR code indicator... 2.5 (previously) not 0.25 ...
            if max(closest_a, closest_b) < cv2.arcLength(square, True) * 2.5 and temp <= 50: # temp / max(closest_a, closest_b) <= DISTANCE_TOLERANCE: #
                # Determine placement of other indicators
                angle_a = get_angle(get_center(distances_to_contours[closest_a]), center)
                angle_b = get_angle(get_center(distances_to_contours[closest_b]), center)

                # temp
                # print('Angle a : {}, angle b : {}'.format(angle_a, angle_b))

                # dari kiri ke kanan bagian atas mulai 0 - 180 derajat. sebaliknya 0 - (-179) derajat
                if (angle_a > 160 or angle_a < -160) and -110 < angle_b < -70:          # A
                    east = distances_to_contours[closest_a]
                    south = distances_to_contours[closest_b]
                    main = square
                    detected = True
                    # print('Ini adalah yang A')
                elif -110 < angle_a < -70 and (angle_b > 160 or angle_b < -160):        # another A
                    east = distances_to_contours[closest_b]
                    south = distances_to_contours[closest_a]
                    main = square
                    detected = True
                    # print('Ini adalah another A bro')
                elif -65 < angle_a < -25 and -20 < angle_b < 20:        # C
                    east = square
                    south = distances_to_contours[closest_a]
                    main = distances_to_contours[closest_b]
                    detected = True
                    # print('Ini adalah yang C')
                elif 125 < angle_a < 165 and 70 < angle_b < 110:        # B
                    east = distances_to_contours[closest_a]
                    south = square
                    main = distances_to_contours[closest_b]
                    detected = True
                    # print('Ini adalah yang B')

                if detected:

                    # temporary
                    tx, ty = get_center(main)
                    main_box = (tx, ty)
                    tx, ty = get_center(east)
                    east_box = (tx, ty)
                    tx, ty = get_center(south)
                    south_box = (tx, ty)

                    # determine the x, y and center of the qr code
                    global qr_center
                    left, right, qr_center = get_box(east_box, south_box, east, south)    # original
                    # left, right, qr_center = get_box(south_box, east_box, south, east)

                    east_corners.append(east)
                    south_corners.append(south)
                    main_corners.append(square)

                    # print a text
                    cv2.putText(output, "Main", main_box, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
                    cv2.putText(output, "East", east_box, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
                    cv2.putText(output, "South", south_box, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
                    # draw qr code details
                    cv2.rectangle(output, left, right, (255, 255, 255), 2)
                    cv2.circle(output, qr_center, 7, (255, 255, 255), cv2.FILLED)
                    cv2.putText(output, "Center", qr_center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

                    # width and height contour
                    global contour_width, contour_height
                    contour_width = math.fabs(left[0] - right[0])
                    contour_height = math.fabs(left[1] - right[1])
                    # print('qr code width : {}'.format(contour_width))
                # print("Width : {}, height : {}".format(contour_width, contour_height))

                    # qr code detected then stop this looping
                    # print ('success then stop')
                    break
                    # print('Stop looping because of the qr code has been found')

    # print(type(np.array(main_corners)))
    # print(len(squares))
    if debug:
        # Draw debug information onto frame before outputting it
        cv2.drawContours(output, squares, -1, (255, 255, 255), 4)  # (5, 5, 5), 2)
        # cv2.drawContours(output, main_corners, -1, (0, 0, 128), 3)
        # cv2.drawContours(output, east_corners, -1, (0, 128, 0), 3)
        # cv2.drawContours(output, south_corners, -1, (128, 0, 0), 3)

    # return output, detected, edged        # with edged detected
    return output, detected


"""
# note

get_direction bagian x, blum menggunakan yang pembagian sbgaimana pada modul ini. toleransinya masih 50 bukan 0. ...

another option to create rectangle over qr codes:
rect = cv2.minAreaRect()
box = cv2.cv.boxPoints(rect)
box = np.int0(box)
cv2.drawContours(image, [box], 0, (color), thickness)
"""

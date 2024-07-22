import cv2
import numpy as np
from cmath import sqrt
import threading
from copy import copy
import math

##############
### Vsiual ###
##############
def get_blackboard_blocks(info_blocks, goal_us, goal_them):
   #info_blocks = {'green': [(441, 319), (357, 118)], 'orange': [(284, 161), (209, 120)], 'blue': [(116, 308), (514, 67)]}
   blocks = {}
   for i in range(len(info_blocks['green'])):
      x, y  = info_blocks['green'][i][0], info_blocks['green'][i][1]
      blocks[str((x, y))] = [x, y, 1, inRectangle([x, y],  goal_us), inRectangle([x, y], goal_them)]

   for i in range(len(info_blocks['orange'])):
      x, y  = info_blocks['orange'][i][0], info_blocks['orange'][i][1]
      blocks[str((x, y))] = [x, y, 2, inRectangle([x, y],  goal_us), inRectangle([x, y],goal_them)]

   for i in range(len(info_blocks['blue'])):
      x, y  = info_blocks['blue'][i][0], info_blocks['blue'][i][1]
      blocks[str((x, y))] = [x, y, 3, inRectangle([x, y],  goal_us), inRectangle([x, y],goal_them)]

   return blocks


def isSquare(approx, min_side=25, tol=0.1):
    res = True
    if len(approx) == 4:
        x_1 = approx[0][0][0]
        y_1 = approx[0][0][1]
        x_2 = approx[1][0][0]
        y_2 = approx[1][0][1]
        x_3 = approx[2][0][0]
        y_3 = approx[2][0][1]
        x_4 = approx[3][0][0]
        y_4 = approx[3][0][1]

        l_1 = ((x_1-x_2)**2+(y_1-y_2)**2)**0.5
        l_2 = ((x_2-x_3)**2+(y_2-y_3)**2)**0.5
        l_3 = ((x_3-x_4)**2+(y_3-y_4)**2)**0.5
        l_4 = ((x_4-x_1)**2+(y_4-y_1)**2)**0.5
        l = [l_1, l_2, l_3, l_4]
        for i in range(3):
            r = l[i]/l[i+1]
            if r > 1+tol or r < 1-tol:
                res = False
        for i in range(4):
            if l[i] < min_side:
                res = False

        l_13 = ((x_1-x_3)**2+(y_1-y_3)**2)
        l_24 = ((x_2-x_4)**2+(y_2-y_4)**2)
        if l_13 == 0 or l_24 == 0:
            res = False
        elif (l_1**2+l_2**2)/l_13 > 1+tol or (l_1**2+l_2**2)/l_13 < 1-tol or (l_3**2+l_4**2)/l_13 > 1+tol or (l_3**2+l_4**2)/l_13 < 1-tol:
            res = False
        elif (l_1**2+l_4**2)/l_24 > 1+tol or (l_1**2+l_4**2)/l_24 < 1-tol or (l_2**2+l_3**2)/l_24 > 1+tol or (l_2**2+l_3**2)/l_24 < 1-tol:
            res = False
        else:
            pass
    else:
        res = False
    return res


def isRectangle(approx, min_side=25, tol=0.1):
    res = True
    if len(approx) == 4:
        x_1 = approx[0][0][0]
        y_1 = approx[0][0][1]
        x_2 = approx[1][0][0]
        y_2 = approx[1][0][1]
        x_3 = approx[2][0][0]
        y_3 = approx[2][0][1]
        x_4 = approx[3][0][0]
        y_4 = approx[3][0][1]

        l_1 = ((x_1-x_2)**2+(y_1-y_2)**2)**0.5
        l_2 = ((x_2-x_3)**2+(y_2-y_3)**2)**0.5
        l_3 = ((x_3-x_4)**2+(y_3-y_4)**2)**0.5
        l_4 = ((x_4-x_1)**2+(y_4-y_1)**2)**0.5
        l = [l_1, l_2, l_3, l_4]
        for i in range(4):
            if l[i] < min_side:
                res = False

        l_13 = ((x_1-x_3)**2+(y_1-y_3)**2)
        l_24 = ((x_2-x_4)**2+(y_2-y_4)**2)
        if l_13 == 0 or l_24 == 0:
            res = False
        elif (l_1**2+l_2**2)/l_13 > 1+tol or (l_1**2+l_2**2)/l_13 < 1-tol or (l_3**2+l_4**2)/l_13 > 1+tol or (l_3**2+l_4**2)/l_13 < 1-tol:
            res = False
        elif (l_1**2+l_4**2)/l_24 > 1+tol or (l_1**2+l_4**2)/l_24 < 1-tol or (l_2**2+l_3**2)/l_24 > 1+tol or (l_2**2+l_3**2)/l_24 < 1-tol:
            res = False
        else:
            pass
    else:
        res = False
    return res


def orderCorners(approx):
    res = [0, 0, 0, 0]
    ind_max = 0
    ind_min = 0
    ind_1 = 0
    ind_2 = 0
    coor_sum = 0
    for i in range(4):
        if approx[i][0][0]+approx[i][0][1] > coor_sum:
            coor_sum = approx[i][0][0]+approx[i][0][1]
            ind_max = i
        else:
            pass
    for i in range(4):
        if approx[i][0][0]+approx[i][0][1] < coor_sum:
            coor_sum = approx[i][0][0]+approx[i][0][1]
            ind_min = i
        else:
            pass
    res[0] = approx[ind_min]
    res[2] = approx[ind_max]
    for i in range(4):
        if i != ind_min and i != ind_max:
            ind_1 = i
        else:
            pass
    for i in range(4):
        if i != ind_min and i != ind_max and i != ind_1:
            ind_2 = i
        else:
            pass
    if abs(approx[ind_min][0][1]-approx[ind_1][0][1]) < abs(approx[ind_min][0][1]-approx[ind_2][0][1]):
        res[1] = approx[ind_1]
        res[3] = approx[ind_2]
    else:
        res[1] = approx[ind_2]
        res[3] = approx[ind_1]
    return res


def corners2contour(corners):
    contour = [[corners[0]], [corners[1]], [corners[2]], [corners[3]]]
    return contour
    

def inRectangle(coord, rect):
    # coord = [x, y]
    # rect = [[x1, y1], [x2, y2], [x3, y3], [x4, y4]], ordered with orderCorners
    x = coord[0]
    y = coord[1]
    x1 = rect[0][0]
    x2 = rect[1][0]
    x3 = rect[2][0]
    x4 = rect[3][0]
    y1 = rect[0][1]
    y2 = rect[1][1]
    y3 = rect[2][1]
    y4 = rect[3][1]
    res = True
    if (x < x1 and x < x4) or (x > x2 and x > x3):
        res = False
    elif (y < y1 and y < y2) or (y > y3 and y > y4):
        res = False
    x_thing = (x-x1)/(x2-x1)
    y_thing = (y-y1)/(y2-y1)
    if (y2 > y1 and x_thing > y_thing) or (y2 < y1 and x_thing < y_thing):
        res = False
    x_thing = (x-x2)/(x3-x2)
    y_thing = (y-y2)/(y3-y2)
    if (x3 < x2 and x_thing < y_thing) or (x3 > x2 and x_thing > y_thing):
        res = False
    x_thing = (x-x3)/(x4-x3)
    y_thing = (y-y3)/(y4-y3)
    if (y4 > y3 and x_thing < y_thing) or (y4 < y3 and x_thing > y_thing):
        res = False
    x_thing = (x-x4)/(x1-x4)
    y_thing = (y-y4)/(y1-y4)
    if (x1 < x4 and x_thing > y_thing) or (x1 > x4 and x_thing < y_thing):
        res = False
    return res


def getCenter(rect):
    # rect = [[x1, y1], [x2, y2], [x3, y3], [x4, y4]]
    center = [0, 0]
    center[0] = (rect[0][0]+rect[1][0]+rect[2][0]+rect[3][0])/4
    center[1] = (rect[0][1]+rect[1][1]+rect[2][1]+rect[3][1])/4
    return center


def get_angle(x, y, x_ro, y_ro):
    theta = 0
    if y < y_ro and x != x_ro:
        theta = np.arctan2(y_ro-y, x_ro-x)
    elif y > y_ro and x != x_ro:
        theta = np.arctan2(y_ro-y, x_ro-x)+2*np.pi
    elif x == x_ro and y < y_ro:
        theta = 0.5*np.pi
    elif x == x_ro and y > y_ro:
        theta = 1.5*np.pi
    elif x > x_ro and y == y_ro:
        theta = np.pi
    elif x < x_ro and y == y_ro:
        theta = 2*np.pi
    return theta


def detAruco(frame_ad, arucoDict, arucoParams, contrastParams=[0.75, 0.825, 1, 1.125, 1.25, 1.5]):
    font = cv2.FONT_HERSHEY_COMPLEX
    frame = frame_ad
    info_arucos = []

    for e in contrastParams:
        contrast = cv2.convertScaleAbs(frame_ad, alpha=e)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(contrast, arucoDict, parameters=arucoParams)
        if np.shape(corners) != (0,):
            for i in range(np.shape(corners)[0]):
                corners_i = corners[i]
                corners_i = corners_i[0]
                corner_i_1 = (int(corners_i[0][0]),int(corners_i[0][1]))
                corner_i_2 = (int(corners_i[1][0]),int(corners_i[1][1]))
                corner_i_3 = (int(corners_i[2][0]),int(corners_i[2][1]))
                corner_i_4 = (int(corners_i[3][0]),int(corners_i[3][1]))
                corner_i_text = (int(corners_i[0][0]),int(corners_i[0][1])-10)
                id_i = str(ids[i][0])
                cv2.line(frame, corner_i_1, corner_i_2, (0, 255, 0), 5)
                cv2.line(frame, corner_i_2, corner_i_3, (0, 255, 0), 5)
                cv2.line(frame, corner_i_3, corner_i_4, (0, 255, 0), 5)
                cv2.line(frame, corner_i_4, corner_i_1, (0, 255, 0), 5)
                cv2.putText(frame, 'ArUco, ID: '+id_i, corner_i_text, font, 0.5, (0, 0, 0))
                center = getCenter([[corners_i[0][0], corners_i[0][1]], [corners_i[1][0], corners_i[1][1]], [corners_i[2][0], corners_i[2][1]], [corners_i[3][0], corners_i[3][1]]])
                
                x2, y2, x3, y3 = corner_i_2[0], corner_i_2[1], corner_i_3[0], corner_i_3[1]

                angle = get_angle(x2, y2, x3, y3)

                info_arucos.append((center, angle, id_i))

    return frame, info_arucos


def color_Detection(img, contrastParams=[1]):
    # parameters for filtering detected areas
    eps = 5
    min_area, max_area = 350, 700
    pos_blocks = {'green': [], 'orange': [], 'blue': []}

    for e in contrastParams:
        img_i = cv2.convertScaleAbs(img, alpha=e)
        # Convert the img in
        # BGR(RGB color space) to
        # HSV(hue-saturation-value)
        # color space
        hsvFrame = cv2.cvtColor(img_i, cv2.COLOR_BGR2HSV)

        # Set range for orange color and
        # define mask
        orange_lower = np.array([1, 50, 70], np.uint8)
        orange_upper = np.array([15, 255, 255], np.uint8)
        orange_mask = cv2.inRange(hsvFrame, orange_lower, orange_upper)

        # Set range for green color and
        # define mask
        green_lower = np.array([25, 50, 70], np.uint8)
        green_upper = np.array([89, 255, 255], np.uint8)
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

        # Set range for blue color and
        # define mask
        blue_lower = np.array([96, 50, 70], np.uint8)
        blue_upper = np.array([115, 255, 255], np.uint8)
        blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

        # Morphological Transform, Dilation
        # for each color and bitwise_and operator
        # between img and mask determines
        # to detect only that particular color
        kernal = np.ones((5, 5), "uint8")

        # For orange color
        orange_mask = cv2.dilate(orange_mask, kernal)
        res_orange = cv2.bitwise_and(img, img,
                                    mask=orange_mask)

        # For green color
        green_mask = cv2.dilate(green_mask, kernal)
        res_green = cv2.bitwise_and(img, img,
                                    mask=green_mask)

        # For blue color
        blue_mask = cv2.dilate(blue_mask, kernal)
        res_blue = cv2.bitwise_and(img, img,
                                mask=blue_mask)

        # Creating contour to track orange color
        contours, hierarchy = cv2.findContours(orange_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)

        #...

        # Creating contour to track orange color
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (area > min_area) and (area < max_area):  # limit size of detected squares
                x, y, w, h = cv2.boundingRect(contour)
            # the recognized colored rectangles must be squares
                if int(h) - eps <= int(w) <= int(h) + eps:
                    img = cv2.rectangle(img, (x, y),
                                    (x + w, y + h),
                                    (0, 0, 255), 2)

                    cv2.putText(img, "orange Colour", (x, y),
                                cv2.FONT_HERSHEY_PLAIN, 1.0,
                                (0, 0, 255))

                    pos_blocks['orange'].append((int(x+w/2), int(y+h/2)))
                    cv2.circle(img, (int(x+w/2), int(y+h/2)), 5, (0, 0, 255), -1)

        # Creating contour to track green color
        contours, hierarchy = cv2.findContours(green_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > min_area) and (area < max_area):
                x, y, w, h = cv2.boundingRect(contour)
                if int(h) - eps <= int(w) <= int(h) + eps:
                    img = cv2.rectangle(img, (x, y),
                                        (x + w, y + h),
                                        (0, 255, 0), 2)

                    cv2.putText(img, "Green Colour", (x, y),
                                cv2.FONT_HERSHEY_PLAIN,
                                1.0, (0, 255, 0))

                    pos_blocks['green'].append((int(x+w/2), int(y+h/2)))
                    cv2.circle(img, (int(x+w/2), int(y+h/2)), 5, (0, 255, 0), -1)

        # Creating contour to track blue color
        contours, hierarchy = cv2.findContours(blue_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > min_area) and (area < max_area):
                x, y, w, h = cv2.boundingRect(contour)
                if int(h) - eps <= int(w) <= int(h) + eps:
                    img = cv2.rectangle(img, (x, y),
                                        (x + w, y + h),
                                        (255, 0, 0), 2)

                    cv2.putText(img, "Blue Colour", (x, y),
                                cv2.FONT_HERSHEY_PLAIN,
                                1.0, (255, 0, 0))

                    pos_blocks['blue'].append((int(x+w/2), int(y+h/2)))
                    cv2.circle(img, (int(x+w/2), int(y+h/2)), 5, (255, 0, 0), -1)

    # Program Termination
    #cv2.imshow("Multiple Color Detection in Real-TIme", img)

    return img, pos_blocks


def aruco_detection(img):
    robotID = 2
    pos_blocks = {'robot': [], 'opponent': []}

    # aruco detection initialisation
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
    arucoParams = cv2.aruco.DetectorParameters_create()

    (corners, ids, rejected) = cv2.aruco.detectMarkers(
        img, arucoDict, parameters=arucoParams)

    # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned
            # in top-left, top-right, bottom-right, and bottom-left
            # order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(img, (cX, cY), 1, (0, 0, 255), -1)

            if markerID == robotID:
                hp = (int((topRight[0] + topLeft[0])/2.0),
                      int((topRight[1] + topLeft[1])/2.0))
                cv2.circle(img, (hp[0], hp[1]), 1, (0, 0, 255), -1)
                cv2.line(img, hp, (cX, cY), (255, 0, 0), 1)

            cv2.putText(img, str(
                markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    return img, ids


##################
### Trajectory ###
##################
def theta_real_output_small(theta):
    return 0.9*theta #0.7*(0.0202*theta**3 + 0.0036*theta**2 + 0.871*theta - 0.0121)


def theta_real_output_big(theta):
    return 0.95*theta #0.95*(0.0202*theta**3 + 0.0036*theta**2 + 0.871*theta - 0.0121)


def dist_real_output(dist):
    return dist*0.005 #0.75*(0.8438*dist + 20.692)*0.005


def theta_to_u(th, u):
    u = 0.7 * u
    d = 18.45*10**(-2) # m
    r = 3.6*10**(-2) # m
    v = u * r * 2 * np.pi # m/s
    if th < 30*np.pi/180:
        th = theta_real_output_small(th)
    else:
        th = theta_real_output_big(th)
    if abs(th) > np.pi and th > 0:
        th = -2*np.pi + th
        dt = abs(th * d / 2 / v) # s
        u = -u
    elif abs(th) > np.pi and th < 0:
        th = -2*np.pi - th
        dt = abs(th * d / 2 / v) # s
    else:
        dt = abs(th * d / 2 / v) # s
        if th < 0:
            u = -u
    return u, dt


def s_to_u(s, u):
    dist = dist_real_output(np.linalg.norm(s, 2))
    r = 2.1*10**(-2) # m
    v = u * r * 2 * np.pi # rps
    dt = dist / v  # s
    return u, dt


def coordinates_to_inputs(x, y, th_ro, v):
    u_l, u_r, t = np.zeros((len(x)-1)*2), np.zeros((len(x)-1)*2), np.zeros((len(x)-1)*2)
    
    th_list = np.zeros(len(x))
    th_list[0] = th_ro*np.pi/180

    for i in range(len(x)-1):
        th_dir = get_angle(x[i+1], y[i+1], x[i], y[i])
        th_turn = th_list[i] - th_dir

        u, dt = theta_to_u(th_turn, v)
        u_l[2*i], u_r[2*i], t[2*i] = -u, u, dt

        s = np.array([x[i+1]-x[i], y[i+1]-y[i]])
        u, dt = s_to_u(s, v)
        u_l[2*i+1], u_r[2*i+1], t[2*i+1] = u, u, dt
        th_list[i+1] = th_dir
    return u_l, u_r, t


def back_it_up(dist, u):
    s = np.array([dist, 0, 0])
    u, dt = s_to_u(s, u)
    return np.array([-u]), np.array([-u]), np.array([dt])


def dist_between_points(x1, y1, x2, y2):
    return np.sqrt((x1-x2)**2+(y1-y2)**2) # distance in pixels


def in_rect(P, corners):  # P = [] coordinate point and corners = [[x, y], [x, y], [x, y], [x, y]] in a sequence!
    angle = 0

    ext_rect = copy(list(corners))
    ext_rect.append(ext_rect[0])

    for i in range(len(corners)):
        b = np.array(P)
        a, c = np.array(ext_rect[i]), np.array(ext_rect[i+1])

        ba = a-b
        bc = c-b
    
        cosine_angle = np.dot(ba, bc) / ((np.linalg.norm(ba) * np.linalg.norm(bc)) + 0.001)
        angle += math.degrees(np.arccos(cosine_angle))
    in_rect = angle >= 0.95*360
    
    return in_rect


def target_zone(frame, pt1, pt2,  info_blocks, pten, thickness, has_block, goal_them, color):
    if color == 'green':
        color_ = (70, 130, 70)
    else:
        color_ = (150, 60, 30)
   
    d = thickness/2
    pt1 = [int(pt1[0]), int(pt1[1])]
    pt2 = [int(pt2[0]), int(pt2[1])]
    pt3 = 0

    cv2.line(frame, pt1, pt2, (0, 0, 255), 2)
    
    blocks = [item for sublist in info_blocks.values() for item in sublist ]
    blocks.append(pten)

    rico = (pt2[1] - pt1[1] + 0.001) / (pt2[0] - pt1[0] + 0.001)
    m = -1/rico

    c1 = np.array([int(pt1[0] + d/np.sqrt(1+m**2)) , int(pt1[1] + d*m/np.sqrt(1+m**2))])   #punt, rico, afstand ---> nieuw punt op die rechte
    c2 = np.array([int(pt2[0] + d/np.sqrt(1+m**2)) , int(pt2[1] + d*m/np.sqrt(1+m**2))])
    c3 = np.array([int(pt2[0] - d/np.sqrt(1+m**2)) , int(pt2[1] - d*m/np.sqrt(1+m**2))]) 
    c4 = np.array([int(pt1[0] - d/np.sqrt(1+m**2)) , int(pt1[1] - d*m/np.sqrt(1+m**2))])


    rect_corner1 = np.array([c1, c2, pt2, pt1 ])
    rect_corner2 = np.array([pt1, c4, c3, pt2])
    rect_contour1 = np.array(corners2contour(rect_corner1))
    rect_contour2 = np.array(corners2contour(rect_corner2))
    cv2.drawContours(frame, [rect_contour1], 0, color_, 3)
    cv2.drawContours(frame, [rect_contour2], 0,color_, 3)

    distance_ref = 1000

    for coord in blocks:
        distance_block = np.sqrt((coord[1]-pt1[1])**2 + (coord[0]-pt1[0])**2)
        isendpoint = int(coord[0]) == pt2[0] and int(coord[1]) == pt2[1]
        if (in_rect(coord,rect_corner1) or  in_rect(coord, rect_corner2)) and not isendpoint and distance_block<=distance_ref:
            coord_in_rect = inRectangle(coord, goal_them)
            if not (has_block and coord_in_rect):
                pt3 = [int(coord[0]),  int(coord[1])]
                distance_ref = distance_block

        
    cv2.putText(frame, "pt1", (c1[0], c1[1]), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255))
    cv2.putText(frame, "pt2", (c2[0], c2[1]), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255))
    cv2.putText(frame, "pt3", (c3[0], c3[1]), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255))
    cv2.putText(frame, "pt4", (c4[0], c4[1]), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255))
    
    if pt3 == 0:
        print('gaat rechtdoor')
        return frame, pt2, False
        
    x_snij = (pt3[1] - pt1[1] - (m*pt3[0] - rico*pt1[0])) / (rico-m)   #snijpunten van twee rechten berekenen
    y_snij = rico*(x_snij-pt1[0]) + pt1[1]




    c5 = np.array([int(x_snij + d/np.sqrt(1+m**2)) , int(y_snij + d*m/np.sqrt(1+m**2))])
    c6 = np.array([int(x_snij - d/np.sqrt(1+m**2)) , int(y_snij - d*m/np.sqrt(1+m**2))])

    if (int(pt3[0]) == int(pten[0]) and int(pt3[1]) == int(pten[1])):
      c5 = np.array([int(x_snij + (1.5*d)/np.sqrt(1+m**2)) , int(y_snij + (1.5*d*m)/np.sqrt(1+m**2))])   #two times the width if enemy in rect
      c6 = np.array([int(x_snij - (1.5*d)/np.sqrt(1+m**2)) , int(y_snij - (1.5*d*m)/np.sqrt(1+m**2))])        

    cv2.putText(frame, "pt5", (c5[0], c5[1]), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255))
    cv2.putText(frame, "pt6", (c6[0], c6[1]), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255))

    
    if in_rect(pt3, rect_corner1):
        print('gaat naar pt6')
        return frame, c6, True
    else:
        print('gaat naar pt5')
        return frame, c5, True


################
### STRATEGY ###
################
def strategy(we, blocks, them, goal_them, goal_us):
    '''
    ---Blackboard---
    blocks: {'[x, y]' (string of np array): [x, y, points, goal_us (bool), goal_them (bool)] (np array), ...} (dictionary)
    we: (x, y, theta) (tuple)
    them: (x, y) (tuple)
    goal_them: coordinates of their goal
    goal_us: coordinates of their goal
    '''

    w_best = -1
    coord_best = 0

    x_en = them[0]
    y_en = them[1]

    weight_goal = 1e9
    counter = 0
    co = []

    for coord in blocks:
        goal_us_bool = blocks[coord][3]
        if goal_us_bool:
            counter += 1
            co.append([blocks[coord][0], blocks[coord][1]])

    if counter == 2:
        if np.sqrt((co[0][0]-co[1][0])**2+(co[0][1]-co[1][1]**2)) > 50:
            weight_goal = 1e9
    elif counter > 2:
        weight_goal = -1e9

    for coord in blocks:
        points, goal_us_bool, goal_them_bool = blocks[coord][2], blocks[coord][3], blocks[coord][4]
        x = np.array([we[0], blocks[coord][0], getCenter(goal_them)[0]])
        y = np.array([we[1], blocks[coord][1], getCenter(goal_them)[1]])
        distance_from_enemy = np.sqrt((x_en-blocks[coord][0])**2+(y_en-blocks[coord][1])**2)

        if not goal_them_bool and distance_from_enemy > 80:
            u_l, u_r, t = coordinates_to_inputs(x, y, we[2], 2.91)
            w = points / (t[0]+t[1]+t[2]+2*t[3]) + weight_goal*goal_us_bool
        else:
            w = -2
        

        if w > w_best:
            w_best = w
            coord_best = coord
    
    if coord_best == 0:
        return ctgt_defense(them, goal_us)[0], ctgt_defense(them, goal_us)[1], True


    return blocks[coord_best][0], blocks[coord_best][1], False


def ctgt_defense(them, goal_us):
    # them = (x_en, y_en, th_en)
    # goal_us = ((..., ...), (..., ...), (..., ...), (..., ...))
    x_en = them[0]
    y_en = them[1]
    center_us = getCenter(goal_us)
    x_center = center_us[0]
    y_center = center_us[1]
    rico = (y_en-y_center)/(x_en-x_center+0.001)
    target_line = [y_center-rico*x_center, rico]
    ctgt = (x_center, y_center)
    for i in range(4):
        j = i+1
        if j == 4:
            j = 0
        
        x_gl1 = goal_us[i][0]
        x_gl2 = goal_us[j][0]
        y_gl1 = goal_us[i][1]
        y_gl2 = goal_us[j][1]
        rico_gl = (y_gl2-y_gl1)/(x_gl2-x_gl1+0.001)
        goal_line = [y_gl1-rico_gl*x_gl1, rico_gl]

        x_int = (goal_line[0]-target_line[0])/(target_line[1]-goal_line[1])
        y_int = goal_line[0]+goal_line[1]*x_int

        dist = np.sqrt((x_int-x_en)**2+(y_int-y_en)**2)
        dist_prev = np.sqrt((ctgt[0]-x_en)**2+(ctgt[1]-y_en)**2)

        if ((x_int <= x_gl1+5 and x_int >= x_gl2-5) or (x_int >= x_gl1-5 and x_int <= x_gl2+5)) and ((y_int <= y_gl1+5 and y_int >= y_gl2-5) or (y_int >= y_gl1-5 and y_int <= y_gl2+5)) and dist < dist_prev:
            ctgt = (x_int, y_int)
    return ctgt


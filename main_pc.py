import time
import threading
import bluetooth
import keyboard
import numpy as np
import cv2

from bluetooth_classes import ServerReceiveThread, ServerSendThread
from ictm_toolbox import *

##############################
#   GENERAL INITALISATIONS   #
##############################

font = cv2.FONT_HERSHEY_COMPLEX

# initialise capture
IP_adress = '192.168.1.13'
cap = cv2.VideoCapture('http://'+IP_adress+':8000/stream.mjpg')
#cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture('output_2.avi')

# aruco detection initialisation
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
arucoParams = cv2.aruco.DetectorParameters_create()
contrastParams = np.array([0.75, 1, 1.25, 1.5])
arucoStartPos = (0, 0)

# playing field initialisation
def nothing(x):
    pass
#cv2.namedWindow('capture')
#cv2.createTrackbar('l_canny', 'capture', 50, 255, nothing)
#cv2.createTrackbar('u_canny', 'capture', 150, 255, nothing)
n = 0

corners_pf = [[0, 0], [0, 0], [0, 0], [0, 0]]
area_pf = 0
corners_big = [[0, 0], [0, 0], [0, 0], [0, 0]]
area_big = 0
diff = 0
diff_big = 0
area = 0

corners_sq_big_1 = [[0, 0], [0, 0], [0, 0], [0, 0]]
area_sq_big_1 = 0
diff_sq_big_1 = 0

corners_sq_big_2 = [[0, 0], [0, 0], [0, 0], [0, 0]]
area_sq_big_2 = 0
diff_sq_big_2 = 0
aruco_not_found = True

# playing field initialisation
while(cap.isOpened() and n < 25):
    ret, frame = cap.read()
    frame_pf = frame
    frame_ad = frame
    canny = cv2.Canny(frame_pf, 50, 150)
    contours, _ = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.01*cv2.arcLength(contour, True), True)#True for closed shapes
        if isSquare(approx, tol=0.25):
            x_1 = approx[0][0][0]
            y_1 = approx[0][0][1]
            cv2.drawContours(frame, [approx], 0, (0, 255, 0), 5)
            cv2.putText(frame, 'Square', (x_1, y_1-10), font, 0.5, (0, 0, 0))
            ordered_approx = orderCorners(approx)
            diff_sq_big_1 = 0
            for i in range(4):
                diff_sq_big_1 += abs(ordered_approx[i][0][0]-corners_sq_big_1[i][0])+abs(ordered_approx[i][0][1]-corners_sq_big_1[i][1])
            diff__sq_big_2 = 0
            for i in range(4):
                diff_sq_big_2 += abs(ordered_approx[i][0][0]-corners_sq_big_2[i][0])+abs(ordered_approx[i][0][1]-corners_sq_big_2[i][1])
            area = cv2.contourArea(approx)
            
            if diff_sq_big_1 > 25 and area > area_sq_big_1 and diff_sq_big_2 > 250: # and bool_sq_big_1:
                corners_sq_big_1[0] = ordered_approx[0][0]
                corners_sq_big_1[1] = ordered_approx[1][0]
                corners_sq_big_1[2] = ordered_approx[2][0]
                corners_sq_big_1[3] = ordered_approx[3][0]
                area_sq_big_1 = area
            elif diff_sq_big_2 > 25 and area > area_sq_big_2 and diff_sq_big_1 > 250: # and bool_sq_big_2:
                corners_sq_big_2[0] = ordered_approx[0][0]
                corners_sq_big_2[1] = ordered_approx[1][0]
                corners_sq_big_2[2] = ordered_approx[2][0]
                corners_sq_big_2[3] = ordered_approx[3][0]
                area_sq_big_2 = area
            else:
                pass
        elif isRectangle(approx, tol=0.05):
            x_1 = approx[0][0][0]
            y_1 = approx[0][0][1]
            cv2.drawContours(frame, [approx], 0, (0, 255, 0), 5)
            cv2.putText(frame, 'Rectangle', (x_1, y_1-10), font, 0.5, (0, 0, 0))
            ordered_approx = orderCorners(approx)
            diff = 0
            for i in range(4):
                diff += abs(ordered_approx[i][0][0]-corners_pf[i][0])+abs(ordered_approx[i][0][1]-corners_pf[i][1])
            diff_big = 0
            for i in range(4):
                diff_big += abs(ordered_approx[i][0][0]-corners_big[i][0])+abs(ordered_approx[i][0][1]-corners_big[i][1])
            area = cv2.contourArea(approx)
            if diff > 25 and diff_big > 25 and area > area_pf and area < area_big:
                corners_pf[0] = ordered_approx[0][0]
                corners_pf[1] = ordered_approx[1][0]
                corners_pf[2] = ordered_approx[2][0]
                corners_pf[3] = ordered_approx[3][0]
                area_pf = area
            elif diff_big > 25 and area > area_big:
                corners_big[0] = ordered_approx[0][0]
                corners_big[1] = ordered_approx[1][0]
                corners_big[2] = ordered_approx[2][0]
                corners_big[3] = ordered_approx[3][0]
                area_big = area
            else:
                pass
        else:
            pass
    # aruco detection
    for e in contrastParams:
        contrast = cv2.convertScaleAbs(frame_ad, alpha=e)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(contrast, arucoDict, parameters=arucoParams)
        if np.shape(corners) != (0,):
            for i in range(np.shape(corners)[0]):
                corners_i = corners[i]
                corners_i = corners_i[0]
                center = getCenter(corners_i)
                eps = abs(center[0]-arucoStartPos[0])+abs(center[1]-arucoStartPos[1])
                if eps > 25 and ids[i][0] == 2:
                    arucoStartPos = center
                    aruco_not_found = False
    if not aruco_not_found:
        n += 1
    if ret:
        cv2.imshow('initialisation', frame)
    if cv2.waitKey(1) == ord('q'):
        break

if np.all(corners_pf) == 0:
    corners_pf = corners_big

if inRectangle(arucoStartPos, corners_sq_big_1):
    goal_us = corners_sq_big_1
    goal_them = corners_sq_big_2
else:
    goal_us = corners_sq_big_2
    goal_them = corners_sq_big_1

matrix = cv2.getPerspectiveTransform(np.float32(corners_pf), np.float32([[0, 0], [600, 0], [600, 400], [0, 400]]))
goal_us = cv2.perspectiveTransform(np.array([((goal_us[0][0], goal_us[0][1]), (goal_us[1][0], goal_us[1][1]), (goal_us[2][0], goal_us[2][1]), (goal_us[3][0], goal_us[3][1]))], dtype=np.float32), matrix)
goal_them = cv2.perspectiveTransform(np.array([((goal_them[0][0], goal_them[0][1]), (goal_them[1][0], goal_them[1][1]), (goal_them[2][0], goal_them[2][1]), (goal_them[3][0], goal_them[3][1]))], dtype=np.float32), matrix)
goal_us = goal_us[0]
goal_them = goal_them[0]
check_start = True
has_block = False
get_block = False
path_interrupted = False
ctgt = None
watchdog = 0
enemy_counter = 0

def server_send(threadName, port):
    # Bluetooth connection
    server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    server_sock.bind(("", port)) # same port as the receiving port on the ev3-brick
    server_sock.listen(1)  # listen for a connection
    client_sock, address = server_sock.accept()  # accept the connection
    print("Accepted connection from ", address)
    get_back_in_there = False
    defense = False
    ctgt_prev = (0, 0)
    help_i_am_stuck = 0


    # global variables
    global has_block, ctgt, get_block, check_start, goal_us, goal_them, watchdog, enemy_counter
    if check_start:
        check_start = False
        img_start = np.zeros([400, 600, 3], dtype=np.uint8)
        cv2.line(img_start, (200, 220), (400, 220), (255, 255, 255), 5)
        cv2.putText(img_start, 'Press any key to start', (200, 200), font, 0.5, (255, 255, 255))
        cv2.imshow('Start menu', img_start)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        print('''Let's go!!!''')

    IP_adress = '192.168.1.13'
    cap = cv2.VideoCapture('http://'+IP_adress+':8000/stream.mjpg')
    
    while cap.isOpened():
        try:
            cap = cv2.VideoCapture('http://'+IP_adress+':8000/stream.mjpg')
            # Aruco detection initialisation
            _, frame = cap.read()
            unwarped_frame = frame
            frame = cv2.warpPerspective(frame, matrix, (600, 400))

            # arcuodetection
            frame_aruco, info_arucos = detAruco(unwarped_frame, arucoDict, arucoParams, contrastParams=contrastParams)

            # Search robots and get info
            check_list_arucos = []
            for i in range(len(info_arucos)):
                check_list_arucos.append(int(info_arucos[i][2]))
                if int(info_arucos[i][2]) == 2:
                    th_ro = info_arucos[i][1]*180/np.pi
                    x_ro, y_ro = info_arucos[i][0]
                    x_ro, y_ro = cv2.perspectiveTransform(np.array([((x_ro, y_ro), (x_ro, y_ro), (x_ro, y_ro), (x_ro, y_ro))], dtype=np.float32), matrix)[0][0]
                else:
                    th_en = info_arucos[i][1]*180/np.pi
                    x_en, y_en = info_arucos[i][0]
                    x_en, y_en = cv2.perspectiveTransform(np.array([((x_en, y_en), (x_en, y_en), (x_en, y_en), (x_en, y_en))], dtype=np.float32), matrix)[0][0]

            check = abs(x_ro-ctgt_prev[0])+abs(y_ro-ctgt_prev[1])
            if check > 100:
                help_i_am_stuck += 1
            else:
                help_i_am_stuck = 0

            if 2 in check_list_arucos:
                watchdog = 0

            if 2 not in check_list_arucos:
                watchdog += 1
                get_back_in_there = True

            if np.sqrt((x_ro-x_en)**2+(y_ro-y_en)**2) < 80:
                enemy_counter += 1
            else:
                enemy_counter = 0


            # Search blocks
            list_blocks = color_Detection(frame, contrastParams=[1, 1, 1])[1]

            # algorithm to find next coordinate
            if not get_back_in_there:
                if ctgt == None:
                    if has_block:
                        x_1, y_1 = getCenter(goal_them)[0], getCenter(goal_them)[1]
                        ctgt = tuple(target_zone(frame, (x_ro, y_ro), (x_1, y_1), list_blocks, (x_en, y_en),  80, has_block, goal_them, 'blue')[1])
                        path_interrupted = target_zone(frame, (x_ro, y_ro), (x_1, y_1), list_blocks, (x_en, y_en),  80, has_block, goal_them, 'blue')[2]
                    else:
                        #x_1, y_1 = weighed_distance_selector(list_blocks, (x_ro, y_ro), (x_en, y_en))
                        x_1, y_1, defense = strategy((x_ro, y_ro, th_ro), get_blackboard_blocks(list_blocks, goal_us, goal_them), (x_en, y_en), goal_them, goal_us)
                        ctgt = tuple(target_zone(frame, (x_ro, y_ro), (x_1, y_1), list_blocks, (x_en, y_en), 80, has_block, goal_them, 'blue')[1])
                        path_interrupted = target_zone(frame, (x_ro, y_ro), (x_1, y_1), list_blocks, (x_en, y_en),  80, has_block, goal_them, 'blue')[2]
                else:
                    ctgt = tuple(target_zone(frame, (x_ro, y_ro), ctgt, list_blocks, (x_en, y_en), 80, has_block, goal_them, 'green')[1])
                    path_interrupted = target_zone(frame, (x_ro, y_ro), ctgt, list_blocks, (x_en, y_en), 80, has_block, goal_them, 'green')[2]
                ctgt_prev = ctgt

                # Pop-up window
                frame_vis_ar = frame
                frame_vis_cd = frame

                frame_vis = detAruco(frame_vis_ar, arucoDict, arucoParams, contrastParams=contrastParams)[0]
                frame_vis = color_Detection(frame_vis_cd)[0]

                cv2.line(frame_vis, (int(x_ro), int(y_ro)), (int(x_1), int(y_1)), (0,0,255), 5) #red line from robot to target
                cv2.imshow('capture', frame_vis)

            if cv2.waitKey(1) == ord('q'):
                break
            
            if enemy_counter > 3:
                u_l, u_r, t = back_it_up(60, 2.91)
                has_block = False
                ctgt = None
            
            elif help_i_am_stuck > 5:
                u_l, u_r, t = back_it_up(60, 2.91)
                has_block = False
                ctgt = None
                help_i_am_stuck = 0
            
            elif path_interrupted:
                x = np.array([x_ro, ctgt[0]])
                y = np.array([y_ro, ctgt[1]])
                u_l, u_r, t = coordinates_to_inputs(x, y, th_ro, 2.91) # still to be adapted, only first part to be executed/predection whether you get block
                if t[1] > 2:
                    t[1] = 1.5
                elif t[1] > 1.5:
                    t[1] = 1.25
                elif t[1] > 1:
                    t[1] = 0.9
                
                ctgt = None

            elif not get_back_in_there:
                # calculate path
                x = np.array([x_ro, ctgt[0]])
                y = np.array([y_ro, ctgt[1]])
                u_l, u_r, t = coordinates_to_inputs(x, y, th_ro, 2.91) # still to be adapted, only first part to be executed/predection whether you get block

                '''if t[1] > 3:
                    t[1] = 2.5
                elif t[1] > 2.5:
                    t[1] = 2'''
                if t[1] > 2:
                    t[1] = 1.5
                elif t[1] > 1.5:
                    t[1] = 1.25
                elif t[1] > 1:
                    t[1] = 0.9
                else:
                    if has_block and not path_interrupted:
                        u_l_b, u_r_b, t_b = back_it_up(60, 2.91)
                        u_l = np.concatenate((u_l, u_l_b))
                        u_r = np.concatenate((u_r, u_r_b))
                        t = np.concatenate((t, t_b))
                        has_block = False
                        ctgt = None
                    elif defense:
                        ctgt = None
                        defense = False
                    #elif path_interrupted:
                    #    ctgt = None
                    else:
                        has_block = True
                        ctgt = None
            else:
                # define inputs to turn 180 degrees and ride 200 pixels
                u_turn, t_turn = theta_to_u(np.pi, 2.91)
                u_drive, t_drive = s_to_u(np.array([150, 0, 0]), 2.91)
                if watchdog < 2:
                    u_l, u_r, t = np.array([[0, 0], [0, 0], [t_turn, t_drive]])
                else:
                    u_l, u_r, t = np.array([[u_turn, -u_drive], [u_turn, -u_drive], [0, t_drive]])
                    has_block = False
                get_back_in_there = False        

            print('ctgt', ctgt)
            print('has block', has_block)
            print('path interrupted', path_interrupted)
            message = str(str(list(u_l)) + ',' + str(list(u_r)) + ',' + str(list(t)))
            message_as_bytes = str.encode(str(message))
            client_sock.send(message_as_bytes)
            
                
            if np.sum(t) > 2:
                cv2.waitKey(int(1.2*1000*np.sum(t)))
            else:
                cv2.waitKey(int(1.2*1000*np.sum(t)))
        except:
            pass



sendport1 = 29

thread1 = ServerSendThread("sendthread1", sendport1, server_send)
thread1.start()

cap.release()
cv2.destroyAllWindows()
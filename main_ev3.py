#!/usr/bin/env python3

from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.sensor.lego import TouchSensor, UltrasonicSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, Motor, SpeedRPS

import time
import threading
import bluetooth

from bluetooth_classes import ClientReceiveThread


def client_receive(threadName, port, address):

    m_left = Motor(OUTPUT_D)
    m_right = LargeMotor(OUTPUT_A)

    sock = bluetooth.BluetoothSocket( bluetooth.RFCOMM ) # initiate socket, the channel on the ev3-brick
    sock.connect((address, port)) # connect the socket to the computer given the mac-adress and port

    while True:
        try:
            # Message from pc
            message_as_bytes = sock.recv(1024)
            message = message_as_bytes.decode()
            message = message.split('],[')
            u_l = list(map(float, message[0].strip('][').split(', '))) # speed in RPS
            u_r = list(map(float, message[1].strip('][').split(', ')))
            t = list(map(float, message[2].strip('][').split(', '))) # time in seconds
            print('u', u_l, u_r, t)

            # Loop over commands
            for i in range(len(u_l)):
                if i < len(u_l)-1:
                    m_left.on_for_seconds(SpeedRPS(u_l[i]), t[i], brake=False, block=False)
                    m_right.on_for_seconds(SpeedRPS(u_r[i]), t[i], brake=False, block=False)
                else:
                    m_left.on_for_seconds(SpeedRPS(u_l[i]), t[i], brake=False, block=False)
                    m_right.on_for_seconds(SpeedRPS(u_r[i]), t[i], brake=False, block=False)
                curr_time = time.time()
                while time.time() < curr_time+t[i]*0.95:
                    a = True
        except:
            pass
            

    sock.close()



receiveport1 = 29
PC_address = '3C:91:80:80:35:48'


thread1 = ClientReceiveThread("receivethread1", receiveport1, PC_address, client_receive)
thread1.start()

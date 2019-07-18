
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import socket
import threading
import time
import glob
import cv2.aruco as aruco





markerSize = 0.08
Detect = False



rc='rc'
p=' '
mtx = np.load('/home/argus/opencv/mtx.npy')
dist = np.load('/home/argus/opencv/dist.npy')


drone3 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#drone3.setsockopt(socket.SOL_SOCKET, 25, 'wlx503eaaa238eb'.encode())


def command():
	t=0
	while True:
	     drone3.sendto('command'.encode(), 0, ('192.168.10.1', 8889)) #enter to comand mode
	     #drone2.sendto('command'.encode(), 0, ('192.168.10.1', 8889))
         #time.sleep(5)
	     t=t+1
	     if t>300 :
	        break 
commandThread = threading.Thread(target=command)
commandThread.daemon = True

commandThread.start()



drone3.sendto('command'.encode(), 0, ('192.168.10.1', 8889)) #enter to comand mode
input('streamon')
drone3.sendto('streamon'.encode(), 0, ('192.168.10.1', 8889)) #enter to comand mode
input('Takeoff')

drone3.sendto('takeoff'.encode(), 0, ('192.168.10.1', 8889)) #enter to comand mode
input('up')
drone3.sendto('go 0 0 40 100'.encode(), 0, ('192.168.10.1', 8889))
input('----')


cap = cv2.VideoCapture('udp://192.168.11.1:11111')
#cap = cv2.VideoCapture(0)



while (True):
    ret, frame = cap.read()
    # operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()

    #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

    if np.all(ids != None):
        rvec, tvec,_objPoints = aruco.estimatePoseSingleMarkers(corners[0],markerSize, mtx, dist) #Estimate pose of each marker and return the values rvet and tvec---different from camera coefficients
        #(rvec-tvec).any() # get rid of that nasty numpy value array error
        
        Detect = True
        
        r = rvec[0][0][2]
        x = tvec[0][0][0]
        y = tvec[0][0][1]
        z = tvec[0][0][2]
    
        if  r > 0.1 :
            d = str(-10)
        elif r < -0.1:
            d = str(10)
        elif abs(r) < 0.2 :
            d = str(0)
        
        if  x > 0.1 :
            a = str(40)
        elif x < -0.1:
            a = str(-40)
        elif abs(x) <= 0.1 :
            a = str(0)
            
        if  y < -0.2 :
            c = str(45)
        elif y > 0.2:
            c = str(-45)
        elif abs(y) <= 0.2 :
            c = str(0)
        
        if  z > 0.5 :
            b = str(40)
        else:
            b = str(0)
        
        a = '0'    
        

        go = rc+p+a+p+b+p+c+p+d
        print(r)
        print('x--',x,'y--',y,'z--',z)
        print(go)
        drone3.sendto(go.encode(), 0, ('192.168.10.1', 8889)) #enter to comand mode  
        
        aruco.drawAxis(frame, mtx, dist, rvec[0], tvec[0], 0.1) #Draw Axis
        aruco.drawDetectedMarkers(frame, corners) #Draw A square around the markers


        ###### DRAW ID #####
        cv2.putText(frame, "Id: " + str(ids) , (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

    else:
        
        if Detect == True :
            if tvec[0][0][0] > 0 :
                drone3.sendto('rc 0 0 0 30'.encode(), 0, ('192.168.10.1', 8889))
            elif tvec[0][0][0] < 0 :
                drone3.sendto('rc 0 0 0 -30'.encode(), 0, ('192.168.10.1', 8889)) 
        elif Detect == False :
            drone3.sendto('rc 0 0 0 30'.encode(), 0, ('192.168.10.1', 8889))
    cv2.namedWindow('test',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('test',600,400)    # Display the resulting frame
    cv2.imshow('test',frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

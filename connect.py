#

import serial
import time
import cv2 as cv
import numpy as np
from kalman_filter import KalmanFilter

#link from which rescaleFrame() is based on 
#https://www.youtube.com/watch?v=oXlwWbU8l2o

'''
MIT License

Copyright (c) 2020 Jason Dsouza

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

def rescaleFrame(frame, scale=0.75):    #rescales frame size according scale number
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)    #resizes dimensions
    dimensions = (width, height)
    return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA)    #resizes the frame according to new dimensions

#this coded is modifed from the following link:
#https://github.com/murtazahassan/OpenCV-Python-Tutorials-and-Projects/blob/master/Intermediate/objectTracking.py 

def drawBox(img, bbox): #draws a box around 
    x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
    cv.rectangle(img,(x,y),((x + w), (y + h)), (255, 0, 255), 3, 1)


def diff(bbox, maxLoc): #finds difference between lazer location and center of ROI
    x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
    
    xCen = x + int(w / 2)   #finds coordinates of center of ROI
    yCen = y + int(h / 2)

    diffx = int(xCen - maxLoc[0])   #finds difference between center of ROI and lazer coordinates
    diffy = int(yCen - maxLoc[1])

    return diffx, diffy

def predictPath(roi):
    kf = KalmanFilter()    #kalman filter object
    x, y, w, h = int(roi[0]), int(roi[1]), int(roi[2]), int(roi[3]) 
    
    xCen = x + int(w / 2)   #center of region of interest
    yCen = y + int(h / 2)

    predicted = kf.predict(xCen, yCen)  #make and display first prediction with green rectangle
    newX, newY = int(predicted[0] - (w / 2)), int(predicted[1] - (h / 2))
    cv.rectangle(frame,(int(newX), int(newY)), ((int(newX) + w), (int(newY) + h)), (0, 255, 0), 3, 1)
    for i in range(2):  #display next 4 predictions to show potential path
        predicted = kf.predict(predicted[0], predicted[1])
        newX, newY = int(predicted[0] - w / 2), int(predicted[1] - h / 2)

        if newX < 0:    #ensure coordinates are within bounds
            newX = 0
        if newY < 0:
            newY = 0
        if newX > 640:
            newX = 640
        if newY > 480:
            newY = 480

        cv.rectangle(frame,(int(newX), int(newY)), ((int(newX) + w), (int(newY) + h)), (0, 255, 0), 3, 1)   #display green rectangle
    
    return newX, newY, w, h


choice = input("control manually?(y/n)")    #terminates python program, turret is controlled through joystick on arduino side
if choice == 'y' or choice == 'Y':
    exit(0)

if choice == 'n' or choice == 'N':
    ser = serial.Serial("COM3", 9600)   #establishes connection with serial port to arduino
    time.sleep(2)

    capture = cv.VideoCapture(0)    #video capture object, connect to laptop webcam

    tracker = cv.legacy.TrackerCSRT_create()   #tracker object
    isTrue, img = capture.read()
    img = rescaleFrame(img)
    roi = cv.selectROI("Tracking",img ,False ) #selects region of interest on image (ROI)
    tracker.init(img, roi)  #tracks object within ROI
    
    while True: #for each frame in the video
        isTrue, frame = capture.read()  #isTrue is a boolean that delcare if the fram was successfully captured
                                    #frame is a single frame of the video
        frame = rescaleFrame(frame)

        isFound, roi = tracker.update(frame)

        hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV) #converts frame from BGR to HSV colorspace

        lower_red = np.array([0, 0, 255])   #arrays that describe red color
        upper_red = np.array([255, 255, 255]) 

        edited = cv.inRange(hsv_frame, lower_red, upper_red)
        (minVal, maxVal, minLoc, maxLoc) = cv.minMaxLoc(edited)
        

        if isFound:
            drawBox(frame, roi)
            

        if maxLoc[0] != 0 and maxLoc[1] != 0: #roi is around a defined object
            cv.circle(frame, maxLoc, 20, (0, 0, 255), 2, cv.LINE_AA)
            roiEst = predictPath(roi)
            x, y = diff(roiEst,maxLoc)    #compute difference between target and lazer
            print(x, y)

            valueX = 0
            valueY = 0

            if x > 0:
                valueX = 3
            if x < 0:
                valueX = 1
            if y > 0:
                valueY = 2
            if y < 0:
                valueY = 4

            if x > 100:
                valueX = valueX + 5

            if y > 100:
                valueY = valueY + 5
            
            if abs(x) <= 20 and abs(y) <= 20:   #turret is pointing to target, fire
                valueX = 5
                valueY = 5
            
            valueX = str(valueX)
            valueY = str(valueY)

            ser.write(valueX.encode('utf-8'))
            print("sent to arduino (x):")
            print(valueX.encode('utf-8'))

            #print("read from arduino (x): ")
            #print(ser.read())

            ser.write(valueY.encode('utf-8'))
            print("sent to arduino (y):")
            print(valueY.encode('utf-8'))
            
            #print("read from arduino (y): ")
            #print(ser.read())

            time.sleep(0.2)
       
        if not isFound:   #roi does not enclose an object
            cv.putText(frame, "Object Lost", (int(frame.shape[1]/2), int(frame.shape[0]/2)), cv.FONT_HERSHEY_PLAIN,1, (0, 0, 255))

        cv.imshow("Video", frame)   #show the frame in a window called "Video"

        if cv.waitKey(20) & 0xFF==ord('q'): #if 'q' key is pressed, exit loop
            break      

    ser.close()    
    capture.release()   #deletes capture object
    cv.destroyAllWindows()  #delete all windows    

else:
    print("Invalid input")
    exit(0)
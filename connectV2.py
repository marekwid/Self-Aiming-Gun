
import serial
import time
import cv2 as cv
import numpy as np
from lazer_trackerV2 import Lazer_Tracker
from kalman_filter import KalmanFilter

BBOX_WIDTH = 50
BBOX_HEIGHT = 50

#finds difference between lazer location and center of ROI
def diff(bbox, maxLoc): 
    x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
    
    xCen = x + int(w / 2)   #finds coordinates of center of ROI
    yCen = y + int(h / 2)

    diffx = int(xCen - maxLoc[0])   #finds difference between center of ROI and lazer coordinates
    diffy = int(yCen - maxLoc[1])

    return diffx, diffy

#predicts path of object being tracked using the Kalman Filter
def predictPath(roi, frame):
    kf = KalmanFilter()    #kalman filter object
    x, y, w, h = int(roi[0]), int(roi[1]), int(roi[2]), int(roi[3]) 
    
    xCen = x + int(w / 2)   #center of region of interest
    yCen = y + int(h / 2)

    predicted = kf.predict(xCen, yCen)  #make and display first prediction with green rectangle
    newX, newY = int(predicted[0] - (w / 2)), int(predicted[1] - (h / 2))
    cv.circle(frame, (xCen, yCen), 20, (0, 255, 255), 1, cv.LINE_AA)
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

        cv.circle(frame, (xCen, yCen), 20, (0, 255, 255), 1, cv.LINE_AA)
    
    return newX, newY, w, h

#create bounding box from center point for tracker 
def get_bbox(x_point, y_point):
    x = 0
    y = 0

    if int(x_point - 10) >= 0 and int(y_point - 10 >= 0):
        x, y = float(x_point - 10), float(y_point - 10)

    w, h = float(BBOX_WIDTH), float(BBOX_HEIGHT)

    bbox = [x, y, w, h]
    return bbox

#initialize tracker
def tracker(frame, x, y):
    tracker = cv.legacy.TrackerCSRT_create()   #tracker object

    #calculate bounding box
    bbox = get_bbox(x, y)

    #initailize tracker
    tracker.init(frame, bbox)
    return tracker

#run main while loop
def run(tracker, video):
    while True:
        captured, frame = video.read()
        if captured:
            #find and show lazer
            lazer = Lazer_Tracker(frame)
            center = lazer.run(frame)

            #update tracker
            isFound, roi = tracker.update(frame)
            if isFound:
                #find center of object
                X = int(roi[0] + roi[2] / 2)
                Y = int(roi[1] + roi[3] / 2)
                #draw circle around center of object
                cv.circle(frame, (X, Y), 20, (0, 255, 0), 3, cv.LINE_AA)

                #predict path of object and show prediction
                predictPath(roi, frame)
                x = 0
                y = 0

                if center is not None:
                    x, y = diff(roi, center)
            
                valueX = str(x)
                valueY = str(y)
                valueX = valueX  + '\r'
                valueY = valueY  + '\r'
                
                ser.write(valueX.encode('utf-8'))
                print("sent to arduino (x):")
                print(valueX.encode('utf-8'))
                #print("read from arduino (x): ")
                #print(ser.readline().decode('utf-8'))

                time.sleep(0.001)

                ser.write(valueY.encode('utf-8'))
                print("sent to arduino (y):")
                print(valueY.encode('utf-8'))
                #print("read from arduino (y): ")
                #print(ser.readline().decode('utf-8'))
                
                time.sleep(0.001)

                #print("angle X: ")
                #print(ser.readline().decode('utf-8'))

                #time.sleep(0.001)
            else:
                print("could not find object")
            cv.imshow("video", frame)
        else:
            print("Could not read frame")
            exit()
        if cv.waitKey(20) & 0xFF==ord('q'): #if 'q' key is pressed, exit loop
            exit()
            


if __name__ == '__main__' :
    #terminates python program, turret is controlled through joystick attached to arduino
    choice = input("control manually?(y/n)")    
    if choice == 'y' or choice == 'Y':
        exit(0)
    
    if choice == 'n' or choice == 'N':
        #establishes connection with serial port to arduino
        ser = serial.Serial("COM3", 9600)   
        time.sleep(2)

        video = cv.VideoCapture(0)

        if not video.isOpened():
            print("Video device or file couldn't be opened")
            exit()

        captured, first_frame = video.read()

        def select_point(event, x, y, flags, params):
            #if left mousebutton is pressed
            if event == cv.EVENT_LBUTTONDOWN:
                #draw circle around selected point
                cv.circle(first_frame, (int(x), int(y)), 20, (0, 255, 0), 1, cv.LINE_AA)
                cv.imshow('image', first_frame)

                track = tracker(first_frame, x, y)
                run(track, video)
                   
        if captured:
            cv.imshow('image', first_frame)
            #select point on image
            cv.setMouseCallback('image', select_point)  
        else:
            print("Could not find first frame")
            exit()

        # wait for a key to be pressed to exit
        cv.waitKey(0)
 
        # close the window
        cv.destroyAllWindows()

    else:
        print("invalid input")
        exit()
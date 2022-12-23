#The following code is modified from this project:
#https://github.com/bradmontgomery/python-laser-tracker/blob/master/laser_tracker/laser_tracker.py 

'''
Copyright (c) 2008 Brad Montgomery <brad@bradmontgomery.net>

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

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

import sys
import cv2 as cv
import numpy

class Lazer_Tracker(object):

    def __init__(self, frame, hue_min = 20, hue_max = 160, sat_min = 100, sat_max = 255, val_min = 200, val_max = 256):
        
        '''
        frame: current frame in video
        hue_min: minimum hue value that is considered red
        hue_max: maximum hue value that is considered red
        sat_min: minimum saturation
        sat_max: maximum saturation
        val_min: minimum brightness
        val_max: maximum brightness
        '''
        self.frame = frame

        self.hue_min = hue_min
        self.hue_max = hue_max
        self.sat_min = sat_min
        self.sat_max = sat_max
        self.val_min = val_min
        self.val_max = val_max

        self.max_val = {
            'hue' : self.hue_max,
            'saturation' : self.sat_max,
            'value' : self.val_max,
        }

        self.min_val = {
            'hue' : self.hue_min,
            'saturation' : self.sat_min,
            'value' : self.val_min,
        }

        self.channels = {
            'hue': None,
            'saturation': None,
            'value': None,
        }

        self.lazer = None       

    #creates threshold image
    def threshold(self, frame):
        #convert to HSV color space
        hsv_img = cv.cvtColor(frame, cv.COLOR_BGR2HSV) 
        
        # split the video frame into color channels
        h, s, v = cv.split(hsv_img)
        self.channels['hue'] = h
        self.channels['saturation'] = s
        self.channels['value'] = v

        #threshold channels
        for channel in self.channels:

            #pixels less than threshold value are set to 255 (white)
            (t, tmp) = cv.threshold(
            self.channels[channel],  # src
            self.max_val[channel],  # threshold value
            0,  # we dont care because of the selected type
            cv.THRESH_TOZERO_INV  # t type
            )

            #pixels greater than threshold value are set to 255 (white), all others to 0 (black)
            (t, self.channels[channel]) = cv.threshold(
            tmp,  # src
            self.min_val[channel],  # threshold value
            255,  # maxvalue
            cv.THRESH_BINARY  # type
            )

            if channel == 'hue':
                # only works for filtering red color because the range for the hue
                # is split
                self.channels['hue'] = cv.bitwise_not(self.channels['hue'])

        #cv.imshow('Hue', self.channels['hue'])
        #cv.imshow('Saturation', self.channels['saturation'])
        #cv.imshow('Value', self.channels['value'])

        # Perform an AND on HSV components to identify the laser!
        self.lazer = cv.bitwise_and(
            self.channels['hue'],
            self.channels['value']
        )
        self.lazer = cv.bitwise_and(
            self.channels['saturation'],
            self.lazer
        )

        # Merge the HSV components back together.
        hsv_image = cv.merge([
            self.channels['hue'],
            self.channels['saturation'],
            self.channels['value'],
        ])

        return hsv_image
    
    #tracks lazer based on threshold image
    def track(self, frame, mask):
        """
        Track the position of the laser pointer.
        Code taken from
        http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
        """
        center = None

        contours = cv.findContours(mask, cv.RETR_EXTERNAL,
                                     cv.CHAIN_APPROX_SIMPLE)[-2]

        # only proceed if at least one contour was found
        if len(contours) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle
            c = max(contours, key=cv.contourArea)
            ((x, y), radius) = cv.minEnclosingCircle(c)

            moments = cv.moments(c)
            #if the contour area is greater than 0, calculate the centeroid
            if moments["m00"] > 0:
                center = int(moments["m10"] / moments["m00"]), \
                         int(moments["m01"] / moments["m00"])
            else:
                center = int(x), int(y)

            # draw the circle and centroid on the frame,
            cv.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
            cv.circle(frame, center, 5, (0, 0, 255), -1)

            cv.imshow('Lazer point', frame)

            return center


            
    def run(self, frame):
        cv.imshow('frame', frame)
        threshold_img = self.threshold(frame)
        #cv.imshow('HSV', threshold_img)
        cv.imshow('Lazer', self.lazer)
        center = self.track(frame, self.lazer)
        return center

if __name__ == '__main__':
    #connect to computer webcam
    capture = cv.VideoCapture(0)    

    while(True):
        successful, frame = capture.read()
        if not successful:  # no image captured... end the processing
                sys.stderr.write("Could not read camera frame. Quitting\n")
                sys.exit(1)
        lazer = Lazer_Tracker(frame)
        center = lazer.run(frame)

        #if 'q' is pressed, end program
        if cv.waitKey(1) & 0xFF == ord('q'):
            sys.exit(1)



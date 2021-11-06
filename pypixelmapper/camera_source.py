# -*- coding: utf-8 -*-
"""
Created on Sun Oct 24 06:24:26 2021

@author: SimonGhionea
"""

import cv2
import _thread

def openCaptureSource():
    if False:
        cap = cv2.VideoCapture(0)  #Foscam X1 format address - others will be different
        def make_1080p():
            cap.set(3, 1920)
            cap.set(4, 1080)
        make_1080p();
    
    if True:
        #cap = cv2.VideoCapture('rtsp://192.168.1.204:554/onvif1');
        cap = cv2.VideoCapture('rtsp://admin:123456@192.168.1.104:554/cam/realmonitor?channel=1&subtype=0')
        
    return cap;


class videosource():
    def __init__(self, source):
        self.source = source
        self.currentFrame = None
        self.polygon = None
        self.point1 = [None,None]
        self.point2 = [None,None]
        self.zscale = 1
        self.resolution = []
        self.fov = []
    
    def release(self):
        self.source.release();

def startCapture():
    cap = openCaptureSource();
    
    vsource = videosource(cap)
    #vsource.resolution = camera_resolution
    
    #Function to continuoulsy get video data and update our videosource with the latest frame
    #this is needed to keep the frame buffer fresh with new data.  This function
    #will be running in its own thread that gets started below
    def updateFrame(videosource):
        while(True):
            ret, videosource.currentFrame = videosource.source.read()
            #cv2.waitKey(1)
    
    #Starts continuosly updating the images in a thread - if we don't do this, old images get stuck in the video buffer
    _thread.start_new_thread(updateFrame,(vsource,))
    #time.sleep(1)
    
    return vsource;
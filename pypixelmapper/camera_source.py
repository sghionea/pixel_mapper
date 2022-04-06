# -*- coding: utf-8 -*-
"""
Created on Sun Oct 24 06:24:26 2021

@author: SimonGhionea
"""

import cv2
import _thread

def openCaptureSource():
    if False:
        cap = cv2.VideoCapture(1)  #Foscam X1 format address - others will be different
        def make_1080p():
            cap.set(3, 1920)
            cap.set(4, 1080)
        make_1080p();
        
    if False:
        camera_resolution = (1920,1080);
        
        vc = cv2.VideoCapture(1, cv2.CAP_DSHOW);
        
        # frame size
        vc.set(cv2.CAP_PROP_FRAME_WIDTH, camera_resolution[0]);
        vc.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_resolution[1]);
        
        # # optional fps
        # if(c.fps is not None):
        #     vc.set(cv2.CAP_PROP_FPS, c.fps);
            
        # # optional zoom
        # if(c.zoom is not None):
        #     # works well for the LOGITECH C810 cameras
        #     vc.set(cv2.CAP_PROP_ZOOM,c.zoom);
        
        # optional focus
        #if(c.focus is not None):
        # works well for the LOGITECH C810 cameras
        vc.set(cv2.CAP_PROP_AUTOFOCUS,0);
        #vc.set(cv2.CAP_PROP_FOCUS,10);
        vc.set(cv2.CAP_PROP_FOCUS,0); # far field
        
        #vc.set(cv2.CAP_PROP_EXPOSURE,-9); #sunny
        #vc.set(cv2.CAP_PROP_EXPOSURE,-6); #dusk
        #vc.set(cv2.CAP_PROP_EXPOSURE,-4); #pitchblak
        vc.set(cv2.CAP_PROP_EXPOSURE,0); #pitchblak
        # else:
        #     vc.set(cv2.CAP_PROP_AUTOFOCUS,1);
            
        # optional directshow propwindow
        #if(c.propwindow):
            #vc.set(cv2.CAP_PROP_SETTINGS,0); #popup window
        vc.set(cv2.CAP_PROP_SETTINGS,0); #popup window
        
        cap = vc;
    
    if True:
        #cap = cv2.VideoCapture('rtsp://192.168.1.204:554/onvif1');
        cap = cv2.VideoCapture('rtsp://admin:123456@1G021E7PAA00187.lan:554/cam/realmonitor?channel=1&subtype=0')
        
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
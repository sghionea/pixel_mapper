import cv2
import _thread
import time
from cv2 import aruco
import sys
import numpy as np
import pandas as pd
from pathlib import Path

from imutils import contours, grab_contours
from skimage import metrics, measure
import imutils
from playsound import playsound
import matplotlib.path as pltPath

import pypixelmapper.camera_calibration as camera_calibration
from pypixelmapper.camera_source import startCapture
#import pypixelmapper.arucomarkers as arucomarkers
import pypixelmapper.light_control as light_control
import pypixelmapper.xlightscontrollerconnections as xlightscontrollerconnections

from pypixelmapper.paths import workdir
cam = 'ipc3'
#cam = 'c920c1f0'
interframe_sleep_delay = 1.5;
#interframe_sleep_delay = 1.5;
datadir = workdir+cam+"/"

#%% Setup E1.31 sACN light control
# ulist = [
#     #light_control.Universe(universe=2000,channelstart=3,channelcount=303),
    
#     # FPP
#     # BUSH L2 to center
#     light_control.Universe(universe=10,channelstart=1,channelcount=450),
    
#     # BUSH L1 leftmost
#     #light_control.Universe(universe=11,channelstart=1,channelcount=450),
    
#     # # BUSH R1 to center
#     # light_control.Universe(universe=11,channelstart=1,channelcount=450),
    
#     # # BUSH R2 to center
#     # light_control.Universe(universe=11,channelstart=1,channelcount=450),    
#     ];
# #lights = light_control.Lights(ulist,pixelcount=100);
# lights = light_control.Lights(ulist,pixelcount=300);

# NO UNDERSCORES
if True:
    allmodels = xlightscontrollerconnections.load_all_model_info('Controller_Connections.csv')
    ulist = [
        # 2021/11/17
        #xlightscontrollerconnections.convert_to_modeluniverse(allmodels,'Bush R1'),
        #xlightscontrollerconnections.convert_to_modeluniverse(allmodels,'Bush R2'),
        
        # 2021/11/19 night
        #light_control.ModelUniverses('bigbush_east',universe=10, channelstart=10000, channelcount=70*3), # 10000 to 10209
        #light_control.ModelUniverses('bigbush_north',universe=10, channelstart=10210, channelcount=406*3),# 10210 to 11427
        #light_control.ModelUniverses('bigbush-south',universe=10, channelstart=11575, channelcount=275*3),# 11575 to 12399
        #light_control.ModelUniverses('bigbush-top',universe=10, channelstart=14000, channelcount=400*3),# separate big top strand
        
        # 2021/11/25 day from office
        #xlightscontrollerconnections.convert_to_modeluniverse(allmodels,'bigbushsouth'),

        # 2021/11/29
        xlightscontrollerconnections.convert_to_modeluniverse(allmodels,'bigbushnorth'),
        
    ];
    lights = light_control.LightStepper(ulist,
                                        unicasthost="fpp.lan"
                                        );
else:
    # GARAGE TEST
    ulist = [
        #light_control.ModelUniverses('garage-old-strand',universe=10000,    channelstart=303,           channelcount=100*3),
        light_control.ModelUniverses('garage-test-strand-2',universe=10000,    channelstart=1,           channelcount=166*3),
    ];
    lights = light_control.LightStepper(ulist,
                                        unicasthost="espixelstick02.lan"
                                        );
#%%
# determine new path for captures
capdir = Path(workdir)/'newcaps'/cam/','.join([u.name for u in ulist])
capdir.mkdir(parents=True,exist_ok=True);
existing_captures = sorted(list([x.name for x in capdir.iterdir() if x.is_dir()]));
if(len(existing_captures)>0):
    next_name = '{:02d}'.format(int(existing_captures[-1])+1)
else:
    next_name = '00';
capdir = capdir / next_name
capdir.mkdir(parents=True,exist_ok=True);

# move any aruco images in workdir to this folder
aruco1 = Path(workdir)/'aruco_img.png';
aruco1dest = capdir/'aruco_img.png';
aruco2 = Path(workdir)/'aruco_img_wireframe.png';
aruco2dest = capdir/'aruco_img_wireframe.png';
aruco3 = Path(workdir)/'aruco_detections.npz';
aruco3dest = capdir/'aruco_detections.npz';
if(aruco1.exists()):
    aruco1.replace(aruco1dest);
if(aruco2.exists()):
    aruco2.replace(aruco2dest);
if(aruco3.exists()):
    aruco3.replace(aruco3dest);

#%% light control begin
lights.start(fps=40);
#onval = [20,20,20];
#onval = [100,100,100];
#onvalall = [50,50,50];
#onvalall = [100,0,0];

#onvalall = [100,100,100];
#onval = [255,0,0];
#onval = [100,100,100]; # garage light on, outaside dusk
#onval = [255,0,0]; # outside 1548 45 minutes before sunset

# SUNNY FULL
# onvalall = [255,255,255];
# onval = [255,255,255];

# # DUSK EARLY
# onvalall = [200,200,200];
# onval = [200,200,200];

# DUSK / DARK LATE
onvalall = [100,100,100];
onval = [100,100,100];

# # VERY DARK
# onvalall = [10,10,10];
# onval = [25,25,25];

# # NIGHT OUTSIDE C920 FROM INSIDE OFFICE, BIG BUSH
# onvalall = [25,25,25];
# onval = [100,100,100];

#onval = [150,150,150];
#onval = [255,255,255];
lights.all_on(onval);

#%% test one
#lights.set_one(1,onval)
# #%%
# lights.set_one(237,[255,255,255]);
# #%%
# lights.set_one(238,[255,255,255]);
# #%%
# lights.set_one(239,[255,255,255]);
#%% Open camera for capture
#cap = openCaptureSource();

# cameraMatrixInit = np.array([[ 1000.,    0., 1920./2.],
#                               [    0., 1000., 1080./2.],
#                               [    0.,    0.,           1.]])
# camera_matrix = cameraMatrixInit;

# distCoeffsInit = np.zeros(5,'float32');
# dist_coeffs = distCoeffsInit;

camera_matrix, dist_coeffs = camera_calibration.load_coefficients(datadir+'calib.yaml');

#%% class polygon selection
CANVAS_SIZE = (600,800)
FINAL_LINE_COLOR = (255, 255, 255)
WORKING_LINE_COLOR = (127, 127, 127)
class PolygonDrawer(object):
    def __init__(self, window_name,videosource):
        self.window_name = window_name # Name for our window
        self.done = False # Flag signalling we're done
        self.current = (0, 0) # Current position, so we can draw the line-in-progress
        self.points = [] # List of points defining our polygon
        self.videosource = videosource
    def on_mouse(self, event, x, y, buttons, user_param):
        # Mouse callback that gets called for every mouse event (i.e. moving, clicking, etc.)
        if self.done: # Nothing more to do
            return
        if event == cv2.EVENT_MOUSEMOVE:
            # We want to be able to draw the line-in-progress, so update current mouse position
            self.current = (x, y)
        elif event == cv2.EVENT_LBUTTONDOWN:
            # Left click means adding a point at current position to the list of points
            print("Adding point 1 #%d with position(%d,%d)" % (len(self.points), x, y))
            self.points.append((x, y))
        elif event == cv2.EVENT_RBUTTONDOWN:
            # Right click means we're done
            print("Adding point 1 #%d with position(%d,%d)" % (len(self.points), x, y))
            self.done = True
    def run(self):
        # Let's create our working window and set a mouse callback to handle events
        cv2.namedWindow(self.window_name, flags=cv2.WINDOW_NORMAL)
        
        while(not self.done):
            # This is our drawing loop, we just continuously draw new images
            # and show them in the named window
            cv2.resizeWindow(self.window_name, 800, 600);
            #cv2.imshow(self.window_name, np.zeros(CANVAS_SIZE, np.uint8))
            cv2.imshow(self.window_name,self.videosource.currentFrame)
            cv2.waitKey(1)
            cv2.setMouseCallback(self.window_name, self.on_mouse)
            height, width, channels = self.videosource.currentFrame.shape
            CANVAS_SIZE = (height,width)
            #canvas = np.zeros(CANVAS_SIZE, np.uint8)
            if (len(self.points) > 0):
                # Draw all the current polygon segments
                cv2.polylines(self.videosource.currentFrame, np.array([self.points]), False, FINAL_LINE_COLOR, 1)
                # And  also show what the current segment would look like
                cv2.line(self.videosource.currentFrame, self.points[-1], self.current, WORKING_LINE_COLOR)
            # Update the window
            cv2.imshow(self.window_name, self.videosource.currentFrame)
            # And wait 50ms before next iteration (this will pump window messages meanwhile)
            if cv2.waitKey(50) == 27: # ESC hit
                self.done = True

        # User finised entering the polygon points, so let's make the final drawing
        #canvas = np.zeros(CANVAS_SIZE, np.uint8)
        # of a filled polygon
        if (len(self.points) > 0):
            cv2.fillPoly(self.videosource.currentFrame, np.array([self.points]), FINAL_LINE_COLOR)
        # And show it
        cv2.imshow(self.window_name, self.videosource.currentFrame)
        # Waiting for the user to press any key
        cv2.waitKey(1000)
        self.videosource.polygon = self.points
        cv2.destroyWindow(self.window_name)
        return self.videosource.currentFrame

#%%
# https://longervision.github.io/2017/03/13/ComputerVision/OpenCV/opencv-external-posture-estimation-ChArUco-board/
# https://ksimek.github.io/2013/08/13/intrinsic/
# https://www.elephantrobotics.com/docs/myCobot-en/6-kit/2-preparation/2-calibrate_camera.html
# 

#%% Open capture source
vsource = startCapture();

# #%%
# #Polygon masking - turn on all pixels and allow the user to draw a polygon around them to prevent
# #detecting light sources outside the area of interest
# lights.all_on(onvalall);  #turn all pixels on
# #lights.all_on([255,255,255]);  #turn all pixels on
# time.sleep(1)
# polyd = PolygonDrawer("Polygon1",vsource)
# image = polyd.run()
# print("Polygon = %s" % vsource.polygon)

# #function to get user input during pixel mapping
# #if the image processing can't find a pixel, the user can click on its location to specify the coordinates
# #alternatively, the user can click outside the polygon area of interest and the coordinates will be set to 0,0 (no pixel found)
# def on_mouse( event, x, y, buttons, mystuff):
#     index = mystuff[0]
#     videosource = mystuff[1]
#     if event == cv2.EVENT_LBUTTONDOWN:
#         # Left click means adding a point at current position to the list of points
#         path = pltPath.Path(videosource.polygon)
#         if path.contains_point([x,y]):
#             videosource.outputpoints[index] = [index,x,y]
#             print("Adding point #%d with position(%d,%d)" % (index, x, y))
#         else:
#             videosource.outputpoints[index] = [index,0,0]
#             print("Adding point #%d with position(%d,%d)" % (index,0,0))
   
#%%
lights.all_off()
time.sleep(1)
cv2.namedWindow("Camera1", flags=cv2.WINDOW_NORMAL)
cv2.resizeWindow("Camera1", 800, 600);

#%% if data file exists, read it now
# if(Path(workdir+'/data.csv').exists()):
#     load_dat = pd.read_csv(workdir+'/data.csv',names=['uni','px','x','y']);
# else:
#     load_dat = None;
load_dat = None;

#%%
def getImageOff():
    lights.all_off()
    time.sleep(interframe_sleep_delay)
    image_off = vsource.currentFrame
    print("image_off");
    return image_off;
def getImageOn():
    unum,index = lights.step_on(onval);
    time.sleep(interframe_sleep_delay)
    image = vsource.currentFrame
    print("image")
    return image,unum,index;
def getImageAll():
    lights.all_on(onvalall);
    time.sleep(interframe_sleep_delay)
    image = vsource.currentFrame
    print("image ALL")
    return image;

imageAll = getImageAll();
#cv2.imwrite(workdir+'/cap_allon.png'.format(),imageAll);
cv2.imwrite(str(capdir/ 'cap_allon.png'.format()),imageAll);
cv2.imshow("Camera1",imageAll)
cv2.resizeWindow("Camera1", 800, 600);
cv2.waitKey(500);

lights.next_step_reset();
while lights.next_step():    # select/pick next pixel (true while valid, false while not)
    #retval = lights.next_step();    # select/pick next pixel
    
    # check if we already have point
    m,index,r = lights.get_step();
    # if( (load_dat is not None) and (load_dat.query('uni == @unum and index == @index').shape[0] == 1)):
    #     print('Skipping because data was already collected')
    #     continue;
    
    if(True):
        image_off = getImageOff();
        cv2.imshow("Camera1",image_off)
        #cv2.resizeWindow("Camera1", 800, 600);
        cv2.waitKey(500)
        
        #universe.source.send_data(data=[0,0,0]*(index) + onval + [0,0,0]*(universe.pixelcount-index-1))
        # time.sleep(0.7)
        # image = vsource.currentFrame
        # print("image")
        image,unum,index = getImageOn();
        cv2.imshow("Camera1",image);
        cv2.waitKey(500);
        
        unum = r[0].universe;
        modelname = m.name;
        
        cv2.imwrite(str(capdir/'cap{:s}_U{:d}_{:03d}_imgoff.png'.format(modelname,unum,index)),image_off);
        cv2.imwrite(str(capdir/'cap{:s}_U{:d}_{:03d}_imgon.png'.format(modelname,unum,index)),image);        
    
    if(False):
        unum,index = lights.step_on(onval);
        time.sleep(2);
lights.all_off();
time.sleep(0.5);


#%%

# #%%
# pixels_captured = 0;
# cv2.namedWindow("Pixel Mapping", flags=cv2.WINDOW_NORMAL);
# #pause = False;

# while True:
#     #ret, frame = cap.read();
#     frame = vsource.currentFrame;
#     #print('Read frame ({:d})'.format(ret))
    
#     frameDisplay = cv2.putText(frame.copy(), 
#                                         text = 'Pixel {:d}'.format(pixels_captured),
#                                         org = (100,100),
#                                         fontFace = cv2.FONT_HERSHEY_DUPLEX,
#                                         fontScale = 1.5,
#                                         color = (255,255,255),
#                                         thickness = 2,
#                                         lineType = cv2.LINE_AA
#                                         );
#     cv2.imshow("Pixel Mapping", frameDisplay);
    
#     key = cv2.waitKeyEx(50)
#     print(key)
#     #if key != -1:
#     #    print(key)
#     if  key == 27: # ESC hit
#         break;
#     elif key == 32: # SPACE hit
#         print('SPACE HIT');
#         lights.set_next(onval);
#         pixels_captured+=1;        

#%% cleanup / quit    
#cap.release();
#lights.all_off();
lights.stop();
vsource.release();
cv2.destroyWindow("Pixel Mapping")

#%% load image
if __name__ == '__main__':
    sys.exit(0);

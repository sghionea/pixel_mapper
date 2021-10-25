import cv2
import _thread
import time
from cv2 import aruco
import sys
import numpy as np

import pypixelmapper.camera_calibration as camera_calibration
from pypixelmapper.camera_source import startCapture
#import pypixelmapper.arucomarkers as arucomarkers
import pypixelmapper.light_control as light_control

from pypixelmapper.paths import workdir
datadir = workdir+"ipc2/"

#%% Setup E1.31 sACN light control
ulist = [
    light_control.Universe(universe=4000,channelcount=300),
    ];
lights = light_control.Lights(ulist,pixelcount=100);
lights.start(fps=40);
onval = [20,20,20];
lights.all_on(onval);

#%% Open camera for capture
#cap = openCaptureSource();

# cameraMatrixInit = np.array([[ 1000.,    0., 1920./2.],
#                               [    0., 1000., 1080./2.],
#                               [    0.,    0.,           1.]])
# camera_matrix = cameraMatrixInit;

# distCoeffsInit = np.zeros(5,'float32');
# dist_coeffs = distCoeffsInit;

camera_matrix, dist_coeffs = camera_calibration.load_coefficients(datadir+'calib.yaml');

#%%
# https://longervision.github.io/2017/03/13/ComputerVision/OpenCV/opencv-external-posture-estimation-ChArUco-board/
# https://ksimek.github.io/2013/08/13/intrinsic/
# https://www.elephantrobotics.com/docs/myCobot-en/6-kit/2-preparation/2-calibrate_camera.html
# 

#%% Open capture source
vsource = startCapture();

#%%
pixels_captured = 0;
cv2.namedWindow("Pixel Mapping", flags=cv2.WINDOW_NORMAL);
#pause = False;

while True:
    #ret, frame = cap.read();
    frame = vsource.currentFrame;
    #print('Read frame ({:d})'.format(ret))
    
    frameDisplay = cv2.putText(frame.copy(), 
                                        text = 'Pixel {:d}'.format(pixels_captured),
                                        org = (100,100),
                                        fontFace = cv2.FONT_HERSHEY_DUPLEX,
                                        fontScale = 1.5,
                                        color = (255,255,255),
                                        thickness = 2,
                                        lineType = cv2.LINE_AA
                                        );
    cv2.imshow("Pixel Mapping", frameDisplay);
    
    key = cv2.waitKeyEx(50)
    print(key)
    #if key != -1:
    #    print(key)
    if  key == 27: # ESC hit
        break;
    elif key == 32: # SPACE hit
        print('SPACE HIT');
        lights.set_next(onval);
        pixels_captured+=1;        

#%% cleanup / quit    
#cap.release();
lights.stop();
vsource.release();
cv2.destroyWindow("Pixel Mapping")

#%% load image
if __name__ == '__main__':
    sys.exit(0);

#%% load image
frame = cv2.imread(workdir + "aruco_img.png")
gray = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY)
corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters = arucoParams)
print('Finding poses for markers',ids.flatten().tolist());
rvecs, tvecs, _objpts = aruco.estimatePoseSingleMarkers(corners, size_of_marker , camera_matrix, dist_coeffs);

# select the marker we want
idx = np.where(ids==markerid_to_search)[0][0]
rvec = rvecs[idx];
tvec = tvecs[idx];
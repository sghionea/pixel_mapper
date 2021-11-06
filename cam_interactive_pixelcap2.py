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

from pypixelmapper.paths import workdir
datadir = workdir+"ipc2/"

#%% Setup E1.31 sACN light control
ulist = [
    light_control.Universe(universe=2000,channelstart=3,channelcount=303),
    ];
lights = light_control.Lights(ulist,pixelcount=100);
lights.start(fps=40);
#onval = [20,20,20];
#onval = [100,100,100];
onvalall = [50,50,50];
onval = [150,150,150];
#onval = [255,255,255];
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

#%%
#Polygon masking - turn on all pixels and allow the user to draw a polygon around them to prevent
#detecting light sources outside the area of interest
lights.all_on(onvalall);  #turn all pixels on
#lights.all_on([255,255,255]);  #turn all pixels on
time.sleep(1)
polyd = PolygonDrawer("Polygon1",vsource)
image = polyd.run()
print("Polygon = %s" % vsource.polygon)

#function to get user input during pixel mapping
#if the image processing can't find a pixel, the user can click on its location to specify the coordinates
#alternatively, the user can click outside the polygon area of interest and the coordinates will be set to 0,0 (no pixel found)
def on_mouse( event, x, y, buttons, mystuff):
    index = mystuff[0]
    videosource = mystuff[1]
    if event == cv2.EVENT_LBUTTONDOWN:
        # Left click means adding a point at current position to the list of points
        path = pltPath.Path(videosource.polygon)
        if path.contains_point([x,y]):
            videosource.outputpoints[index] = [index,x,y]
            print("Adding point #%d with position(%d,%d)" % (index, x, y))
        else:
            videosource.outputpoints[index] = [index,0,0]
            print("Adding point #%d with position(%d,%d)" % (index,0,0))
   
#%%
lights.all_off()
time.sleep(1)
cv2.namedWindow("Camera1", flags=cv2.WINDOW_NORMAL)
cv2.resizeWindow("Camera1", 800, 600);

#%% if data file exists, read it now
if(Path(workdir+'/data.csv').exists()):
    load_dat = pd.read_csv(workdir+'/data.csv',names=['uni','px','x','y']);
else:
    load_dat = None;

#%%
def getImageOff():
    lights.all_off()
    time.sleep(0.7)
    image_off = vsource.currentFrame
    print("image_off");
    return image_off;
def getImageOn():
    unum,index = lights.step_on(onval);
    time.sleep(0.7)
    image = vsource.currentFrame
    print("image")
    return image,unum,index;
results = {};
while lights.next_step():    # select/pick next pixel (true while valid, false while not)
    #retval = lights.next_step();    # select/pick next pixel
    
    # check if we already have point
    unum,index = lights.get_step();
    if( (load_dat is not None) and (load_dat.query('uni == @unum and index == @index').shape[0] == 1)):
        print('Skipping because data was already collected')
        continue;
    
    if(True):
        #unum,index = retval
        attempts = 1
        success = False;
        while attempts >0:
            #pixelout = {}
            # lights.all_off()
            # time.sleep(0.7)
            # image_off = vsource.currentFrame
            # print("image_off")
            
            image_off = getImageOff();
            cv2.imwrite(workdir+'/cap_U{:d}_{:03d}_imgoff.png'.format(unum,index),image_off);
            cv2.imshow("Camera1",image_off)
            cv2.resizeWindow("Camera1", 800, 600);
            cv2.waitKey(500)
            
            #universe.source.send_data(data=[0,0,0]*(index) + onval + [0,0,0]*(universe.pixelcount-index-1))
            # time.sleep(0.7)
            # image = vsource.currentFrame
            # print("image")
            image,unum,index = getImageOn();
            cv2.imwrite(workdir+'/cap_U{:d}_{:03d}_imgon.png'.format(unum,index),image);
            cv2.imshow("Camera1",image);
            cv2.waitKey(500);
            
            
            ####MASK OUT THE PORTIONS OF THE IMAGES WE DON'T CARE ABOUT###########
            height, width, channels = image_off.shape
            canvas = np.zeros((height,width), np.uint8)
            polymask = cv2.fillPoly(canvas, np.array([vsource.polygon]), [255,255,255])
            masked_image_off = cv2.bitwise_and(image_off,image_off, mask = polymask)
            masked_image = cv2.bitwise_and(image,image, mask = polymask)
            #######################IMAGE DIFFERENCE###############################
            #https://www.pyimagesearch.com/2017/06/19/image-difference-with-opencv-and-python/
            gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
            gray_off = cv2.cvtColor(masked_image_off, cv2.COLOR_BGR2GRAY)
            # compute the Structural Similarity Index (SSIM) between the two
            # images, ensuring that the difference image is returned
            #(score, diff) = measure.compare_ssim(gray, gray_off, full=True)
            (score, diff) = metrics.structural_similarity(gray, gray_off, full=True)
            diff = (diff * 255).astype("uint8")
            print("SSIM: {}".format(score))
            # threshold the difference image, followed by finding contours to
            # obtain the regions of the two input images that differ
            thresh = cv2.threshold(diff, 0, 255,cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
            cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cnts = grab_contours(cnts)
            diff_mask = thresh
            diff_masked_gray = cv2.bitwise_and(gray, gray, mask = diff_mask)
            blurred = cv2.GaussianBlur(diff_masked_gray, (11, 11), 0)
            thresh2 = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]
            # perform a series of erosions and dilations to remove
            # any small blobs of noise from the thresholded image
            #thresh = cv2.erode(thresh, None, iterations=2)
            #thresh = cv2.dilate(thresh, None, iterations=4)
            # perform a connected component analysis on the thresholded
            # image, then initialize a mask to store only the "large"
            # components
            labels = measure.label(thresh2, connectivity=2, background=0)
            mask = np.zeros(thresh2.shape, dtype="uint8")
            # loop over the unique components
            for label in np.unique(labels):
                # if this is the background label, ignore it
                if label == 0:
                    continue
                # otherwise, construct the label mask and count the
                # number of pixels 
                labelMask = np.zeros(thresh2.shape, dtype="uint8")
                labelMask[labels == label] = 255
                numPixels = cv2.countNonZero(labelMask)
                # if the number of pixels in the component is sufficiently
                # large, then add it to our mask of "large blobs"
                if numPixels > 50:
                    mask = cv2.add(mask, labelMask)
            # find the contours in the mask, then sort them from left to
            # right
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)
            print(cnts)
            cnts = cnts[0] if (imutils.is_cv2() or imutils.is_cv4()) else cnts[1]
            if len(cnts) > 0:
                cnts = contours.sort_contours(cnts)[0]
                # loop over the contours
                for (i, c) in enumerate(cnts):
                    # draw the bright spot on the image
                    (x, y, w, h) = cv2.boundingRect(c)
                    ((cX, cY), radius) = cv2.minEnclosingCircle(c)
                    cv2.circle(image, (int(cX), int(cY)), int(radius),
                        (0, 0, 255), 3)
                    cv2.putText(image, str(cX) + " " + str(cY), (x, y - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
                    # show the output image
                    cv2.imshow("Camera1", image)
                    cv2.waitKey(1)
                if len(cnts)== 1:
                    attempts = 0
                    print("Pixel " + str(index) + " coordinates: [" + str(x) + "," + str(y) + "]")
                    #vsource.outputpoints[unum*170+index] = [unum*170+index,cX,cY]
                    results[(unum,index)] = [unum,index,cX,cY];
                    success = True;
                else:
                    print("too many bright spots, trying again!")
                    attempts = attempts + 1
                    cv2.imshow("Camera1",image)
                    cv2.waitKey(5)
            else:
                print("No bright spots found")
                attempts = attempts + 1
            if attempts >= 2:
                print('too many points attempts - click on the pixel to locate or click outside of polygon to skip') 
                # #playsound('alert.wav')
                # cv2.imshow("Camera1", image)
                # cv2.setMouseCallback("Camera1",on_mouse,[unum*170+index,vsource])
                # done = 0
                # while done == 0:
                #     cv2.waitKey(50)
                #     #if vsource.outputpoints[unum*170+index] != [0,0,0]:
                #     if (unum,index) in results:
                #         done = 1
                #     else:
                #         done = 0
                done = 0;
                attempts = 0
        if success:
            with open(workdir+'/data.csv', 'a') as outfile:
                outfile.write(str(unum) + ',' + str(index) + ',' + str(results[(unum,index)][2]) + ',' + str(results[(unum,index)][3]) + "\n")
        im2 = cv2.putText(image.copy(), 
            text = 'U {:d} pxnum {:03d} success={:d}'.format(unum,index,success),
            org = (100,100),
            fontFace = cv2.FONT_HERSHEY_DUPLEX,
            fontScale = 2,
            color = (255,255,255),
            thickness = 2,
            lineType = cv2.LINE_AA
        );
        cv2.imwrite(workdir+'/cap_U{:d}_{:03d}.png'.format(unum,index),im2)


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
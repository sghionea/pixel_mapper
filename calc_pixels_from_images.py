import cv2
import _thread
import time
from cv2 import aruco
import sys
import numpy as np
import pandas as pd
import os
from pathlib import Path

from imutils import contours, grab_contours
from skimage import metrics, measure
import imutils
from playsound import playsound
import matplotlib.path as pltPath

import pypixelmapper.camera_calibration as camera_calibration
import pypixelmapper.arucomarkers as arucomarkers

from pypixelmapper.paths import workdir
datadir = workdir+"ipc3/"
camname = datadir.split('/')[2];
capturedir = workdir+"captures/"+camname+"/";

#%% elements
elements = os.listdir(capturedir);
print('Have these elements in {:s}'.format(capturedir));
print(elements);
element = elements[1];
elementcapdir = capturedir+element+"/"
print('Using element',element);

#%% class polygon selection
CANVAS_SIZE = (600,800)
FINAL_LINE_COLOR = (255, 255, 255)
WORKING_LINE_COLOR = (127, 127, 127)
class PolygonDrawer(object):
    def __init__(self, window_name,image):
        self.window_name = window_name # Name for our window
        self.done = False # Flag signalling we're done
        self.current = (0, 0) # Current position, so we can draw the line-in-progress
        self.points = [] # List of points defining our polygon
        #self.videosource = videosource
        self.imagesource = image;
        self.polygon = [];
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
            cv2.imshow(self.window_name,self.imagesource)
            cv2.waitKey(1)
            cv2.setMouseCallback(self.window_name, self.on_mouse)
            height, width, channels = self.imagesource.shape
            CANVAS_SIZE = (height,width)
            #canvas = np.zeros(CANVAS_SIZE, np.uint8)
            if (len(self.points) > 0):
                # Draw all the current polygon segments
                cv2.polylines(self.imagesource, np.array([self.points]), False, FINAL_LINE_COLOR, 1)
                # And  also show what the current segment would look like
                cv2.line(self.imagesource, self.points[-1], self.current, WORKING_LINE_COLOR)
            # Update the window
            cv2.imshow(self.window_name, self.imagesource)
            # And wait 50ms before next iteration (this will pump window messages meanwhile)
            if cv2.waitKey(50) == 27: # ESC hit
                self.done = True

        # User finised entering the polygon points, so let's make the final drawing
        #canvas = np.zeros(CANVAS_SIZE, np.uint8)
        # of a filled polygon
        if (len(self.points) > 0):
            cv2.fillPoly(self.imagesource, np.array([self.points]), FINAL_LINE_COLOR)
        # And show it
        cv2.imshow(self.window_name, self.imagesource)
        # Waiting for the user to press any key
        cv2.waitKey(1000)
        self.polygon = self.points
        cv2.destroyWindow(self.window_name)
        return self.imagesource
    
    def view(self):
        # Let's create our working window and set a mouse callback to handle events
        cv2.namedWindow(self.window_name, flags=cv2.WINDOW_NORMAL)
        
        # User finised entering the polygon points, so let's make the final drawing
        #canvas = np.zeros(CANVAS_SIZE, np.uint8)
        # of a filled polygon
        if (len(self.polygon) > 0):
            cv2.fillPoly(self.imagesource, np.array([self.polygon]), (255,255,255,0.5))
        # And show it
        cv2.imshow(self.window_name, self.imagesource)
        # Waiting for the user to press any key
        cv2.waitKey(1000)
        #self.polygon = self.points
        #cv2.destroyWindow(self.window_name)
        #return self.imagesource


#%%
# https://longervision.github.io/2017/03/13/ComputerVision/OpenCV/opencv-external-posture-estimation-ChArUco-board/
# https://ksimek.github.io/2013/08/13/intrinsic/
# https://www.elephantrobotics.com/docs/myCobot-en/6-kit/2-preparation/2-calibrate_camera.html
# 

#%% Examine captures
captures = os.listdir(elementcapdir);
print('Have these captures in {:s}'.format(elementcapdir));
print(captures);

#%% Test 1 2 3 4 5 6 7 8
mycapname = captures[3];
mycapdir = elementcapdir+""+mycapname;
print('Using',mycapdir)


#%% Load images
#datadir = workdir+"webcam/"
images = np.array([mycapdir + "/" + f for f in os.listdir(mycapdir) if f.endswith("imgoff.png") ])
#order = np.argsort([int(p.split(".")[-1].split("_")[-1]) for p in images])
#order = np.argsort([int(p.split("/")[-1]) for p in images])
#images_off = images[order]
images_off = sorted(images);

images = np.array([mycapdir + "/" + f for f in os.listdir(mycapdir) if f.endswith("imgon.png") ])
#order = np.argsort([int(p.split(".")[-1].split("_")[-1]) for p in images])
#order = np.argsort([int(p.split("/")[-1]) for p in images])
#images_off = images[order]
images_on = sorted(images);

file_img_aruco = mycapdir + "/" + "aruco_img.png";

images = np.array([mycapdir + "/" + f for f in os.listdir(mycapdir) if f.endswith("allon.png") ])
if(len(images)>0):
    file_img_all = images[0];
else:
    file_img_all = file_img_aruco;

# #%% Open capture source
# vsource = startCapture();



#%%
#Polygon masking - turn on all pixels and allow the user to draw a polygon around them to prevent
#detecting light sources outside the area of interest
# lights.all_on(onval);  #turn all pixels on
# #lights.all_on([255,255,255]);  #turn all pixels on
# time.sleep(1)
print('Reading {:s} for polygon drawing'.format(file_img_all));
frame = cv2.imread(file_img_all)
polyd = PolygonDrawer("Polygon1",frame)
fname_poly = mycapdir+'/polygon.npy';
if( os.path.exists(fname_poly) ):
    polyd.polygon = np.load(fname_poly);
    polyd.view();
else:
    image = polyd.run()
    print("Polygon = %s" % polyd.polygon)
    np.save(mycapdir+'/polygon',polyd.polygon);

# #%%
# lights.all_off()
# time.sleep(1)
# cv2.namedWindow("Camera1", flags=cv2.WINDOW_NORMAL)
# cv2.resizeWindow("Camera1", 800, 600);

# #%% if data file exists, read it now
# if(Path(workdir+'/data.csv').exists()):
#     load_dat = pd.read_csv(workdir+'/data.csv',names=['uni','px','x','y']);
# else:
#     load_dat = None;

# #%%
# def getImageOff():
#     lights.all_off()
#     time.sleep(0.7)
#     image_off = vsource.currentFrame
#     print("image_off");
#     return image_off;
# def getImageOn():
#     unum,index = lights.step_on(onval);
#     time.sleep(0.7)
#     image = vsource.currentFrame
#     print("image")
#     return image,unum,index;

#%% test
def redfinder(image):
    def nothing(x):
        pass
    
    # Load in image
    #image = cv2.imread('1.jpg')
    
    # Create a window
    cv2.namedWindow('image')
    
    # create trackbars for color change
    cv2.createTrackbar('HMin','image',0,179,nothing) # Hue is from 0-179 for Opencv
    cv2.createTrackbar('SMin','image',0,255,nothing)
    cv2.createTrackbar('VMin','image',0,255,nothing)
    cv2.createTrackbar('HMax','image',0,179,nothing)
    cv2.createTrackbar('SMax','image',0,255,nothing)
    cv2.createTrackbar('VMax','image',0,255,nothing)
    
    # Set default value for MAX HSV trackbars.
    cv2.setTrackbarPos('HMax', 'image', 179)
    cv2.setTrackbarPos('SMax', 'image', 255)
    cv2.setTrackbarPos('VMax', 'image', 255)
    
    # Initialize to check if HSV min/max value changes
    hMin = sMin = vMin = hMax = sMax = vMax = 0
    phMin = psMin = pvMin = phMax = psMax = pvMax = 0
    
    output = image
    wait_time = 33
    
    while(1):
    
        # get current positions of all trackbars
        hMin = cv2.getTrackbarPos('HMin','image')
        sMin = cv2.getTrackbarPos('SMin','image')
        vMin = cv2.getTrackbarPos('VMin','image')
    
        hMax = cv2.getTrackbarPos('HMax','image')
        sMax = cv2.getTrackbarPos('SMax','image')
        vMax = cv2.getTrackbarPos('VMax','image')
    
        # Set minimum and max HSV values to display
        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])
    
        # Create HSV Image and threshold into a range.
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        output = cv2.bitwise_and(image,image, mask= mask)
    
        # Print if there is a change in HSV value
        if( (phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax) ):
            print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (hMin , sMin , vMin, hMax, sMax , vMax))
            phMin = hMin
            psMin = sMin
            pvMin = vMin
            phMax = hMax
            psMax = sMax
            pvMax = vMax
    
        # Display output image
        cv2.imshow('image',output)
    
        # Wait longer to prevent freeze for videos.
        if cv2.waitKey(wait_time) & 0xFF == ord('q'):
            break
    
    cv2.destroyAllWindows()



#%%
results = {};
#while lights.next_step():    # select/pick next pixel (true while valid, false while not)
cv2.namedWindow("Pixel Mapping", flags=cv2.WINDOW_NORMAL);
for fon,foff in zip(images_on,images_off):
    print(fon)
    print(foff)
    #retval = lights.next_step();    # select/pick next pixel
    
    # universe/pixel
    unum = int(os.path.basename(fon).split('_')[1].replace('U',''));
    index = int(os.path.basename(fon).split('_')[2]);
    
    # # check if we already have point
    # unum,index = lights.get_step();
    # if( (load_dat is not None) and (load_dat.query('uni == @unum and index == @index').shape[0] == 1)):
    #     print('Skipping because data was already collected')
    #     continue;
    
    #unum,index = retval
    attempts = 1
    success = False;
    # while attempts >0:
        
    #pixelout = {}
    # lights.all_off()
    # time.sleep(0.7)
    image_off = cv2.imread(foff);
    print("image_off")
    
    # image_off = getImageOff();
    # cv2.imwrite(workdir+'/cap_U{:d}_{:03d}_imgoff.png'.format(unum,index),image_off);
    # cv2.imshow("Camera1",image_off)
    # cv2.resizeWindow("Camera1", 800, 600);
    # cv2.waitKey(500)
    
    # #universe.source.send_data(data=[0,0,0]*(index) + onval + [0,0,0]*(universe.pixelcount-index-1))
    # # time.sleep(0.7)
    # # image = vsource.currentFrame
    # # print("image")
    # image,unum,index = getImageOn();
    # cv2.imwrite(workdir+'/cap_U{:d}_{:03d}_imgon.png'.format(unum,index),image);
    # cv2.imshow("Camera1",image);
    # cv2.waitKey(500);
    
    image = cv2.imread(fon);
    print("image")
    #break;
    
    
    ####MASK OUT THE PORTIONS OF THE IMAGES WE DON'T CARE ABOUT###########
    height, width, channels = image_off.shape
    canvas = np.zeros((height,width), np.uint8)
    polymask = cv2.fillPoly(canvas, np.array([polyd.polygon]), [255,255,255])
    masked_image_off = cv2.bitwise_and(image_off,image_off, mask = polymask)
    masked_image = cv2.bitwise_and(image,image, mask = polymask)
    
    ##### IMAGE DIFFERENCE BY SUBTRACTING
    diff = cv2.subtract(masked_image,masked_image_off);
    #cv2.imshow("difference",diff);
    #break;
    
    ####PICK OUT ONLY RED COLORS
    if False:
        #image = cv2.imread('1.jpg')
        result = diff.copy()
        cvtimage = cv2.cvtColor(masked_image, cv2.COLOR_BGR2HSV)
        #lower = np.array([125,50,50])
        lower = np.array([125,25,25])
        upper = np.array([255,255,255])
        #lower = np.array([100,0,0])
        #upper = np.array([255,255,255])
        mask = cv2.inRange(cvtimage, lower, upper)
        redresult = cv2.bitwise_and(result, result, mask=mask)
        cv2.imshow("t",redresult);
        #break;
    redresult = cv2.cvtColor(diff.copy(), cv2.COLOR_BGR2HSV);
    
    # b,g,r = cv2.split(image.copy())
    # rg = r - g
    # rb = r - b
    # rg = np.clip(rg, 0, 255)
    # rb = np.clip(rb, 0, 255)
    
    # mask1 = cv2.inRange(rg, 50, 255)
    # mask2 = cv2.inRange(rb, 50, 255)
    # mask = cv2.bitwise_and(mask1, mask2)
    # cv2.imshow("test",mask)
    
    #redfinder(masked_image.copy())
    #break;
    
    #######################IMAGE DIFFERENCE###############################
    # #https://www.pyimagesearch.com/2017/06/19/image-difference-with-opencv-and-python/
    # gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
    # gray_off = cv2.cvtColor(masked_image_off, cv2.COLOR_BGR2GRAY)
    # # compute the Structural Similarity Index (SSIM) between the two
    # # images, ensuring that the difference image is returned
    # #(score, diff) = measure.compare_ssim(gray, gray_off, full=True)
    # (score, diff) = metrics.structural_similarity(gray, gray_oxff, full=True)
    # diff = (diff * 255).astype("uint8")
    # print("SSIM: {}".format(score))
    
    # threshold the difference image, followed by finding contours to
    # obtain the regions of the two input images that differ
    #graydiff = cv2.cvtColor(redresult, cv2.COLOR_HSV2GRAY);
    _,_,graydiff = cv2.split(redresult);
    #thresh = cv2.threshold(graydiff.copy(), 240, 255,cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
    thresh = cv2.threshold(graydiff.copy(), 50, 255,cv2.THRESH_BINARY_INV)[1]
    
    
    if False:
        #cnts = grab_contours(cnts1)
        #diff_mask = thresh
        #diff_masked_gray = cv2.bitwise_and(graydiff, graydiff, mask = diff_mask)
        blurred = cv2.GaussianBlur(thresh, (11, 11), 0)
        thresh2 = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]
        
        cnts1 = cv2.findContours(thresh.copy(), cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        contourimg = cv2.drawContours(cv2.cvtColor(thresh.copy(), cv2.COLOR_GRAY2BGR), cnts1[0], -1, (0,255,0), 3)
        cv2.imshow("cnts",contourimg)
        break;
        
        # cv2.imshow("graydiff",graydiff)
        # cv2.imshow("thresh",thresh)
        # #cv2.imshow("diff_masked_gray",diff_masked_gray);
        # cv2.imshow("blurred",blurred)
        cv2.imshow("thresh2",thresh2)
        #break;
    
    # perform a series of erosions and dilations to remove
    # any small blobs of noise from the thresholded image
    thresh3 = cv2.erode(thresh, None, iterations=2)
    thresh3 = cv2.dilate(thresh3, None, iterations=4)
    # perform a connected component analysis on the thresholded
    # image, then initialize a mask to store only the "large"
    # components
    labels = measure.label(thresh3, connectivity=2, background=0)
    mask = np.zeros(thresh3.shape, dtype="uint8")
    #cv2.imshow('Mask initial',mask);
    # loop over the unique components
    for label in np.unique(labels):
        # if this is the background label, ignore it
        if label == 0:
            continue
        # otherwise, construct the label mask and count the
        # number of pixels 
        labelMask = np.zeros(thresh3.shape, dtype="uint8")
        labelMask[labels == label] = 255
        numPixels = cv2.countNonZero(labelMask)
        #print('label {:d} numpixels={:d}'.format(label,numPixels));
        # if the number of pixels in the component is sufficiently
        # large, then add it to our mask of "large blobs"
        if numPixels > 50:
            mask = cv2.add(mask, labelMask)
    #cv2.imshow('Mask final',mask);
    #break;
    # find the contours in the mask, then sort them from left to
    # right
    cnts = cv2.findContours(mask.copy(), cv2.RETR_LIST,
        cv2.CHAIN_APPROX_SIMPLE)
    #print(cnts)
    cnts = cnts[0] if (imutils.is_cv2() or imutils.is_cv4()) else cnts[1]
    #break;
    
    imageAnno = image.copy();
    if len(cnts) > 0:
        cnts = contours.sort_contours(cnts)[0]
        # outer-most is always index 0, so skip this one
        cnts = cnts[1:];
        
        # loop over the contours
        for (i, c) in enumerate(cnts):
            # draw the bright spot on the image
            (x, y, w, h) = cv2.boundingRect(c)
            ((cX, cY), radius) = cv2.minEnclosingCircle(c)
            cv2.circle(imageAnno, (int(cX), int(cY)), int(radius),
                (255, 0, 255), 2)
            cv2.putText(imageAnno, str(cX) + " " + str(cY), (x, y - 15),
                cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 0, 255), 1)
            cv2.waitKey(1);
        
        windowName = "Pixel Mapping cap {:s}".format(mycapname);
        cv2.putText(imageAnno, 
            text = 'U {:d} pxnum {:03d} success={:d}'.format(unum,index,success),
            org = (100,100),
            fontFace = cv2.FONT_HERSHEY_DUPLEX,
            fontScale = 2,
            color = (255,0,0),
            thickness = 2,
            lineType = cv2.LINE_AA
        );
        
        if len(cnts)== 1:
            attempts = 0
            print("Pixel " + str(index) + " coordinates: [" + str(x) + "," + str(y) + "]")
            #vsource.outputpoints[unum*170+index] = [unum*170+index,cX,cY]
            results[(unum,index)] = [unum,index,cX,cY,True];
            success = True;
        else:
            #print("too many bright spots - click on pixel to locate or click outside polygon to skip")
            print("confused - click on pixel to locate or click outside polygon to skip")

            
            #function to get user input during pixel mapping
            #if the image processing can't find a pixel, the user can click on its location to specify the coordinates
            #alternatively, the user can click outside the polygon area of interest and the coordinates will be set to 0,0 (no pixel found)
            def on_mouse( event, x, y, buttons, mystuff):
                unum = mystuff[0]
                index = mystuff[1]
                if event == cv2.EVENT_LBUTTONDOWN:
                    # Left click means adding a point at current position to the list of points
                    path = pltPath.Path(polyd.polygon)
                    if path.contains_point([x,y]):
                        #videosource.outputpoints[index] = [index,x,y]
                        print("Adding point #%d #%d with position(%d,%d)" % (unum, index, x, y))
                        results[(unum,index)] = [unum,index,x,y,False];
                    else:
                        #videosource.outputpoints[index] = [index,0,0]
                        print("Adding point #%d #%d with position(%d,%d)" % (unum, index, 0, 0))
                        results[(unum,index)] = [unum,index,0,0,False];

            cv2.imshow(windowName,imageAnno)
            cv2.setMouseCallback(windowName,on_mouse,[unum,index]);
            #cv2.waitKey(5)
            done = False;
            while not done:
                cv2.waitKey(50)
                #if vsource.outputpoints[unum*170+index] != [0,0,0]:
                if (unum,index) in results:
                    done = True
                else:
                    done = False
            # else:
            #     print("No bright spots found")
            #     attempts = attempts + 1
            # if attempts >= 2:
            #     print('too many points attempts - click on the pixel to locate or click outside of polygon to skip') 
            #     # #playsound('alert.wav')
            #     # cv2.imshow("Camera1", image)
            #     # cv2.setMouseCallback("Camera1",on_mouse,[unum*170+index,vsource])
            #     # done = 0
            #     # while done == 0:
            #     #     cv2.waitKey(50)
            #     #     #if vsource.outputpoints[unum*170+index] != [0,0,0]:
            #     #     if (unum,index) in results:
            #     #         done = 1
            #     #     else:
            #     #         done = 0
            #done = 0;
            #attempts = 0
        
        #if success:
            #with open(workdir+'/data.csv', 'a') as outfile:
                #outfile.write(str(unum) + ',' + str(index) + ',' + str(results[(unum,index)][2]) + ',' + str(results[(unum,index)][3]) + "\n")

        # show the final output image
        image_show = cv2.imshow(windowName, imageAnno);
        key = cv2.waitKey(50);
        if(key==27):    # ESC
            break;

#%% Save pixel results
#dfres = pd.DataFrame(results,columns=["universe","index","x","y","auto"]);
dfres = pd.DataFrame.from_dict(results,orient="index",columns=["universe","index","x","y","auto"])
dfres.to_csv(elementcapdir+"data_{:s}.csv".format(mycapname));

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
#lights.stop();
#vsource.release();
cv2.destroyWindow(windowName);

#%% load image
if __name__ == '__main__':
    sys.exit(0);

#%% pose estimations
markerid_to_search = 40;

# aruco info
aruco_dict,size_of_marker = arucomarkers.getArucoInfo();
arucoParams = arucomarkers.getArucoDetector();

# camera calibration
camera_matrix, dist_coeffs = camera_calibration.load_coefficients(datadir+'calib.yaml');

# read aruco registration image and detect markers
frame = cv2.imread(mycapdir + "/aruco_img.png")

gray = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY)
corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters = arucoParams)
frame2 = aruco.drawDetectedMarkers(frame.copy(), corners, ids, (0,255,0));
print('Finding poses for markers',ids.flatten().tolist());
rvecs, tvecs, _objpts = aruco.estimatePoseSingleMarkers(corners, size_of_marker , camera_matrix, dist_coeffs);

# select the marker we want
idx = np.where(ids==markerid_to_search)[0][0]
rvec = rvecs[idx];
tvec = tvecs[idx];

#draw axis
length_of_axis = size_of_marker/2;
frame2 = aruco.drawAxis(frame2, camera_matrix, dist_coeffs, rvec, tvec, length_of_axis)
cv2.imshow("registration",frame2);
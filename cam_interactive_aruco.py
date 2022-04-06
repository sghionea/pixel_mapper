import cv2
import _thread
import time
from cv2 import aruco
import sys
import numpy as np

import pypixelmapper.camera_calibration as camera_calibration
from pypixelmapper.camera_source import startCapture
import pypixelmapper.arucomarkers as arucomarkers
from pypixelmapper.paths import workdir
datadir = workdir+"ipc3/"
#datadir = workdir+"c920c1f0/"

#%% Aruco setup
arucoParams = arucomarkers.getArucoDetector();
arucoParams.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX;
arucoParams.adaptiveThreshConstant = 15; # default is 7.0
arucoParams.cornerRefinementWinSize = 3; # default is 5
arucoParams.detectInvertedMarker = True;
#arucoParams.adaptiveThreshConstant = ; # default is 7.0
aruco_dict, size_of_marker = arucomarkers.getArucoInfo();


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
# class videosource():
#     def __init__(self, source):
#         self.source = source
#         self.currentFrame = None
#         self.polygon = None
#         self.point1 = [None,None]
#         self.point2 = [None,None]
#         self.zscale = 1
#         self.resolution = []
#         self.fov = []

# vsource = videosource(cap)
# vsource.resolution = camera_resolution

# #Function to continuoulsy get video data and update our videosource with the latest frame
# #this is needed to keep the frame buffer fresh with new data.  This function
# #will be running in its own thread that gets started below
# def updateFrame(videosource):
#     while(True):
#         ret, videosource.currentFrame = videosource.source.read()
#         #cv2.waitKey(1)

# #Starts continuosly updating the images in a thread - if we don't do this, old images get stuck in the video buffer
# _thread.start_new_thread(updateFrame,(vsource,))
# time.sleep(1)

#%% LOOK FOR MARKER ID
markerid_to_search = 40;

#%%
frame_captured = 0;
cv2.namedWindow("Pixel Mapping", flags=cv2.WINDOW_NORMAL);
pause = False;
while True:
    #ret, frame = cap.read();
    frame = vsource.currentFrame;
    if(frame is None):
        print('Frame reqturned empty, continue')
        continue;
    #print(frame.shape);
    #print('Read frame ({:d})'.format(ret))

    if not pause:
        # show the frame
        #cv2.imshow("Pixel Mapping",frame)
        
        #cv2.resizeWindow("Calibration", 800, 450);

    # # any chessboards found here?    
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict);

    # if len(corners)>0:
    #     # SUB PIXEL DETECTION
    #     for corner in corners:
    #         cv2.cornerSubPix(gray, corner,
    #                          winSize = (3,3),
    #                          zeroZone = (-1,-1),
    #                          criteria = criteria)
    #     res2 = cv2.aruco.interpolateCornersCharuco(corners,ids,gray,board)
    #     if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%1==0:
    #         allCorners.append(res2[1])
    #         allIds.append(res2[2])

    # decimator+=1
        #frame_remapped = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)    # for fisheye remapping
        #frame_remapped_gray = cv2.cvtColor(frame_remapped, cv2.COLOR_BGR2GRAY)
        frame_remapped = frame.copy();
        frame_remapped_gray = cv2.cvtColor(frame_remapped, cv2.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame_remapped_gray, aruco_dict, parameters=arucoParams)  # First, detect markers
        #aruco.refineDetectedMarkers(frame_remapped_gray, board, corners, ids, rejectedImgPoints);
        #corners, ids, rejectedCorners, recoveredIds = aruco.refineDetectedMarkers(frame_remapped_gray, board, corners, ids, rejectedImgPoints);
        #print(recoveredIds);
        
        #print(ids);
        if ids is not None: # if there is at least one marker detected
            #charucoretval, charucoCorners, charucoIds = aruco.interpolateCornersCharuco(corners, ids, frame_remapped_gray, board)
            
            #im_with_charuco_board = aruco.drawDetectedCornersCharuco(frame_remapped.copy(), charucoCorners, charucoIds, (0,255,0))
            im_with_charuco_board = aruco.drawDetectedMarkers(frame_remapped.copy(), corners, ids, (0,255,0));
            
            #rvecs, tvecs = aruco.estimatePoseSingleMarkers(corners, size_of_marker, camera_matrix, dist_coeffs);  # posture estimation from a charuco board
            rvecs, tvecs, _objpts = aruco.estimatePoseSingleMarkers(corners, size_of_marker , camera_matrix, dist_coeffs);
            #print(rvecs);
            #im_with_charuco_board = aruco.drawAxis(im_with_charuco_board, camera_matrix, dist_coeffs, rvecs, tvecs, 100);  # axis length 100 can be changed according to your requirement
            length_of_axis = 100;
            for i in range(len(tvecs)):
                im_with_charuco_board = aruco.drawAxis(im_with_charuco_board, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], length_of_axis)
            
            # if len(ids)>18:
            #     retval, rvec, tvec = aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, board, camera_matrix, dist_coeffs,np.zeros(4),np.zeros(4))  # posture estimation from a charuco board
            #     ##retval, rvec, tvec = aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, board)  # posture estimation from a charuco board
            #     if retval == True:
            #         #im_with_charuco_board = aruco.drawAxis(im_with_charuco_board, camera_matrix, dist_coeffs, rvec, tvec, 100)  # axis length 100 can be changed according to your requirement
            #         im_with_charuco_board = aruco.drawAxis(im_with_charuco_board, camera_matrix, dist_coeffs, rvec, tvec, 100)  # axis length 100 can be changed according to your requirement
            #         #cv2.imshow("charucoboard", im_with_charuco_board)
                    
            #     # save this frame
            #     cv2.imwrite(workdir + "cal_img_{:06d}.png".format(frame_captured), frame_remapped);
            #     frame_captured += 1;
            
            
            #myframe = frame_remapped.copy();
            cv2.imshow("Pixel Mapping", im_with_charuco_board)
        else:
            #im_with_charuco_left = frame_remapped
            cv2.imshow("Pixel Mapping",frame);
            #myframe = frame.copy();
            im_with_charuco_board = frame_remapped;
        
        #ids = None;
        #cv2.imshow("charucoboard", im_with_charuco_board)

    
    
    
    
    key = cv2.waitKeyEx(50)
    print(key)
    #if key != -1:
    #    print(key)
    if  key == 27: # ESC hit
        break;
    elif key == 32: # SPACE hit
        print('SPACE HIT');
        if(not pause):
            print('UNPAUSED, save and pause');
            # take snapshots and record pose informations, and pause update
            cv2.imwrite(workdir + "aruco_img.png", frame_remapped);
            cv2.imwrite(workdir + "aruco_img_wireframe.png", im_with_charuco_board);
            im_with_charuco_board = cv2.putText(im_with_charuco_board, 
                                                text = 'Captured, and now paused',
                                                org = (100,100),
                                                fontFace = cv2.FONT_HERSHEY_DUPLEX,
                                                fontScale = 2,
                                                color = (255,255,255),
                                                thickness = 2,
                                                lineType = cv2.LINE_AA
                                                );
            cv2.imshow("Pixel Mapping", im_with_charuco_board);
            np.savez(workdir+"aruco_detections.npz",corners=corners,ids=ids,rejectedImgPoints=rejectedImgPoints)
            pause = True;
        else:
            pause = False;

#%% cleanup / quit    
#cap.release();
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
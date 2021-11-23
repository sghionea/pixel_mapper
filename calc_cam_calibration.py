# -*- coding: utf-8 -*-
"""
Created on Sat Oct 23 16:13:08 2021

@author: SimonGhionea
"""

import numpy as np
import cv2, PIL, os
from cv2 import aruco
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
#%matplotlib nbagg
import yaml

import pypixelmapper.camera_calibration as camera_calibration
import pypixelmapper.arucomarkers as arucomarkers
from pypixelmapper.paths import workdir
#datadir = workdir+"ipc3/"
datadir = workdir+"c920c1f0/"
#%% 
# https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/sandbox/ludovic/aruco_calibration_rotation.html

#%% Setup charucoboard information and working directory
#workdir = "./workdir/"
# aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50);
# # board = aruco.CharucoBoard_create(9, 6, 15, 12, aruco_dict)
# # #imboard = board.draw((2000, 2000))
# #cv2.imwrite(workdir + "chessboard.tiff", imboard);

# squareLength = 15   # Here, our measurement unit is mm
# markerLength = 12   # Here, our measurement unit is mm
# board = aruco.CharucoBoard_create(9, 6, squareLength, markerLength, aruco_dict)
# #imboard = board.draw((2000, 2000))
# #cv2.imwrite(workdir + "chessboard.tiff", imboard);

# #Meanwhile, create aruco detector with default parameters.
# arucoParams = aruco.DetectorParameters_create();

#%% Aruco setup
arucoParams = arucomarkers.getArucoDetector();
aruco_dict, board, imboard = arucomarkers.getCharucoInfo();


#%% load images
#datadir = workdir+"webcam/"
images = np.array([datadir + f for f in os.listdir(datadir) if f.endswith(".png") ])
order = np.argsort([int(p.split(".")[-2].split("_")[-1]) for p in images])
images = images[order]
images

#%%
def read_chessboards(images):
    """
    Charuco base pose estimation.
    """
    print("POSE ESTIMATION STARTS:")
    allCorners = []
    allIds = []
    decimator = 0
    # SUB PIXEL CORNER DETECTION CRITERION
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

    for im in images:
        print("=> Processing image {0}".format(im))
        frame = cv2.imread(im)
        gray = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)
        
        corners, ids, rejectedCorners, recoveredIds = aruco.refineDetectedMarkers(gray, board, corners, ids, rejectedImgPoints);
        
        if len(corners)>0:
            # SUB PIXEL DETECTION
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize = (3,3),
                                 zeroZone = (-1,-1),
                                 criteria = criteria)
            res2 = cv2.aruco.interpolateCornersCharuco(corners,ids,gray,board)
            if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%1==0:
                allCorners.append(res2[1])
                allIds.append(res2[2])

        decimator+=1

    imsize = gray.shape
    return allCorners,allIds,imsize

allCorners,allIds,imsize=read_chessboards(images);

#%%
def calibrate_camera(allCorners,allIds,imsize):
    """
    Calibrates the camera using the dected corners.
    """
    print("CAMERA CALIBRATION")

    # cameraMatrixInit = np.array([[ 1000.,    0., imsize[0]/2.],
    #                              [    0., 1000., imsize[1]/2.],
    #                              [    0.,    0.,           1.]])

    # distCoeffsInit = np.zeros((5,1))
    #flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    flags = (cv2.CALIB_RATIONAL_MODEL)
    #flags = (cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO);
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
                      charucoCorners=allCorners,
                      charucoIds=allIds,
                      board=board,
                      imageSize=imsize,
                      #cameraMatrix=cameraMatrixInit,
                      cameraMatrix=None,
                      #distCoeffs=distCoeffsInit,
                      distCoeffs = np.zeros(5,'float32'),
                      flags=flags,
                      criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors, perViewErrors
ret, mtx, dist, rvecs, tvecs, perViewErrors = calibrate_camera(allCorners,allIds,imsize);
print('Calibration returned',ret);

#%% Print the camera calibration error
error = 0

for i in range(len(allCorners)):
    imgPoints, _ = cv2.projectPoints(allCorners[i], rvecs[i], tvecs[i], mtx, dist)
    error += cv2.norm(imgPointsArray[i], imgPoints, cv2.NORM_L2) / len(imgPoints)

print("Total error: ", error / len(allCorners))

#%% check results
i=50 # select image id
plt.figure()
frame = cv2.imread(images[i])
img_undist = cv2.undistort(frame,mtx,dist,None)
plt.subplot(1,2,1)
plt.imshow(frame)
plt.title("Raw image")
plt.axis("off")
plt.subplot(1,2,2)
plt.imshow(img_undist)
plt.title("Corrected image")
plt.axis("off")
plt.show()

#%%
#np.savez(datadir+'calib.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs);
camera_calibration.save_coefficients(mtx, dist, datadir+'calib.yaml')
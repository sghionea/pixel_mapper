import cv2
import _thread
import time
from cv2 import aruco
import sys
import numpy as np

#%% Setup charucoboard information and working directory
workdir = "./workdir/"
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50);
# board = aruco.CharucoBoard_create(9, 6, 15, 12, aruco_dict)
# #imboard = board.draw((2000, 2000))
#cv2.imwrite(workdir + "chessboard.tiff", imboard);

squareLength = 15   # Here, our measurement unit is mm
markerLength = 12   # Here, our measurement unit is mm
board = aruco.CharucoBoard_create(9, 6, squareLength, markerLength, aruco_dict)
#imboard = board.draw((2000, 2000))
#cv2.imwrite(workdir + "chessboard.tiff", imboard);

#Meanwhile, create aruco detector with default parameters.
arucoParams = aruco.DetectorParameters_create();


#%%

if False:
    sys.exit(0);    


camera_resolution = [2048,1536]   #resolution (in pixels) of the camera [Horizontal, Vertical]
#camera_resolution = [1920,1080]   #resolution (in pixels) of the camera [Horizontal, Vertical]
#camera_resolution = [640,480]   #resolution (in pixels) of the camera [Horizontal, Vertical]
#cap = cv2.VideoCapture('rtsp://username:password@cam.ip.add.ress:88/videoMain')  #Foscam X1 format address - others will be different

if False:
    cap = cv2.VideoCapture(0)  #Foscam X1 format address - others will be different
    def make_1080p():
        cap.set(3, camera_resolution[0])
        cap.set(4, camera_resolution[1])
    make_1080p();

#rtsp://192.168.1.204:554/onvif1

if True:
    #cap = cv2.VideoCapture('rtsp://192.168.1.204:554/onvif1')
    cap = cv2.VideoCapture('rtsp://admin:123456@192.168.1.104:554/cam/realmonitor?channel=1&subtype=0')

cameraMatrixInit = np.array([[ 1000.,    0., camera_resolution[0]/2.],
                              [    0., 1000., camera_resolution[1]/2.],
                              [    0.,    0.,           1.]])
camera_matrix = cameraMatrixInit;

distCoeffsInit = np.zeros(5,'float32');
dist_coeffs = distCoeffsInit;

#%%
# https://longervision.github.io/2017/03/13/ComputerVision/OpenCV/opencv-external-posture-estimation-ChArUco-board/
# https://ksimek.github.io/2013/08/13/intrinsic/
# https://www.elephantrobotics.com/docs/myCobot-en/6-kit/2-preparation/2-calibrate_camera.html
# 

#%% prepare to estimate charuco stuff
allCorners = []
allIds = []
decimator = 0
# SUB PIXEL CORNER DETECTION CRITERION
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001);

#%%
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

vsource = videosource(cap)
vsource.resolution = camera_resolution

#Function to continuoulsy get video data and update our videosource with the latest frame
#this is needed to keep the frame buffer fresh with new data.  This function
#will be running in its own thread that gets started below
def updateFrame(videosource):
    while(True):
        ret, videosource.currentFrame = videosource.source.read()
        #cv2.waitKey(1)

#Starts continuosly updating the images in a thread - if we don't do this, old images get stuck in the video buffer
_thread.start_new_thread(updateFrame,(vsource,))
time.sleep(1)

#%%
frame_captured = 0;
cv2.namedWindow("Calibration", flags=cv2.WINDOW_NORMAL);
while True:
    #ret, frame = cap.read();
    frame = vsource.currentFrame;
    #print('Read frame ({:d})'.format(ret))

    if True:
        # show the frame
        cv2.imshow("Calibration",frame)
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
        frame_remapped = frame;
        frame_remapped_gray = cv2.cvtColor(frame_remapped, cv2.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame_remapped_gray, aruco_dict, parameters=arucoParams)  # First, detect markers
        #aruco.refineDetectedMarkers(frame_remapped_gray, board, corners, ids, rejectedImgPoints);
        corners, ids, rejectedCorners, recoveredIds = aruco.refineDetectedMarkers(frame_remapped_gray, board, corners, ids, rejectedImgPoints);
        #print(recoveredIds);
        
        #print(ids);
        if ids is not None: # if there is at least one marker detected
            charucoretval, charucoCorners, charucoIds = aruco.interpolateCornersCharuco(corners, ids, frame_remapped_gray, board)
            
            im_with_charuco_board = aruco.drawDetectedCornersCharuco(frame_remapped.copy(), charucoCorners, charucoIds, (0,255,0))
            
            
            if len(ids)>18:
                retval, rvec, tvec = aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, board, camera_matrix, dist_coeffs,np.zeros(4),np.zeros(4))  # posture estimation from a charuco board
                ##retval, rvec, tvec = aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, board)  # posture estimation from a charuco board
                if retval == True:
                    #im_with_charuco_board = aruco.drawAxis(im_with_charuco_board, camera_matrix, dist_coeffs, rvec, tvec, 100)  # axis length 100 can be changed according to your requirement
                    im_with_charuco_board = aruco.drawAxis(im_with_charuco_board, camera_matrix, dist_coeffs, rvec, tvec, 100)  # axis length 100 can be changed according to your requirement
                    #cv2.imshow("charucoboard", im_with_charuco_board)
                    
                # save this frame
                cv2.imwrite(workdir + "cal_img_{:06d}.png".format(frame_captured), frame_remapped);
                frame_captured += 1;
            
            cv2.imshow("Calibration", im_with_charuco_board)
        else:
            im_with_charuco_left = frame_remapped
        
        #ids = None;
        #cv2.imshow("charucoboard", im_with_charuco_board)

    
    
    
    
    key = cv2.waitKeyEx(50)
    #print(key)
    #if key != -1:
    #    print(key)
    if  key == 27: # ESC hit
        break;

#%% cleanup / quit    
cap.release();
cv2.destroyWindow("Calibration")
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
# #display alignment overlays until the user hits ESK key
# done = False
# yellow_line_location = round(vsource.resolution[0]/3)
# while done == False:
#     image = vsource.currentFrame
#     cv2.namedWindow("Alignment", flags=cv2.WINDOW_NORMAL)
#     #cv2.namedWindow("Alignment")
#     #overlay a vertical blue line at the centerpoint of the picture
#     image = cv2.line(image,(round(vsource.resolution[0]/2),0),(round(vsource.resolution[0]/2),round(vsource.resolution[1])),(255,0,0),5)
#     #overlay a horiztonal blue line a the centerpoint of the picture
#     image = cv2.line(image,(0,round(vsource.resolution[1]/2)),(vsource.resolution[0],round(vsource.resolution[1]/2)),(255,0,0),5)
#     #overlay two yellow vertical lines equidistant from the center
#     image = cv2.line(image,(round(vsource.resolution[0]/2+yellow_line_location),0),(round(vsource.resolution[0]/2+yellow_line_location),vsource.resolution[1]),(0,255,0),5)
#     image = cv2.line(image,(round(vsource.resolution[0]/2-yellow_line_location),0),(round(vsource.resolution[0]/2-yellow_line_location),vsource.resolution[1]),(0,255,0),5)
#     cv2.imshow("Alignment",image)
#     cv2.resizeWindow("Alignment", 800, 450);
#     key = cv2.waitKeyEx(50)
#     print(key)
#     #if key != -1:
#     #    print(key)
#     if  key == 27: # ESC hit
#         done = True
#     elif key == 39:  #right arrow
#         yellow_line_location = min(yellow_line_location + 5,vsource.resolution[0]/2-5)
#     elif key == 37:  #left arrow
#         yellow_line_location = max(yellow_line_location - 5,5)
        
# cv2.destroyWindow("Alignment")
# vsource.source.release();
import cv2
import _thread
import time
from cv2 import aruco
import sys
import numpy as np
import pandas as pd
import os
from pathlib import Path

import vedo
import copy

from imutils import contours, grab_contours
from skimage import metrics, measure
import imutils
from playsound import playsound
import matplotlib.path as pltPath
from matplotlib import pyplot as plt

import pypixelmapper.camera_calibration as camera_calibration
import pypixelmapper.arucomarkers as arucomarkers

from dataclasses import dataclass

import re

from pypixelmapper.paths import workdir

#%% defs
# camname = 'ipc3';
# #model = 'bushL2l'
# model = 'bushL1c'

#dothis = ('ipc3','bushL1c');
#dothis = ('ipc3','bushL2l');
#dothis = ('ipc3','Bush R1,Bush R2');
#dothis = ('ipc3','bigbush-east');
dothis = ('ipc3','bigbushnorth');

# camname = 'c920c1f0';
# model = 'bigbush-south,bigbush-top'
#dothis = ('c920c1f0','bigbush-south,bigbush-top');


camname,model = dothis;

datadir = workdir+camname+"/"
#camname = datadir.split('/')[2];
#capturedir = workdir+"captures/"+camname+"/";
capturedir = workdir+"newcaps/"+camname+"/";

#%% camera calibration
camera_matrix, dist_coeffs = camera_calibration.load_coefficients(datadir+'calib.yaml');

#%% elements
elements = os.listdir(capturedir);
print('Have these elements in {:s}'.format(capturedir));
print(elements);

#%%
#element = elements[5];
#element = 'bushL1c';
#element = 'bushL2l';
element = model;
#element = 'Bush R1,Bush R2'
elementcapdir = capturedir+element+"/"
print('Using element',element);

#% Examine captures
captures = [p for p in os.listdir(elementcapdir) if os.path.isdir(os.path.join(elementcapdir,p))];
print('Have these captures in {:s}'.format(elementcapdir));
print(captures);

# datafiles = np.array([mycapdir + "/" + f for f in os.listdir(elementcapdir) if f.startswith("data_") and f.endswith(".csv") ]);
# print('Datafiles:');
# print(datafiles);



#%% Handy funcs
# https://courses.ece.cornell.edu/ece5990/ECE5725_Fall2020_Projects/Dec_21_Demo/Drawing%20Robot/eam348_mm2994_W/index.html
# https://answers.opencv.org/question/175705/mapping-aruco-markers/
# https://towardsdatascience.com/3-d-reconstruction-with-vision-ef0f80cbb299

def getMarkerCenter(corners):
 px = (corners[0][0] + corners[1][0] + corners[2][0]+ corners[3][0]) * 0.25
 py = (corners[0][1] + corners[1][1] + corners[2][1]+ corners[3][1]) * 0.25
 return [px,py]

def getMarkerRotation(corners):
 unit_x_axis = [1.,0.]
 center = getMarkerCenter(corners)
 right_edge_midpoint = (corners[1]+corners[2])/2.
 unit_vec = (right_edge_midpoint-center)/np.linalg.norm(right_edge_midpoint-center)
 angle = np.arccos(np.dot(unit_x_axis,unit_vec))
 return angle

# def inversePerspective(rvec, tvec):
#    R, _ = cv2.Rodrigues(rvec)
#    R = np.array(R).T #this was np.matrix but had error
#    invTvec = np.dot(-R, np.array(tvec))
#    invRvec, _ = cv2.Rodrigues(R)
#    return invRvec, invTvec

def normalize(v):
   if np.linalg.norm(v) == 0 : return v
   return v / np.linalg.norm(v)

#%% Line Intersections
# https://pretagteam.com/question/nearest-intersection-point-to-many-lines-in-python
def intersect(P0, P1):
    """
    P0 and P1 are NxD arrays defining N lines.
    D is the dimension of the space.This
    function
    returns the least squares intersection of the N
    lines from the system given by eq.13 in
       http: //cal.cs.illinois.edu/~johannes/research/LS_line_intersect.pdf.
    """
    
    # generate all line direction vectors
    n = (P1 - P0) / np.linalg.norm(P1 - P0, axis = 1)[: , np.newaxis] # normalized
    
    # generate the array of all projectors
    projs = np.eye(n.shape[1]) - n[: ,: , np.newaxis] * n[: , np.newaxis] # I - n * n.T
    # see fig.1
    
    # generate R matrix and q vector
    R = projs.sum(axis = 0)
    q = (projs @ P0[: ,: , np.newaxis]).sum(axis = 0)
    
    # solve the least squares problem for the
    # intersection point p: Rp = q
    p = np.linalg.lstsq(R, q, rcond = None)[0]
    
    return p


#%% Load datafiles and aruco reference images
@dataclass
class CaptureSet:
    name : str;
    capdir : str;
    images : dict;
    data : pd.DataFrame();

    def findArucoPoses(self,show=False):
        #markerid_to_search = 40;
        #markerid_to_search = 41;
        
        # aruco info
        aruco_dict,size_of_marker = arucomarkers.getArucoInfo();
        arucoParams = arucomarkers.getArucoDetector();
        # arucoParams.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX;
        # arucoParams.adaptiveThreshConstant = 5; # default is 7.0
        arucoParams.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX;
        arucoParams.adaptiveThreshConstant = 15; # default is 7.0
        arucoParams.cornerRefinementWinSize = 3; # default is 5
        arucoParams.detectInvertedMarker = True;
        
        # TRY
        def drawDetected(frame,corners,ids):
            frame2 = aruco.drawDetectedMarkers(frame.copy(), corners, ids, (0,255,0));
            return frame2;
        def doDetect(frame):
            # # camera calibration
            # camera_matrix, dist_coeffs = camera_calibration.load_coefficients(datadir+'calib.yaml');
            #h,  w = self.images['aruco'].shape[:2]
            h,  w = frame.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix,dist_coeffs,(w,h),1,(w,h))
            
            # read aruco registration image and detect markers
            #frame = cv2.imread(mycapdir + "/aruco_img.png")
            #frame = self.images['aruco'];
            
            gray = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters = arucoParams)
            #frame2 = aruco.drawDetectedMarkers(frame.copy(), corners, ids, (0,255,0));
            frame2 = drawDetected(frame, corners, ids);
            return corners,ids,rejectedImgPoints,gray,frame2,newcameramtx;
        
        # check for pre-saved info
        have_aruco_reference = False;
        fdetect = (Path(self.capdir)/'aruco_detections.npz');
        # vet it
        if(fdetect.exists()):
            print('Load Aruco data directly from saved file')
            # found predetected info
            arucofile = np.load(fdetect,allow_pickle=True);
            
            if(len(arucofile['ids'].shape)>0):
                #raise NotImplementedError("Found a valid aruco info in .npz (FIXME)");
                corners = arucofile['corners'];
                ids = arucofile['ids'];
                rejectedImgPoints = arucofile['rejectedImgPoints'];
                print('Loaded markers',ids);
            arucofile.close();  
            
        criteria = "(ids is None)";
        #criteria = "(ids is None) or (37 not in ids)";
        if(not have_aruco_reference ):
            frame_tries = [
                self.images['aruco'],
                self.images['all'],
                ]
            for num,frame in enumerate(frame_tries):
                print('Trying main frame',num,end='')
                if frame is None:
                    continue;
                corners,ids,rejectedImgPoints,gray,frame2,newcameramtx = doDetect(frame);
                if(not eval(criteria)):
                    print(' was good');
                    print(ids);
                    break;
                else:
                    #print('fail');
                    pass;
            #if(ids is None):

            if(eval(criteria)):
                for num,fname in enumerate(self.imgson.tolist()+self.imgsoff.tolist()):
                    print('Consider',num)
                    frame = cv2.imread(str(fname));
                    corners,ids,rejectedImgPoints,gray,frame2,newcameramtx = doDetect(frame);
                    if(not eval(criteria)):
                        print('Trying',num,fname.name,end='')
                        print(' was good');
                            #cv2.imshow("aruco cap frame {:d}".format(num),frame2);
                        ofn = 'aruco_find'+'_'.join(fname.name.split('_')[1:]);
                        cv2.imwrite(str(fname.parent / ofn),frame2);
                        ofn = 'aruco_find'+'_'.join(fname.name.split('_')[1:])+'_REF';
                        cv2.imwrite(str(fname.parent / ofn),frame);
                        #break;
                    else:
                        #print('fail');
                        pass;
                raise RuntimeError('generated files');

        
        print('Finding poses for markers',ids.flatten().tolist());
        #raise RuntimeError;
        rvecs, tvecs, _objpts = aruco.estimatePoseSingleMarkers(corners, size_of_marker , camera_matrix, dist_coeffs);
        
        # # select the marker we want
        # idx = np.where(ids==markerid_to_search)[0][0]
        # rvec = rvecs[idx];
        # tvec = tvecs[idx];
        
        # #draw axis
        # length_of_axis = size_of_marker/2;
        # frame2 = aruco.drawAxis(frame2, camera_matrix, dist_coeffs, rvec, tvec, length_of_axis)

        if(show):
            cv2.imshow("registration",frame2);
        
        # remember the relevant tvec/rvec
        self.rvecs = rvecs;
        self.tvecs = tvecs;
        self.ids = ids;
        self.arucoframe = frame2;
        self.newcameramtx = newcameramtx;
        
        # # a particular aruco marker
        # self.rvec = rvec;
        # self.tvec = tvec;
        # print('rvec',rvec);
        # print('tvec',tvec);
        
        # self.morePoseCalculations();
    
    def poseCalculate(self,markerid_to_search=40,show=False):
        #markerid_to_search = 40;
        #markerid_to_search = 41;
        
        # aruco info
        aruco_dict,size_of_marker = arucomarkers.getArucoInfo();
        # arucoParams = arucomarkers.getArucoDetector();
        
        # # # camera calibration
        # # camera_matrix, dist_coeffs = camera_calibration.load_coefficients(datadir+'calib.yaml');
        # h,  w = self.images['aruco'].shape[:2]
        # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix,dist_coeffs,(w,h),1,(w,h))
        
        # # read aruco registration image and detect markers
        # #frame = cv2.imread(mycapdir + "/aruco_img.png")
        # frame = self.images['aruco'];
        
        # gray = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY)
        # corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters = arucoParams)
        # frame2 = aruco.drawDetectedMarkers(frame.copy(), corners, ids, (0,255,0));
        
        # print('Finding poses for markers',ids.flatten().tolist());
        # rvecs, tvecs, _objpts = aruco.estimatePoseSingleMarkers(corners, size_of_marker , camera_matrix, dist_coeffs);
        
        # select the marker we want
        idx = np.where(self.ids==markerid_to_search)[0][0]
        rvec = self.rvecs[idx];
        tvec = self.tvecs[idx];
        
        #draw axis
        length_of_axis = size_of_marker/2;
        frame2 = aruco.drawAxis(self.arucoframe, camera_matrix, dist_coeffs, rvec, tvec, length_of_axis)
        if(show):
            cv2.imshow("registration",frame2);
        
        # remember the relevant tvec/rvec
        #self.rvecs = rvecs;
        #self.tvecs = tvecs;
        #self.ids = ids;
        self.arucoframe = frame2;
        #self.newcameramtx = newcameramtx;
        
        # a particular aruco marker
        self.rvec = rvec;
        self.tvec = tvec;
        print('rvec',rvec);
        print('tvec',tvec);
        
        self.morePoseCalculations();
        
        
    def morePoseCalculations(self):
        rvec = self.rvec[0];
        tvec = self.tvec[0];
        
        #https://www.youtube.com/watch?v=a17fK5kG2NI&ab_channel=GOPM
        R, _ = cv2.Rodrigues(rvec);
        Rinv = np.linalg.inv(R);
        #Rinv = R.T   # this is the same result as inversion in above line
        
        # camera position expressed in world frame OXYZ coordinates
        #P = -Rinv * tvec;
        P = np.matmul(-Rinv,tvec);
        #P = -np.matrix(Rinv)*np.matrix(tvec)np.np
        #P = 
        
        # projection matrix
        Pmat = np.matrix(np.hstack((R,self.tvec.T)));
        
        #thetax, thetay, thetaz = rotationMatrixToEulerAngles(R)
        # theta = math.sqrt(rvec[0]**2 + rvec[1]**2 + rvec[2]**2);
        # v = rvec/theta;
        
        # euler decomposition
        _,_,_,_,_,_,euler_angles_degrees = cv2.decomposeProjectionMatrix(Pmat);
        euler_angles_degrees*=-1;
        
        self.P = P;
        self.Pmat = Pmat;
        self.Rmat = R;
        self.Rmatinv = Rinv;
        self.euler_angles_degrees = euler_angles_degrees;
        
    def fileList(self):
        imgson = np.array([Path(self.capdir + "/" + f) for f in os.listdir(self.capdir) if f.endswith("imgon.png") and not f.startswith("aruco") ]);
        imgsoff= np.array([Path(self.capdir + "/" + f) for f in os.listdir(self.capdir) if f.endswith("imgoff.png") and not f.startswith("aruco")]);
        # if(len(images)>0):
        #     file_img_all = images[0];
        # else:
        #     file_img_all = file_img_aruco;
        
        universes = [(x.name.split('_')[1].replace('U','')) for x in imgson];
        index = [(x.name.split('_')[2]) for x in imgson];
        elems = [x.name.split('_')[0][3:] for x in imgson if x.name.find('aruco')];
        plist = [(int(x),int(y)) for x,y in zip(universes,index)];
        
        
        self.imgson = imgson;
        self.imgsoff = imgsoff;
        self.unis = universes;
        self.index = index;
        self.elems = elems;
        self.plist = plist;
        
    def projectLine(self,universe,index):
        """
        Projects this pixel position's location onto the aruco plane
        computed from aruco marker camera pose information.
        
        Returns line segment between camera and plane in real-world aruco coordinates.

        Parameters
        ----------
        universe : TYPE
            DESCRIPTION.
        index : TYPE
            DESCRIPTION.

        Returns
        -------
        P : TYPE
            DESCRIPTION.
        P1 : TYPE
            DESCRIPTION.

        """
        Pmat = self.Pmat; # projection matrix to project onto real-world aruco plane
        
        # test
        # now map the image pixel location to the right-angle ruler coordinate system
        # help from:
            # https://stackoverflow.com/questions/34140150/reverse-of-opencv-projectpoints
        
        # projection matrix
        mtx = self.newcameramtx;
        #Lcam=mtx.dot(np.hstack((R,ds.tvec.T)));
        Lcam=mtx.dot(Pmat);
        
        # pixell locations
        px=self.data.loc[(universe,index)]['x']
        py=self.data.loc[(universe,index)]['y']
        # px=1000
        # py=1019
        
        # if px and py == 0, then no pixel was detected/found/visible in this view
        # return early, None
        if( (px == 0) and (py == 0)):
            return None,None;
        
        # defined on Z=0 real-world plane
        Z=0
        
        # calculate
        X=np.linalg.inv(np.hstack((Lcam[:,0:2],np.array([[-1*px],[-1*py],[-1]])))).dot((-Z*Lcam[:,2]-Lcam[:,3]))
        
        
        #myline = vedo.shapes.Line(p0=P,p1=[X[0],X[1],0]);
        #P1 = np.array([X[0], X[1],0]);
        #P1 = np.array([X[0][0], X[1][0],0]);
        #P1 = np.array([np.array(X[0])[0], np.array(X[1])[0], 0])
        
        
        P1 = np.array([X[0].tolist()[0][0], X[1].tolist()[0][0], 0]);
        
        # line goes from P to P1 
        
        return self.P,P1

    
datasets = [];
for c in captures:
    print('Capture',c)
    mycapdir = elementcapdir+""+c;

    file_img_aruco = mycapdir + "/" + "aruco_img.png";
    
    images = np.array([mycapdir + "/" + f for f in os.listdir(mycapdir) if f.endswith("allon.png") ])
    if(len(images)>0):
        file_img_all = images[0];
    else:
        file_img_all = file_img_aruco;
    
    # load images
    im_aruco = cv2.imread(file_img_aruco);
    im_all = cv2.imread(file_img_all);
     
    # load csv datas
    datafile = elementcapdir+"data_{:s}.csv".format(c);
    if(os.path.exists(datafile)):
        print('Loading ',datafile);
        #data = pd.read_csv(datafile,index_col=0)
        data = pd.read_csv(datafile)
        data = data.set_index(['universe','index']);
    else:
        data= None;
    
    obj = CaptureSet(
        name = c,
        capdir = mycapdir,
        images = {'aruco':im_aruco,'all':im_all},
        data = data
        );
    obj.fileList();
    obj.findArucoPoses(show=False);
    #obj.poseCalculate()
    datasets.append(obj);
    
#%% For each capture, what aruco marker is being shown?
for ds in datasets:
    print('Cap',ds.name,'values',ds.ids.T[0].tolist())
ids_detected = np.hstack([ds.ids.T[0] for ds in datasets])
print('Detected markers',ids_detected)
counts = np.bincount(ids_detected);
modeval = np.argmax(counts);
modevalcnt = counts[modeval];
print('Most frequent marker {:d} (found {:d} times)'.format(modeval,modevalcnt))
modefoundin = [ds for ds in datasets if modeval in ds.ids]
modenotfoundin = [ds for ds in datasets if (ds not in modefoundin)]
print('caps',[ds.name for ds in modenotfoundin],'does not have the mode value')


#%% for those captures where the mode marker was not found, try to find a relationship to it
# relative positioning o aruco markers
# https://aliyasineser.medium.com/calculation-relative-positions-of-aruco-markers-eee9cc4036e3
def inversePerspective(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    R = np.matrix(R).T
    invTvec = np.dot(R, np.matrix(-tvec))
    invRvec, _ = cv2.Rodrigues(R)
    return invRvec, invTvec
def relativePosition(rvec1, tvec1, rvec2, tvec2):
    """ Get relative position for rvec2 & tvec2. Compose the returned rvec & tvec to use composeRT with rvec2 & tvec2 """
    rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape((3, 1))
    rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))
    # Inverse the second marker
    invRvec, invTvec = inversePerspective(rvec2, tvec2)
    info = cv2.composeRT(rvec1, tvec1, invRvec, invTvec)
    composedRvec, composedTvec = info[0], info[1]
    composedRvec = composedRvec.reshape((3, 1))
    composedTvec = composedTvec.reshape((3, 1))
    return composedRvec, composedTvec
# https://stackoverflow.com/questions/58355040/how-to-transform-camera-pose-between-aruco-markers
for ds in modenotfoundin:
    # find
    #otherids = np.hstack([x.ids.T[0] for x in datasets if x != ds]);
    #other_ds_with_our_ids = [x for x in datasets if ds.ids in x.ids and x != ds]
    other_ds_with_our_ids = [
        x for x in datasets
        if len(set(ds.ids.flatten().tolist()).intersection(x.ids.flatten().tolist()))>0 and 
        x != ds
    ]
    
    # can't continue?
    if(len(other_ds_with_our_ids)==0):
        raise ValueError("Cannot find another capture with overlapping markers")
    
    # continue
    print('Working on non-conforming cap',ds.name, 'which has', ds.ids.T[0].tolist());
    print('\talso in caps:',[x.name for x in other_ds_with_our_ids])
    otherids = np.hstack([x.ids.T[0] for x in other_ds_with_our_ids])
    otherids = otherids[otherids != modeval] # remove the modeval
    # otherids = otherids[otherids == ds.ids.T[0]]
    #sotherids = set(otherids);
    #sotherids.discard(modeval);
    counts2 = np.bincount(otherids);
    mode2val = np.argmax(counts2);
    mode2valvalcnt = counts2[mode2val];
    reduce_to_caps = [
        x for x in other_ds_with_our_ids
        if mode2val in x.ids
    ]
    print('\t=> refer {:d} from {:s} to entire-set mode {:d}'.format(mode2val,
                                str([x.name for x in reduce_to_caps]),modeval))
    print('\t====>>>',[x.name for x in reduce_to_caps])
    print('-'*20);

    for x2 in reduce_to_caps:
        print('\t\t cap {:s} compose {:d} to {:d}'.format(x2.name,mode2val,modeval))
        # select the marker we want
        idx = np.where(x2.ids==modeval)[0][0]
        print('\t'*3 + 'marker {:d} = idx {:d}'.format(modeval,idx))
        firstRvec = x2.rvecs[idx];
        firstTvec = x2.tvecs[idx];

        idx = np.where(x2.ids==mode2val)[0][0]
        print('\t'*3 + 'marker {:d} = idx {:d}'.format(mode2val,idx))
        secondRvec = x2.rvecs[idx];
        secondTvec = x2.tvecs[idx];
        
        print('\t'*3 + 'tvec1',firstTvec)
        print('\t'*3 + 'tvec2',secondTvec)
        
        ##################
        # firstRvec, firstTvec = firstRvec.reshape((3, 1)), firstTvec.reshape((3, 1))
        # secondRvec, secondTvec = secondRvec.reshape((3, 1)), secondTvec.reshape((3, 1))
    
        # composedRvec, composedTvec = relativePosition(firstRvec, firstTvec, secondRvec, secondTvec)
        # print('\t\t\t',composedRvec.T)
        # print('\t\t\t',composedTvec.T)
        ##################
        
        # tvec1 = np.transpose(firstTvec[0]) # tvec of 'm' marker
        # tvec2 = np.transpose(secondTvec[0]) # tvec of 'n' marker
        tvec1 = firstTvec[0];
        tvec2 = secondTvec[0];
        dtvec = tvec1 - tvec2 # vector from 'm' to 'n' marker in the camera's coordinate system
        print('\t'*3 + 'dtvec=',dtvec)
        print('\t'*3 + 'dtvecnorm=',np.linalg.norm(dtvec))
        
        # get the markers' rotation matrices respectively
        rvec1= firstRvec;
        rvec2= secondRvec;
        R1 = cv2.Rodrigues(rvec1[0])[0]
        R2 = cv2.Rodrigues(rvec2[0])[0]
        
        # tvec11 = np.matmul(-R1.T, tvec1) # camera pose in 'm' 1 marker's coordinate system
        # tvec22 = np.matmul(-R2.T, tvec2) # camera pose in 'n' 2 marker's coordinate system
        tvec11 = np.matmul(-R1.T, tvec1) # camera pose in 'm' 1 marker's coordinate system
        tvec22 = np.matmul(-R2.T, tvec2) # camera pose in 'n' 2 marker's coordinate system
        print('\t'*3 + 'cam pose in 1 marker system=',tvec11)
        print('\t'*3 + 'cam pose in 2 marker system=',tvec22)
        
        # translational difference between markers in 'm' 1 marker's system,
        # basically the origin of 'n' 2
        dtvec1 = np.matmul(-R1.T, dtvec)
        print('\t'*3 + 'diff between markers 1 and 2 in 1 coordinate system=',dtvec1)
        
        # this gets me the same as tvec_mm,
        # but this only works, if 'm' marker is seen
        #tvec_nm = dtvec_m + np.matmul(-R_m.T, tvec_n)
        tvec21a = dtvec1 + np.matmul(-R1.T, tvec2)
        print('\t'*3 + 'tvec21a=',tvec21a);
        
        # something with the rvec difference must give the transformation(???)
        # drvec = rvec_m-rvec_n
        # drvec_m = np.transpose(np.matmul(R_m.T, np.transpose(drvec))) # transformed to 'm' 1 marker
        # dR_m = cv2.Rodrigues(drvec_m)[0]
        drvec = rvec1-rvec2
        drvec_1 = np.transpose(np.matmul(R1.T, np.transpose(drvec))) # transformed to 'm' 1 marker
        dR_1 = cv2.Rodrigues(drvec_1)[0]
        dR_1b = R1.dot(R2.T)
        # print('\t'*3 + 'dR_1 =',dR_1)
        # print('\t'*3 + 'dR_1b=',dR_1b)
        
        # I want to transform tvec_nn 22 with a single matrix,
        # so it would be interpreted in 'm' 1  marker's system
        tvec21b = dtvec1 + np.matmul(dR_1.T, tvec22)
        print('\t'*3 + 'tvec21b=',tvec21b)
        

#%% Show poses
nrows = int(len(datasets)/2);
ncols = int(len(datasets)/int(len(datasets)/2));
if(nrows+ncols<len(datasets)):
    ncols += len(datasets)-(nrows+ncols)
fig, ax_arr = plt.subplots(nrows,ncols, sharex=True, sharey=True);
axs = ax_arr.flatten();
for cnt,ds in enumerate(datasets):
    print(cnt)
    ax = axs[cnt];
    #d = datasets[cnt];
    ds.poseCalculate(modeval);
    #ax.imshow(d.images['aruco'])
    ax.imshow(ds.arucoframe)
    ax.set_title(ds.name)
fig.tight_layout();
    
    

#%% notesz-component of tvec is distance from camera to the aruco marker
# https://stackoverflow.com/questions/68019526/how-can-i-get-the-distance-from-my-camera-to-an-opencv-aruco-marker


#%% notes - PnP diagram
# https://doughtmw.github.io/posts/ArUco-Detection-HoloLens-4

#%% undergrad project related to what i'm trying to do
# https://courses.ece.cornell.edu/ece5990/ECE5725_Fall2020_Projects/Dec_21_Demo/Drawing%20Robot/eam348_mm2994_W/index.html

# aruco marker layman document
# https://docs.google.com/document/d/1QU9KoBtjSM2kF6ITOjQ76xqL7H0TEtXriJX5kwi9Kgc/edit#

# tst
# https://miaodx.github.io/blogs/unrealcv_digest/camera_pose/

#%% project 2d point to 3d point
# https://stackoverflow.com/questions/34140150/reverse-of-opencv-projectpoints
# https://stackoverflow.com/questions/12299870/computing-x-y-coordinate-3d-from-image-point

#%%
# https://learnopencv.com/rotation-matrix-to-euler-angles/
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6
    print('Singular',singular)

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

#%% vedo test

import math
cones = [];
otheractors = [];
lines = [];

universe = 2000
index = 60;

for ds in datasets:
    # plot camera pose representation as a cone
    P = ds.P;
    euler_angles_degrees = ds.euler_angles_degrees;
    cone = vedo.shapes.Cone(P,r=100,height=165,axis=(1,0,0));
    cone = cone.rotateX(euler_angles_degrees[0],rad=False,locally=True);
    cone = cone.rotateY(-euler_angles_degrees[1],rad=False,locally=True);
    cone = cone.rotateY(euler_angles_degrees[2],rad=False,locally=True);
    cones.append(cone);
    
    
    P,P1 = ds.projectLine(universe, index);
    #myline = vedo.shapes.Line(p0=P,p1=[X[0],X[1],0]);
    myline = vedo.shapes.Line(p0=P,p1=P1);
    lines.append(myline);

    
origin_aruco = vedo.shapes.Rectangle(p1=(0,0),p2=(165,165),c='white'); #xy plane
origin_aruco = origin_aruco.rotateX(0);

# intersection of all the lines
P0 = np.zeros((len(lines),3),float)
P1 = np.zeros((len(lines),3),float)
for cnt,l in enumerate(lines):
    p0 = l.points()[0];
    p1 = l.points()[1];
    P0[cnt,:] = p0;
    P1[cnt,:] = p1;
closest_intersection = intersect(P0,P1);
pclosest = vedo.shapes.Sphere(closest_intersection.T.tolist()[0],r=20)

title = vedo.shapes.Text2D('Element {:s}\nPixel -> Universe: {:d} , Index: {:}'.format(element,universe,index), s=0.9, c='white');
p = vedo.Plotter(bg='black',axes=1);
actors_to_show = [origin_aruco]+cones+lines+[pclosest,title];
p.show(actors_to_show);
p.close();
#vedo.interactive().close()

# test
#elementcapdir

#%% Main Loop - find closest points
# assume that first capture contains entirety of universe,index list
#pixellist = datasets[0].data.index.tolist();
pixellist = datasets[0].plist;

# select an element
if False:
    df = pd.DataFrame({'plist':pixellist,'elems':datasets[0].elems})
    elemselect = 'bigbush-top';
    #elemselect = 'bigbush-south';
    #r = df.query("elems == 'Bush R1'");
    #r = df.query("elems == 'Bush R2'");
    r = df.query("elems == @elemselect");
    #r = df.query("elems == 'bigbush-south'");
    pixellist = r['plist'].tolist();
else:
    elemselect = None;

closest_point = [];
point_spheres = [];
successful_triangulation = [];
for cnt in range(len(pixellist)):
    universe,index = pixellist[cnt];
    print(cnt,universe,index);
    
    # loop through datasets and get the pixels' line projection to the work coordinate system
    lineP0 = [];
    lineP1 = [];
    #for ds in datasets:
    #for ds in datasets[1:]:     #bigbush-top
    for ds in [datasets[0],datasets[2]]: # bigbush-south, bigbush-top
    #for ds in [datasets[0],datasets[2]]: # bigbush-south
    #for ds in [datasets[0],datasets[2],datasets[4]]:
    #for ds in datasets[0:3]: # bigbushnorth
    #for ds in [ds for ds in datasets if ds.name in ['00','03','07']]:
    #for ds in [ds for ds in datasets if ds.name in ['03','04','07']]:
    #for ds in [ds for ds in datasets if ds.name in ['00','04','07']]: # bush R1
    #for ds in [ds for ds in datasets if ds.name in ['00','04','07']]: # bush R2
    #for ds in [ds for ds in datasets if ds.name in ['00','01','02','04','07']]: # bush R2
        P,P1 = ds.projectLine(universe, index);
        #myline = vedo.shapes.Line(p0=P,p1=[X[0],X[1],0]);
        #myline = vedo.shapes.Line(p0=P,p1=P1);
        #lines.append(myline);
        if(P is not None):
            lineP0.append(P);
            lineP1.append(P1);
        else:
            print('\tcapture={:s} not found'.format(ds.name))
    
    # ensure at least 2 
    if( len(lineP0)>=2 ):
        P0 = np.vstack(lineP0);
        P1 = np.vstack(lineP1);
        closest_intersection = intersect(P0,P1);
        sphere_closest = vedo.shapes.Sphere(closest_intersection.T.tolist()[0],r=15)
        #points.append(sphere_closest);
        
        closest_point.append(closest_intersection.T[0]);
        point_spheres.append(sphere_closest);
        successful_triangulation.append(True);
    
    else:
        print('\tNot enough captures saw this pixel.');
        successful_triangulation.append(False);
        closest_point.append(None);
        point_spheres.append(None);
        #print('\t\tUse geometric means');
        #n = cnt-1;
        #m = cnt+1;
        
        #closest_point.append(np.array((0.0,0.0,0.0)))
        #closest_point.append(np.array((np.nan,np.nan,np.nan)))
    #break;

#%% those without triangulations, use geometric mean of surrounding ones
for cnt in range(len(pixellist)):
    if(successful_triangulation[cnt]):
        continue;
    universe,index = pixellist[cnt];
    print(cnt,universe,index);
    print('\tNot enough captures saw this pixel.');
    nd = 1;
    n = cnt-nd;
    while(closest_point[n] is None):
        nd += 1;
        n = cnt-nd;
    mi = 1;
    m = cnt+mi;
    while(m<len(pixellist) and closest_point[m] is None):
        mi += 1;
        m = cnt+mi;
    if(m==len(pixellist)):
        #find last non-none entry and use that, offset by 20 in each direction
        for cnt2,p in enumerate(closest_point[::-1]):
            if p is not None:
                
                #avg = p;
                #print('\t\tReplace index {:d}'.format(len(pixellist)-cnt2));
                nowpoint = len(pixellist)-cnt2-2;
                prevpoint = len(pixellist)-cnt2-3;
                p1 = closest_point[nowpoint];
                p2 = closest_point[prevpoint];
                dv = p2-p1;
                avg = p1-dv;
                print('\t\tReplace with dv between {:d} and {:d} {:s}'.format(nowpoint,prevpoint,str(dv)));
                
                break;
    else:
        print('\t\tReplace with geometric means between {:d}({:d}) and {:d}({:d})'.format(n,nd,m,mi));
        avg = np.mean(np.vstack([closest_point[n],closest_point[m]]),axis=0);
    
    
    closest_point[cnt] = avg;
    point_spheres[cnt] = vedo.shapes.Sphere(avg,r=15,c='yellow');


# tweaks to manually modify an outlier
if(model=='bigbush-south,bigbush-top' and elemselect == 'bigbush-south'):
    #bigbush-south
    # manually modify
    pxn = 42;
    closest_point[pxn][1] = -500;
    # pnt = closest_point[pxn]
    # point_spheres[pxn] = vedo.shapes.Sphere(pnt,r=15,c='blue');
    
elif(model=='bigbush-south,bigbush-top' and elemselect == 'bigbush-top'):
    #bigbush-south
    # manually modify
    pxn = 165;
    closest_point[pxn][1] = 150;
    print('Manual point tweaks for',elemselect)

# make spheres
#pts = np.vstack(closest_point);
    
#%% (optional) FIT TOPLANE
if True:
    pts = np.vstack(closest_point);
    pts2 = vedo.pointcloud.removeOutliers(pts, 300,neighbors=5);
    plane = vedo.fitPlane(pts2)
    plane = plane.scale(1.2);
    
    pts3 = [];
    # project onto plane
    for p1 in pts:
        #print(p1)
        pA = p1+3000*plane.normal;
        pB = p1-3000*plane.normal;
        raytrace = plane.intersectWithLine(p0=pA,p1=pB);
        pts3.append(raytrace[0]);
        #raise RuntimeError();
    
    cones = [];
    for ds in datasets:
        # plot camera pose representation as a cone
        P = ds.P;
        euler_angles_degrees = ds.euler_angles_degrees;
        cone = vedo.shapes.Cone(P,r=100,height=165,axis=(1,0,0));
        cone = cone.rotateX(euler_angles_degrees[0],rad=False,locally=True);
        cone = cone.rotateY(-euler_angles_degrees[1],rad=False,locally=True);
        cone = cone.rotateY(euler_angles_degrees[2],rad=False,locally=True);
        cones.append(cone);
    
    origin_aruco = vedo.shapes.Rectangle(p1=(0,0),p2=(165,165),c='white'); #xy plane
    origin_aruco = origin_aruco.rotateX(0);
    
    title = vedo.shapes.Text2D('Element {:s}'.format(element), s=0.9, c='white');
    p = vedo.Plotter(bg='black',axes=1);

    # rotation
    import scipy.spatial.transform
    R = scipy.spatial.transform.Rotation.align_vectors(
        np.array([plane.normal]), np.array([[0.0,0.0,1.0]])
        #np.array([plane.normal]), np.array([[0.0,1.0,0.0]])
        );
    pts4 = np.vstack(pts3).dot(R[0].as_matrix())
    #pt = vedo.shapes.Points(pts3,c='red');
    pt = vedo.shapes.Points(pts4,c='red');
    
    actors_to_show = [title,origin_aruco]+cones+[plane,pt];
    p.show(actors_to_show);
    p.close();
    

    
    # out = np.vstack(pts3);
    # from pypixelmapper.xlightsmodel import outputXlightsModelFile
    # outputXlightsModelFile(out,pixellist,model+'fittoplane');
    
    out = np.vstack(pts4);
    from pypixelmapper.xlightsmodel import outputXlightsModelFile
    #outputXlightsModelFile(out,pixellist,model+'fittoplanerot');


#%% vedo show rendering of points and situation
cones = [];
for ds in datasets:
    # plot camera pose representation as a cone
    P = ds.P;
    euler_angles_degrees = ds.euler_angles_degrees;
    cone = vedo.shapes.Cone(P,r=100,height=165,axis=(1,0,0));
    cone = cone.rotateX(euler_angles_degrees[0],rad=False,locally=True);
    cone = cone.rotateY(-euler_angles_degrees[1],rad=False,locally=True);
    cone = cone.rotateY(euler_angles_degrees[2],rad=False,locally=True);
    cones.append(cone);

origin_aruco = vedo.shapes.Rectangle(p1=(0,0),p2=(165,165),c='white'); #xy plane
origin_aruco = origin_aruco.rotateX(0);

title = vedo.shapes.Text2D('Element {:s}'.format(element), s=0.9, c='white');
p = vedo.Plotter(bg='black',axes=1);
#actors_to_show = [title,origin_aruco]+cones+point_spheres;
pts_vedo = vedo.shapes.Points(np.vstack(closest_point),c='red');
actors_to_show = [title,origin_aruco]+cones+[pts_vedo];
p.show(actors_to_show);
p.close();

#%% vedo show rendering of points and situation
cones = [];
for ds in datasets:
    # plot camera pose representation as a cone
    P = ds.P;
    euler_angles_degrees = ds.euler_angles_degrees;
    cone = vedo.shapes.Cone(P,r=100,height=165,axis=(1,0,0));
    cone = cone.rotateX(euler_angles_degrees[0],rad=False,locally=True);
    cone = cone.rotateY(-euler_angles_degrees[1],rad=False,locally=True);
    cone = cone.rotateY(euler_angles_degrees[2],rad=False,locally=True);
    cones.append(cone);

origin_aruco = vedo.shapes.Rectangle(p1=(0,0),p2=(165,165),c='white'); #xy plane
origin_aruco = origin_aruco.rotateX(0);

title = vedo.shapes.Text2D('Element {:s}'.format(element), s=0.9, c='white');
p = vedo.Plotter(bg='black',axes=1,interactive=False);
mypoints = copy.copy(point_spheres);
actors_to_show = [title,origin_aruco]+cones+mypoints;

curr = 0;
def myfnc(evt):
    global curr;
    mesh = evt.actor;
    #print(evt.keyPressed);
    #return;
    if evt.keyPressed == "Right":
        curr+=1;
        universe,pixel = pixellist[curr]
        point = mypoints[curr];
        
        print(universe,pixel);
        
        point.color('yellow');
        if(curr!=1):
            mypoints[curr-1].color('red');
        
        #print("click mesh and press c")
        
        if(curr>len(pixellist)):
            print('Finished!')
            curr -= 1;
            
        title.text('Element {:s} - Uni:{:d} Index:{:d}'.format(element,universe,pixel));
        #return
    # printc("mesh :", mesh.filename, c=mesh.color())
    # printc("point:", mesh.picked3d, c="v")
    # cpt = Point(pos=mesh.picked3d, r=20, c="v").pickable(False)
    # plt.add(cpt)
p.addCallback('KeyPress', myfnc)
p.show(actors_to_show);

# for (universe,pixel),point in zip(pixellist,point_spheres):
#     print(universe,pixel);
#     if(p.escaped):
#         break;
    
vedo.interactive();

p.close();

#%% write as xlights model
out = np.vstack(closest_point);
from pypixelmapper.xlightsmodel import outputXlightsModelFile

#outputXlightsModelFile(out,pixellist,model);
#outputXlightsModelFile(out,pixellist,model,scalefactor=5.0);
outputXlightsModelFile(out,pixellist,model,scalefactor=10);
#outputXlightsModelFile(out,pixellist,'test1');

#%% Test Calculate Line Intersections

P0 = np.zeros((len(lines),3),float)
P1 = np.zeros((len(lines),3),float)
for cnt,l in enumerate(lines):
    p0 = l.points()[0];
    p1 = l.points()[1];
    P0[cnt,:] = p0;
    P1[cnt,:] = p1;
closest_intersection = intersect(P0,P1);

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
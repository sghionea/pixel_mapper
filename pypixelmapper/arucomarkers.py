# -*- coding: utf-8 -*-
"""
Created on Sun Oct 24 06:24:55 2021

@author: SimonGhionea
"""
import cv2
from cv2 import aruco

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50);

def getCharucoInfo():
    #aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50);
    # board = aruco.CharucoBoard_create(9, 6, 15, 12, aruco_dict)
    
    squareLength = 15   # Here, our measurement unit is mm
    markerLength = 12   # Here, our measurement unit is mm
    board = aruco.CharucoBoard_create(9, 6, squareLength, markerLength, aruco_dict)
    
    imboard = board.draw((2000, 2000))
    #cv2.imwrite(workdir + "chessboard.tiff", imboard);
    
    return aruco_dict, board, imboard;

def getArucoInfo():
    size_of_marker = 175;
    
    return aruco_dict, size_of_marker;

def getArucoDetector():
    #Meanwhile, create aruco detector with default parameters.
    arucoParams = aruco.DetectorParameters_create();
    
    return arucoParams;
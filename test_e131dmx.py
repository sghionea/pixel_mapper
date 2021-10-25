# -*- coding: utf-8 -*-
"""
Created on Sat Oct 23 20:25:05 2021

@author: SimonGhionea
"""

import cv2
import socket   
import sys  
import json
import time
from lumos import DMXSource
import _thread
import matplotlib.path as pltPath
#from playsound import playsound

#%% Notes
"""
Test DMX stuff
"""

#%%
class Universe:
	def __init__(self,source,channelcount):
		self.source = source
		self.pixelcount = int(channelcount/3)

#EDIT THESE LINES AS NEEDED******************************************************
#Configure the Universes to send to when controlling the pixels
#format is as follows:
#U1 = Universe(DMXSource(universe=UNIVERSE NUMBER),NUMBER OF CHANNELS IN THE UNIVERSE)  #for RGB pixels, there are three channels per pixel
U1 = Universe(DMXSource(universe=4000),512);
#U2 = Universe(DMXSource(universe=4001),512);
#U3 = Universe(DMXSource(universe=4002),512);
# U4 = Universe(DMXSource(universe=2003),510)
# U5 = Universe(DMXSource(universe=2004),60)
#universes = [U1,U2,U3,U4,U5]
universes = [U1]
totalpixels = 100                 #total number of pixels to map
onval = [100,100,100]             #RGB value to use when turning the pixels on for detection

#%%
def all_off():
	#shut all pixels off
	for universe in universes:
		universe.source.send_data(data=[0,0,0]*universe.pixelcount)
def everyother():
	#turn every other pixel on - for testing
	for universe in universes:
		universe.source.send_data(data=onval*universe.pixelcount)
		counter = 0
		for i,element in enumerate(data):
			if counter == 3 or counter == 4 or counter == 5:
				data[i] = 0
			counter = counter + 1
			if counter > 5:
				counter = 0
def all_on():
	#turn all pixels on white
	for universe in universes:
		universe.source.send_data(data=onval*universe.pixelcount)
        
#%%
all_on();
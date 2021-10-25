# -*- coding: utf-8 -*-
"""
Created on Sat Oct 23 20:25:05 2021

@author: SimonGhionea
"""
import sacn
import time
from dataclasses import dataclass

#%% Notes
"""
Test DMX stuff
"""

#%% setup sacn sender and universes
class Universe():
    universe : int;
    pixelcount : int;
    def __init__(self,universe,channelcount):
        self.universe = universe;
        self.pixelcount = int(channelcount/3)

#EDIT THESE LINES AS NEEDED******************************************************
#Configure the Universes to send to when controlling the pixels
#format is as follows:
#U1 = Universe(DMXSource(universe=UNIVERSE NUMBER),NUMBER OF CHANNELS IN THE UNIVERSE)  #for RGB pixels, there are three channels per pixel
U1 = Universe(universe=4000,channelcount=300);
#universes = [U1,U2,U3,U4,U5]
universes = [U1]
totalpixels = 100                 #total number of pixels to map
#onval = [100,100,100]             #RGB value to use when turning the pixels on for detection
onval = [25,25,25]             #RGB value to use when turning the pixels on for detection


sender = sacn.sACNsender(fps=40)  # provide an IP-Address to bind to if you are using Windows and want to use multicast
sender.start()  # start the sending thread
#sender.activate_output(4000)  # start sending out data in the 1st universe

# start sending out data in the defined universes
for u in universes:
    sender.activate_output(u.universe);
    #sender[u.universe].multicast = True  # set multicast to True
    sender[u.universe].destination = "fpp.lan"
    # Keep in mind that if multicast is on, unicast is not used

# #sender[1].multicast = True  # set multicast to True
# sender[1].destination = "fpp.lan"  # or provide unicast information.
# # Keep in mind that if multicast is on, unicast is not used
# sender[1].dmx_data = (1, 2, 3, 4)  # some test DMX data

# time.sleep(10)  # send the data for 10 seconds
# sender.stop()  # do not forget to stop the sender


#%%

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
    for u in universes:
        sender[u.universe].dmx_data = (onval*u.pixelcount);
def all_off():
    #shut all pixels off
    for u in universes:
        sender[u.universe].dmx_data = ([0,0,0]*u.pixelcount);
def test_chase():
    for u in universes:
        pxcnt = u.pixelcount;
        for px in range(pxcnt):
            #print(px)
            #print(px,[0,0,0]*px + onval + [0,0,0]*(pxcnt-px-1));
            sender[u.universe].dmx_data = [0,0,0]*px + onval + [0,0,0]*(pxcnt-px-1);
            time.sleep(0.1);
    all_off();
        
#%%
all_on();

#%%
all_off();

#%%
test_chase();

#%% stop sacn
sender.stop()  # do not forget to stop the sender
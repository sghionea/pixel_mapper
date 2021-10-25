# -*- coding: utf-8 -*-
"""
Created on Sun Oct 24 22:09:56 2021

@author: SimonGhionea
"""

import sacn
import time

class Universe():
    universe : int;
    pixelcount : int;
    def __init__(self,universe,channelcount):
        self.universe = universe;
        self.pixelcount = int(channelcount/3)
        
class Lights():
    _universes = [];
    _pixelcount = 0;
    _sender = None;
    
    _step_current_universe_index = 0;
    _step_current_pixel_index = 0;
    
    def __init__(self, universe_list : list,pixelcount=100):
        self._universes = universe_list;
        self._pixelcount = pixelcount;
        
    def start(self,fps=40):
        sender = sacn.sACNsender(fps=fps)  # provide an IP-Address to bind to if you are using Windows and want to use multicast
        sender.start()  # start the sending thread
        #sender.activate_output(4000)  # start sending out data in the 1st universe
        
        # start sending out data in the defined universes
        for u in self._universes:
            sender.activate_output(u.universe);
            #sender[u.universe].multicast = True  # set multicast to True
            sender[u.universe].destination = "fpp.lan"
            # Keep in mind that if multicast is on, unicast is not used
        
        self._sender = sender;
        
    def stop(self):
        self._sender.stop();
    
    def all_on(self,onval = [150,150,150]):
        #turn all pixels on white
        for u in self._universes:
            self._sender[u.universe].dmx_data = (onval*u.pixelcount);
            
    def all_off(self):
        #shut all pixels off
        for u in self._universes:
            self._sender[u.universe].dmx_data = ([0,0,0]*u.pixelcount);
            
    def set_one(self, u : Universe, pxnum, onval = [150,150,150]):
        pxcnt = u.pixelcount;
        self._sender[u.universe].dmx_data = [0,0,0]*pxnum + onval + [0,0,0]*(pxcnt-pxnum-1);
    
    def set_next_reset(self):
        self._step_current_universe_index = 0;
        self._step_current_pixel_index = 0;

    def set_next(self, onval = [150,150,150]):
        uidx = self._step_current_universe_index
        if( uidx < len(self._universes)):
            u = self._universes[uidx];
            
            self.set_one(u,self._step_current_pixel_index,onval);
            
            if(self._step_current_pixel_index+1>u.pixelcount):
                # increment universe
                self._step_current_pixel_index = 0;
                self._step_current_universe_index += 1;
            else:
                # increment pixel
                self._step_current_pixel_index += 1;
                
            return True;
        else:
            return False;
            
    def test_chase(self,onval = [150,150,150]):
        for u in self._universes:
            pxcnt = u.pixelcount;
            for px in range(pxcnt):
                #print(px)
                #print(px,[0,0,0]*px + onval + [0,0,0]*(pxcnt-px-1));
                self._sender[u.universe].dmx_data = [0,0,0]*px + onval + [0,0,0]*(pxcnt-px-1);
                time.sleep(0.1);
        all_off();
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 24 22:09:56 2021

@author: SimonGhionea
"""

import sacn
import time

class Universe():
    universe : int;
    channelcount : int;
    #pixelcount : int;
    #skippixels : int;
    channelstart : int;
    channelstop : int;
    universesize : int;
    
    def __init__(self,universe,channelcount,channelstart=1,universesize=512):
        self.universe = universe;
        self.channelcount = channelcount;
        #self.pixelcount = int((channelcount-channelstart)/3);
        #self.skippixels = int(channelstart/3);
        self.channelstart = channelstart;
        self.universesize = universesize;
        self.channelstop = channelstart+channelcount;
        
        print('\t\tU={:d} start={:d} count={:d} (stop={:d})'.format(self.universe,self.channelstart,self.channelcount,self.channelstop));
    
class ModelUniverses():
    name = '';
    universes = [];
    universe_ranges = [];
    startuniverse : int;
    pixelcount : int;
    #skippixels : int;
    channelcount : int;
    channelstart : int;
    universesize : int;
    
    eff_univ_start : int;
    eff_univ_stop : int;
    eff_abs_channel_start : int;
    eff_abs_channel_stop : int;
    eff_univ_channel_start : int;
    eff_univ_channel_stop : int;
    
    _step_current_universe_index = -1;
    _step_current_pixel_index = -1;
    
    _step_current_channel = 1;
    
    def __init__(self,name,universe:int,channelcount:int,channelstart=1,universesize=512):
        self.name = name;
        self.startuniverse = universe;
        self.channelcount = channelcount;
        #self.pixelcount = int((channelcount-channelstart-1)/3)+1;
        #self.skippixels = int(channelstart/3);
        self.channelstart = channelstart;
        self.universesize = universesize;
        
        self.universes = [];
        self.universe_ranges = [];
        
        self._compute();
    
    def _compute(self):
        #chns = int(self.channelcount-self.channelstart+1);
        chns = self.channelcount;
        self.eff_abs_channel_start = self.channelstart;
        self.eff_abs_channel_stop = self.channelstart + chns;
        print('-'*5 + self.name + '-'*5);
        print('Abs Chs {:d} to {:d}'.format(self.eff_abs_channel_start,self.eff_abs_channel_stop));
        print('Chns',chns);
        
        # is first channel occurring outside of our base universe?
        n_universe_increment = 0;
        n_universe_increment = int(self.channelstart/self.universesize);
        print('Uni Increment',n_universe_increment)
        relchannelstart = self.channelstart-(n_universe_increment*self.universesize)
        print('relchannelstart',relchannelstart)
        
        # # how does this break-down across universes?
        # #break_across_first = (self.channelstart + chns) > self.universesize+1;
        # universecount = int(chns/self.universesize)+1;
        # universecountrem = chns%self.universesize;
        # break_across_first = ((relchannelstart + chns) > self.universesize+1) and (universecount<2)
        # # if(universecountrem==chns):
        # #     universecount-=1; # subtract 1 if perfect fit (don't create new uni)
            
        # if(break_across_first):
        #     print('break across first')
        #     universecount+=1;
        # print('{:d} universes (rem: {:})'.format(universecount,universecountrem))
        
        self.eff_univ_start = self.startuniverse+n_universe_increment;
        #self.eff_univ_stop = self.startuniverse+n_universe_increment+universecount-1;
        #self.eff_univ_channel_start = self.eff_abs_channel_start;
        self.eff_univ_channel_start = relchannelstart;
        # if(break_across_first):
        #     #self.eff_univ_channel_stop = universecountrem-(self.universesize - self.channelstart + 1);
        #     #self.eff_univ_channel_stop = universecountrem-(self.universesize - relchannelstart + 1);
        #     self.eff_univ_channel_stop = universecountrem;
        # elif(universecount==1):
        #     self.eff_univ_channel_stop = relchannelstart+universecountrem
        # else:
        #     self.eff_univ_channel_stop = universecountrem;
            
        # print('U{:d} Ch {:d}  --> to --> U{:d} Ch {:d}'.format(
        #     self.eff_univ_start,self.eff_univ_channel_start,
        #     self.eff_univ_stop,self.eff_univ_channel_stop,
        #     ));
        print('starting point U{:d} Ch {:d}'.format(
            self.eff_univ_start,self.eff_univ_channel_start,
            ));
        
        # assign / fill universes
        lastabsch = self.eff_abs_channel_start;
        lastunich = self.eff_univ_channel_start;
        lastuni = n_universe_increment+self.startuniverse;
        remaining_channels = chns;
        while remaining_channels > 0:
            print('-Loop lastuni={:d}'.format(lastuni));
            chstart = lastunich;
            
            # will overflow this universe?
            unichns = remaining_channels
            if(lastunich+remaining_channels > self.universesize):
                print('fill to end of universe')
                unichns = self.universesize - chstart;
                lastunich = 1;
            else:
                lastunich += unichns;
                
            u = Universe(
                lastuni,
                channelcount = unichns,
                channelstart = chstart,
                universesize = self.universesize,
                );
            self.universes.append( u );
            
            # what is this universe's absolute range?
            unistart = lastabsch;
            unistop = lastabsch+unichns-1;
            rng = range(unistart,unistop,1);
            print('\t',rng);
            self.universe_ranges.append(rng);
            
            #lastch = unistop+1;
            lastabsch = unistop+1;
            #lastunich = chstart+unichns+1;
            lastuni += 1;
            remaining_channels -= unichns;
            #lastunichns = unichns;
            #remaining_channels -= unichns;
            print('remaining',remaining_channels)
                
        
        # lastch = self.eff_abs_channel_start;
        # lastunichns = 0;
        # remaining_channels = chns;
        # for i in range(universecount):
        #     chstart = 1;
        #     if(i==0 and relchannelstart != 0):
        #         #chstart = self.eff_abs_channel_start;
        #         chstart = relchannelstart;
                
        #     unichns = self.universesize;
        #     # if(universecount==1):
        #     #     unichns = chns;
        #     # elif(i==universecount-1):
        #     #     unichns = self.eff_univ_channel_stop;
        #     # if(i==0 and break_across_first):
        #     #     unichns = self.universesize - chstart+1;
        #     # elif(i==universecount-1 and universecount==2 and break_across_first):                
        #     #     unichns = self.eff_univ_channel_stop-lastunichns;
            
        #     if(not break_across_first):
        #         # 1 universe
        #         if(universecount == 1):
        #             unichns = chns;
        #         # last universe
        #         elif(i==universecount-1):
        #             unichns = self.eff_univ_channel_stop;
        #     else:
        #         if(i==0):
        #             unichns = self.universesize - chstart+1;
        #         # elif(i==universecount-1 and universecount==2):                
        #         #     unichns = self.eff_univ_channel_stop-lastunichns;
        #         elif(i==universecount-1):
        #             unichns = remaining_channels;
            
        #     u = Universe(
        #         self.eff_univ_start + i,
        #         channelcount = unichns,
        #         channelstart = chstart,
        #         universesize = self.universesize,
        #         );
        #     self.universes.append( u );
            
        #     # what is this universe's absolute range?
        #     unistart = lastch;
        #     unistop = lastch+unichns-1;
        #     rng = range(unistart,unistop,1);
        #     print('\t',rng);
        #     self.universe_ranges.append(rng);
            
        #     lastch = unistop+1;
        #     #lastunichns = unichns;
        #     remaining_channels -= unichns;
        #     print('remaining',remaining_channels)
        
        #print('{:d} rgb pixels'.format(self.pixelcount));
        print('{:d} rgb pixels'.format(int(chns/3)));
        s = int(sum([u.channelcount for u in self.universes])/3);
        print('{:d} rgb pixels'.format(s));
        self.pixelcount = int(chns/3);
        #self.eff_univ_start = self.universe;
        #self.eff_univ_stop = 
        print('-'*20);
        
    #def next_step_reset(self):
        #self._step_current_universe_index = -1;
        #self._step_current_pixel_index = -1;
    
    
    # def _pixel_to_universe(self,index):
    #     for cnt,rng in enumerate(self.universe_ranges):
    #         #print(cnt,rng);
    #         if(index*3 in rng):
    #             #print('pixel {:d} belongs to universe index {:}'.format(index,cnt));
    #             return cnt;
            
    #     return False;
    
    def _chnabs_to_universe(self,absolute_channel):
        for cnt,rng in enumerate(self.universe_ranges):
            #print(cnt,rng);
            if(absolute_channel in rng):
                #print('pixel {:d} belongs to universe index {:}'.format(index,cnt));
                return cnt;
            
        return False;
    
    
    def mapPixel(self,index):
        #
        # test
        #
        # 
        #
        
        if(index > self.pixelcount):
            raise(ValueError("Index exceeded pixel count"));
            
        # 1-based indexing in DMX world
        #index -= 1;
        
        # absolute channel
        this_channel_start = self.eff_abs_channel_start + 3*(index-0);
        #print('Asked for pixel {:d}/{:d} (chnabs={:d})'.format(index,self.pixelcount,this_channel_start));
        
        # which of our consecutive universes does this pixel belong to?
        #uidx = self._pixel_to_universe(index);
        uidx = self._chnabs_to_universe(this_channel_start);
        #print(uidx);
        u = self.universes[uidx];
        #print('pixel {:d} belongs to universe index {:d} U={:d}'.format(index,uidx,u.universe));
        
        # which channel inside the universe
        #ustart = self.universe_ranges[uidx].start;
        ustart = u.channelstart;
        #universe_size_offset = self.universe_ranges[uidx].start;
        universe_size_offset = (u.universe-self.startuniverse)*self.universesize
        # if(uidx==0):
        #     start_absolute_channel_for_univ = ustart+(this_channel_start-self.channelstart)
        # else:
        #     start_absolute_channel_for_univ = (uidx*self.universesize+1)-self.universe_ranges[uidx].start
        # if(uidx==0):
        #     start_absolute_channel_for_univ = ustart+(this_channel_start-self.channelstart)
        # else:
        #     start_absolute_channel_for_univ = (uidx*self.universesize+1)-self.universe_ranges[uidx].start
        #start_absolute_channel_for_univ = ustart+(this_channel_start-self.channelstart)
        start_absolute_channel_for_univ = this_channel_start-universe_size_offset-1
        print('Asked for px {:d}/{:d} (chnabs={:d},uidx={:d},u={:d},chnuni={:d})'.format(
            index,
            self.pixelcount,
            this_channel_start,
            uidx,
            u.universe,
            start_absolute_channel_for_univ
        ));
        
        # figure out red, green, blue
        ured = u;
        absred = start_absolute_channel_for_univ
        ugrn = u;
        absgrn = start_absolute_channel_for_univ+1;
        ublu = u;
        absblu = start_absolute_channel_for_univ+2;
        
        # catch condition where RGB crosses universe boundary
        if(absgrn >= self.universesize):
            absgrn-=self.universesize
            ugrn = self.universes[uidx+1];
        if(absblu >= self.universesize):
            absblu-=self.universesize
            ublu = self.universes[uidx+1];
            print('<<<<pixel crosses universe boundary>>>>');
            
        print('R:{:d},{:d}   G:{:d},{:d}   B:{:d},{:d}'.format(
            ured.universe,absred,
            ugrn.universe,absgrn,
            ublu.universe,absblu
        ))
        
            
        return (ured,absred,ugrn,absgrn,ublu,absblu);

    def all_ch(self,onval=100):
        #skippixels = 
        
        # returns dmx data for all_on for this model
        dmxdata_univ_dict = {};
        for u in self.universes:
            # blanks + onvalues + remainders (if any)
            blankspre = u.channelstart-1;
            numbers = u.channelcount;
            blankspost= self.universesize - (u.channelstart + u.channelcount);
            print('U{:d} preblanks:{:d} active:{:d} postblanks:{:d}'.format(
                u.universe,blankspre,numbers,blankspost)
                );
            
            # method 1
            #u_data = [0,0,0]*blankspre + (onval*u.pixelcount);
            #u_data = [0]*blankspre + [onval]*(numbers) + [0]*blankspost;
            
            # method 2
            u_data = [0]*self.universesize;
            rng = range(u.channelstart,u.channelstart+u.channelcount);
            print('Iterate',rng)
            for i in rng:
                #print(i)
                u_data[i-1] = onval;
            
            dmxdata_univ_dict[u.universe] = u_data;
        return dmxdata_univ_dict;
    
    def all_on(self, onval = 100):
        return self.all_ch(onval=onval);
    
    def all_off(self):
        return self.all_ch(onval=0);
        
        
class LightStepper():
    _models = [];
    _model_pixel_ranges = [];
    _unique_universes = set();
    
    _pixelcount = 0;
    
    _sender = None;
    _unicasthost = None;
    
    # _step_current_universe_index = -1;
    _step_current_pixel_index = 0;
    _last_step_u = 0;
    _last_step_idx = 0;
    
    def __init__(self, models : list, unicasthost="espixelstick02.lan"):
        self._models = models;
        #self._pixelcount = pixelcount;
        self._unicasthost = unicasthost;
        
        # count pixels
        self.pixelcount = sum([m.pixelcount for m in self._models]);
        print('LightStepper initialized with {:d} pixels across {:d} models'.format(self.pixelcount,len(self._models)))
        
        # set up pixel ranges (running sum)
        lastpx = 1;
        self._model_pixel_ranges = [];
        for i,count in enumerate([m.pixelcount for m in self._models]):
            thisbeg = lastpx;
            thisend = lastpx + count;
            lastpx = thisend;
            thisone = range(thisbeg,thisend);
            self._model_pixel_ranges += [thisone];
            print(i,thisone)
            
        # unique universes
        self._unique_universes = set();
        for m in self._models:
            for u in m.universes:
                self._unique_universes.add(u.universe);
        print('Unique universes:',self._unique_universes)
    
    def _px_to_mapped_model(self,index):
        for cnt,rng in enumerate(self._model_pixel_ranges):
            print(cnt,rng);
            if(index in rng):
                #print('pixel {:d} belongs to universe index {:}'.format(index,cnt));
            #    return cnt;
                print('Model Index',cnt);
                m = self._models[cnt];
                themodelpixel = index-rng.start
                ret = m.mapPixel(themodelpixel);
                
                
            
                return (m,themodelpixel,ret);
            
    def set_one(self,index,onval=[0,25,0]):
        m,px,r = self._px_to_mapped_model(index);
        
        # r, g, b
        #set_universes = set([r[0],r[2],r[4]]);
        set_universes = m.universes;
        
        # setup blank universe dicts
        u_dict = {};
        for u in set_universes:
            u_dict[u.universe] = [0]*m.universesize;
        
        # set r
        u_dict[r[0].universe][r[1]] = onval[0];
        
        # set g
        u_dict[r[2].universe][r[3]] = onval[1];
        
        # set b
        u_dict[r[4].universe][r[5]] = onval[2];
        
        # assign to sender
        for uval, vals in u_dict.items():
            print('Set {:d} to'.format(uval),vals);
            self._sender[uval].dmx_data = vals;
            

        # # method 2
        # u_data = [0]*self.universesize;
        # rng = range(u.channelstart,u.channelstart+u.channelcount);
        # #print('Iterate',rng)
        # for i in rng:
        #     #print(i)
        #     u_data[i-1] = onval;
        return m,px,r;
        
        
    def start(self,fps=40):
        sender = sacn.sACNsender(fps=fps)  # provide an IP-Address to bind to if you are using Windows and want to use multicast
        sender.start()  # start the sending thread
        #sender.activate_output(4000)  # start sending out data in the 1st universe
        
        # start sending out data in the defined universes
        for unum in self._unique_universes:
            sender.activate_output(unum);
            #sender[u.universe].multicast = True  # set multicast to True
            #sender[u.universe].destination = "fpp.lan"
            #sender[u.universe].destination = "espixelstick02.lan"
            sender[unum].destination = self._unicasthost;
            # Keep in mind that if multicast is on, unicast is not used
        
        self._sender = sender;
        
    def stop(self):
        self._sender.stop();
    
    def all_ch(self,onval = [150,150,150]):
        #onval = test;
        #turn all pixels on white
        # for u in self._universes:
        #     self._sender[u.universe].dmx_data = [0,0,0]*u.skippixels + (onval*u.pixelcount);
        
        # gather for each model
        dmxdictlists = [];
        for m in self._models:
            r = m.all_ch(onval[0]);
            dmxdictlists.append(r);
            
        finald = {};
        # FIXME
        # # merge universes
        # finalu = {};
        # for u in self._unique_universes:
        #     finalu[u] = [0]*self.universesize;
        # for key,val in dmxdictlists.items():
        #     for cnt,v in enumerate(val):
        #         finalu
        
        # TODO: only final model universes will be used
        for u in dmxdictlists:
            for uval,vals in u.items():
                if(uval in finald):
                    
                    # already exists, merge
                    for i in range(len(vals)):
                        
                        # assume this value if existing is zero
                        if( finald[uval][i] == 0):
                            finald[uval][i] = vals[i];
                    
                else:
                    # direct copy
                    finald[uval] = vals;
                
                # # direct copy
                # finald[uval] = vals;
        
        
        # assign to sender
        for uval, vals in finald.items():
            print('Set {:d} to'.format(uval),vals);
            self._sender[uval].dmx_data = vals;
        
        return finald;
    
    def all_on(self, onval = [150,150,150]):
        return self.all_ch(onval);
        
    def all_off(self):
        return self.all_ch([0,0,0]);
        #pass;
            
    # def all_off(self):
    #     #shut all pixels off
    #     for u in self._universes:
    #         self._sender[u.universe].dmx_data = [0,0,0]*u.skippixels + [0,0,0]*u.pixelcount;
            
    # def set_one(self, u : Universe, pxnum, onval = [150,150,150]):
    #     pxcnt = u.pixelcount;
    #     self._sender[u.universe].dmx_data = [0,0,0]*u.skippixels + [0,0,0]*pxnum + onval + [0,0,0]*(pxcnt-pxnum-1);
    
    def set_next_reset(self):
        #self._step_current_universe_index = -1;
        self._step_current_pixel_index = 0;

    # def set_next(self, onval = [150,150,150]):
    #     uidx = self._step_current_universe_index
    #     if( uidx < len(self._universes)):
    #         u = self._universes[uidx];
    #         setidx = self._step_current_pixel_index;
            
    #         self.set_one(u,setidx,onval);
            
    #         if(self._step_current_pixel_index+1>u.pixelcount):
    #             # increment universe
    #             self._step_current_pixel_index = 0;
    #             self._step_current_universe_index += 1;
    #         else:
    #             # increment pixel
    #             self._step_current_pixel_index += 1;
                
    #         return (u.universe,setidx);
    #     else:
    #         return False;
        
    def next_step_reset(self):
        self.set_next_reset();
    
    def next_step(self):
        newidx = self._step_current_pixel_index+1;
        if(newidx<=self.pixelcount):
            # increment pixel and return info
            self._step_current_pixel_index = newidx;
            print('Now set to pixel {:d}'.format(self._step_current_pixel_index))
            
            return True;
        else:
            # end
            return False;

    # def next_step(self):
    #     uidx = self._step_current_universe_index;
    #     u = self._universes[uidx];
    #     idx = self._step_current_pixel_index;
        
    #     if( idx + 1 > u.pixelcount ):
    #         # increment universe
    #         self._step_current_pixel_index = 0;
    #         self._step_current_universe_index += 1;
            
    #         # check for universe overflow
    #         uidx = self._step_current_universe_index;
    #         if(uidx <= len(self._universes)):
    #             print('Universe Overflow, return False');
    #             return False;
    #         else:
    #             print('Next Step 1 U{:d} {:d} = {:d}'.format(self._step_current_universe_index,self._step_current_pixel_index,self._step_current_pixel_index+u.skippixels))
    #             return True;
    #     else:
    #         # increment pixel
    #         self._step_current_pixel_index += 1;
    #         print('Next Step 2 U{:d} {:d} = {:d}'.format(self._step_current_universe_index,self._step_current_pixel_index,self._step_current_pixel_index+u.skippixels))
    #         return True;
        
    #     #     return (u.universe,setidx);
    #     # else:
    #     #     return False;
    
    def get_step(self):
        #print(self._step_current_pixel_index)
        m,px,r = self._px_to_mapped_model(self._step_current_pixel_index);
        px+=1;
        
        return (m,px,r)
    
    def step_on(self,onval = [150,150,150]):
        # uidx = self._step_current_universe_index;
        # u = self._universes[uidx];
        # setidx = self._step_current_pixel_index;
        # self.set_one(u,setidx,onval);
        # return u.universe,setidx+u.skippixels;
        m,px,r = self.set_one(self._step_current_pixel_index, onval);
        px+=1
        print(r);
        return r[0].universe,px;
        
            
    # def test_chase(self,onval = [150,150,150]):
    #     for u in self._universes:
    #         pxcnt = u.pixelcount;
    #         for px in range(pxcnt):
    #             #print(px)
    #             #print(px,[0,0,0]*px + onval + [0,0,0]*(pxcnt-px-1));
    #             self._sender[u.universe].dmx_data = [0,0,0]*u.skippixels + [0,0,0]*px + onval + [0,0,0]*(pxcnt-px-1);
    #             time.sleep(0.25);
    #     all_off();

        
class Lights():
    _universes = [];
    _pixelcount = 0;
    _sender = None;
    _unicasthost = None;
    
    _step_current_universe_index = -1;
    _step_current_pixel_index = -1;
    
    def __init__(self, universe_list : list,pixelcount=100, unicasthost="espixelstick02.lan"):
        self._universes = universe_list;
        self._pixelcount = pixelcount;
        self._unicasthost = unicasthost;
        
    def start(self,fps=40):
        sender = sacn.sACNsender(fps=fps)  # provide an IP-Address to bind to if you are using Windows and want to use multicast
        sender.start()  # start the sending thread
        #sender.activate_output(4000)  # start sending out data in the 1st universe
        
        # start sending out data in the defined universes
        for u in self._universes:
            sender.activate_output(u.universe);
            #sender[u.universe].multicast = True  # set multicast to True
            #sender[u.universe].destination = "fpp.lan"
            #sender[u.universe].destination = "espixelstick02.lan"
            sender[u.universe].destination = self._unicasthost;
            # Keep in mind that if multicast is on, unicast is not used
        
        self._sender = sender;
        
    def stop(self):
        self._sender.stop();
    
    def all_on(self,onval = [150,150,150]):
        #turn all pixels on white
        for u in self._universes:
            self._sender[u.universe].dmx_data = [0,0,0]*u.skippixels + (onval*u.pixelcount);
            
    def all_off(self):
        #shut all pixels off
        for u in self._universes:
            self._sender[u.universe].dmx_data = [0,0,0]*u.skippixels + [0,0,0]*u.pixelcount;
            
    def set_one(self, u : Universe, pxnum, onval = [150,150,150]):
        pxcnt = u.pixelcount;
        self._sender[u.universe].dmx_data = [0,0,0]*u.skippixels + [0,0,0]*pxnum + onval + [0,0,0]*(pxcnt-pxnum-1);
    
    def set_next_reset(self):
        self._step_current_universe_index = -1;
        self._step_current_pixel_index = -1;

    def set_next(self, onval = [150,150,150]):
        uidx = self._step_current_universe_index
        if( uidx < len(self._universes)):
            u = self._universes[uidx];
            setidx = self._step_current_pixel_index;
            
            self.set_one(u,setidx,onval);
            
            if(self._step_current_pixel_index+1>u.pixelcount):
                # increment universe
                self._step_current_pixel_index = 0;
                self._step_current_universe_index += 1;
            else:
                # increment pixel
                self._step_current_pixel_index += 1;
                
            return (u.universe,setidx);
        else:
            return False;
        
    def next_step_reset(self):
        self.set_next_reset();

    def next_step(self):
        uidx = self._step_current_universe_index;
        u = self._universes[uidx];
        idx = self._step_current_pixel_index;
        
        if( idx + 1 > u.pixelcount ):
            # increment universe
            self._step_current_pixel_index = 0;
            self._step_current_universe_index += 1;
            
            # check for universe overflow
            uidx = self._step_current_universe_index;
            if(uidx <= len(self._universes)):
                print('Universe Overflow, return False');
                return False;
            else:
                print('Next Step 1 U{:d} {:d} = {:d}'.format(self._step_current_universe_index,self._step_current_pixel_index,self._step_current_pixel_index+u.skippixels))
                return True;
        else:
            # increment pixel
            self._step_current_pixel_index += 1;
            print('Next Step 2 U{:d} {:d} = {:d}'.format(self._step_current_universe_index,self._step_current_pixel_index,self._step_current_pixel_index+u.skippixels))
            return True;
        
        #     return (u.universe,setidx);
        # else:
        #     return False;
    
    def get_step(self):
        uidx = self._step_current_universe_index;
        u = self._universes[uidx];
        setidx = self._step_current_pixel_index;
        #self.set_one(u,setidx,onval);
        return u.universe,setidx+u.skippixels;
    
    def step_on(self,onval = [150,150,150]):
        uidx = self._step_current_universe_index;
        u = self._universes[uidx];
        setidx = self._step_current_pixel_index;
        self.set_one(u,setidx,onval);
        return u.universe,setidx+u.skippixels;
            
    def test_chase(self,onval = [150,150,150]):
        for u in self._universes:
            pxcnt = u.pixelcount;
            for px in range(pxcnt):
                #print(px)
                #print(px,[0,0,0]*px + onval + [0,0,0]*(pxcnt-px-1));
                self._sender[u.universe].dmx_data = [0,0,0]*u.skippixels + [0,0,0]*px + onval + [0,0,0]*(pxcnt-px-1);
                time.sleep(0.25);
        all_off();
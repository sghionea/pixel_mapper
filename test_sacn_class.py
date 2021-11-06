# -*- coding: utf-8 -*-
"""
Created on Sat Oct 23 20:25:05 2021

@author: SimonGhionea
"""

import pypixelmapper.light_control as light_control
import time

#%%
ulist = [
    light_control.Universe(universe=2000,channelstart=9,channelcount=309),
    ];
lights = light_control.Lights(ulist,pixelcount=100);

#%% start sender
lights.start(fps=40);

#%%
lights.all_on();

#%%
lights.set_next()

#%%
lights.set_next_reset();

#%%
lights.all_off();

#%%
lights.test_chase([100,100,100]);

#%%
lights.set_one(ulist[0],99)

#%%
lights.next_step_reset();
while lights.next_step():
    unum,index = lights.step_on([255,255,255]);
    time.sleep(0.1);

#%%
lights.next_step_reset();
lights.next_step()
unum,index = lights.step_on([255,255,255]);

#%%
lights.stop();

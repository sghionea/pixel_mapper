# -*- coding: utf-8 -*-
"""
Created on Sat Oct 23 20:25:05 2021

@author: SimonGhionea
"""

import pypixelmapper.light_control as light_control

#%%
ulist = [
    light_control.Universe(universe=4000,channelcount=300),
    ];
lights = light_control.Lights(ulist,pixelcount=100);

#%% start sender
lights.start(fps=40);

#%%
lights.set_next()

#%%
lights.set_next_reset();

#%%
lights.all_off();

#%%
lights.stop();

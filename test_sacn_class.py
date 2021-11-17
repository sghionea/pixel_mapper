# -*- coding: utf-8 -*-
"""
Created on Sat Oct 23 20:25:05 2021

@author: SimonGhionea
"""

import pypixelmapper.light_control as light_control
import pypixelmapper.xlightscontrollerconnections as xlightscontrollerconnections
import time

# #%%
# # garage test
# # ulist = [
# #     light_control.Universe(universe=2000,channelstart=9,channelcount=309),
# #     ];
# # lights = light_control.Lights(ulist,pixelcount=100);
# # outside bushes test
# ulist = [
#     # FPP
#     # # BUSH L2 to center
#     # light_control.ModelUniverses('BushL2c',universe=100,channelstart=1,channelcount=450),
    
#     # # BUSH R1 (near center)
#     # light_control.ModelUniverses('BushR1c',universe=200,channelstart=1,channelcount=600),

#     # BUSH R1 to center (global pathway)
#     #light_control.ModelUniverses('BushL2c',universe=10,channelstart=454,channelcount=450),

    
#     #light_control.ModelUniverses('testbig',universe=20,channelstart=1,channelcount=450),
    
    
#     # # GARAGE TEST - order
#     # light_control.ModelUniverses('garage_new_strand',universe=10000,    channelstart=3,             channelcount=100*3),
#     # light_control.ModelUniverses('garage_old_strand',universe=10000,    channelstart=303+(50*3),    channelcount=20*3),
    
#     # GARAGE TEST - order
#     #light_control.ModelUniverses('garage_new_strand',universe=10000,    channelstart=3,             channelcount=100*3),
#     light_control.ModelUniverses('garage_old_strand',universe=10000,    channelstart=303,           channelcount=100*3),
    
#     # # GARAGE TEST - first model occurs in other universe
#     # #light_control.ModelUniverses('garage_new_strand',universe=10000,    channelstart=3,             channelcount=100*3),
#     # light_control.ModelUniverses('garage_old_strand_last',universe=10000,    channelstart=303+(3*80),           channelcount=10*3),
    
#     #light_control.ModelUniverses('garage_test_2ndstrandB',universe=10000,channelstart=302,channelcount=100*3),
    
#     #light_control.ModelUniverses('garage_first_strand',universe=10000,channelstart=3,channelcount=100*3),
#     #light_control.ModelUniverses('garage_test_2ndstrand',universe=10000,channelstart=303,channelcount=100*3),
#     #light_control.ModelUniverses('garage_test_2ndstrandB',universe=10000,channelstart=302,channelcount=100*3),

#     ];
# lights = light_control.LightStepper(ulist,
#                                     unicasthost="espixelstick02.lan"
#                                     );

#%% fpp test model number
allmodels = xlightscontrollerconnections.load_all_model_info('Controller_Connections.csv')
ulist = [
    xlightscontrollerconnections.convert_to_modeluniverse(allmodels,'Bush R1'),
    xlightscontrollerconnections.convert_to_modeluniverse(allmodels,'Bush R2'),
];
lights = light_control.LightStepper(ulist,
                                    unicasthost="fpp.lan"
                                    );

#%% test modeluniverse class functions
#print(ulist[0].testMap(1))
print(ulist[1].mapPixel(70));
print(ulist[1].mapPixel(71));
print(ulist[1].mapPixel(72));

#%% test modeluniverse class functions
#print(ulist[0].testMap(1))
r = ulist[0].all_on();
print(r);

#%% test lightstepper class
r = lights._px_to_mapped_model(100)
print(r[0].name,r[1])
print('')

r = lights._px_to_mapped_model(101)
print(r[0].name,r[1])
print('')

r = lights._px_to_mapped_model(102)
print(r[0].name,r[1])
print('')

#%%
setoneval = [150,150,150]

#%% test lightstepper class
r = lights.all_on([100,100,100]);
#print(r[0].name,r[1])



#%% start sender
lights.start(fps=40);

#%%
#lights.all_on([25,25,25]);
#lights.all_on([75,75,75]);
lights.all_on([255]*3);

#%%
#lights.set_next()

#%%
#lights.set_next_reset();

#%%
lights.all_off();

#%%
lights.test_chase([100,100,100]);


#%%
lights.set_one(1,onval=setoneval)

#%%
lights.set_one(169,onval=setoneval)

#%%
lights.set_one(170,onval=setoneval)

#%%
lights.set_one(171,onval=setoneval)

#%%
lights.next_step_reset();
ret = lights.get_step()
#%%
lights.step_on()

#%%
lights.next_step_reset();
while lights.next_step():
    unum,index = lights.step_on([50,100,50]);
    time.sleep(0.15);
    #break;
lights.all_off();

#%%
lights.next_step_reset();
lights.next_step()
unum,index = lights.step_on([255,255,255]);

#%%
lights.stop();

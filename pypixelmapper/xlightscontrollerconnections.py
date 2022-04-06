# -*- coding: utf-8 -*-
"""
Created on Sun Oct 31 07:42:29 2021

@author: SimonGhionea
"""

import csv
import re
from pypixelmapper.light_control import ModelUniverses

#%%
def read_xl_controller_connections(fname = 'C:\Simon\Personal\dmxpixels\pixel_mapper\Controller_connections.csv'):
    controller_strings = [];
    with open(fname, newline='') as f:
      reader = csv.reader(f)
      controller_str = '';
      pixel_ports = [];
      for row in reader:
        print(row)
        
        # out of controller
        if(controller_str == ''):
            if(len(row)>0):
                print('Starting controller');
                controller_str = row;
        
        # in controller
        if(not controller_str == ''):
            if(len(row)==0):
                print('done with controller');
                controller_strings.append((controller_str,pixel_ports));
                controller_str = '';
                pixel_ports = [];
                # done with controller
                #break; # stop after firsts
            elif(row[0]=="Output"):
                # this is a header
                print('header row')
                pass;
            elif(row[0].find('Pixel Port')>=0):
                print('pixel port');
                pixel_ports.append(row);
        
        #print('')
    
    return controller_strings;

def separate_info_to_dict(l):
    d = {}
    for s in l:
        split = s.split(':');
        key = split[0];
        val = split[-1];
        d[key] = int(val);
        #print(s);
    #print(d)
    return d;

def parse_pixel_port_strings(info):
    pp = info[2][1];
    allmodels = {};
    for row in pp:
        strs_port_info = [];
        port = None;
        if(row[1] == ''):
            #print(row[0],'unused');
            pass;
        else:
            # pixel port in use
            #break;
            strs_port_info = re.findall(r'\((.*?)\)',row[0]);
            port = int(re.findall(r'Port ([0-9]*)\(',row[0])[0]);
            port_info = separate_info_to_dict(strs_port_info)
            
            print('-'*25);
            print('Port ',port,strs_port_info);
            print('-'*25);
            
            # iterate each model on this port
            port_models = [];
            for modelnum,mstr in enumerate(row[1:]):
                if(mstr != ''):
                    modelname = re.findall(r'^(.*?)\(',mstr)[0];
                    #print('Model {:d}'.format(modelnum),modelname)
                    model_info = re.findall(r'\((.*?)\)',mstr);
                    minfo = separate_info_to_dict(model_info);
                    #print('\t',model_info);
                #break;
                    port_models.append((modelname,minfo));
                    allmodels[modelname] = dict(port=port,port_info=port_info,minfo=minfo);
            print(port_models)
    return allmodels;

def load_all_model_info(fname = 'C:\Simon\Personal\dmxpixels\pixel_mapper\Controller_connections.csv'):
    info = read_xl_controller_connections(fname);
    allmodels = parse_pixel_port_strings(info);  
    return allmodels;
            #break;
        #print(row)

def convert_to_modeluniverse(allmodels,modelname='Bush R1'):
    startchannel = allmodels[modelname]['minfo']['SC'];
    chns = allmodels[modelname]['minfo']['CHANS'];
    
    #ModelUniverses('garage_old_strand',universe=10000,    channelstart=303+(3*50),           channelcount=100*3),
    m = ModelUniverses(modelname, universe=10, channelcount=chns, channelstart=startchannel)
    
    return m;

#%% test code

if __name__ == "__main__":
    allmodels = load_all_model_info()
    #m = convert_to_modeluniverse(allmodels,'Bush R1');
    m = convert_to_modeluniverse(allmodels,'bigbushsouth');
    
    # print('');
    # print('ALL MODELS:');
    # print(allmodels);
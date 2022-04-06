# -*- coding: utf-8 -*-
"""
Created on Sun Oct 31 07:42:29 2021

@author: SimonGhionea
"""

import numpy as np

xLights_output_scale_factor = 10.0  #scaling factor to reduce the size of the xLights output file to a reasonable size

def outputXlightsModelFile(out , pixellist = None, modelname = 'testmodel', scalefactor = 10.0):
    xLights_output_scale_factor = scalefactor;
    #Create output file in xlights format
    #xlights format:
    #commas separate columns (width) (X), semicolons separate rows (height) (Y), "|" separate depth (Z)
    #pixel numbers (index of pixel) go between
    
    #determine the max size of the matrix to use for the model.  
    dataout = np.array(out)/xLights_output_scale_factor   #divide numbers down to a reasonable resolution so the file is not too large
    dataout = np.nan_to_num(dataout,copy=False)
    dataout = (dataout.round()).astype('i')
    xmin = np.nanmin(dataout[:,0])
    xmax = np.nanmax(dataout[:,0])
    ymin = np.nanmin(dataout[:,1])
    ymax = np.nanmax(dataout[:,1])
    zmin = np.nanmin(dataout[:,2])
    zmax = np.nanmax(dataout[:,2])
    
    if(zmax==zmin):
        zmin -= 1;
        
    if(ymax==ymin):
        ymin -= 1;
    
    #start with a matrix filled with zeros.  
    xlout = np.zeros([xmax-xmin,zmax-zmin,ymax-ymin],'i')
    #xlout = np.zeros([xmax-xmin,ymax-ymin,zmax-zmin],'i')
    #for each pixel, populate the location in the matrix with the index number of the pixel
    #locations in the matrix without a pixel will remain zeros
    
    #print("Found " + str(len(dataout)) + " pixel locations")
    #print(dataout)
    
    #global axis   xLights axis
    #    X             X
    #    Y             Z
    #    Z             Y
    if(pixellist is None):
        for i, point in enumerate(dataout):
        	xlout[point[0]-xmin-1,-1*(point[2]-zmin-1),point[1]-ymin-1] = i+1;
    else:
        for (cnt, ((universe, index), point)) in enumerate(zip(pixellist,dataout)):
            #xlout[point[0]-xmin-1,-1*(point[2]-zmin-1),point[1]-ymin-1] = index;
            xlout[point[0]-xmin-1,1*(point[2]-zmin-1),point[1]-ymin-1] = cnt+1;
            
            #xlout[point[0]-xmin-1,point[1]-ymin-1,-1*(point[2]-zmin-1)] = index;
            print('Pixellist',cnt+1,universe,index,point);
    
    #create an output string in xLights format
    outstring = ""
    for i, z in enumerate(xlout):
    	for j, y in enumerate(z):
    		for k,x in enumerate(y):
    			if x!= 0:
    				outstring = outstring + str(x)
    			if k != len(y)-1:
    				outstring = outstring + ","
    		if j != len(z)-1:
    			outstring = outstring + ";"
    	if i != len(xlout)-1:
    		outstring = outstring + "|"
    outxml = '<?xml version="1.0" encoding="UTF-8"?><custommodel name="{:s}" parm1="'.format(modelname) + str(xmax-xmin) + '" parm2="' + str(zmax-zmin) + '" Depth="' + str(ymax-ymin) + '" StringType="RGB Nodes" Transparency="0" PixelSize="2" ModelBrightness="" Antialias="1" StrandNames="" NodeNames="" CustomModel="' + outstring + '" SourceVersion="2020.37"  ></custommodel>'
    f = open(modelname+'.xmodel','w')
    f.write(outxml)
    f.close()
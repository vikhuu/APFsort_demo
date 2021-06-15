# -*- coding: utf-8 -*-
"""
Created on Sat Aug 29 01:23:37 2020

@author: AD

giản lược chương trình, xóa các hàm không cần thiết
thêm hàm tính FPS
"""

import snap7
from snap7 import util 

import sys
import numpy as np
import cv2 as cv
from timeit import default_timer as timer
import math

#-----------------------------------------------------------------------------
red    = (0,0,255)
green  = (0,255,0)
blue   = (255,0,0)
purple = (255,0,125)
cyan   = (255,255,0)
orange = (0,125,255)
yellow = (0,255,255) 
black  = (0,0,0)

#-----------------------------------------------------------------------------
    
# Control valve 
def controlVal( i , pulseStack_i, flag ):
    # Check stack and pop 1st Ni value in that
    if len(pulseStack_i) != 0 and not(flag):
        data = pulseStack_i.pop(0)
        writeDB( plc, 11, offset[i-1], 4, data) #send value to corresponding valve
        writeMbit( plc, 2, i-1, True ) #send flag

# PLC: switch status memory bit        
def writeMbit( dev, byte, bit, cmd ):
    data = dev.read_area( 0x83, 0, byte, 1 ) 
    util.set_bool( data, 0, bit, cmd ) # set cmd cho #bit của data 
    dev.write_area( 0x83, 0, byte, data )   

# PLC: write value in Data Blocks
def writeDB( dev, dbnum, offset , dtype, value ):
    data = dev.db_read( dbnum, offset, dtype )
    if dtype == 4:
        util.set_dword( data, 0, value )
    elif dtype == 2:
        util.set_int( data, 0, value )
    dev.db_write( dbnum, offset, data )
        
# PLC: read value in Data Blocks
def readDB( dev, dbnum, offset, dtype ):
    data = dev.db_read( dbnum, offset, dtype )
    if   dtype == 4 :
        return util.get_dword(data, 0)
    elif dtype == 2 :
        return util.get_int( data, 0 )

# PLC: READ status memory bit        
def readMbit( dev, byte, bit ):
    data = dev.read_area( 0x83, 0, byte, 1 ) 
    return util.get_bool( data, 0, bit )

# PLC: READ memory byte and covert to binary-string
def readMB( dev, byte ):
    data = dev.read_area( 0x83, 0, byte, 1 )
    int_data = (list(data)[0]) 
    return [(int_data & (1<<x))>>x for x in (7,6,5,4,3,2,1,0)]
    
#-----------------------------------------------------------------------------    
# Global  variable
Hmin1, Smin1, Vmin1 = 0, 100, 55
Hmax1, Smax1, Vmax1 = 45,255,255
Hmin2, Smin2, Vmin2 = 0, 100, 55
Hmax2, Smax2, Vmax2 = 20,255,255

a1,i1 = 3, 2 # kernel size and interations of Morphology Transform
a2,i2 = 3, 2 # a1,i1 for OPEN, a2,i2 for CLOSE

minArea = 3000   # min area to confirm these are an objects 
minObject = 5000 # min area to ensure removed-object
smallObj = 11000 # small Object 
redPercent = 30  # ratio percent of RED
detectBorder = True

k0, k1, k2 = 140, 180, 240 # x_l0, x_k1, y_k2 line

ppr = 600 #pulse per revolution
Dg = 100 # diameter of gear
Li = [ 125 ,150 ,175 ,200, 225, 250, 275, 300 ] # distance between the pair-valve-i and camera

CAM = 0 
connectPLC = True
IP = '192.168.0.1'
RACK = 0
SLOT = 0

delay = 300 # delay time(ms) of valve open
speed = 50 # motor speed, from 0-100%
#-----------------------------------------------------------------------------
offset = (0,4,8,12,16,20,24,28) #offset of datablock
flagCheck = [False]*9
pulseStack = ( [],[],[],[],[],[],[],[] ) # Ni-stack
Objlst = []
fps = 0
FPS = 0
#-----------------------------------------------------------------------------
k = ppr/(math.pi*Dg)
n = [int(Li[i]*k) for i in range(8)] # Pulse-distance corresponding to Valve-i
PLCconnectivity = connectPLC
#-----------------------------------------------------------------------------
# Connecting with PLC
if PLCconnectivity :
    # plc = snap7.client.Client()
    # plc.connect( IP, RACK, SLOT )    
    # if not plc.get_connected():
    #     print('Cannot Connect to PLC')
    #     sys.exit()   
    # for i in range(8):
    #     writeMbit( plc, 2, i, False ) # reset Flgi bit (M2.i)
    # # write delay to DB10.DBD2
    # writeDB( plc, 10, 2, 4, delay )
    # # write speed to DB10.DBW0
    # writeDB( plc, 10, 0, 2, speed )
    # # reset encoder
    # writeMbit(plc,0,1,True)
    # writeMbit(plc,0,1,False)  
    try:
        plc = snap7.client.Client()
        plc.connect( IP, RACK, SLOT )    
        if not plc.get_connected():
            print('Cannot Connect to PLC')
            sys.exit()   
        for i in range(8):
            writeMbit( plc, 2, i, False ) # reset Flgi bit (M2.i)
        # write delay to DB10.DBD2
        writeDB( plc, 10, 2, 4, delay )
        # write speed to DB10.DBW0
        writeDB( plc, 10, 0, 2, speed )
        # reset encoder
        writeMbit(plc,0,1,True)
        writeMbit(plc,0,1,False)
    except :
        print('Cannot Connect to PLC')
        PLCconnectivity = False

# Connecting with Camera
cap = cv.VideoCapture( CAM ) # Choice chanel-0
if not cap.isOpened():
    print('Cannot Open Camera')
    sys.exit()

t1 = timer()    
#-----------------------------------------------------------------------------        
# Main program
while 1:
    # Get continuous frame 
    receive,frame = cap.read()
    if not receive: # Check connect
        print('Cannot receive frame (stream End?). Exiting ... ')
        break
    yLen = frame.shape[:2][0] - 1
    xLen = frame.shape[:2][1] - 1
    temp = frame.copy() # create temp to draw on it

    # Preprocessing
    hsv = cv.cvtColor( frame, cv.COLOR_BGR2HSV ) # convert to HSV from BGR chanel
    kernel1 = cv.getStructuringElement( cv.MORPH_RECT, (a1,a1) )
    kernel2 = cv.getStructuringElement( cv.MORPH_RECT, (a2,a2) )
    min1 = np.array( [Hmin1, Smin1, Vmin1], np.uint8 )  # GREEN + RED range1
    max1 = np.array( [Hmax1, Smax1, Vmax1], np.uint8 )
    min2 = np.array( [Hmin2, Smin2, Vmin2], np.uint8 )  # RED range2
    max2 = np.array( [Hmax2, Smax2, Vmax2], np.uint8 )
    thresh1 = cv.inRange( hsv, min1, max1 ) # Thresholding : Green+Red   
    thresh1 = cv.morphologyEx( thresh1, cv.MORPH_OPEN , kernel1, iterations = i1)
    thresh1 = cv.morphologyEx( thresh1, cv.MORPH_CLOSE, kernel2, iterations = i2)   
    thresh2 = cv.inRange( hsv, min2, max2 ) # Thresholding : Red
    thresh2 = cv.morphologyEx( thresh2, cv.MORPH_OPEN , kernel1, iterations = i1)
    thresh2 = cv.morphologyEx( thresh2, cv.MORPH_CLOSE, kernel2, iterations = i2)
    
    
    # Multi-objects tracking
    contours,hierachy = cv.findContours( thresh1,
                                         cv.RETR_EXTERNAL,
                                         cv.CHAIN_APPROX_SIMPLE )
    if len(contours) > 0 :  # If any contour exist do...
        # Check each contours if they are an object then tracking these
        Objlst = list([]) # Create empty list to contain Objects ( the Object's Stack )
        typeObj = lambda r,area: ((1 if area>smallObj else 2) if r>=redPercent 
                                 else (3 if area>smallObj 
                                 else 4 if area>minObject else 5))
        col = lambda t: green if t < 3 else red if t == 5 else blue
        for cnt in contours:  
            # calculate area of an object
            M = cv.moments( cnt ) 
            area = int(M['m00']/1000)*1000 #rounding number   
            if area < minArea:
                continue
            flagBorder = False
            for j in range(len(cnt)):
                flgx = cnt[j][0][0] == 0 or cnt[j][0][0] == xLen
                flgy = cnt[j][0][1] == 0 or cnt[j][0][1] == yLen 
                if flgx or  flgy :
                    flagBorder = True
                    break
            # Remove noisy-Object 
            if flagBorder and detectBorder:
                continue
            # Get type of an object
            xj,yj,wj,hj = cv.boundingRect( cnt )                        
            thresh_i = thresh2[yj:yj+hj,xj:xj+wj].copy() # choice the region in second-thresh
            contours_i,hierachy_i = cv.findContours( thresh_i,
                                                    cv.RETR_EXTERNAL,
                                                    cv.CHAIN_APPROX_SIMPLE )
            if len(contours_i) > 0:
                si = 0
                for cnt in contours_i:
                    sii = cv.contourArea( cnt )
                    si += sii
                si = int(si/1000)*1000
                ratio_i = int(int(si/area*100)/5)*5
                t = typeObj(ratio_i,area)  
            else:
                t = typeObj(0,area) 
            # Draw ROI and return centroid
            cx = int(M['m10']/M['m00']) #centroid coordinates
            cy = int(M['m01']/M['m00']) 
            cv.line( temp, (cx-5,cy-5), (cx+5,cy+5), col(t), 2 )
            cv.line( temp, (cx-5,cy+5), (cx+5,cy-5), col(t), 2 )
            cv.putText( temp, '#%s(%s)'%(t,int(area/100)), (xj-5,yj-5),
                        cv.FONT_HERSHEY_SIMPLEX, 1, col(t), 1, cv.LINE_AA )
            cv.putText( temp, '(%s,%s)'%(cx,cy), (xj,yj+hj+25),
                        cv.FONT_HERSHEY_SIMPLEX, 1, col(t), 1, cv.LINE_AA )
            cv.rectangle( temp, (xj,yj), (xj+wj, yj+hj), col(t), 2)
            # Add an object to the Object's Stack
            Objlst.append((cx,cy,area,t))

            
    # Classify Objects   
    if len(Objlst) != 0: # if any Object exits do...
        if PLCconnectivity :
            NoP = readDB(plc,10,6,4) #number of pulse
        # Check each Object in the Object's Stack and classificate it
        for obj in Objlst :
            ix,iy,t  = obj[0] , obj[1] , obj[3]    
            if   t == 1 and iy > k2 : #Group 1
                if (k0 < ix < k1) and not(flagCheck[0]) :
                    if PLCconnectivity :
                        pulseStack[0].append( NoP + n[0] )
                    print('Group 1')
                    flagCheck[0] = True
                elif (ix < k0) and flagCheck[0] :
                    flagCheck[0] = False                    
            elif t == 1 and iy < k2 : #Group 2
                if (k0 < ix < k1) and not(flagCheck[1]) :
                    if PLCconnectivity :
                        pulseStack[1].append( NoP + n[1] )
                    print('Group 2')
                    flagCheck[1] = True
                elif (ix < k0) and flagCheck[1] :
                    flagCheck[1] = False             
            elif t == 2 and iy > k2 : #Group 3
                if (k0 < ix < k1) and not(flagCheck[2]) :
                    if PLCconnectivity :
                        pulseStack[2].append( NoP + n[2] )
                    print('Group 3')
                    flagCheck[2] = True
                elif (ix < k0) and flagCheck[2] :
                    flagCheck[2] = False               
            elif t == 2 and iy < k2 : #Group 4
                if (k0 < ix < k1) and not(flagCheck[3]) :
                    if PLCconnectivity :
                        pulseStack[3].append( NoP + n[3] )
                    print('Group 4')
                    flagCheck[3] = True
                elif (ix < k0) and flagCheck[3] :
                    flagCheck[3] = False            
            elif t == 3 and iy > k2 : #Group 5
                if (k0 < ix < k1) and not(flagCheck[4]) :
                    if PLCconnectivity :
                        pulseStack[4].append( NoP + n[4] )
                    print('Group 5')
                    flagCheck[4] = True
                elif (ix < k0) and flagCheck[4] :
                    flagCheck[4] = False                
            elif t == 3 and iy < k2 : #Group 6
                if (k0 < ix < k1) and not(flagCheck[5]) :
                    if PLCconnectivity :
                        pulseStack[5].append( NoP + n[5] )
                    print('Group 6')
                    flagCheck[5] = True
                elif (ix < k0) and flagCheck[5] :
                    flagCheck[5] = False               
            elif t == 4 and iy > k2:  #Group 7
                if (k0 < ix < k1) and not(flagCheck[6]) :
                    if PLCconnectivity :
                        pulseStack[6].append( NoP + n[6] )
                    print('Group 7')
                    flagCheck[6] = True
                elif (ix < k0) and flagCheck[6] :
                    flagCheck[6] = False                
            elif t == 4 and iy < k2 : #Group 8
                if (k0 < ix < k1) and not(flagCheck[7]) :
                    if PLCconnectivity :
                        pulseStack[7].append( NoP + n[7] )
                    print('Group 8')
                    flagCheck[7] = True
                elif (ix < k0) and flagCheck[7] :
                    flagCheck[7] = False                
            elif t == 5 :             #Group 9
                if (k0 < ix < k1) and not(flagCheck[8]) :
                    print('Remove')
                    flagCheck[8] = True
                elif (ix < k0) and flagCheck[8] :
                    flagCheck[8] = False
         

    # Send signal to PLC
    # Wait and Trigger the corresponding output on PLC
    if PLCconnectivity :
        flagVal = readMB( plc, 2 )
        controlVal(1, pulseStack[0], flagVal[7])
        controlVal(2, pulseStack[1], flagVal[6])
        controlVal(3, pulseStack[2], flagVal[5])
        controlVal(4, pulseStack[3], flagVal[4])
        controlVal(5, pulseStack[4], flagVal[3])
        controlVal(6, pulseStack[5], flagVal[2])
        controlVal(7, pulseStack[6], flagVal[1])
        controlVal(8, pulseStack[7], flagVal[0])        


    # Show frame 
    cv.line( temp, (5,5), (5,35), blue, 2)            
    cv.line( temp, (5,5), (35,5), blue, 2)
    cv.putText( temp, 'X', (45,25),
               cv.FONT_HERSHEY_SIMPLEX, 1, blue, 1, cv.LINE_AA )
    cv.putText( temp, 'Y', (5,65),
               cv.FONT_HERSHEY_SIMPLEX, 1, blue, 1, cv.LINE_AA )
    cv.line( temp, (k0,0),(k0,yLen), blue, 1)
    cv.line( temp, (k1,0),(k1,yLen), blue, 1)
    cv.line( temp, (0,k2),(xLen,k2), blue, 2)
    cv.putText( temp, 'k0', (k0-16,yLen-10),
               cv.FONT_HERSHEY_SIMPLEX, 0.4, blue, 1, cv.LINE_AA )
    cv.putText( temp, 'k1', (k1-14,yLen-10),
               cv.FONT_HERSHEY_SIMPLEX, 0.4, blue, 1, cv.LINE_AA )
    cv.putText( temp, 'k2', (0+5,k2-5),
               cv.FONT_HERSHEY_SIMPLEX, 0.4, blue, 1, cv.LINE_AA )
    cv.putText( temp, "%.2iFPS"%(FPS), (xLen-60,20 ),
               cv.FONT_HERSHEY_SIMPLEX, 0.6, blue, 1, cv.LINE_AA )
    cv.imshow( 'Object', temp )
    cv.imshow( 'Thresh1', thresh1 )

    
    # Wait Esc to quit    
    if cv.waitKey(1) % 0xFF == 27 : #Esc-button press
        break
    
    # Show FPS
    fps+=1 
    if fps == 30:
        t2 = timer() 
        FPS = int(30/(t2-t1))
        fps = 0
        t1 = timer()
 #----------------------------------------------------------------------------   
cap.release()
cv.destroyAllWindows()
#plc.disconnect()


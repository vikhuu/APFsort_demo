# -*- coding: utf-8 -*-
"""
Created on Sat Aug 29 01:23:37 2020

@author: AD

giản lược chương trình, xóa các hàm không cần thiết
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
purple = (255,0,125)
cyan   = (255,255,0)
orange = (0,125,255)
blue   = (255,0,0)
yellow = (0,255,255) 
black  = (0,0,0)

#-----------------------------------------------------------------------------
# Check object belong to group
def checkGroup( index ,ix, k0, k1 ):
    global flagCheck, pulseStack, n # dùng lst Global chứa cờ nhớ
    if (k0 < ix < k1) and not(flagCheck[index-1]) :
        if index < 9 and PLCconnectivity :
            N0 = int( readDB(plc, 10, 6, 4) )
            pulseStack[i-1].append( N0 + n[i-1] )
        print('Group %s'%index if index < 9 else 'Removed!')
        flagCheck[index-1] = True
    elif (ix < k0) and flagCheck[index-1] :
        flagCheck[index-1] = False
    
    
# Control valve 
def controlVal( i ):
    global pulseStack, doFlag
    # Check stack and pop 1st Ni value in that
    if len(pulseStack[i-1]) != 0 and not(readMbit( plc, 2, i-1 )):
        data = pulseStack[i-1].pop(0)
        writeDB( plc, 11, offset[i-1], 4, data) #send value to corresponding valve
        writeMbit( plc, 2, i-1, True ) #block function
    # Signal valve active
    if readMbit( plc, 1, i-1) and doFlag[i-1]:
        print('val %s!' %(i))
        doFlag[i-1] = False   
    elif not(readMbit( plc, 1, i-1 )) and not(doFlag[i-1]):
        doFlag[i-1] = True
        

# PLC: switch status memory bit        
def writeMbit( dev, byte, bit, cmd ):
    '''
    Switch status of Memory bit
    eg. writeMbit( myplc, 1, 5, 1)
        -> modify M1.5 ON
    - read data from PLC -> modify data -> rewrite data to PLC

    Parameters
    ----------
    dev : object
        device connect, eg. plc s7-1200.
    byte : int
        Byte address.
    bit : int
        Bit location of the byte (0->7).
    cmd : boolean
        True or False, 1 or 0 .

    '''
    data = dev.read_area( 0x83, 0, byte, 1 ) 
    util.set_bool( data, 0, bit, cmd ) # set cmd cho #bit của data 
    dev.write_area( 0x83, 0, byte, data )   


# PLC: write value in Data Blocks
def writeDB( dev, dbnum, offset , amount, value ):
    '''
    Write value into a variable on Data Block 
    eg. writeDB( myplc, 9, 0, 4, 500)
        On DB9 write 500 into a variable have a offset 0.0 with dword format 

    Parameters
    ----------
    dev : object
        device connect, eg. plc s7-1200.
    dbnum : int
        Data Block number.
    offset : int
        Offset of corresponding variable in DB.
    amount : int
        byte is '1'', word is '2', double word is '4'.
    value : int
        value write in variable.


    '''
    data = dev.db_read( dbnum, offset, amount )
    util.set_dword( data, offset, value )
    dev.db_write( dbnum, offset, data )
        

# PLC: read value in Data Blocks
def readDB( dev, dbnum, offset, dtype ):
    '''
    Read value from a Data Block
    eg. readDB( plc, 9, 2, 2 ) : read DB9.DBW2 (data block 9, Data word offset 2)
        readDB( plc, 10, 0, 4 ) : read DB10.DBD0 (data block 9, Data Dword offset 0)

    Parameters
    ----------
    dev : object
        device connect, eg. plc s7-1200.
    dbnum : unsigned integer
        Offset of corresponding variable in DB.
    offset : unsigned integer
        Offset of corresponding variable in DB.
    dtype : unsigned integer
        Data type: word is 2, double word is 4.

    Returns
    -------
    integer
        Value want to read from Data block.

    '''
    data = dev.db_read( dbnum, offset, dtype )
    if   dtype == 4 :
        return util.get_dword(data, 0)
    elif dtype == 2 :
        return util.get_int( data, 0 )


# PLC: READ status memory bit        
def readMbit( dev, byte, bit ):
    '''
    Read status of Memory bit
    eg. readMbit( myplc, 2, 7)
        -> read M2.7 
    - read 'bytearea' from PLC -> read 'bit' within 'bytearray'
    
    Parameters
    ----------
    dev : object
        device connect, eg. plc s7-1200.
    byte : int
        Byte address.
    bit : int
        Bit location of the byte (0->7).

    '''
    data = dev.read_area( 0x83, 0, byte, 1 ) 
    return util.get_bool( data, 0, bit )

    
#-----------------------------------------------------------------------------    
# Global  variable
min1 = np.array( [0, 100, 55], np.uint8 )  # GREEN + RED
max1 = np.array( [45,255,255], np.uint8 )
min2 = np.array( [0, 100, 55], np.uint8 )  # RED
max2 = np.array( [20,255,255], np.uint8 )

a1,i1 = 3, 2 # kernel size and interations of Morphology Transform
a2,i2 = 3, 2 # a1,i1 for OPEN, a2,i2 for CLOSE

minArea = 3000   # min area to confirm these are an objects 
minObject = 5000 # min area to ensure removed-object
smallObj = 11000 # small Object 
redPercent = 30  # ratio percent of RED
detectBorder = True

k0, k1, k2 = 140, 180, 240 # x_l0, x_k1, y_k2 line

ppr = 2000 #pulse per revolution
Dg = 100 # diameter of gear
Li = [ 25 ,50 ,75 ,100, 125, 150, 175, 200 ] # distance between the pair-valve-i and camera

CAM = 0 
IP = '192.168.0.1'
RACK = 0
SLOT = 0
connectPLC = True
delay = 300 # delay time(ms) of valve open
speed = 50 # motor speed, from 0-100%

offset = (0,4,8,12,16,20,24,28) #offset of datablock
flagCheck = [False]*9
pulseStack = ( [],[],[],[],[],[],[],[] ) # Ni-stack
doFlag = [False]*8
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
        # Check each Object in the Object's Stack and classificate it
        for obj in Objlst :
            ix,iy,t  = obj[0] , obj[1] , obj[3]    
            if   t == 1 and iy > k2 : #Group 1
                 checkGroup( 1 ,ix, k0, k1 )
            elif t == 1 and iy < k2 : #Group 2
                 checkGroup( 2 ,ix, k0, k1 )               
            elif t == 2 and iy > k2 : #Group 3
                 checkGroup( 3 ,ix, k0, k1 )                
            elif t == 2 and iy < k2 : #Group 4
                 checkGroup( 4 ,ix, k0, k1 )
            elif t == 3 and iy > k2 : #Group 5
                 checkGroup( 5 ,ix, k0, k1 )                
            elif t == 3 and iy < k2 : #Group 6
                 checkGroup( 6 ,ix, k0, k1 )                
            elif t == 4 and iy > k2:  #Group 7
                 checkGroup( 7 ,ix, k0, k1 )                
            elif t == 4 and iy < k2 : #Group 8
                checkGroup( 8 ,ix, k0, k1 )               
            elif t == 5 :             #Group 9
                checkGroup( 9 ,ix, k0, k1 )
         

    # Send signal to PLC
    # Wait and Trigger the corresponding output on PLC
    if PLCconnectivity :
        controlVal(1)
        controlVal(2)
        controlVal(3)
        controlVal(4)
        controlVal(5)
        controlVal(6)
        controlVal(7)
        controlVal(8)        


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
    t2 = timer()
    
    if (t2 - t1) >= 1:
        FPS = fps
        fps = 0
        t1= timer()
 #----------------------------------------------------------------------------   
cap.release()
cv.destroyAllWindows()
#plc.disconnect()


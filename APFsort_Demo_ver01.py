"""
Date : 09/09/2020

Project : Agricultural Products & Fruits sorting machine (APFsort) DEMO ver1.0
Author  :  Kh.D.Vi 

    Program:
    - Python 3.7
    - Image processing library: OpenCV
    - Communication library: Snap7
    - GUI: Tkinter
    PLC:
    - S7-1200

"""
import numpy as np
import cv2 as cv
from tkinter import *
from tkinter import messagebox
from PIL import ImageTk,Image
import snap7
from snap7 import util 
from timeit import default_timer as timer
import math

#-----------------------------------------------------------------------------
# Define Main window
root = Tk()
root.title('APFsort v1.0')
root.iconbitmap('eye.ico')
root.geometry( "1350x700" )
#-----------------------------------------------------------------------------
# Global variable
rangeCol1 = [0,60,70,40,255,255]
rangeCol2 = [0,60,70,15,255,255]

a1,i1 = 3, 2 # kernel size and interations of Morphology Transform
a2,i2 = 3, 2 # a1,i1 for OPEN, a2,i2 for CLOSE
detectBorder = True

minArea = 3000   # min area to confirm these are an objects 
minObject = 20000 # min area to ensure removed-object
smallObj = 4000 # small Object 
redPercent = 65  # ratio percent of RED

ppr = 600 #pulse per revolution
n1,n2 = 40,16
Dg = 113 # diameter of gear ()
k0, k1, k2 = 150, 200, 315 # x_l0, x_k1, y_k2 line
# Li = [ 251 ,351 ,557, 620, 830, 893, 1103, 1166 ]
Li = [ 235 ,335 ,530, 600, 810, 870, 1080, 1140 ] # distance between the pair-valve-i and camera

CAM = 0 
IP = '192.168.0.1'
RACK = 0
SLOT = 0
PLCconnectivity = True
delay = 200 # delay time(ms) of valve open
speed = 65 # motor speed, from 0-100%
#-----------------------------------------------------------------------------
# Parameters
offset1 = (0,4,8,12,16,20,24,28) #offset of datablock
flagCheck = [False]*9
pulseStack = ( [],[],[],[],[],[],[],[] ) # Ni-stack
Objlst = list([])
t1 = timer()
fps = 0
FPS = 0
flgw = True
n = [int( Li[i]*((ppr*n1/n2)/(math.pi*Dg)) ) for i in range(8)] # Pulse-distance corresponding to Valve-i
red    = (0,0,255)
green  = (0,255,0)
blue   = (255,0,0)
ci = [0,0,0,0,0,0,0,0]
IPcop = IP
RACKcop = RACK
SLOTcop = SLOT
CAMcop = CAM
PLCconnectivity_cop = PLCconnectivity
#-----------------------------------------------------------------------------
def controlVal( i , pulseStack_i, flag ):
    global offset1
    if len(pulseStack_i) > 0 and not(flag):
        data = pulseStack_i.pop(0)
        writeDB( plc, 11, offset1[i-1], 4, data) #send value to corresponding valve
        writeMbit( plc, 2, i-1, True ) #send flag
# PLC: switch status memory bit        
def writeMbit( dev, byte, bit, cmd ):
    data = dev.read_area(0x83, 0, byte, 1)
    util.set_bool( data, 0, bit, cmd ) # set cmd for #bit of data bytearray
    dev.write_area( 0x83, 0, byte, data )   
# PLC: write value in Data Blocks
def writeDB( dev, dbnum, offset , dtype, value ):
    if dtype == 4:
        data = bytearray([0,0,0,0])
        util.set_dword( data, 0, value )
    elif dtype == 2:
        data = bytearray([0,0])
        util.set_int( data, 0, value )
    else:
        pass
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
# initial setup for cam and plc
def reset_PLC():
    global speed, delay, plc
    try:
        for i in range(8):
            writeMbit( plc, 1, i, False )
            writeMbit( plc, 2, i, False ) # reset Flgi bit (M2.i)     
        writeDB( plc, 10, 2, 4, delay ) # write delay to DB10.DBD2    
        writeDB( plc, 10, 0, 2, speed ) # write speed to DB10.DBW0
        writeMbit(plc,0,1,True) # reset encoder
        writeMbit(plc,0,1,False)
        flagCheck= [False]*9
        messagebox.showinfo("PLC","PLC have been Connected and Reset")
    except:
        messagebox.showwarning("PLC","Cannot receive signal")

def connect_PLC():
    global plc,PLCconnectivity,IP,RACK,SLOT
    if PLCconnectivity : 
        try:
            plc = snap7.client.Client()
            plc.connect( IP, RACK, SLOT )    
            if not plc.get_connected():
                messagebox.showwarning("PLC","Unreachable peer")
                PLCconnectivity = False
            else: 
                reset_PLC()
        except :
            messagebox.showerror("PLC","Cannot Connect PLC \n Error : 101")
            PLCconnectivity = False
    else :
         messagebox.showwarning("PLC","PLC have been Disconnected")
                     
def connect_CAM():
    global CAM, cap
    # Connecting Camera
    cap = cv.VideoCapture( CAM )
    if not cap.isOpened():                      
        messagebox.showerror("Camera","Cannot Open Camera")
        printlog("ERROR: Cannot Open Camera")
    else:
        messagebox.showinfo("Camera","Camera have been connected")

def set_true(window):
    global flgw
    flgw = True
    window.destroy()
    show_frame1()
    
def set_false():
    global flgw
    flgw = False

def apply_setting():
    global CAM,IP,RACK,SLOT,IPcop,RACKcop,SLOTcop
    if messagebox.askokcancel("Parametes","Apply new setting ?") :
        flag = IP==IPcop and RACK==RACKcop and SLOT == SLOTcop
        if CAMcop == CAM and not(flag):
            delete_log()
            printlog("INFO: Apply new parameter for PLC")
            IP,RACK,SLOT = IPcop,RACKcop,SLOTcop
            connect_PLC()
        elif CAMcop != CAM and flag:
            delete_log()
            printlog("INFO: Apply new parameter for CAM")            
            CAM = CAMcop
            connect_CAM()
        elif CAMcop != CAM and not(flag):
            delete_log()
            printlog("INFO: Apply new parameter forSET PLC & CAM")
            IP,RACK,SLOT = IPcop,RACKcop,SLOTcop
            CAM = CAMcop
            connect_CAM()
            connect_PLC()
        elif CAMcop == CAM and flag:
            messagebox.showinfo("Parametes","Value does not change")
            printlog("INFO: Apply new parameter Unsuccessful" )
        else:
            pass

def rst_plc():
    global PLCconnectivity, PLCconnectivity_cop
    if messagebox.askokcancel("Parameters", "RESET PLC ?") :
        printlog("WARNING: Reset PLC")        
        PLCconnectivity = PLCconnectivity_cop
        connect_PLC()
    else:
        pass

#----------------------------------------------------------------------------
# Define function
def slide1act(value):
    global rangeCol1
    rangeCol1[0] = int(value)
def slide2act(value):
    global rangeCol1
    rangeCol1[3] = int(value)
def slide3act(value):
    global rangeCol1
    rangeCol1[1] = int(value)
def slide4act(value):
    global rangeCol1
    rangeCol1[4] = int(value)
def slide5act(value):
    global rangeCol1
    rangeCol1[2] = int(value)
def slide6act(value):
    global rangeCol1
    rangeCol1[5] = int(value)
def slide7act(value):
    global rangeCol2
    rangeCol2[0] = int(value)
def slide8act(value):
    global rangeCol2
    rangeCol2[3] = int(value)
def slide9act(value):
    global rangeCol2
    rangeCol2[1] = int(value)
def slide10act(value):
    global rangeCol2
    rangeCol2[4] = int(value)
def slide11act(value):
    global rangeCol2
    rangeCol2[2] = int(value)
def slide12act(value):
    global rangeCol2
    rangeCol2[5] = int(value)
    
def slide13act(value):
    global speed
    speed = int(value)
    if PLCconnectivity :
        writeDB( plc, 10, 0, 2, speed ) # write speed to DB10.DBW0
def slide14act(value):
    global delay
    delay = int(value)
    if PLCconnectivity :
        writeDB( plc, 10, 2, 4, delay ) # write delay to DB10.DBD2  
def slide15act(value):
    global redPercent
    redPercent = int(value)
    
def callback_quit():
    if messagebox.askokcancel("Quit", "Do you really want to quit?"):
        root.destroy()
def slctcamera(value):
    global CAMcop
    CAMcop = int(value[-1])

def rst_copvar():
    global CAM,IP,RACK,SLOT,IPcop,RACKcop,SLOTcop,PLCconnectivity_cop
    CAMcop = CAM
    IPcop = IP
    RACKcop = RACK
    SLOTcop = SLOT
    PLCconnectivity_cop = PLCconnectivity

def slctvar1( var ):
    global IPcop,RACKcop,SLOTcop
    if var == "IP":
        try:
            IPcop = entryIP.get()
            labelplc["text"]="IP: %s | RACK: %s | SLOT: %s"%(IPcop,RACKcop,SLOTcop)
            entryIP.delete(0,END)
            entryIP.insert(0,IPcop)
        except ValueError:
            pass
    elif var == "RACK":
        try:
            RACKcop = int(entryRACK.get())
            labelplc["text"]="IP: %s | RACK: %s | SLOT: %s"%(IPcop,RACKcop,SLOTcop)
            entryRACK.delete(0,END)
            entryRACK.insert(0,str(RACKcop))
        except ValueError:
            pass
    elif var == "SLOT":
        try:
            SLOTcop = int(entrySLOT.get())
            labelplc["text"]="IP: %s | RACK: %s | SLOT: %s"%(IPcop,RACKcop,SLOTcop)
            entrySLOT.delete(0,END)
            entrySLOT.insert(0,str(SLOTcop))
        except ValueError:
            pass  
    else:
        pass

def slct_connect( ):
    global PLCconnectivity_cop
    PLCconnectivity_cop = PLCconnect.get()

def slct_brdrObj():
    global detectBorder
    detectBorder = brdrVar.get()

def slctvar2(var):
    global n,Li,ppr,Dg,n1,n2
    if var == "van1":
        try:
            Li[0] = int(van1Entry.get())
            van1Label["text"] = "%s mm"%str(Li[0])
            van1Entry.delete(0,END)
            van1Entry.insert(0,str(Li[0]))
            n[0] = int(Li[0]*((ppr*n1/n2)/(math.pi*Dg)) )
        except ValueError:
            pass
    elif var == "van2":
        try:
            Li[1] = int(van2Entry.get())
            van2Label["text"] = "%s mm"%str(Li[1])
            van2Entry.delete(0,END)
            van2Entry.insert(0,str(Li[1]))
            n[1] = int(Li[1]*((ppr*n1/n2)/(math.pi*Dg)) )
        except ValueError:
            pass
    elif var == "van3":
        try:
            Li[2] = int(van3Entry.get())
            van3Label["text"] = "%s mm"%str(Li[2])
            van3Entry.delete(0,END)
            van3Entry.insert(0,str(Li[2]))
            n[2] = int(Li[2]*((ppr*n1/n2)/(math.pi*Dg)) )
        except ValueError:
            pass    
    elif var == "van4":
        try:
            Li[3] = int(van4Entry.get())
            van4Label["text"] = "%s mm"%str(Li[3])
            van4Entry.delete(0,END)
            van4Entry.insert(0,str(Li[3]))
            n[3] = int(Li[3]*((ppr*n1/n2)/(math.pi*Dg)) )
        except ValueError:
            pass        
    elif var == "van5":
        try:
            Li[4] = int(van5Entry.get())
            van5Label["text"] = "%s mm"%str(Li[4])
            van5Entry.delete(0,END)
            van5Entry.insert(0,str(Li[4]))
            n[4] = int(Li[4]*((ppr*n1/n2)/(math.pi*Dg)) )
        except ValueError:
            pass        
    elif var == "van6":
        try:
            Li[5] = int(van6Entry.get())
            van6Label["text"] = "%s mm"%str(Li[5])
            van6Entry.delete(0,END)
            van6Entry.insert(0,str(Li[5]))
            n[5] = int(Li[5]*((ppr*n1/n2)/(math.pi*Dg)) )
        except ValueError:
            pass     
    elif var == "van7":
        try:
            Li[6] = int(van7Entry.get())
            van7Label["text"] = "%s mm"%str(Li[6])
            van7Entry.delete(0,END)
            van7Entry.insert(0,str(Li[6]))
            n[6] = int(Li[6]*((ppr*n1/n2)/(math.pi*Dg)) )
        except ValueError:
            pass        
    elif var == "van8":
        try:
            Li[7] = int(van8Entry.get())
            van8Label["text"] = "%s mm"%str(Li[7])
            van8Entry.delete(0,END)
            van8Entry.insert(0,str(Li[7]))
            n[7] = int(Li[7]*((ppr*n1/n2)/(math.pi*Dg)) )
        except ValueError:
            pass         
    elif var == "Dgr":
        try:
            Dg = int(DgrEntry.get())
            DgrLabel["text"] = "%s mm"%str(Dg)
            DgrEntry.delete(0,END)
            DgrEntry.insert(0,str(Dg))
            n = [int(Li[i]*((ppr*n1/n2)/(math.pi*Dg)) ) for i in range(8)]
        except ValueError:
            pass             
    elif var == "ppr":
        try:
            ppr = int(pprEntry.get())
            pprLabel["text"] = "%s ppr"%str(ppr)
            pprEntry.delete(0,END)
            pprEntry.insert(0,str(ppr))
            n = [int(Li[i]*((ppr*n1/n2)/(math.pi*Dg)) ) for i in range(8)]
        except ValueError:
            pass   
    elif var == "ratio":
        try:
            n1 = int(n1Entry.get())
            n2 = int(n2Entry.get())
            nraLabel["text"] = "%s | %s"%(str(n1),str(n2))
            n1Entry.delete(0,END)
            n2Entry.delete(0,END)
            n1Entry.insert(0,str(n1))
            n2Entry.insert(0,str(n2))
            n = [int(Li[i]*((ppr*n1/n2)/(math.pi*Dg)) ) for i in range(8)]
        except ValueError:
            pass             
    else:
        pass

def slctvar3(var):
    global minArea, minObject, smallObj, k0, k1, k2
    if var == "mArea":    
        try:
            minArea = int(mAreaEntry.get())
            mAreaLabel["text"]= "%s"%str(minArea)
            mAreaEntry.delete(0,END)
            mAreaEntry.insert(0,str(minArea))
        except ValueError:
            pass
    elif var == "mObj":
        try:
            minObject = int(mObjEntry.get())
            mObjLabel["text"]= "%s"%str(minObject)
            mObjEntry.delete(0,END)
            mObjEntry.insert(0,str(minObject))
        except ValueError:
            pass
    elif var == "sObj":
        try:
            smallObj = int(sObjEntry.get())
            sObjLabel["text"]= "%s"%str(smallObj)
            sObjEntry.delete(0,END)
            sObjEntry.insert(0,str(smallObj))
        except ValueError:
            pass
    elif var == "k0":
        try:
            k0 = int(k0Entry.get())
            k0Label["text"]= "%s"%str(k0)
            k0Entry.delete(0,END)
            k0Entry.insert(0,str(k0))
        except ValueError:
            pass
    elif var == "k1":
        try:
            k1 = int(k1Entry.get())
            k1Label["text"]= "%s"%str(k1)
            k1Entry.delete(0,END)
            k1Entry.insert(0,str(k1))
        except ValueError:
            pass
    elif var == "k2":
        try:
            k2 = int(k2Entry.get())
            k2Label["text"]= "%s"%str(k2)
            k2Entry.delete(0,END)
            k2Entry.insert(0,str(k2))
        except ValueError:
            pass
    else:
        pass
    
def slctvar4(var):
    global a1,i1,a2,i2
    if var == "a1i1":
        try:
            a1 = int(a1Entry.get())
            ai1Label["text"]= "%s | %s"%(str(a1),str(i1))
            a1Entry.delete(0,END)
            a1Entry.insert(0,str(a1))
        except ValueError:
            pass
        try:
            i1 = int(i1Entry.get())
            ai1Label["text"]= "%s | %s"%(str(a1),str(i1))
            i1Entry.delete(0,END)
            i1Entry.insert(0,str(i1))
        except ValueError:
            pass
    elif var == "a2i2":
        try:
            a2 = int(a2Entry.get())
            ai2Label["text"]= "%s | %s"%(str(a2),str(i2))
            a2Entry.delete(0,END)
            a2Entry.insert(0,str(a2))
        except ValueError:
            pass            
        try:
            i2 = int(i2Entry.get())
            ai2Label["text"]= "%s | %s"%(str(a2),str(i2))
            i2Entry.delete(0,END)
            i2Entry.insert(0,str(i2))
        except ValueError:
            pass
    else:
        pass

def switchval(val):
    global plc
    if PLCconnectivity:
        try:
            if val == "1":
                writeMbit( plc, 1, 0, True )
            elif val == "2":
                writeMbit( plc, 1, 1, True )
            elif val == "3":
                writeMbit( plc, 1, 2, True )
            elif val == "4":
                writeMbit( plc, 1, 3, True )
            elif val == "5":
                writeMbit( plc, 1, 4, True )
            elif val == "6":
                writeMbit( plc, 1, 5, True )
            elif val == "7":
                writeMbit( plc, 1, 6, True )
            elif val == "8":
                writeMbit( plc, 1, 7, True )
            else:
                pass
        except:
            messagebox.showerror("PLC","Can not switch val")
    else:
        messagebox.showwarning("PLC","PLC have been Disconnected")
    
def rst_cnt():
    if messagebox.askokcancel("Parameters","RESET Counter ?") and PLCconnectivity :
        writeMbit( plc, 0, 2, True )
        writeMbit( plc, 0, 2, False)
        delete_log()
        print("RESET COUNTER & LOG")
        printlog("WARNING: Reset Counter & Log")
    else:
        delete_log()
        print("RESET LOG")
        printlog("WARNING: Reset log")
        pass
def delete_log():
    textlog["state"]= NORMAL
    textlog.delete(1.0,"end")     
    textlog["state"]= DISABLED    
    
def printlog(text): 
    textlog["state"]= NORMAL
    a = int( textlog.index('end-1c').split(".")[0] )
    if a > 8:
        textlog.delete(1.0,2.0)        
        textlog.insert(END,text+"\n")
    else:
        textlog.insert(END,text+"\n")
    textlog["state"]=DISABLED
#-----------------------------------------------------------------------------
def show_frame1():
    global flagCheck,pulseStack,Objlst,fps,FPS,t1
    if PLCconnectivity :
        try:
            CnctPLCLabel["text"] = "Successful connecting with PLC"
            CnctPLCLabel["fg"] = "green"
            readdata = plc.read_area(0x83,0,0,26) # read MB0->MB25
            NoP = util.get_dword(readdata, 4) #  present value of ENCODER
        except :
            messagebox.showerror("PLC","Cabel error")
            CnctPLCLabel["text"] = "PLC Disconnected"
            CnctPLCLabel["fg"] = "red"
            printlog("ERROR: PLC Cable")  
    receive,frame = cap.read()
    if not receive: # Check connect
        messagebox.showerror("CAMERA",'Cannot receive frame')
        printlog("ERROR: Camera Cable")        
    else:
        yLen = frame.shape[:2][0] - 1
        xLen = frame.shape[:2][1] - 1
        temp = frame.copy() # create temp to draw on it
        # Preprocessing
        hsv = cv.cvtColor( frame, cv.COLOR_BGR2HSV ) # convert to HSV from BGR chanel
        kernel1 = cv.getStructuringElement( cv.MORPH_RECT, (a1,a1) )
        kernel2 = cv.getStructuringElement( cv.MORPH_RECT, (a2,a2) )
        min1 = np.array( rangeCol1[:3], np.uint8 )  # GREEN + RED range1
        max1 = np.array( rangeCol1[3:], np.uint8 )
        min2 = np.array( rangeCol2[:3], np.uint8 )  # RED range2
        max2 = np.array( rangeCol2[3:], np.uint8 )
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
                cv.putText( temp, '#%s(%s)'%(t,int(area)), (xj-5,yj-5),
                            cv.FONT_HERSHEY_SIMPLEX, 1, col(t), 1, cv.LINE_AA )
                cv.putText( temp, '(%s,%s)'%(cx,cy), (xj,yj+hj+25),
                            cv.FONT_HERSHEY_SIMPLEX, 1, col(t), 1, cv.LINE_AA )
                cv.rectangle( temp, (xj,yj), (xj+wj, yj+hj), col(t), 2)
                # Add an object to the Object's Stack
                Objlst.append((cx,cy,area,t))   
        # Classify Objects   
        if len(Objlst) > 0: # if any Object exits do...
            # Check each Object in the Object's Stack and classificate it
            for obj in Objlst :
                ix,iy,t  = obj[0] , obj[1] , obj[3]    
                if   t == 1 and ix > k2 : #Group 1(A)
                    if (k0 < iy < k1) and not(flagCheck[0]) :
                        if PLCconnectivity :
                            pulseStack[0].append( NoP + n[0] )
                        print('Group 1')
                        printlog('Type: 1A (val 1)')
                        flagCheck[0] = True
                    elif (iy < k0) and flagCheck[0] :
                        flagCheck[0] = False                    
                elif t == 1 and ix < k2 : #Group 2(B)
                    if (k0 < iy < k1) and not(flagCheck[1]) :
                        if PLCconnectivity :
                            pulseStack[1].append( NoP + n[1] )
                        print('Group 2')
                        printlog('Type: 1B (val 2)')
                        flagCheck[1] = True
                    elif (iy < k0) and flagCheck[1] :
                        flagCheck[1] = False             
                elif t == 2 and ix > k2 : #Group 3
                    if (k0 < iy < k1) and not(flagCheck[2]) :
                        if PLCconnectivity :
                            pulseStack[2].append( NoP + n[2] )
                        print('Group 3')
                        printlog('Type: 2A (val 3)')
                        flagCheck[2] = True
                    elif (iy < k0) and flagCheck[2] :
                        flagCheck[2] = False               
                elif t == 2 and ix < k2 : #Group 4
                    if (k0 < iy < k1) and not(flagCheck[3]) :
                        if PLCconnectivity :
                            pulseStack[3].append( NoP + n[3] )
                        print('Group 4')
                        printlog('Type: 2B (val 4)')
                        flagCheck[3] = True
                    elif (iy < k0) and flagCheck[3] :
                        flagCheck[3] = False            
                elif t == 3 and ix > k2 : #Group 5
                    if (k0 < iy < k1) and not(flagCheck[4]) :
                        if PLCconnectivity :
                            pulseStack[4].append( NoP + n[4] )
                        print('Group 5')
                        printlog('Type: 3A (val 5)')
                        flagCheck[4] = True
                    elif (iy < k0) and flagCheck[4] :
                        flagCheck[4] = False                
                elif t == 3 and ix < k2 : #Group 6
                    if (k0 < iy < k1) and not(flagCheck[5]) :
                        if PLCconnectivity :
                            pulseStack[5].append( NoP + n[5] )
                        print('Group 6')
                        printlog('Type: 3B (val 6)')
                        flagCheck[5] = True
                    elif (iy < k0) and flagCheck[5] :
                        flagCheck[5] = False               
                elif t == 4 and ix > k2:  #Group 7
                    if (k0 < iy < k1) and not(flagCheck[6]) :
                        if PLCconnectivity :
                            pulseStack[6].append( NoP + n[6] )
                        print('Group 7')
                        printlog('Type: 4A (val 7)')
                        flagCheck[6] = True
                    elif (iy < k0) and flagCheck[6] :
                        flagCheck[6] = False                
                elif t == 4 and ix < k2 : #Group 8
                    if (k0 < iy < k1) and not(flagCheck[7]) :
                        if PLCconnectivity :
                            pulseStack[7].append( NoP + n[7] )
                        print('Group 8')
                        printlog('Type: 4B (val 8)')
                        flagCheck[7] = True
                    elif (iy < k0) and flagCheck[7] :
                        flagCheck[7] = False                
                elif t == 5 :             #Group 9
                    if (k0 < iy < k1) and not(flagCheck[8]) :
                        print('Remove')
                        printlog('Type: 5 (Remove)')
                        flagCheck[8] = True
                    elif (iy < k0) and flagCheck[8] :
                        flagCheck[8] = False               
        cv.line( temp, (5,5), (5,35), blue, 2)                    
        cv.line( temp, (5,5), (35,5), blue, 2)
        cv.putText( temp, 'X', (45,25),
                   cv.FONT_HERSHEY_SIMPLEX, 1, blue, 1, cv.LINE_AA )
        cv.putText( temp, 'Y', (5,65),
                   cv.FONT_HERSHEY_SIMPLEX, 1, blue, 1, cv.LINE_AA )
        cv.line( temp, (0,k0),(xLen,k0), blue, 1)
        cv.line( temp, (0,k1),(xLen,k1), blue, 1)
        cv.line( temp, (k2,0),(k2,yLen), blue, 2)
        cv.putText( temp, 'k0', (3,k0-5),
                   cv.FONT_HERSHEY_SIMPLEX, 0.4, blue, 1, cv.LINE_AA )
        cv.putText( temp, 'k1', (3,k1-5),
                   cv.FONT_HERSHEY_SIMPLEX, 0.4, blue, 1, cv.LINE_AA )
        cv.putText( temp, 'k2', (k2+5,15), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.4, blue, 1, cv.LINE_AA ) 
        cv.putText( temp, "B", (160,25),
                   cv.FONT_HERSHEY_SIMPLEX, 1, red,2, cv.LINE_AA)
        cv.putText( temp, "A", (480,25),
                   cv.FONT_HERSHEY_SIMPLEX, 1, red,2, cv.LINE_AA)           
        cv.putText( temp, "%.2iFPS"%(FPS), (xLen-60,20 ),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, blue, 1, cv.LINE_AA )
        fps+=1 # Show FPS 
        if fps == 30:
            t2 = timer() 
            FPS = int(30/(t2-t1))
            fps = 0
            t1 = timer() 
        showframe1 = cv.cvtColor(temp, cv.COLOR_BGR2RGBA)
        tkf1 = Image.fromarray(showframe1)
        tkf2 = Image.fromarray(thresh1)
        tkframe1 = ImageTk.PhotoImage(image=tkf1)
        tkframe2 = ImageTk.PhotoImage(image=tkf2)
        labelFrame1.tkframe1 = tkframe1 #Shows frame for display 1
        labelFrame1.configure(image=tkframe1)
        labelFrame2.tkframe2 = tkframe2 #Shows frame for display 2
        labelFrame2.configure(image=tkframe2)   
        # Send and Receive Signal from PLC
        global perateLabel,i1Label,i2Label,i3Label,i4Label
        if PLCconnectivity :
            # convert bytearray to signal
            flagOp = bool([( list(readdata)[0] & (1<<x))>>x for x in (7,6,5,4,3,2,1,0)][7]) #M0.0
            flagMotor = bool([( list(readdata)[0] & (1<<x))>>x for x in (7,6,5,4,3,2,1,0)][4]) #M0.3
            flagVal = [( list(readdata)[2] & (1<<x))>>x for x in (7,6,5,4,3,2,1,0)] #M2.x
            doVal = [( list(readdata)[3] & (1<<x))>>x for x in (7,6,5,4,3,2,1,0)] #M3.x
            # Control valve
            controlVal(1, pulseStack[0], flagVal[7])
            controlVal(2, pulseStack[1], flagVal[6])
            controlVal(3, pulseStack[2], flagVal[5])
            controlVal(4, pulseStack[3], flagVal[4])
            controlVal(5, pulseStack[4], flagVal[3])
            controlVal(6, pulseStack[5], flagVal[2])
            controlVal(7, pulseStack[6], flagVal[1])
            controlVal(8, pulseStack[7], flagVal[0])  
            # Read Counter
            ci = [util.get_int(readdata[10:],i) for i in range(0,15,2)]
            i1Label["text"] = "Type 1 : %s" %str(ci[0]+ci[1])
            i2Label["text"] = "Type 2 : %s" %str(ci[2]+ci[3])
            i3Label["text"] = "Type 3 : %s" %str(ci[4]+ci[5])
            i4Label["text"] = "Type 4 : %s" %str(ci[6]+ci[7])
            # Indicate status of Val(Q)
            if doVal[7]:
                bgri.itemconfig(led1, fill="orange")
            else:
                bgri.itemconfig(led1, fill="cyan4")
            if doVal[6]:
                bgri.itemconfig(led2, fill="orange")
            else:
                bgri.itemconfig(led2, fill="cyan4")
            if doVal[5]:
                bgri.itemconfig(led3, fill="orange")
            else:
                bgri.itemconfig(led3, fill="cyan4") 
            if doVal[4]:
                bgri.itemconfig(led4, fill="orange")
            else:
                bgri.itemconfig(led4, fill="cyan4")
            if doVal[3]:
                bgri.itemconfig(led5, fill="orange")
            else:
                bgri.itemconfig(led5, fill="cyan4")                 
            if doVal[2]:
                bgri.itemconfig(led6, fill="orange")
            else:
                bgri.itemconfig(led6, fill="cyan4")   
            if doVal[1]:
                bgri.itemconfig(led7, fill="orange")
            else:
                bgri.itemconfig(led7, fill="cyan4")                  
            if doVal[0]:
                bgri.itemconfig(led8, fill="orange")
            else:
                bgri.itemconfig(led8, fill="cyan4") 
            # Operation Indicate
            if flagOp :
                OperateLabel["text"] = "System : RUNNING"
                OperateLabel["fg"] = "green"
            else:
                OperateLabel["text"] = "System : STOPED"
                OperateLabel["fg"] = "red"
            if flagMotor :
                MotorLabel["text"] = "Motor : RUNNING"
                MotorLabel["fg"] = "green"
            else:
                MotorLabel["text"] = "Motor : STOPED"
                MotorLabel["fg"] = "red"                
        else:
            CnctPLCLabel["text"] = "Disconnecting with PLC"
            CnctPLCLabel["fg"] = "red"                           
        if flgw:
            root.after(8, show_frame1) #wait 10ms to show
            
def show_frame2():
    _,frame = cap.read()
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    temp1 = frame.copy()
    temp2 = frame.copy()
    min1 = np.array( rangeCol1[:3], np.uint8 )  # GREEN + RED range1
    max1 = np.array( rangeCol1[3:], np.uint8 )
    min2 = np.array( rangeCol2[:3], np.uint8 )  # RED range2
    max2 = np.array( rangeCol2[3:], np.uint8 )
    kernel1 = cv.getStructuringElement( cv.MORPH_RECT, (a1,a1) )
    kernel2 = cv.getStructuringElement( cv.MORPH_RECT, (a2,a2) )  
    thresh1 = cv.inRange(hsv, min1, max1)
    thresh1 = cv.morphologyEx( thresh1, cv.MORPH_OPEN , kernel1, iterations = i1)
    thresh1 = cv.morphologyEx( thresh1, cv.MORPH_CLOSE, kernel2, iterations = i2)   
    thresh2 = cv.inRange( hsv, min2, max2 ) # Thresholding : Red
    thresh2 = cv.morphologyEx( thresh2, cv.MORPH_OPEN , kernel1, iterations = i1)
    thresh2 = cv.morphologyEx( thresh2, cv.MORPH_CLOSE, kernel2, iterations = i2)
    range1 = cv.bitwise_and(temp1,temp1, mask = thresh1)
    range2 = cv.bitwise_and(temp2,temp2, mask = thresh2)
    range1 = cv.cvtColor(range1, cv.COLOR_BGR2RGBA)
    range2 = cv.cvtColor(range2, cv.COLOR_BGR2RGBA) 
    tkRange1 = Image.fromarray(range1)
    tkRange2 = Image.fromarray(range2)
    imgtk1 = ImageTk.PhotoImage(image=tkRange1)
    imgtk2 = ImageTk.PhotoImage(image=tkRange2)
    displayRange1.imgtk1 = imgtk1 #Shows frame for display 1
    displayRange1.configure(image=imgtk1)
    displayRange2.imgtk2 = imgtk2 #Shows frame for display 2
    displayRange2.configure(image=imgtk2)
    if not(flgw):
        window2.after(8, show_frame2) #wait 10ms to show
    
def range_window():
    set_false() #disable main image-processing
    global displayRange1,displayRange2,window2
    window2 = Toplevel()
    window2.title('Set Range' )
    window2.iconbitmap('eye.ico')
    window2.geometry( "1310x680" )
    window2.grab_set()
    # Create subFrame
    subframe1 = LabelFrame( window2, text='Range 1', height=160, width=620, padx=10, pady=10 )
    subframe1.grid( row=0, column=0, rowspan=1, columnspan=1, sticky='nsew',  
                padx= 5, pady= 5 )
    subframe1.grid_propagate(False)
    subframe2 = LabelFrame( window2, text='Range 2', height=160, width=620, padx=10, pady=10 )
    subframe2.grid( row=0, column=1, rowspan=1, columnspan=1, sticky= 'nsew',
                padx= 5, pady= 5 )
    subframe2.grid_propagate(False) 
    # subFrame1
    slider1 = Scale( subframe1, from_=0, to=179, label="Hue min", length = 180, 
                    orient=HORIZONTAL, command = slide1act )
    slider1.set(rangeCol1[0])
    slider2 = Scale(subframe1, from_=0, to=179, label="Hue max", 
                    orient=HORIZONTAL, command = slide2act )
    slider2.set(rangeCol1[3])
    slider3 = Scale( subframe1, from_=0, to=255, label="Saturation min", length = 180, 
                    orient=HORIZONTAL, command = slide3act )
    slider3.set(rangeCol1[1])
    slider4 = Scale(subframe1, from_=0, to=255, label="Saturation max", 
                    orient=HORIZONTAL, command = slide4act )
    slider4.set(rangeCol1[4])
    slider5 = Scale( subframe1, from_=0, to=255, label="Value min", length = 180, 
                    orient=HORIZONTAL, command = slide5act )
    slider5.set(rangeCol1[2])
    slider6 = Scale(subframe1, from_=0, to=255, label="Value max", 
                    orient=HORIZONTAL, command = slide6act )
    slider6.set(rangeCol1[5])
    slider1.grid( row=0, column=0, rowspan=1, columnspan=1, padx=8,  
                 sticky='nsew')
    slider2.grid( row=1, column=0, rowspan=1, columnspan=1, padx=8,   
                 sticky='nsew')
    slider3.grid( row=0, column=1, rowspan=1, columnspan=1, padx=8,   
                 sticky='nsew')
    slider4.grid( row=1, column=1, rowspan=1, columnspan=1, padx=8,   
                 sticky='nsew')
    slider5.grid( row=0, column=2, rowspan=1, columnspan=1, padx=8,  
                 sticky='nsew')
    slider6.grid( row=1, column=2, rowspan=1, columnspan=1, padx=8,   
                 sticky='nsew')
    # subFrame2
    slider7 = Scale( subframe2, from_=0, to=179, label="Hue min", length = 180, 
                    orient=HORIZONTAL, command = slide7act )
    slider7.set(rangeCol2[0])
    slider8 = Scale(subframe2, from_=0, to=179, label="Hue max",
                    orient=HORIZONTAL, command = slide8act )
    slider8.set(rangeCol2[3])
    slider9 = Scale( subframe2, from_=0, to=255, label="Saturation min", length = 180,
                    orient=HORIZONTAL, command = slide9act )
    slider9.set(rangeCol2[1])
    slider10 = Scale(subframe2, from_=0, to=255, label="Saturation max",
                    orient=HORIZONTAL, command = slide10act )
    slider10.set(rangeCol2[4])
    slider11 = Scale( subframe2, from_=0, to=255, label="Value min", length = 180, 
                    orient=HORIZONTAL, command = slide11act )
    slider11.set(rangeCol2[2])
    slider12 = Scale(subframe2, from_=0, to=255, label="Value max", 
                    orient=HORIZONTAL, command = slide12act )
    slider12.set(rangeCol2[5])
    slider7.grid( row=0, column=0, rowspan=1, columnspan=1, padx=8, 
                 sticky='nsew')
    slider8.grid( row=1, column=0, rowspan=1, columnspan=1, padx=8,
                 sticky='nsew')
    slider9.grid( row=0, column=1, rowspan=1, columnspan=1, padx=8,
                 sticky='nsew')
    slider10.grid( row=1, column=1, rowspan=1, columnspan=1, padx=8,
                 sticky='nsew')
    slider11.grid( row=0, column=2, rowspan=1, columnspan=1, padx=8,
                 sticky='nsew')
    slider12.grid( row=1, column=2, rowspan=1, columnspan=1,  
                 sticky='nsew')
    # showFrame
    displayRange1 = Label( window2, text= 'Range1')
    displayRange1.grid( row=1, column=0, rowspan=1, columnspan=1, padx=5, pady=5,
                 sticky='nsew')
    displayRange2 = Label( window2, text= 'Range2' )
    displayRange2.grid( row=1, column=1, rowspan=1, columnspan=1, padx=5, pady=5,
                 sticky='nsew' )
    try:
        show_frame2()
    except:
        messagebox.showerror("Set Range", "Cannot run program")
    window2.protocol("WM_DELETE_WINDOW", lambda:set_true(window2))  

def setting_window():
    set_false() #disable main image-processing
    rst_copvar()
    window3 = Toplevel()
    window3.title('Setting Parameters' )
    window3.iconbitmap('eye.ico')
    window3.geometry( "900x630" )
    window3.grab_set()
    #-------------------------------------------------------------------------
    global entryIP,entryRACK,entrySLOT,labelplc,PLCconnect,brdrVar
    global van1Entry,van2Entry,van3Entry,van4Entry,van5Entry,van6Entry,van7Entry,van8Entry
    global van1Label,van2Label,van3Label,van4Label,van5Label,van6Label,van7Label,van8Label
    global DgrEntry,pprEntry,DgrLabel,pprLabel,nraLabel,n1Entry,n2Entry
    global mAreaLabel,mObjLabel,sObjLabel,mAreaEntry,mObjEntry,sObjEntry,k0Label,k1Label,k2Label,k0Entry,k1Entry,k2Entry
    global ai1Label,ai2Label,a1Entry,i1Entry,a2Entry,i2Entry
    selectvar1 = StringVar(window3)
    selectvar1.set("CAM %s"%str(CAM))
    PLCconnect =  BooleanVar()
    PLCconnect.set(PLCconnectivity)
    brdrVar = BooleanVar()
    brdrVar.set(detectBorder)
    #-------------------------------------------------------------------------
    # create subpara-frame
    subpara1 = LabelFrame( window3, height=400, width=275, text="PLC & CAMERA", padx=5, pady=5, relief=GROOVE, borderwidth=5 )
    subpara2 = LabelFrame( window3, height=400, width=275, text="Valve Distance", padx=5, pady=5, relief=GROOVE, borderwidth=5 )
    subpara3 = LabelFrame( window3, height=400, width=275, text="Filter & Region", padx=5, pady=5, relief=GROOVE, borderwidth=5 )
    # subpara4 = LabelFrame( window3, height=400, width=275, text="Frame4", padx=5, pady=5, relief=GROOVE, borderwidth=5 )
    subpara5 = LabelFrame( window3, height=200, width=275, text="PLC Parameters", padx=5, pady=5, relief=GROOVE, borderwidth=5 )
    subpara6 = LabelFrame( window3, height=200, width=275, text="Mechanical Parameters", padx=5, pady=5, relief=GROOVE, borderwidth=5 )
    subpara7 = LabelFrame( window3, height=200, width=275, text="Pre-Processing parameters", padx=5, pady=5, relief=GROOVE, borderwidth=5 )
    # subpara8 = LabelFrame( window3, height=200, width=275, text="Frame8", padx=5, pady=5, relief=GROOVE, borderwidth=5 )
    subpara1.grid(row=0, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    subpara2.grid(row=0, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    subpara3.grid(row=0, column=2, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    subpara5.grid(row=1, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    subpara6.grid(row=1, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    subpara7.grid(row=1, column=2, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    # sub-parameters 1 -------------------------------------------------------------------
    labelcam = Label( subpara1, text="Select CAMERA", width= 20)
    slctCam = OptionMenu( subpara1, selectvar1,"CAM 0","CAM 1","CAM 2","CAM 3", command=slctcamera)
    labelplc = Label( subpara1, text="IP: %s | RACK: %s | SLOT: %s"%(IPcop,RACKcop,SLOTcop), width=20, relief=SUNKEN)
    entryIP = Entry( subpara1, width=10, borderwidth=5)
    buttonIP = Button( subpara1, text="Select IP", padx=10, pady=4, command=lambda:slctvar1("IP"))
    entryRACK = Entry( subpara1, width=10, borderwidth=5)
    buttonRack = Button( subpara1, text="Select RACK", padx=10, pady=4, command=lambda:slctvar1("RACK"))
    entrySLOT = Entry( subpara1, width=10, borderwidth=5)
    buttonSLOT = Button( subpara1, text="Select SLOT", padx=10, pady=4, command=lambda:slctvar1("SLOT"))
    buttonSAVE = Button( subpara1, text="APPLY SETTING", padx=10, pady=4, borderwidth=5, command=apply_setting )
    subofsubpara1 = LabelFrame(subpara1, height=50, width=265, text="PLC operation" , padx=3, pady=3 )
    rstPLC = Button( subofsubpara1, text="RESET\nPLC", padx=3, pady=5, borderwidth=3, command=rst_plc )
    rstCTR = Button( subofsubpara1, text="RESET\nCounter", padx=3, pady=5, borderwidth=3, command=rst_cnt )
    entryIP.insert(0,IP)
    entryRACK.insert(0,str(RACK))
    entrySLOT.insert(0,str(SLOT))
    labelcam.grid(row=0, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    slctCam.grid(row=0, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    labelplc.grid(row=1, column=0, rowspan=1, columnspan=2, padx=5, pady=15, sticky='nsew')
    entryIP.grid(row=2, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    buttonIP.grid(row=2, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    entryRACK.grid(row=3, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    buttonRack.grid(row=3, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    entrySLOT.grid(row=4, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    buttonSLOT.grid(row=4, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    buttonSAVE.grid(row=5, column=0, rowspan=1, columnspan=2, padx=5, pady=5, sticky='nsew')
    subofsubpara1.grid(row=6, column=0, rowspan=1, columnspan=2, padx=3, pady=3, sticky='nsew')
    Radiobutton( subofsubpara1, text="Connect PLC", variable=PLCconnect, value=True, 
                command=slct_connect ).grid(row=0, column=0, padx=5, pady=5, sticky='W')
    Radiobutton( subofsubpara1, text="Disconnect PLC", variable=PLCconnect, value=False, 
                command=slct_connect ).grid(row=1, column=0, padx=5, pady=5, sticky='W')
    rstPLC.grid(row=0, column=1, rowspan=2, columnspan=1, padx=6, pady=3, sticky='nsew')
    rstCTR.grid(row=0, column=2, rowspan=2, columnspan=1, padx=6, pady=3, sticky='nsew')
    # sub-parameters 5 -------------------------------------------------------------------
    delaySlider = Scale( subpara5, from_=1, to=1000, label="Valve's Open-time (ms)", length = 240, orient=HORIZONTAL, command=slide14act  )
    speedSlider = Scale( subpara5, from_=0, to=100, label="Conveyor Speed (%)", length = 240, orient=HORIZONTAL, command=slide13act )
    delaySlider.set(delay)
    speedSlider.set(speed)
    delaySlider.grid( row=0, column=0, rowspan=1, columnspan=1, padx=5, pady=10, sticky='nsew')
    speedSlider.grid( row=1, column=0, rowspan=1, columnspan=1, padx=5, pady=10, sticky='nsew')
    # sub-parameters 2 -------------------------------------------------------------------
    van1Label = Label( subpara2, text="%s mm"%str(Li[0]), padx= 6, pady= 8, width=9, relief=SUNKEN)
    van2Label = Label( subpara2, text="%s mm"%str(Li[1]), padx= 6, pady= 8, width=9, relief=SUNKEN)
    van3Label = Label( subpara2, text="%s mm"%str(Li[2]), padx= 6, pady= 8, width=9, relief=SUNKEN)
    van4Label = Label( subpara2, text="%s mm"%str(Li[3]), padx= 6, pady= 8, width=9, relief=SUNKEN)
    van5Label = Label( subpara2, text="%s mm"%str(Li[4]), padx= 6, pady= 8, width=9, relief=SUNKEN)
    van6Label = Label( subpara2, text="%s mm"%str(Li[5]), padx= 6, pady= 8, width=9, relief=SUNKEN)
    van7Label = Label( subpara2, text="%s mm"%str(Li[6]), padx= 6, pady= 8, width=9, relief=SUNKEN)
    van8Label = Label( subpara2, text="%s mm"%str(Li[7]), padx= 6, pady= 8, width=9, relief=SUNKEN)
    van1Entry = Entry( subpara2, width=12, borderwidth=5 )
    van2Entry = Entry( subpara2, width=12, borderwidth=5 )
    van3Entry = Entry( subpara2, width=12, borderwidth=5 )
    van4Entry = Entry( subpara2, width=12, borderwidth=5 )
    van5Entry = Entry( subpara2, width=12, borderwidth=5 )
    van6Entry = Entry( subpara2, width=12, borderwidth=5 )
    van7Entry = Entry( subpara2, width=12, borderwidth=5 )
    van8Entry = Entry( subpara2, width=12, borderwidth=5 )
    van1Button = Button( subpara2, text="Vavle 1", padx=6, pady=4, command=lambda:slctvar2("van1"))
    van2Button = Button( subpara2, text="Vavle 2", padx=6, pady=4, command=lambda:slctvar2("van2"))
    van3Button = Button( subpara2, text="Vavle 3", padx=6, pady=4, command=lambda:slctvar2("van3"))
    van4Button = Button( subpara2, text="Vavle 4", padx=6, pady=4, command=lambda:slctvar2("van4"))
    van5Button = Button( subpara2, text="Vavle 5", padx=6, pady=4, command=lambda:slctvar2("van5"))
    van6Button = Button( subpara2, text="Vavle 6", padx=6, pady=4, command=lambda:slctvar2("van6"))
    van7Button = Button( subpara2, text="Vavle 7", padx=6, pady=4, command=lambda:slctvar2("van7"))
    van8Button = Button( subpara2, text="Vavle 8", padx=6, pady=4, command=lambda:slctvar2("van8"))
    van1Entry.insert(0, str(Li[0]))
    van2Entry.insert(0, str(Li[1]))
    van3Entry.insert(0, str(Li[2]))
    van4Entry.insert(0, str(Li[3]))
    van5Entry.insert(0, str(Li[4]))
    van6Entry.insert(0, str(Li[5]))
    van7Entry.insert(0, str(Li[6]))
    van8Entry.insert(0, str(Li[7]))
    van1Label.grid(row=0, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van2Label.grid(row=1, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van3Label.grid(row=2, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van4Label.grid(row=3, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van5Label.grid(row=4, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van6Label.grid(row=5, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van7Label.grid(row=6, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van8Label.grid(row=7, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van1Entry.grid(row=0, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van2Entry.grid(row=1, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van3Entry.grid(row=2, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van4Entry.grid(row=3, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van5Entry.grid(row=4, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van6Entry.grid(row=5, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van7Entry.grid(row=6, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van8Entry.grid(row=7, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van1Button.grid(row=0, column=2, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van2Button.grid(row=1, column=2, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van3Button.grid(row=2, column=2, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van4Button.grid(row=3, column=2, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van5Button.grid(row=4, column=2, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van6Button.grid(row=5, column=2, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van7Button.grid(row=6, column=2, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    van8Button.grid(row=7, column=2, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    # sub-parameters 6 -------------------------------------------------------------------
    DgrLabel = Label( subpara6, text="%s mm"%str(Dg), padx= 6, pady= 8, relief=SUNKEN)
    pprLabel = Label( subpara6, text="%s ppr"%str(ppr), padx= 6, pady= 8, relief=SUNKEN)
    nraLabel = Label( subpara6, text= "%s | %s"%(str(n1),str(n2)), padx=8, relief=SUNKEN)
    DgrEntry = Entry( subpara6, width=8, borderwidth=5 )
    pprEntry = Entry( subpara6, width=8, borderwidth=5 )
    n1Entry = Entry( subpara6, width=4, borderwidth=5 )
    n2Entry = Entry( subpara6, width=4, borderwidth=5 )
    DgrButton = Button( subpara6, text="Diameter gear", padx=6, pady=6, borderwidth=2, command=lambda:slctvar2("Dgr")) 
    pprButton = Button( subpara6, text="Pulse Per Revolution", padx=6, pady=6, borderwidth=2, command=lambda:slctvar2("ppr")) 
    nraButton = Button( subpara6, text="Ratio seclection", padx=6, pady=6, borderwidth=2, command=lambda:slctvar2("ratio")) 
    DgrEntry.insert(0, str(Dg))
    pprEntry.insert(0, str(ppr))
    n1Entry.insert(0,str(n1))
    n2Entry.insert(0,str(n2))
    DgrLabel.grid(row=0, column=0, rowspan=1, columnspan=1, padx=5, pady=10, sticky='nsew')
    pprLabel.grid(row=1, column=0, rowspan=1, columnspan=1, padx=5, pady=10, sticky='nsew')
    DgrEntry.grid(row=0, column=1, rowspan=1, columnspan=2, padx=5, pady=10, sticky='nsew')
    pprEntry.grid(row=1, column=1, rowspan=1, columnspan=2, padx=5, pady=10, sticky='nsew')
    DgrButton.grid(row=0, column=3, rowspan=1, columnspan=1, padx=5, pady=10, sticky='nsew')
    pprButton.grid(row=1, column=3, rowspan=1, columnspan=1, padx=5, pady=10, sticky='nsew')
    nraLabel.grid(row=2, column=0, rowspan=1, columnspan=1, padx=5, pady=10, sticky='nsew')
    n1Entry.grid(row=2, column=1, rowspan=1, columnspan=1, padx=1, pady=10, sticky='nsew')
    n2Entry.grid(row=2, column=2, rowspan=1, columnspan=1, padx=1, pady=10, sticky='nsew')
    nraButton.grid(row=2, column=3, rowspan=1, columnspan=1, padx=5, pady=10, sticky='nsew')   
    # sub-parameters 3 -------------------------------------------------------------------------
    mAreaLabel = Label( subpara3, text="%s"%str(minArea), padx= 6, pady= 6, width=9, relief=SUNKEN)
    mObjLabel = Label( subpara3, text="%s"%str(minObject), padx= 6, pady= 6, width=9, relief=SUNKEN)
    sObjLabel = Label( subpara3, text="%s"%str(smallObj), padx= 6, pady= 6, width=9, relief=SUNKEN)
    k0Label = Label( subpara3, text="%s"%str(k0), padx= 6, pady= 8, width=9, relief=SUNKEN)
    k1Label = Label( subpara3, text="%s"%str(k1), padx= 6, pady= 8, width=9, relief=SUNKEN)
    k2Label = Label( subpara3, text="%s"%str(k2), padx= 6, pady= 8, width=9, relief=SUNKEN)
    RedSlider = Scale( subpara3, from_=0, to=100, label="Red percent (%)", length = 220, orient=HORIZONTAL, command=slide15act  )
    mAreaEntry = Entry( subpara3, width=12, borderwidth=5 )
    mObjEntry = Entry( subpara3, width=12, borderwidth=5 )
    sObjEntry = Entry( subpara3, width=12, borderwidth=5 )    
    k0Entry = Entry( subpara3, width=12, borderwidth=5 )
    k1Entry = Entry( subpara3, width=12, borderwidth=5 )
    k2Entry = Entry( subpara3, width=12, borderwidth=5 ) 
    mAreaButton = Button( subpara3, text="Filter 1", padx=6, pady=5, command=lambda:slctvar3("mArea"))
    mObjButton = Button( subpara3, text="Filter 2", padx=6, pady=5, command=lambda:slctvar3("mObj"))
    sObjButton = Button( subpara3, text="Filter 3", padx=6, pady=5, command=lambda:slctvar3("sObj"))   
    k0Button = Button( subpara3, text="K0 line", padx=6, pady=5, command=lambda:slctvar3("k0"))
    k1Button = Button( subpara3, text="K1 line", padx=6, pady=5, command=lambda:slctvar3("k1"))
    k2Button = Button( subpara3, text="K2 line", padx=6, pady=5, command=lambda:slctvar3("k2"))
    mAreaEntry.insert(0, str(minArea))
    mObjEntry.insert(0, str(minObject))
    sObjEntry.insert(0, str(smallObj))
    k0Entry.insert(0, str(k0))
    k1Entry.insert(0, str(k1))
    k2Entry.insert(0, str(k2))
    RedSlider.set(redPercent)
    mAreaLabel.grid(row=0, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    mObjLabel.grid(row=1, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    sObjLabel.grid(row=2, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    mAreaEntry.grid(row=0, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    mObjEntry.grid(row=1, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    sObjEntry.grid(row=2, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    mAreaButton.grid(row=0, column=2, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    mObjButton.grid(row=1, column=2, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    sObjButton.grid(row=2, column=2, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    RedSlider.grid(row=3, column=0, rowspan=1, columnspan=3, padx=5, pady=10, sticky='nsew')
    k0Label.grid(row=4, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    k1Label.grid(row=5, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    k2Label.grid(row=6, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    k0Entry.grid(row=4, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    k1Entry.grid(row=5, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    k2Entry.grid(row=6, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    k0Button.grid(row=4, column=2, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    k1Button.grid(row=5, column=2, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    k2Button.grid(row=6, column=2, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    # sub-parameters 7 -------------------------------------------------------------------------
    ai1Label = Label( subpara7, text="%s | %s"%(str(a1),str(i1)), padx= 3, pady= 6, width=7, relief=SUNKEN )
    ai2Label = Label( subpara7, text="%s | %s"%(str(a2),str(i2)), padx= 3, pady= 6, width=7, relief=SUNKEN )  
    a1Entry = Entry( subpara7, width=7, borderwidth=1 )
    i1Entry = Entry( subpara7, width=7, borderwidth=1 )
    a2Entry = Entry( subpara7, width=7, borderwidth=1 )
    i2Entry = Entry( subpara7, width=7, borderwidth=1 )    
    ai1Button = Button( subpara7, text="a1 & i1", padx=3, pady=10, borderwidth=5, command=lambda:slctvar4("a1i1"))
    ai2Button = Button( subpara7, text="a2 & i2", padx=3, pady=10, borderwidth=5, command=lambda:slctvar4("a2i2")) 
    brdrBox = Checkbutton( subpara7, text="Detect Border Object", variable=brdrVar, padx=6, pady=15, command=slct_brdrObj )
    a1Entry.insert(0, str(a1))
    i1Entry.insert(0, str(i1))
    a2Entry.insert(0, str(a2))
    i2Entry.insert(0, str(i2)) 
    ai1Label.grid(row=0, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    ai2Label.grid(row=1, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    a1Entry.grid(row=0, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    i1Entry.grid(row=0, column=2, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    a2Entry.grid(row=1, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    i2Entry.grid(row=1, column=2, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    ai1Button.grid(row=0, column=3, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    ai2Button.grid(row=1, column=3, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
    brdrBox.grid(row=2, column=0, rowspan=1, columnspan=4, padx=5, pady=5, sticky='w')       
    #-------------------------------------------------------------------------
    window3.protocol("WM_DELETE_WINDOW", lambda:set_true(window3))

def About():
    messagebox.showinfo("About","Agricultural Products & Fruits sorting machine (APFsort) DEMO ver1.0\nAuthor: Duc Vi & Nhan Nghia - K16 HUTECH")   

#-----------------------------------------------------------------------------
# config root menu
menuset = Menu( root )
root.config( menu= menuset )
menuset.add_command( label="Set Range", command= range_window )
menuset.add_command( label="Parameters", command = setting_window )
menuset.add_command( label="About", command = About )
connect_PLC()
connect_CAM() 
# Create subFrame
frame1 = Frame( root, height=140, width=220, padx=10, pady=5, bg="PaleGreen3" )
frame2 = Frame( root, height=140, width=420, padx=10, pady=5, bg="PaleGreen3")
frame3 = LabelFrame( root, text="Valve Status", height=140, width=630, padx=10, pady=5, relief=SUNKEN )
display1 = LabelFrame( root, height=500, width=630, text="Result", padx=5, pady=5)
display2 = LabelFrame( root, height=500, width=630, text="Binary-Mask", padx=5, pady=5 )
frame1.grid( row=0, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew') 
frame2.grid( row=0, column=1, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
frame3.grid( row=0, column=2, rowspan=1, columnspan=2, padx=5, pady=5, sticky='nsew')
display1.grid( row=1, column=0, rowspan=1, columnspan=2, padx=5, pady=5, sticky='nsew')
display2.grid( row=1, column=2, rowspan=1, columnspan=2, padx=5, pady=5, sticky='nsew')
labelFrame1 = Label( display1 )
labelFrame1.pack(fill="both", expand=True)
labelFrame2 = Label( display2 )
labelFrame2.pack(fill="both", expand=True)
# Frame3 ----------------------------------------------------------------------
i1Label = Label( frame3, text="Type 1 : 000", padx= 3, pady= 3, width=5, borderwidth=5, relief=SUNKEN)
i2Label = Label( frame3, text="Type 2 : 000", padx= 3, pady= 3, width=5, borderwidth=5, relief=SUNKEN)
i3Label = Label( frame3, text="Type 3 : 000", padx= 3, pady= 3, width=5, borderwidth=5, relief=SUNKEN)
i4Label = Label( frame3, text="Type 4 : 000", padx= 3, pady= 3, width=5, borderwidth=5, relief=SUNKEN)
bgri = Canvas( frame3, width=630, height=50, bg="brown", borderwidth=0 )
led1=bgri.create_oval(20,7, 
                      60,47, fill="cyan4" )
led2=bgri.create_oval(100,7, 
                      140,47, fill="cyan4" )
led3=bgri.create_oval(180,7, 
                      220,47, fill="cyan4" )
led4=bgri.create_oval(260,7, 
                      300,47, fill="cyan4" )
led5=bgri.create_oval(340,7, 
                      380,47, fill="cyan4" )
led6=bgri.create_oval(420,7, 
                      460,47, fill="cyan4" )
led7=bgri.create_oval(500,7, 
                      540,47, fill="cyan4" )
led8=bgri.create_oval(580,7, 
                      620,47, fill="cyan4" )
i1Botton = Button( frame3, text="Val1", padx=3, pady=3, command=lambda:switchval("1"))
i2Botton = Button( frame3, text="Val2", padx=3, pady=3, command=lambda:switchval("2"))
i3Botton = Button( frame3, text="Val3", padx=3, pady=3, command=lambda:switchval("3"))
i4Botton = Button( frame3, text="Val4", padx=3, pady=3, command=lambda:switchval("4"))
i5Botton = Button( frame3, text="Val5", padx=3, pady=3, command=lambda:switchval("5"))
i6Botton = Button( frame3, text="Val6", padx=3, pady=3, command=lambda:switchval("6"))
i7Botton = Button( frame3, text="Val7", padx=3, pady=3, command=lambda:switchval("7"))
i8Botton = Button( frame3, text="Val8", padx=3, pady=3, command=lambda:switchval("8"))
i1Label.grid(row=0, column=0, rowspan=1, columnspan=2, padx=3, pady=3, sticky='nsew')
i2Label.grid(row=0, column=2, rowspan=1, columnspan=2, padx=3, pady=3, sticky='nsew')
i3Label.grid(row=0, column=4, rowspan=1, columnspan=2, padx=3, pady=3, sticky='nsew')
i4Label.grid(row=0, column=6, rowspan=1, columnspan=2, padx=3, pady=3, sticky='nsew')
bgri.grid(row=1, column=0, rowspan=1, columnspan=8, padx=3, pady=3, sticky='nsew')
i1Botton.grid(row=2, column=0, rowspan=1, columnspan=1, padx=3, pady=3, sticky='nsew')
i2Botton.grid(row=2, column=1, rowspan=1, columnspan=1, padx=3, pady=3, sticky='nsew')
i3Botton.grid(row=2, column=2, rowspan=1, columnspan=1, padx=3, pady=3, sticky='nsew')
i4Botton.grid(row=2, column=3, rowspan=1, columnspan=1, padx=3, pady=3, sticky='nsew')
i5Botton.grid(row=2, column=4, rowspan=1, columnspan=1, padx=3, pady=3, sticky='nsew')
i6Botton.grid(row=2, column=5, rowspan=1, columnspan=1, padx=3, pady=3, sticky='nsew')
i7Botton.grid(row=2, column=6, rowspan=1, columnspan=1, padx=3, pady=3, sticky='nsew')
i8Botton.grid(row=2, column=7, rowspan=1, columnspan=1, padx=3, pady=3, sticky='nsew')
# Frame1 ---------------------------------------------------------------------- 
CnctPLCLabel = Label( frame1, text="Disconnect to PLC", fg="red", padx= 6, pady= 7, width=25, relief=SUNKEN) 
OperateLabel = Label( frame1, text="System : STOP", fg="black", padx= 6, pady= 7, width=25, relief=SUNKEN) 
MotorLabel = Label( frame1, text="MOTOR : STOP", fg="black", padx= 6, pady= 7, width=25, relief=SUNKEN) 
CnctPLCLabel.grid(row=0, column=0, rowspan=1, columnspan=1, padx=4, pady=7, sticky='nsew')
OperateLabel.grid(row=1, column=0, rowspan=1, columnspan=1, padx=4, pady=7, sticky='nsew')
MotorLabel.grid(row=2, column=0, rowspan=1, columnspan=1, padx=4, pady=7, sticky='nsew')
# Frame2 ---------------------------------------------------------------------- 
textlog = Text( frame2, height=8, width=48, state= DISABLED)
textlog.grid(row=0, column=0, rowspan=1, columnspan=1, padx=5, pady=5, sticky='nsew')
#------------------------------------------------------------------------------
try:
    show_frame1()
except:
    messagebox.showerror("Imgage Processing", "Cannot run program") 
    printlog("ERROR: Image processing cannot run")
root.protocol("WM_DELETE_WINDOW", callback_quit)  
root.mainloop()

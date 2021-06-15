#!/usr/bin/env python3

#Avoid errors from EPICS about finding multiple copies of the PVs
#Note: This only occurs if running on localhost
#Note: May need to adjust broadcast IP listed in EPICS_CA_ADDR_LIST
import os
os.environ["EPICS_CA_ADDR_LIST"]="192.168.1.255"
os.environ["EPICS_CA_AUTO_ADDR_LIST"]="NO"

import epics

import matplotlib.pyplot as plt
import numpy as np

import threading
import time

camName="cam1"


data_type      = epics.caget(camName + ':det1:DataType_RBV')
data_colorMode = epics.caget(camName + ':det1:ColorMode_RBV')
data_nr        = epics.caget(camName + ':det1:SizeY_RBV')
data_nc        = epics.caget(camName + ':det1:SizeX_RBV')
print("colorMode =", data_colorMode)
print("sizeY =", data_nr)
print("sizeX =", data_nc)

epics.caput(camName + ':det1:ImageMode', 0)  # Get single images

# def RGBorder(matrix):
#     "Convert a (3,N,M) RGB image matrix to (N,M,3) as needed by imshow"
#     R = matrix[0,:,:]
#     G = matrix[1,:,:]
#     B = matrix[2,:,:]

#     matrix_ = np.empty((matrix.shape[1],matrix.shape[2],3))
#     matrix_[:,:,

#Initialize drawing
drawLock = threading.Lock()
fig = plt.figure()
data = None
if data_colorMode == 2: #RGB1
    data = np.zeros((data_nr,data_nc,3))
else:
    print("Unknown colorMode")
    exit()
    data = np.zeros((data_nr,data_nc))
plt.imshow(data)
#plt.show()
#exit()

#Setup callback
def img_callback(pvname=None, value=None, char_value=None, **kw):
    global data
    drawLock.acquire()
    print("img_callback()")

    data = value
    if data_colorMode == 2: #RGB1
        data = data.reshape(data_nr,data_nc,3)
    else:
        exit(1)

    plt.clf()
    plt.imshow(data)
    plt.draw()

    print()
    drawLock.release()

#Fire a trigger to prefill the buffer
epics.caput(camName + ':det1:Acquire', 1)
time.sleep(0.5)

pv = epics.PV(camName + ':image1:ArrayData', auto_monitor=True, callback=img_callback)

#Start trigger thread
trig_on = True
def trigger_thread():
    global trig_on
    while trig_on:
        time.sleep(0.5)
        drawLock.acquire()
        print("Acquire!")
        epics.caput(camName + ':det1:Acquire', 1)
        drawLock.release()
tt = threading.Thread(target=trigger_thread)
tt.start()

#Show plot
plt.show()

#Stop when the plot is closed
trig_on = False
tt.join()

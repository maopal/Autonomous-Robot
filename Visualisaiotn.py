# -*- coding: utf-8 -*-
"""
Created on Wed Feb 10 17:05:42 2021

@author: Main
"""

from vpython import *
import numpy as np 
import serial
import time

ad=serial.Serial('COM6', 115200)
time.sleep(1)

xarrow=arrow(axis=vector(1,0,0),length=3, shaftwidth=.1, color=color.red)
yarrow=arrow(axis=vector(0,1,0), length=3, shaftwidth=.1, color=color.blue)
zarrow=arrow(axis=vector(0,0,1), length=3, shaftwidth=.1, color=color.green)

bboard=box( length=6, width=2, height=.2, opacity=.4, pos=vector(0,0,0))


while(True):
    while (ad.inWaiting()==0):
        pass
    dataPacket = ad.readline()
    dataPacket = str(dataPacket, 'utf-8')
    splitPacket = dataPacket.split(",")
    pitch= float(splitPacket[0])
    roll= float(splitPacket[1])
    yaw= float(splitPacket[2])
    print(pitch, roll, yaw)
    rate(50)
    k=vector(cos(yaw)*cos(pitch), sin(pitch), sin(yaw)*cos(pitch))
    y=vector(0,1,0)
    s =cross(k,y)
    v=cross(s,k)
    vrot = v*cos(roll)+cross(k,v)*sin(roll)
    bboard.axis=k
    bboard.up = vrot 
  


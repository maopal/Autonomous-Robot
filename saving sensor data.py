

#from vpython import *
import numpy as np 
import serial
import time 
import sqlite3
import random
from itertools import count
from matplotlib.animation import FuncAnimation
import pandas as pd
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D

ad=serial.Serial('COM6', 115200)
time.sleep(1)
tablename = 'magnetometer'
conn = sqlite3.connect('tablename.db')
c = conn.cursor()

def create_table():
    try:
        c.execute("CREATE TABLE IF NOT EXISTS tablename(x REAL, y REAL, z REAL)")
        conn.commit()
    except Exception as e:
        print(str(e))
create_table()


def streaming_data():
 while(True):
    while (ad.inWaiting()==0):
        pass
    dataPacket = ad.readline()
    dataPacket = str(dataPacket, 'utf-8')
    splitPacket = dataPacket.split(",")
    try:
     x= float(splitPacket[0])
     y= float(splitPacket[1])
     z= float(splitPacket[2])
    except:
     x =np.nan
     y=np.nan
     z=np.nan
    
    return x, y, z
    


def populatetable(x,y,z):
 try:
     c.execute("INSERT INTO tablename(x, y, z) VALUES (?, ?, ?)", (x, y, z))
     conn.commit()
 except:
     print("testerro")

##Overwite question
delte_table  = input("Do you want to append or overwrite table? Type 'o' for overwrite or type anything else to append: ")
if(delte_table=="o"):
    c.execute("DELETE FROM tablename")
else:
    print("appending")

c.execute("DELETE FROM tablename")

while True:
 print(streaming_data())
 populatetable(streaming_data()[0],streaming_data()[1],streaming_data()[2])
    

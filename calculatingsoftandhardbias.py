# -*- coding: utf-8 -*-
"""
Created on Tue Jun 16 21:52:05 2020

@author: Main
"""

import sqlite3
import pandas as pd
import matplotlib.pyplot as plt 
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from IPython import get_ipython
get_ipython().run_line_magic('matplotlib', 'inline')

tablename = 'magnetometer'
conn = sqlite3.connect('tablename.db')
c = conn.cursor()
saveplots=True

#views whole dataset
df1 = pd.read_sql("SELECT * FROM tablename LIMIT 5000", conn)
df = df1.dropna()
numberofnas = len(df1)-len(df)

def twod_plot(x, y, z, title, save=False):
 fig1 = plt.figure()
 ax1 = fig1.add_subplot(111,title=title)
 
 ax1.scatter(x, y, s=10, c='b', marker="o", label='xy')
 ax1.scatter(x,z, s=10, c='r', marker="o", label='xz')
 ax1.scatter(y,z, s=10, c='g', marker="o", label='yz')
 ax1.set(xlim=(-100, 100), ylim=(-100, 100))
 plt.legend(loc='upper right');
 plt.show()
 if save==True:
  fig1.savefig(r'C:\Users\Main\Documents\Work\Projects\Electronics\STM32\Visualisations\Saved mag data/' +title+ '.png')
 else:
     pass

twod_plot(df.x, df.y, df.z, 'orignal', save=saveplots)

def hard_iron(x,y,z):
    offset_x = (max(x) + min(x)) / 2
    offset_y = (max(y) + min(y)) / 2
    offset_z = (max(z) + min(z)) / 2
    
    xoff = df.x-offset_x
    yoff = df.y-offset_y
    zoff = df.z-offset_z
    return xoff, yoff, zoff, offset_x, offset_y, offset_z

xoff, yoff, zoff = hard_iron(df.x, df.y, df.z)[0:3]

hardx, hardy, hardz = hard_iron(df.x, df.y, df.z)[3:6]

twod_plot(xoff, yoff, zoff, 'Hard-iron corrected', save=saveplots)

def soft_iron(x,y,z):
 avg_delta_x = (max(x) - min(x)) / 2
 avg_delta_y = (max(y) - min(y)) / 2
 avg_delta_z = (max(z) - min(z)) / 2
 avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3

 scale_x = avg_delta / avg_delta_x
 scale_y = avg_delta / avg_delta_y
 scale_z = avg_delta / avg_delta_z
 
 corrected_x = x * scale_x
 corrected_y = y * scale_y
 corrected_z = z * scale_z
 return corrected_x, corrected_y, corrected_z, scale_x, scale_y, scale_z

corrected_x, corrected_y, corrected_z  = soft_iron(xoff, yoff, zoff)[0:3]

softx, softy, softz = soft_iron(xoff, yoff, zoff)[3:6]



twod_plot(corrected_x, corrected_y, corrected_z, 'Soft & Hard Iron corrected', save=saveplots)

def threed_plot(x,y,z,title,save=False):
 fig = plt.figure()
 ax = fig.add_subplot(111, projection='3d', title=title)
 ax.scatter(x,y,z)
 if save==True:
  fig.savefig(r'C:\Users\Main\Documents\Work\Projects\Electronics\STM32\Visualisations\Saved mag data/' +title+ '.png')
 else:
     pass

get_ipython().run_line_magic('matplotlib', 'qt')
threed_plot(df.x,df.y,df.z,'3doriginal', save=saveplots)
threed_plot(corrected_x,corrected_y,corrected_z,'3dcorrected', save=saveplots)



df['hardx'] = xoff
df['hardy'] = yoff
df['hardz'] = zoff

df['hardnsoftx'] = corrected_x
df['hardnsofty'] = corrected_y
df['hardnsoftz'] = corrected_z



print("Soft offset :", softx, softy, softz)

print("Hard offset :", hardx, hardy, hardz)

#df.to_csv(r'C:\Users\Main\Documents\Work\Projects\Electronics\STM32\Visualisations\Saved mag data\df.csv')


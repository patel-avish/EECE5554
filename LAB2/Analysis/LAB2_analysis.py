#!/usr/bin/env python
# coding: utf-8

# In[180]:


import bagpy
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import math
import statistics

#reading the rosbag files
bag_1 = bagreader(r'/home/avish/catkin_ws/src/LAB2/Data/StationaryOS.bag')
bag_2 = bagreader(r'/home/avish/catkin_ws/src/LAB2/Data/MovingOS.bag')
bag_3 = bagreader(r'/home/avish/catkin_ws/src/LAB2/Data/StationaryOCC.bag')
bag_4 = bagreader(r'/home/avish/catkin_ws/src/LAB2/Data/MovingOCC.bag')

#Reading the messages and converting it to CSV file 
message_1 = bag_1.message_by_topic('/rtk_gps_message')
message_2 = bag_2.message_by_topic('/rtk_gps_message')
message_3 = bag_3.message_by_topic('/rtk_gps_message')
message_4 = bag_4.message_by_topic('/rtk_gps_message')

data_1 = pd.read_csv(message_1)
data_2 = pd.read_csv(message_2)
data_3 = pd.read_csv(message_3)
data_4 = pd.read_csv(message_4)

###### Getting the required fields from the messages #########################

##############################################################################
# OPEN SPACE
##############################################################################
###### STATIONARY
eSOS = (data_1['UTM_easting'] - data_1['UTM_easting'].min()).to_numpy()
nSOS = (data_1['UTM_northing'] - data_1['UTM_northing'].min()).to_numpy()
aSOS = (data_1['Altitude']).to_numpy()
tSOS = (data_1['Header.stamp.secs']-data_1['Header.stamp.secs'][0]).to_numpy()
fSOS = (data_1['fixed_quality']).to_numpy()

eSOS_K = statistics.mean(eSOS)
nSOS_K = statistics.mean(nSOS)
###### MOVING
eMOS = (data_2['UTM_easting'] - data_2['UTM_easting'].min()).to_numpy()
nMOS = (data_2['UTM_northing'] - data_2['UTM_northing'].min()).to_numpy()
tMOS = (data_2['Header.stamp.secs']-data_2['Header.stamp.secs'][0]).to_numpy()
fMOS = (data_2['fixed_quality']).to_numpy()

##############################################################################
# OCCULDED SPACE
##############################################################################
###### STATIONARY
eSOCC = (data_3['UTM_easting'] - data_3['UTM_easting'].min()).to_numpy()
nSOCC = (data_3['UTM_northing'] - data_3['UTM_northing'].min()).to_numpy()
aSOCC = (data_3['Altitude']).to_numpy()
tSOCC = (data_3['Header.stamp.secs']-data_3['Header.stamp.secs'][0]).to_numpy()
fSOCC = (data_3['fixed_quality']).to_numpy()

eSOCC_K = statistics.mean(eSOCC)
nSOCC_K = statistics.mean(nSOCC)
###### MOVING
eMOCC = (data_4['UTM_easting'] - data_4['UTM_easting'].min()).to_numpy()
nMOCC = (data_4['UTM_northing'] - data_4['UTM_northing'].min()).to_numpy()
tMOCC = (data_4['Header.stamp.secs']-data_4['Header.stamp.secs'][0]).to_numpy()
fMOCC = (data_4['fixed_quality']).to_numpy()

########################## ERROR ANALYSIS #####################################
error_SOS  = []
error_SOCC = []
aerror_SOS = []
aerror_SOCC= []

for i in range (len(eSOS)):
    error_SOS.append(math.sqrt((eSOS_K-eSOS[i])**2 + (nSOS_K-nSOS[i])**2))
    aerror_SOS.append(abs(statistics.mean(aSOS)-aSOS[i]))

for i in range (len(eSOCC)):
    error_SOCC.append(math.sqrt((eSOCC_K-eSOCC[i])**2 + (nSOCC_K-nSOCC[i])**2))
    aerror_SOCC.append(abs(statistics.mean(aSOCC)-aSOCC[i]))
    
print("Open Space (Stationary)")
mean_SOS = statistics.mean(error_SOS)
std_SOS = statistics.stdev(error_SOS)
amean_SOS = statistics.mean(aerror_SOS)
astd_SOS = statistics.stdev(aerror_SOS)
print("POSITION ERROR => mean:", mean_SOS,", standard deviation:", std_SOS)
print("ALTITUDE ERROR => mean:", amean_SOS,", standard deviation:", astd_SOS)

print("Occluded Space (Stationary)")
mean_SOCC = statistics.mean(error_SOCC)
std_SOCC = statistics.stdev(error_SOCC)
amean_SOCC = statistics.mean(aerror_SOCC)
astd_SOCC = statistics.stdev(aerror_SOCC)
print("POSITION ERROR => mean:", mean_SOCC,", standard deviation:", std_SOCC)
print("ALTITUDE ERROR => mean:", amean_SOCC,", standard deviation:", astd_SOCC)

######################### STATIONARY PLOTS ###############################################
#FIX vs Time
plt.figure()
plt.plot(fSOS,color='k',label='Open-Space (STATIONARY)')
plt.plot(fMOS,color='r',label='Open-Space (MOVING)')
plt.plot(fSOCC,ls=':',c='b',linewidth='3',label='Occluded-Space (STATIONARY)')
plt.plot(fMOCC,ls=':',c='g',linewidth='2',label='Occluded-Space (MOVING)')
plt.title("Fix_Quality vs Time")
plt.xlabel("Time (seconds)")
plt.ylabel("Fix Quality")
plt.legend()
plt.savefig('Fix vs Time', dpi = 200)

#HISTOGRAM
plt.figure()
plt.hist(error_SOS)
plt.title("Open-Space STATIONARY (DISTANCE ERROR)")
plt.xlabel("Distance (meters)")
plt.ylabel("Number of Readings")
plt.savefig('Histogram: Open-Space STATIONARY', dpi = 200)

plt.figure()
plt.hist(error_SOCC)
plt.title("Occluded-Space STATIONARY (DISTANCE ERROR)")
plt.xlabel("Distance (meters)")
plt.ylabel("Number of Readings")
plt.savefig('Histogram: Occluded-Space STATIONARY', dpi = 200)

#EASTING vs NORTHING
plt.figure()
plt.scatter(eSOS,nSOS,s=5,label ='Measured Values')
plt.scatter(eSOS_K,nSOS_K,s=100,marker="X",color = 'r',label ='Known Value')
plt.title("Open-Space STATIONARY (Northing v/s Easting)")
plt.xlabel("UTM_Easting (meters)")
plt.ylabel("UTM_Northing (meters)")
plt.legend()
plt.savefig('Open-Space STATIONARY', dpi = 200)

plt.figure()
plt.scatter(eSOCC,nSOCC,s=5,label ='Measured Values')
plt.scatter(eSOCC_K,nSOCC_K,s=100,marker="X",color = 'r',label ='Known Value')
plt.title("Occluded-Space STATIONARY (Northing v/s Easting)")
plt.xlabel("UTM_Easting (meters)")
plt.ylabel("UTM_Northing (meters)")
plt.legend()
plt.savefig('Occluded-Space STATIONARY', dpi = 200)

########################## OPEN MOVING DATA SET PLOTS ##########################################
x1,y1 = eMOS[0:18],nMOS[0:18]
x2,y2 = eMOS[17:64],nMOS[17:64]
x3,y3 = eMOS[63:75],nMOS[63:75]
x4,y4 = eMOS[74:],nMOS[74:]
a1, b1 = np.polyfit(x1, y1, 1)
a2, b2 = np.polyfit(x2, y2, 1)
a3, b3 = np.polyfit(x3, y3, 1)
a4, b4 = np.polyfit(x4, y4, 1)


# line best fits => y = ax+b
x_pred1 = (y1-b1)/a1
y_pred1 = a1*x1+b1

x_pred2 = (y2-b2)/a2
y_pred2 = a2*x2+b2

x_pred3 = (y3-b3)/a3
y_pred3 = a3*x3+b3

x_pred4 = (y4-b4)/a4
y_pred4 = a4*x4+b4

#Root Mean Squared Error
RMSE_x1 = math.sqrt((1/len(x1))*(((x1-x_pred1)**2).mean())) 
RMSE_y1 = math.sqrt((1/len(y1))*(((y1-y_pred1)**2).mean())) 

RMSE_x2 = math.sqrt((1/len(x2))*(((x2-x_pred2)**2).mean())) 
RMSE_y2 = math.sqrt((1/len(y2))*(((y2-y_pred2)**2).mean()))

RMSE_x3 = math.sqrt((1/len(x3))*(((x3-x_pred3)**2).mean())) 
RMSE_y3 = math.sqrt((1/len(y3))*(((y3-y_pred3)**2).mean()))

RMSE_x4 = math.sqrt((1/len(x4))*(((x4-x_pred4)**2).mean())) 
RMSE_y4 = math.sqrt((1/len(y4))*(((y4-y_pred4)**2).mean()))

RMSE_x = (1/4)*(RMSE_x1+RMSE_x2+RMSE_x3+RMSE_x4)
RMSE_y = (1/4)*(RMSE_y1+RMSE_y2+RMSE_y3+RMSE_y4)

plt.figure()
plt.scatter(eMOS,nMOS,s=5,label ='Measured Values')
plt.plot(x1, a1*x1+b1,color = 'r',label ='LoBF 1')
plt.plot(x2, a2*x2+b2,color = 'g',label ='LoBF 2')
plt.plot(x3, a3*x3+b3,color = 'b',label ='LoBF 3')
plt.plot(x4, a4*x4+b4,color = 'tab:orange',label ='LoBF 4')
#plt.scatter(x_known_os,y_known_os,s=50,marker="X",color = 'r',label ='Known Value')
plt.title("Open-Space MOVING (Northing v/s Easting)")
plt.xlabel("UTM_Easting (meters)")
plt.ylabel("UTM_Northing (meters)")
plt.legend()
plt.savefig('Open-Space MOVING', dpi = 200)

########################## OCCLUDED MOVING DATA SET PLOTS ##########################################
x1,y1 = eMOCC[0:18],nMOCC[0:18]
x2,y2 = eMOCC[17:49],nMOCC[17:49]
x3,y3 = eMOCC[48:59],nMOCC[48:59]
x4,y4 = eMOCC[58:],nMOCC[58:]
a1, b1 = np.polyfit(x1, y1, 1)
a2, b2 = np.polyfit(x2, y2, 1)
a3, b3 = np.polyfit(x3, y3, 1)
a4, b4 = np.polyfit(x4, y4, 1)


# line best fits => y = ax+b
x_pred1 = (y1-b1)/a1
y_pred1 = a1*x1+b1

x_pred2 = (y2-b2)/a2
y_pred2 = a2*x2+b2

x_pred3 = (y3-b3)/a3
y_pred3 = a3*x3+b3

x_pred4 = (y4-b4)/a4
y_pred4 = a4*x4+b4

#Root Mean Squared Error
RMSE_x1 = math.sqrt((1/len(x1))*(((x1-x_pred1)**2).mean())) 
RMSE_y1 = math.sqrt((1/len(y1))*(((y1-y_pred1)**2).mean())) 

RMSE_x2 = math.sqrt((1/len(x2))*(((x2-x_pred2)**2).mean())) 
RMSE_y2 = math.sqrt((1/len(y2))*(((y2-y_pred2)**2).mean()))

RMSE_x3 = math.sqrt((1/len(x3))*(((x3-x_pred3)**2).mean())) 
RMSE_y3 = math.sqrt((1/len(y3))*(((y3-y_pred3)**2).mean()))

RMSE_x4 = math.sqrt((1/len(x4))*(((x4-x_pred4)**2).mean())) 
RMSE_y4 = math.sqrt((1/len(y4))*(((y4-y_pred4)**2).mean()))

RMSE_x = (1/4)*(RMSE_x1+RMSE_x2+RMSE_x3+RMSE_x4)
RMSE_y = (1/4)*(RMSE_y1+RMSE_y2+RMSE_y3+RMSE_y4)

plt.figure()
plt.scatter(eMOCC,nMOCC,s=5,label ='Measured Values')
plt.plot(x1, a1*x1+b1,color = 'r',label ='LoBF 1')
plt.plot(x2, a2*x2+b2,color = 'g',label ='LoBF 2')
plt.plot(x3, a3*x3+b3,color = 'b',label ='LoBF 3')
plt.plot(x4, a4*x4+b4,color = 'tab:orange',label ='LoBF 4')
plt.title("Occluded-Space MOVING (Northing v/s Easting)")
plt.xlabel("UTM_Easting (meters)")
plt.ylabel("UTM_Northing (meters)")
plt.legend()
plt.savefig('Occluded-Space MOVING', dpi = 200)


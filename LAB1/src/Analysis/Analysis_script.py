#!/usr/bin/env python
# coding: utf-8

# In[46]:


import bagpy
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import math
import statistics

#reading the rosbag files
bag_1 = bagreader(r'/home/avish/catkin_ws/src/Data/Walking.bag')
bag_2 = bagreader(r'/home/avish/catkin_ws/src/Data/WideOpenSpace.bag')
bag_3 = bagreader(r'/home/avish/catkin_ws/src/Data/OccludedSpace.bag')

#Reading the messages and converting it to CSV file 
message_1 = bag_1.message_by_topic('/gps')
message_2 = bag_2.message_by_topic('/gps')
message_3 = bag_3.message_by_topic('/gps')

data_1 = pd.read_csv(message_1)
data_2 = pd.read_csv(message_2)
data_3 = pd.read_csv(message_3)


###### Getting the required fields from the messages #########################


##############################################################################
# WALKING DATA SET 
##############################################################################

utm_easting_w = data_1['UTM_easting'] - data_1['UTM_easting'].min()
utm_northing_w = data_1['UTM_northing'] - data_1['UTM_northing'].min()
altitude_w = data_1['Altitude']
time_w = data_1['header.stamp.secs']-data_1['header.stamp.secs'][0]

z_known_w = 4

##############################################################################
# WIDE-OPEN SPACE STATIONARY DATA SET 
##############################################################################

utm_easting_wide = data_2['UTM_easting'] - data_2['UTM_easting'].min()
utm_northing_wide = data_2['UTM_northing'] - data_2['UTM_northing'].min()
altitude_wide = data_2['Altitude']
time_wide = data_2['header.stamp.secs']-data_2['header.stamp.secs'][0]
#time_wide = pd.Series(range(0,276))

# KNOWN CO-ORDINATES and altitude
x_known_wide = 327788.153- data_2['UTM_easting'].min()
y_known_wide = 4689328.130 - data_2['UTM_northing'].min()
z_known_wide = 6

##############################################################################
# OCCLUDED-SPACE STATIONARY DATA SET
##############################################################################

utm_easting_os = data_3['UTM_easting'] - data_3['UTM_easting'].min()
utm_northing_os = data_3['UTM_northing'] - data_3['UTM_northing'].min()
altitude_os = data_3['Altitude']
time_os = data_3['header.stamp.secs']-data_3['header.stamp.secs'][0]

# KNOWN CO-ORDINATES and altitude
x_known_os = 327984 - data_3['UTM_easting'].min()
y_known_os = 4689501 - data_3['UTM_northing'].min()
z_known_os = 5

##############################################################################
##############################################################################
##############################################################################
### ERROR ANALYSIS 
##############################################################################
##############################################################################
##############################################################################


##Line of Best Fit for walking data
x = utm_easting_w.to_numpy()
y = utm_northing_w.to_numpy()
a, b = np.polyfit(x, y, 1)

# line best fits => y = ax+b
x_pred = (y-b)/a
y_pred = a*x+b

#Root Mean Squared Error
MSE_x = (1/len(x))*(((x-x_pred)**2).mean()) 
RMSE_x = math.sqrt(MSE_x)

MSE_y = (1/len(y))*(((y-y_pred)**2).mean()) 
RMSE_y = math.sqrt(MSE_y)

z_w = altitude_w.to_numpy()
error_alt_w = []

for i in range (len(z_w)):
    error_alt_w.append(z_w[i]-z_known_w)



### Stationary data set analyis
x_wide = utm_easting_wide.to_numpy()
y_wide = utm_northing_wide.to_numpy()
z_wide = altitude_wide.to_numpy()


x_os = utm_easting_os.to_numpy()
y_os = utm_northing_os.to_numpy()
z_os = altitude_os.to_numpy()

error_wide = []
error_os =[]
error_alt_wide = []
error_alt_os = []



for i in range (len(x_wide)):
    error_wide.append(math.sqrt((x_known_wide-x_wide[i])**2 + (y_known_wide-y_wide[i])**2))
    error_alt_wide.append(z_wide[i]-z_known_wide)
    
for i in range (len(x_os)):
    error_os.append(math.sqrt((x_known_os-x_os[i])**2 + (y_known_os-y_os[i])**2))
    error_alt_os.append(z_os[i]-z_known_os)


#DISPLAY
print("ERROR STATISTICS")
print()
print("Walking Data Set")
alt_mean_w = statistics.mean(error_alt_w)
alt_median_w = statistics.median(error_alt_w)
alt_std_w = statistics.stdev(error_alt_w)
print("POSITION ERROR (from the line of BEST FITS) => RMSE(x-direction):",RMSE_x,", RMSE(y-direction):", RMSE_y)
print("ALTITUDE ERROR => mean:", alt_mean_w,", median:", alt_median_w,", standard deviation:", alt_std_w)
print()
print("Wide Open Space")
mean_wide = statistics.mean(error_wide)
median_wide = statistics.median(error_wide)
std_wide = statistics.stdev(error_wide)

alt_mean_wide = statistics.mean(error_alt_wide)
alt_median_wide = statistics.median(error_alt_wide)
alt_std_wide = statistics.stdev(error_alt_wide)
print("POSITION ERROR => mean:", mean_wide,", median:", median_wide,", standard deviation:", std_wide)
print("ALTITUDE ERROR => mean:", alt_mean_wide,", median:", alt_median_wide,", standard deviation:", alt_std_wide)

print()
print("Occluded Space")
mean_os = statistics.mean(error_os)
median_os = statistics.median(error_os)
std_os = statistics.stdev(error_os)

alt_mean_os = statistics.mean(error_alt_os)
alt_median_os = statistics.median(error_alt_os)
alt_std_os = statistics.stdev(error_alt_os)
print("POSITION ERROR => mean:", mean_os,", median:", median_os,", standard deviation:", std_os)
print("ALTITUDE ERROR => mean:", alt_mean_os,", median:", alt_median_os,", standard deviation:", std_os)




##############################################################################
# PLOTTING STATIONARY DATA SET
##############################################################################

plt.rcParams.update({'font.size': 6})

#NORTHING VS EASTING
plt.figure()

plt.subplot(2,1,1)
plt.scatter(utm_easting_wide,utm_northing_wide,s=5,label ='Measured Values')
plt.scatter(x_known_wide,y_known_wide,s=50,marker="X",color = 'r',label ='Known Value')
plt.title("WIDE-OPEN SPACE DATA (Northing v/s Easting)")
plt.xlabel("UTM_Easting (meters)")
plt.ylabel("UTM_Northing (meters)")
plt.legend()

plt.subplot(2,1,2)
plt.scatter(utm_easting_os,utm_northing_os,s=5,label ='Measured Values')
plt.scatter(x_known_os,y_known_os,s=50,marker="X",color = 'r',label ='Known Value')
plt.title("OCCLUDED-SPACE DATA (Northing v/s Easting)")
plt.xlabel("UTM_Easting (meters)")
plt.ylabel("UTM_Northing (meters)")
plt.legend()
plt.tight_layout(pad=5.0)

plt.savefig('Stationary-NvsE.png', dpi = 200)

#ALTITUDE VS TIME
plt.figure()

plt.subplot(2,1,1)
plt.scatter(time_wide,altitude_wide,s=5,label ='Measured Values')
plt.axhline(z_known_wide,color = 'r',label ='Known Altitude')
plt.title("WIDE-OPEN SPACE DATA (Altitude v/s Time)")
plt.xlabel("Time (seconds)")
plt.ylabel("Altitude (meters)")
plt.legend()
#plt.show()

plt.subplot(2,1,2)
plt.scatter(time_os,altitude_os,s=5,label ='Measured Values')
plt.axhline(z_known_os,color = 'r',label ='Known Altitude')
plt.title("OCCLUDED-SPACE DATA (Altitude v/s Time)")
plt.xlabel("Time (seconds)")
plt.ylabel("Altitude (meters)")
plt.legend()
#plt.show()
plt.tight_layout(pad=2.0)
plt.savefig('Stationary-AvsT.png', dpi = 200)


#HISTOGRAM
plt.figure()
plt.subplot(1,2,1)
plt.hist(error_wide)
plt.title("WIDE-OPEN SPACE DATA (DISTANCE ERROR)")
plt.xlabel("Distance (meters)")
plt.ylabel("Number of Readings")
#plt.show()

plt.subplot(1,2,2)
plt.hist(error_os)
plt.title("OCCLUDED-SPACE DATA (DISTANCE ERROR)")
plt.xlabel("Distance (meters)")
plt.ylabel("Number of Readings")
#plt.show()
plt.savefig('Stationary-histogram.png', dpi = 200)

# using padding
plt.tight_layout(pad=2.0)


#############################################################################
# PLOTTING WALKING DATA SET
##############################################################################

plt.figure()
plt.scatter(utm_easting_w,utm_northing_w,s=5,label ='Measured Values')
plt.plot(x, a*x+b,color = 'r',label ='Line of Best Fit')
plt.title("WALKING DATA (Northing v/s Easting)")
plt.xlabel("UTM_Easting (meters)")
plt.ylabel("UTM_Northing (meters)")
plt.legend()
plt.savefig('Walking-NvsE.png', dpi = 200)

plt.figure()
plt.scatter(time_w,altitude_w,s=10,label ='Measured Altitude')
plt.axhline(z_known_w,color = 'r',label ='Known Altitude')
plt.title("WALKING DATA (Altitude v/s Time)")
plt.xlabel("Time (seconds)")
plt.ylabel("Altitude (meters)")
plt.legend()
plt.savefig('Walking-AvsT.png', dpi = 200)

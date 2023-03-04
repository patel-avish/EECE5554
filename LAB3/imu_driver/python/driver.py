#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import utm
import serial
import sys
import numpy as np
from imu_driver.msg import *
from imu_driver.msg import Vectornav

## Define the driver method
def driver():
    pub = rospy.Publisher('imu', Vectornav, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    msg = Vectornav()
    
    
    args = rospy.myargv(argv = sys.argv)
    

    connected_port = args[1]
    serial_port = rospy.get_param('~port',connected_port)
    serial_baud = rospy.get_param('~baudrate',115200)

    ser = serial.Serial(serial_port, serial_baud, timeout = 3)
    
    while not rospy.is_shutdown():
        recieve = str(ser.readline()) #read lines from the serial port 
        
        # Read only "VNYMR" data
        if "$VNYMR" in str(recieve):
            data = str(recieve).split(",")
            print(data)
            
            current = rospy.get_rostime()
            rospy.loginfo("Current Time is %i %i", current.secs, current.nsecs)
            
            yaw = float(data[1])
            pitch = float(data[2])
            roll = float(data[3])
            magX = float(data[4])
            magY = float(data[5])
            magZ = float(data[6])
            laccX = float(data[7])
            laccY = float(data[8])
            laccZ = float(data[9])
            avelX = float(data[10])
            avelY = float(data[11])
            avelZ = float(data[12][0:9])
            
            ## Euler Angles to Quaternion
            # convert angles from degrees to radian
            rrad = roll*np.pi/180
            prad = pitch*np.pi/180
            yrad = yaw*np.pi/180
            cr = np.cos(rrad/2)
            sr = np.sin(rrad/2)
            cp = np.cos(prad/2)
            sp = np.sin(prad/2)
            cy = np.cos(yrad/2)
            sy = np.sin(yrad/2)
            
            qw = (cr*cp*cy) + (sr*sp*sy)
            qx = (sr*cp*cy) - (cr*sp*sy)
            qy = (cr*sp*cy) + (sr*cp*sy)
            qz = (cr*cp*sy) - (sr*sp*cy)
            
            #Publishing the message
            msg.header.frame_id = 'imu1_frame'
            msg.header.stamp.secs = int(current.secs)
            msg.header.stamp.nsecs = int(current.nsecs)
            
            msg.imu.orientation.x = qx
            msg.imu.orientation.y = qy
            msg.imu.orientation.z = qz
            msg.imu.orientation.w = qw
            
            msg.imu.linear_acceleration.x = laccX
            msg.imu.linear_acceleration.y = laccY
            msg.imu.linear_acceleration.z = laccZ
            
            msg.imu.angular_velocity.x = avelX
            msg.imu.angular_velocity.y = avelY
            msg.imu.angular_velocity.z = avelZ
            
            msg.mag_field.magnetic_field.x = magX
            msg.mag_field.magnetic_field.y = magY
            msg.mag_field.magnetic_field.z = magZ
            
            msg.VNYMR = recieve
            
            pub.publish(msg)
            

if __name__ == '__main__':
    try:
        driver()
    except rospy.ROSInterruptException:
        pass

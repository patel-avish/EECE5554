#!/usr/bin/env python3

import rospy
import numpy as np
from imu_driver.srv import to_quaternion, to_quaternionResponse

class Q:
    def __init__(self, x=0, y=0, z=0, w=1):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
            
def handle_to_quaternion(req):
    Ex = req.x
    Ey = req.y
    Ez = req.z
    
    QQ = euler_to_quaternion(Ex,Ey,Ez)
    
    return to_quaternionResponse(qx = QQ.x,qy = QQ.y,qz = QQ.z,qw = QQ.w)
    

def euler_to_quaternion_server():
   rospy.init_node('euler_to_quaternion_server')
   s = rospy.Service('euler_to_quaternion',to_quaternion,handle_to_quaternion)
   rospy.spin()


def euler_to_quaternion(roll,pitch,yaw):
    
    # convert angles from degrees to radian
    rrad = roll*np.pi/180
    prad = pitch*np.pi/180
    yrad = yaw*np.pi/180
    
    # get sins and cosins
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
    
    return Q(x=qx, y=qy, z=qz, w=qw)
    
if __name__ == '__main__':
    euler_to_quaternion_server()

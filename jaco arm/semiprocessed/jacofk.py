#!/usr/bin/env python

"""
Created, tested and maintained by Rohit for RBE 501 Final Project with JACO arm
"""

import rospy
import copy
from numpy import *
from math import *
from sensor_msgs.msg import JointState
import tf
from jacoTF import *


"""
The purpose of this node is to read current joint angles from the jaco arm
and publish the cartesian coordinates of the end effector of the JACO arm. 
"""


class jacofk(jacoTF):

    def calcFK(self, msg):
        joint_val = []
        for i in xrange(0,6):
            joint_val.append(msg.position[i])
        q = mat(joint_val)
        #q = self.jacotoDH(q)
        q = self.DHtoJaco(q)
        T_final = 1
        #T_final = T_final * self.TAPI()
        #T_final = T_final * self.TB() 
        T_final = T_final * self.TAPI()
        T_final = T_final * self.TJ1(q[0,0]) 
        T_final = T_final * self.TJ2(q[0,1]) * self.TJ3(q[0,2])
        T_final = T_final * self.TJ3Offset() * self.TJ4(q[0,3])
        T_final = T_final * self.TJ5(q[0,4]) *self.TJ6(q[0,5]) * self.Tend()
        print T_final
        return T_final
        
    def jacotoDH(self, q):
        q[0,0] = -(q.item(0) - radians(180))
        q[0,1] =  (q.item(1) - radians(270))
        q[0,2] = -(q.item(2) - radians(90))
        q[0,3] = -(q.item(3) - radians(180))
        q[0,4] = -(q.item(4) - radians(180))
        q[0,5] = -(q.item(5) - radians(180 + 80))
        return q
    def DHtoJaco(self, q):
        q[0,0] = -q.item(0) + radians(180)
        q[0,1] =  q.item(1) + radians(270)
        q[0,2] = -q.item(2) + radians(90)
        q[0,3] = -q.item(3) + radians(180)
        q[0,4] = -q.item(4) + radians(180)
        q[0,5] = -q.item(5) + radians(180 + 80)
        return q
        
        
    def transform(self, theta, a, alpha, d):
        T = [[cos(theta),
              -sin(theta)* cos(alpha),
               sin(theta)* sin(alpha),
               a * cos(theta)]]
        T.append([ sin(theta),
                  cos(theta) * cos(alpha),
                  -cos(theta)* sin(alpha),
                  a * sin(theta)])
        T.append([0,
                  sin(alpha),
                  cos(alpha),
                  d])
        T.append([0,0,0,1])
        T = mat(T)
        return T
        
    def __init__(self):
        # Initialize Node
        rospy.init_node('rbansal_srao_jacofk')
        
        # Setup publisher and Subscriber
        #self.endeffectorpose = rospy.Publisher('/map_OE', OccupancyGrid, latch=True)
        self.jointconfig = rospy.Subscriber('/jaco/joint_state', JointState , self.calcFK, queue_size=1)
        
        
# This is the program's main function
if __name__ == '__main__':
    
    node = jacofk()
    rospy.spin()
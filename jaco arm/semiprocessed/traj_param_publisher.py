#!/usr/bin/env python

"""
Created, tested and maintained by Rohit for RBE 501 Final Project with JACO arm
"""

import rospy
from numpy import *
from math import *
from sensor_msgs.msg import JointState
from jacoTF import *
from jacoarm.msg import traj_params

"""
The purpose of this node is to create and and publish traj_param_publisher parameters
"""

class traj_param_publisher():
            
        
    def __init__(self, p0, pf, v0, vf, a0, af, t0 = 0, tf=5,tstep = 0.05):
        # Initialize Node
        rospy.init_node('rbansal_srao_traj_param_publisher')
        
        # Setup publisher and Subscriber
        self.pub = rospy.Publisher('/rbe_jacoapi/traj_params', traj_params, latch=True)
        #self.pub = rospy.Subscriber('/rbe_jacoapi/traj_params', traj_params, latch=True)

        self.msg = traj_params()
        self.msg.t0  = t0
        self.msg.tf  = tf
        self.msg.tstep = tstep
        self.msg.pos_x  = p0[0]
        self.msg.pos_y  = p0[1]
        self.msg.pos_z  = p0[2]
        self.msg.pos_xf = pf[0]
        self.msg.pos_yf = pf[1]
        self.msg.pos_zf = pf[2]
        
        self.msg.vel_x  = v0[0]
        self.msg.vel_y  = v0[1]
        self.msg.vel_z  = v0[2]
        self.msg.vel_xf = vf[0]
        self.msg.vel_yf = vf[1]
        self.msg.vel_zf = vf[2]
        
        self.msg.acc_x  = a0[0]
        self.msg.acc_y  = a0[1]
        self.msg.acc_z  = a0[2]
        self.msg.acc_xf = af[0]
        self.msg.acc_yf = af[1]
        self.msg.acc_zf = af[2]
        
        self.pub.publish(self.msg)
        print "Trajectory paramters were published."
        rospy.sleep(1.0)
        
        
        
        
# This is the program's main function
if __name__ == '__main__':
    node = traj_param_publisher((0.21,-0.27,0.49), (0.4,0.1,0.7), (0,0,0),(0,0,0),(0,0,0),(0,0,0))

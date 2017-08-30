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
from jacoarm.msg import trajectorymsg

"""
The purpose of this node is to create and generate a set of waypoints for the
JACO arm to follow, i.e. to create a discretized trajectory. 
"""

class Trajectory():
    
    def axisD(self, p0, pf, v0, vf, a0, af, i):
        axisD = [p0[i], v0[i], a0[i], pf[i], vf[i], af[i]]
        return mat(axisD).T
    
    def pdt(self, t, matrix = False):
        if not matrix:
            return [ 1, t, t**2,   t**3,    t**4,    t**5]
        else:
            return mat([ 1, t, t**2,   t**3,    t**4,    t**5])

        
    def vdt(self, t, matrix = False):
        if not matrix:
            return [ 0, 1, 2*t, 3*(t**2),  4*(t**3),  5*(t**4)]
        else:
            return mat([ 0, 1, 2*t, 3*(t**2),  4*(t**3),  5*(t**4)])
        
    def adt(self, t, matrix = False):
        if not matrix:
            return [ 0, 0,   2,   6*t, 12*(t**2), 20*(t**3)]
        else:
            return mat([ 0, 0,   2,   6*t, 12*(t**2), 20*(t**3)])
        
    def buildA(self, t0, tf):
        A = [self.pdt(t0),
             self.vdt(t0),
             self.adt(t0), 
             self.pdt(tf),
             self.vdt(tf), 
             self.adt(tf)]
        return mat(A)
    
    def getWaypoint(self, t):
        wp = mat([self.pdt(t),
                  self.vdt(t),
                  self.adt(t)])
        return wp
    
    def genTrajectories(self):
        
        xTraj = []
        yTraj = []
        zTraj = []
        
        px_traj = []
        vx_traj = []
        ax_traj = []
        
        py_traj = []
        vy_traj = []
        ay_traj = []
        
        pz_traj = []
        vz_traj = []
        az_traj = []
        
        for i in self.xfrange(self.t0, self.tf, self.tstep):
             xTraj.append([(self.pdt(i, True) * self.xcoeff).item(0), 
                           (self.vdt(i, True) * self.xcoeff).item(0), 
                           (self.adt(i, True) * self.xcoeff).item(0)])
             yTraj.append([(self.pdt(i, True) * self.ycoeff).item(0), 
                           (self.vdt(i, True) * self.ycoeff).item(0), 
                           (self.adt(i, True) * self.ycoeff).item(0)])
             zTraj.append([(self.pdt(i, True) * self.zcoeff).item(0), 
                           (self.vdt(i, True) * self.zcoeff).item(0), 
                           (self.adt(i, True) * self.zcoeff).item(0)])
        
        for i in xrange(0, len(xTraj), 1):
            px_traj.append(xTraj[i][0])
            vx_traj.append(xTraj[i][1])
            ax_traj.append(xTraj[i][2])
            
            py_traj.append(yTraj[i][0])
            vy_traj.append(yTraj[i][1])
            ay_traj.append(yTraj[i][2])
            
            pz_traj.append(zTraj[i][0])
            vz_traj.append(zTraj[i][1])
            az_traj.append(zTraj[i][2])
            
        traj_msg = trajectorymsg()
        
        traj_msg.t0 = self.t0
        traj_msg.tf = self.tf
        traj_msg.tstep = self.tstep
               
        
        traj_msg.posx_traj = px_traj
        traj_msg.velx_traj = vx_traj
        traj_msg.accx_traj = ax_traj
        
        traj_msg.posy_traj = py_traj
        traj_msg.vely_traj = vy_traj
        traj_msg.accy_traj = ay_traj
        
        traj_msg.posz_traj = pz_traj
        traj_msg.velz_traj = vz_traj
        traj_msg.accz_traj = az_traj
        
        self.traj_pub.publish(traj_msg)
        
        
    def traj_callback(self, msg):

        print "Callback was invoked"
        p0 = (msg.pos_x ,msg.pos_y ,msg.pos_z )
        pf = (msg.pos_xf,msg.pos_yf,msg.pos_zf)
        
        v0 = (msg.vel_x ,msg.vel_y ,msg.vel_z )
        vf = (msg.vel_xf,msg.vel_yf,msg.vel_zf)
        
        a0 = (msg.acc_x ,msg.acc_y ,msg.acc_z )
        af = (msg.acc_xf,msg.acc_yf,msg.acc_zf)
        
        self.t0 = msg.t0
        self.tf = msg.tf
        self.tstep = msg.tstep
        
        xd = self.axisD(p0, pf, v0, vf, a0, af, 0)
        yd = self.axisD(p0, pf, v0, vf, a0, af, 1)
        zd = self.axisD(p0, pf, v0, vf, a0, af, 2)
        
        A = self.buildA(self.t0, self.tf)
        Ainv = linalg.pinv(A)
        self.xcoeff = Ainv * xd
        self.ycoeff = Ainv * yd
        self.zcoeff = Ainv * zd
        
        self.genTrajectories()
	
	
	print "Calculated Trajectories"
        
    
    
    def xfrange(self, start, stop, step):
        while not rospy.is_shutdown() and start < stop:
            yield start
            start += step
        
    def __init__(self):
        # Initialize Node
        rospy.init_node('rbansal_srao_Trajectory')
        self.t0 = 0
        self.tf = 0
        self.tstep = 0
        # Setup publisher and Subscriber
        self.traj_pub = rospy.Publisher('/rbe_jacoapi/trajectories', trajectorymsg, latch=True)
        traj_params_sub = rospy.Subscriber('/rbe_jacoapi/traj_params', traj_params, self.traj_callback ,queue_size=1)
        
        
# This is the program's main function
if __name__ == '__main__':
    node = Trajectory()
    rospy.spin()

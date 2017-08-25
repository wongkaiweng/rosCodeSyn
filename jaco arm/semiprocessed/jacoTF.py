#!/usr/bin/env python

"""
Created, tested and maintained by Rohit for RBE 501 Final Project with JACO arm
"""

import rospy
import copy
from numpy import *
from numpy.linalg import norm
from numpy.linalg import pinv
from math import *
from sensor_msgs.msg import JointState
import tf

"""
The purpose of this script is to provide an abstract class that defines jaco tf params.
"""


class jacoTF(object):

    def TB(self):
        rot = self.rotation_m(cos(pi/2), -sin(pi/2), 0, sin(pi/2), cos(pi/2), 0, 0, 0, 1)
        vec = self.translation_v(0,0,0)
        return self.transformation(rot, vec)
    
    def TAPI(self):
        rot = self.rotation_m(1,0,0,0,1,0, 0, 0, 1)
        vec = self.translation_v(0, 0, 0.028)
        return self.transformation(rot, vec)
    
    def TJ1(self, q1):
        rot = self.rotation_m(cos(q1), -sin(q1), 0, -sin(q1), -cos(q1), 0, 0, 0, -1)
        vec = self.translation_v(0,0,0.1370+0.0174)
        return self.transformation(rot, vec)
    
    def TJ2(self, q2):
        rot = self.rotation_m(sin(q2), cos(q2), 0, 0, 0, 1, cos(q2), -sin(q2), 0)
        vec = self.translation_v(0, 0, -0.1181)
        return self.transformation(rot, vec)
    
    def TJ3(self, q3):
        rot = self.rotation_m(-cos(q3), sin(q3), 0, sin(q3), cos(q3), 0, 0, 0, -1)
        vec = self.translation_v( 0.4100, 0, 0)
        return self.transformation(rot, vec)
    
    def TJ3Offset(self):
        rot = self.rotation_m(1, 0, 0, 0, 1, 0, 0, 0, 1)
        vec = self.translation_v(0, 0, 0.0113)
        return self.transformation(rot, vec)
    
    def TJ4(self, q4):
        rot = self.rotation_m(0,0,-1, sin(q4), cos(q4), 0, cos(q4), -sin(q4), 0)
        vec = self.translation_v(0.2070, 0, 0)
        return self.transformation(rot, vec)
    
    def TJ5(self, q5):
        rot = self.rotation_m(cos((radians(-55)))*cos(q5),cos((radians(-55)))*-sin(q5),sin((radians(-55))), sin(q5), cos(q5), 0, -sin((radians(-55)))*cos(q5), sin((radians(-55)))*sin(q5), cos((radians(-55))))
        vec = self.translation_v(cos(radians(55))*0.0750, 0, -sin(radians(55))*0.0750)
        return self.transformation(rot, vec)
    
    def TJ6(self, q6):
        rot = self.rotation_m(cos((radians(55)))*cos(q6),cos((radians(55)))*-sin(q6),sin((radians(55))), sin(q6), cos(q6), 0, -sin((radians(55)))*cos(q6), sin((radians(55)))*sin(q6), cos((radians(55))))
        vec = self.translation_v(-cos(radians(55))*0.0750, 0, -sin(radians(55))*0.0750)
        return self.transformation(rot, vec)
    
    def Tend(self):
        rot = self.rotation_m(1,0,0,0,1,0,0,0,1)
        vec = self.translation_v(0,0,-0.1850)
        return self.transformation(rot, vec)
    
    def rotation_m(self, e11,e12,e13,e21,e22,e23,e31,e32,e33):
        r = [[e11, e12, e13], [e21, e22, e23],[e31, e32, e33]]
        #TODO:add checks for valid rotation matrix 
        return mat(r)
    
    def translation_v(self, x,y,z):
        vec =  mat([x,y,z])
        return vec.transpose()
    
    def transformation(self, rot, vec):
        rot = array(rot)
        T = vstack((rot, array([0,0,0])))
        vec = array(vec)
        vec =  vstack((vec, array([1])))
        T = vstack((T.T, vec.T))
        T = mat(T.T)
        return T
        
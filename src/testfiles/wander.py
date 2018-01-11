#!/usr/bin/env python

""" Simple example of subscribing to sensor messages and publishing
twist messages to the turtlebot.

Author: Nathan Sprague
Version: 4/4/2014
"""

# The string passed to load_manifest needs to match the package name.
import rospy
import math
import random

# Twist is the message type for sending movement commands.
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# globals
SCAN = None

# This function will be called every time a new scan message is
# published.
def scan_callback(scan):
    """ scan will be of type LaserScan """

    # Save a global reference to the most recent sensor state so that
    # it can be accessed in the main control loop.
    global SCAN
    SCAN = scan


# This is the 'main'
def start():
    
    # Turn this into an official ROS node named approach
    rospy.init_node('approach')

    # Subscribe to the /scan topic.  From now on
    # scan_callback will be called every time a new scan message is
    # published.
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    # Create a publisher object for sending Twist messages to the
    # turtlebot_node's velocity topic. 
    vel_pub = rospy.Publisher('/cmd_vel_mux', Twist, queue_size=10) # CAT: Fixed queue size

    # Create a twist object. 
    # twist.linear.x represents linear velocity in meters/s.
    # twist.angular.z represents rotational velocity in radians/s.
    twist = Twist()


    # Wait until the first scan is available.
    while SCAN is None and not rospy.is_shutdown():
        pass

    # Try to maintain this target distance to the wall.
    target = 1.0

    turn = 0
    vel =0

    while not rospy.is_shutdown():
        if  math.isnan(SCAN.ranges[320]) or SCAN.ranges[320] < target:
            turn = random.randint(1,8)


        else:
            if SCAN.ranges[320]/10 >= .3:
                vel=.3
            else:
                vel = SCAN.ranges[320]/10

        twist.linear.x = vel  # forward velocity in meters/second
        twist.linear.x = 0.4  # forward velocity in meters/second
        twist.angular.z = turn   # rotation in radians/second
        turn = 0
        print "Range: ", SCAN.ranges[320], vel
        vel_pub.publish(twist) # These velocities will be applied for .6 seconds
                               # unless another command is sent before that. 
        rospy.sleep(.1)        # Pause for .1 seconds.
        

# This is how we usually call the main method in Python. 
if __name__ == "__main__":
   start()

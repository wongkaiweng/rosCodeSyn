#! /usr/bin/env python

import roslib;
import rospy
import actionlib
import jaco_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
from jaco_msgs.srv import HomeArm
import socket

def cartesian_pose_client(position, orientation):
    """Send a cartesian goal to the action server."""
    action_address = '/jaco_arm_driver/arm_pose/arm_pose'
    client = actionlib.SimpleActionClient(action_address, jaco_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = jaco_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=('jaco_api_origin'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    client.send_goal(goal)

    if client.wait_for_result(rospy.Duration(60.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None

def gripper_client(finger_positions):
    """Send a gripper goal to the action server."""
    action_address = '/jaco_arm_driver/fingers/finger_positions'
    client = actionlib.SimpleActionClient(action_address,
                                          jaco_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = jaco_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])

    # The MICO arm has only two fingers, but the same action definition is used
    if len(finger_positions) < 3:
        goal.fingers.finger3 = 0.0
    else:
        goal.fingers.finger3 = float(finger_positions[2])

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(60.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the gripper action timed-out')
        return None

def home():
    rospy.ServiceProxy("/jaco_arm_driver/in/home_arm", HomeArm)()
    gripper_client((0, 0, 0))

def approach():
    cartesian_pose_client((0.2, 0.5, 0.0), (0.0, 0.738, -0.675, 0.0))

def grab():
    cartesian_pose_client((0.2, 0.68, 0.0), (0.04, -0.75, 0.659, 0.03))
    gripper_client((50, 50, 50))

def position_arm():
    cartesian_pose_client((0.155, -0.586, 0.8), (0.735, 0.09, -0.098, 0.664))

def shower():
    cartesian_pose_client((0.15, -0.615, 0.774), (-0.24, 0.565, 0.757, 0.224))

if __name__ == '__main__':
    rospy.init_node('ice_bucket_challenge')
    while not rospy.is_shutdown():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(('', 1337))

        key = ""
        while key != "x":
            key = s.recvfrom(1)[0]

        s.close()
        
        try:
            home()
            approach()
            grab()
            position_arm()
            shower()
        except rospy.ROSInterruptException:
            print "program interrupted before completion"
#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

#roslaunch ur_gazebo ur5.launch

JOINT_NAMES = ['j2s7s300_joint_1', 'j2s7s300_joint_2', 'j2s7s300_joint_3', 'j2s7s300_joint_4', 'j2s7s300_joint_5', 'j2s7s300_joint_6', 'j2s7s300_joint_7']
Q1 = [0.9217745989347632, 4.893839010211762, 0.1285402289560838, 4.791091006198828, 5.775819249406404, 4.810200258289667, 0.0]
Q2 = [1.5833645008567891, 4.926286938716483, 0.01948879488608728, 4.785513413050978, 5.632924078398825, 4.732378463594495, 0.0]
Q3 = [1.611179234331941, 4.706930137983049, 0.15908469624141863, 4.9392660845748075, 5.908728669521643, 4.851022468662938, 0.0]

client = None

def move1():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=rospy.Duration(4.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_disordered():
    order = [4, 2, 3, 1, 5, 0]
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = [JOINT_NAMES[i] for i in order]
    q1 = [Q1[i] for i in order]
    q2 = [Q2[i] for i in order]
    q3 = [Q3[i] for i in order]
    g.trajectory.points = [
        JointTrajectoryPoint(positions=q1, velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=q2, velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=q3, velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=rospy.Duration(4.0))]
    client.send_goal(g)
    client.wait_for_result()
    
def move_repeated():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    
    d = 2.0
    g.trajectory.points = []
    for i in range(10):
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q1, velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q2, velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q3, velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=rospy.Duration(d)))
        d += 2
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_interrupt():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=rospy.Duration(4.0))]
    
    client.send_goal(g)
    time.sleep(2.0)
    print "Interrupting"
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def main():
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('j2s7s300/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        #move1()
        move_repeated()
        #move_disordered()
        #move_interrupt()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()

#!/usr/bin/env python

import roslib; roslib.load_manifest('jaco_driver')
import rospy
import actionlib
import jaco_msgs.msg
import sys
from jacoarm.msg import trajectorymsg

class follow_traj():
        
    def traj_callback(self, msg):
        px_traj = msg.posx_traj 
        py_traj = msg.posy_traj 
        pz_traj = msg.posz_traj 
        
        if(len(px_traj) != len(py_traj) or len(px_traj) != len(pz_traj)):
            rospy.logwarn("The lengths of the trajectories don't match. Aborting")
            return
        else:
            rospy.sleep(rospy.Duration(msg.t0, 0))
            for i in xrange(0, len(px_traj)):
                rospy.sleep(rospy.Duration(msg.tstep, 0))
                if(rospy.is_shutdown()):
                    rospy.logwarn("Shutdown request received")
                    return 
                self.pose_client(px_traj[i], py_traj[i], pz_traj[i])
                
        
    def pose_client(self, x, y, z):
        
        self.goal.pose.header.frame_id = "/jaco_api_origin"
        
        self.goal.pose.pose.orientation.x = -0.590686044496
        self.goal.pose.pose.orientation.y = -0.519369415388
        self.goal.pose.pose.orientation.z = 0.324703360925
        self.goal.pose.pose.orientation.w = 0.525274342226
        
        
        self.goal.pose.pose.position.x = x
        self.goal.pose.pose.position.y = y
        self.goal.pose.pose.position.z = z

        self.client.wait_for_server()
        rospy.loginfo("Connected to Pose server")
    
        self.client.send_goal(self.goal)
    
        return
    
    def __init__(self):

        # Initialize Node
        rospy.init_node('rbansal_srao_follow_traj')
        
        self.px_traj = []
        self.py_traj = []
        self.pz_traj = []
        
        self.client = actionlib.SimpleActionClient('/jaco/arm_pose', jaco_msgs.msg.ArmPoseAction)
        self.goal = jaco_msgs.msg.ArmPoseGoal()
        
        sub = rospy.Subscriber('/rbe_jacoapi/trajectories', trajectorymsg, self.traj_callback ,queue_size=1)
        

if __name__ == '__main__':
    try:
        node = follow_traj()
	rospy.spin()
    except rospy.ROSInterruptException: 
        rospy.loginfo("Program interrupted before completion")
        

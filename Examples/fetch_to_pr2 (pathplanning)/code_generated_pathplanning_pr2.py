#!/usr/bin/env python

# wave.py: "Wave" the fetch gripper
import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# Note: fetch_moveit_config move_group.launch must be running
# Safety!: Do NOT run this script near people or objects.
# Safety!: There is NO perception.
#          The ONLY objects the collision detection software is aware
#          of are itself & the floor.
if __name__ == '__main__':
    rospy.init_node("hi")

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("left_arm_and_torso", "base_link")

    # Define ground plane
    # This creates objects in the planning scene that mimic the ground
    # If these were not in place gripper could hit the ground
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground1")
    planning_scene.removeCollisionObject("my_front_ground2")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    planning_scene.addCube("my_front_ground1", 0.5, 0.8, 0.25, 0.25)
    planning_scene.addCube("my_front_ground2", 0.5, 0.8, 0.25, 1.20)

    # This is the wrist link not the gripper itself
    gripper_frame = "l_wrist_roll_link"
    # Position and rotation of two "wave end poses"
    gripper_poses = [Pose(Point(0.042, 0.584, 1.426),
                          Quaternion(0.173, -0.693, -0.242, 0.657)),
                     Pose(Point(0.547, 0.30, 0.72),
                          Quaternion(0.0, 0.0, 0.0, 1.0))]

    # Construct a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'

    while not rospy.is_shutdown():
        for pose in gripper_poses:
            # Finish building the Pose_stamped message
            # If the message stamp is not current it could be ignored
            gripper_pose_stamped.header.stamp = rospy.Time.now()
            # Set the message pose
            gripper_pose_stamped.pose = pose

            # Move gripper frame to the pose specified
            move_group.moveToPose(gripper_pose_stamped, gripper_frame)
            result = move_group.get_move_action().get_result()
            if result:
                # Checking the MoveItErrorCode
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Hello there!")
                else:
                    # If you get to this point please search for:
                    # moveit_msgs/MoveItErrorCodes.msg
                    rospy.logerr("Arm goal in state: %s",
                                 move_group.get_move_action().get_state())
            else:
                rospy.logerr("MoveIt! failure no result returned.")

    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    move_group.get_move_action().cancel_all_goals()

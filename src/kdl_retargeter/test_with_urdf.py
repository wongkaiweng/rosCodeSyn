# tests the retargeting solver functions.

import getpass
import unittest
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from PyKDL import *
from functions import *
import numpy as np

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

class TestMethods(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """ get_some_resource() is slow, to avoid calling it for each test use setUpClass()
            and store the result as class variable
        """
        super(TestMethods, cls).setUpClass()
        target_robot = URDF.from_xml_file('/home/{0}/ros_examples/configs/ur5.urdf'.format(getpass.getuser()))
        target_tree = kdl_tree_from_urdf_model(target_robot)
        base_link = 'base_link'
        end_link = 'ee_link'
        cls.target = target_tree.getChain(base_link, end_link)
        cls.angles = [0,0,0,0,0,0]
        cls.eps = forwardKinematics(cls.target,cls.angles)
        """
        cls.eps = np.array([[0.0, 0.0, 0.0],
             [0.0, 0.0, 0.089159],
             [0.0, 0.13585, 0.089159],
             [0.425, 0.016149999999999998, 0.08915900076283102],
             [0.81725, 0.016149999999999998, 0.08915900146687918],
             [0.81725, 0.10915, 0.08915900146687918],
             [0.8172500003397739, 0.10915, -0.005490998533120822],
             [0.8172500003397739, 0.19145, -0.005490998533120822]])
        """

    def test_cost_same_source_and_target(self):
        # eps = forwardKinematics(target,angles)
        self.assertEqual(cost_joints_ee([0,0,0,0,0,0], self.target, self.eps, self.target, 1), 0.0)

    def test_scaled_chain(self):
        self.assertEqual(calculate_rJoints_with_scaled_chain(np.array(self.eps), self.target, \
                                            np.array(self.eps), self.target), 0)

    def test_same_arm(self):
        source = self.target
        source_angles = [-2.4273787950371637, -0.8025726878730972, -1.0610762635509168, \
                        -3.286074344527749, -20.863498365468587, 0.0]
        target_initial_angles = [0,0,0,0,0,0]
        target_bounds = [(-360,360),(-360,360),(-360,360),(-360,360),(-360,360),(-360,360)]
        MODE = 'scale_by_length'
        EE_ratio = 0.0
        ret = retarget(source,source_angles,self.target,target_initial_angles,target_bounds,\
            EE_ratio=EE_ratio, mode=MODE)
        ret_angles = ret[0].tolist()
        ret_ep = forwardKinematics(self.target,ret_angles)
        eps = forwardKinematics(source,source_angles)
        rJoints = calculate_rJoints_with_scaled_chain(np.array(ret_ep), self.target, np.array(eps), source)
        print "rJoints: {0}".format(rJoints)
        self.assertEqual(rJoints<0.01, True)



if __name__ == "__main__":

    TEST_CASE = False

    if TEST_CASE:
        test_suite = unittest.TestLoader().loadTestsFromTestCase(TestMethods)
        unittest.TextTestRunner(verbosity=2).run(test_suite)
    else:
        ## JACO to UR5  ###

        # jaco #
        robot_jaco = URDF.from_xml_file('/home/{0}/ros_examples/configs/jaco_arm.urdf'.format(getpass.getuser()))
        jaco_tree = kdl_tree_from_urdf_model(robot_jaco)
        #print dir(source_tree) # no getSegment
        jaco_base_link = 'jaco_link_base'
        jaco_end_link = 'jaco_link_hand'
        jaco_chain = jaco_tree.getChain(jaco_base_link, jaco_end_link)
        print "Jaco Joints: {0}".format(jaco_chain.getNrOfJoints())
        print "Jaco Segments: {0}".format(jaco_chain.getNrOfSegments())

        jaco_angles = [0.0,2.0,1.3,2.2,2.0,1.0] # move_robot_jaco
        #jaco_angles = [2.2,2.3,1.57,0,0,0] # test_move_jaco\
        #jaco_angles = [1.5,1.7,1.57,0,0,0]



        # UR5 #
        robot_ur5 = URDF.from_xml_file('/home/{0}/ros_examples/configs/ur5.urdf'.format(getpass.getuser()))
        ur5_tree = kdl_tree_from_urdf_model(robot_ur5)
        ur5_base_link = 'base_link'
        ur5_end_link = 'ee_link'
        ur5_chain = ur5_tree.getChain(ur5_base_link, ur5_end_link)
        print "UR5 Joints: {0}".format(ur5_chain.getNrOfJoints())
        print "UR5 Segments: {0}".format(ur5_chain.getNrOfSegments())

        ur5_initial_angles = [0,0,0,0,0,0]
        ur5_bounds = [(-360,360),(-360,360),(-360,360),(-360,360),(-360,360),(-360,360)]


        #youbot
        robot_youbot = URDF.from_xml_file('/home/{0}/ros_examples/configs/youbot.urdf'.format(getpass.getuser()))
        youbot_tree = kdl_tree_from_urdf_model(robot_youbot)
        youbot_tree_base_link = 'arm_link_0'
        youbot_tree_end_link = 'gripper_palm_link'
        youbot_chain = youbot_tree.getChain(youbot_tree_base_link, youbot_tree_end_link)
        print "youbot_tree Joints: {0}".format(youbot_chain.getNrOfJoints())
        print "youbot_tree Segments: {0}".format(youbot_chain.getNrOfSegments())

        youbot_initial_angles = [0,0,0,0,0]
        youbot_bounds = [(-360,360),(-360,360),(-360,360),(-360,360),(-360,360)] ## need to change


        # UR5 to youbot
        source = ur5_chain
        source_angles = jaco_angles #ur5_initial_angles
        target = youbot_chain
        target_initial_angles = youbot_initial_angles
        target_bounds = youbot_bounds

        """
        # UR5 to UR5
        source = ur5_chain
        source_angles = jaco_angles
        target = ur5_chain
        target_initial_angles = ur5_initial_angles
        target_bounds = ur5_bounds
        """

        """
        # jaco to UR5
        source = jaco_chain
        source_angles = jaco_angles
        target = ur5_chain
        target_initial_angles = ur5_initial_angles
        target_bounds = ur5_bounds
        """

        # Test retarget:
        print "Now testing retarget:"

        # Plot the results:
        #plt.ion()
        #mode_list = ['original',"joints_only",'links','scale_by_length','scale_by_unit_length']
        mode_list = []
        MODE = 'scale_by_unit_length'
        fig = plt.figure()
        ax = p3.Axes3D(fig)
        draw(source, source_angles,ax,'b', label='Source Config')
        draw(target, target_initial_angles, ax, 'k', label='Target Init')
        color = ['g','c','r']

        eps = forwardKinematics(source,source_angles)
        if mode_list:
            for mode in mode_list:
                print "Mode: {0}".format(mode)
                for idx, EE_ratio in enumerate([0,0.5,1]):
                    ret = retarget(source,source_angles,target,target_initial_angles,target_bounds,\
                        EE_ratio=EE_ratio, mode=mode)
                    ret_angles = ret[0].tolist()
                    ret_ep = forwardKinematics(target,ret_angles)
                    print "EE_ratio: {0} Final Cost: {1}".format(EE_ratio, \
                        calculate_rJoints_with_scaled_chain(np.array(ret_ep), target, np.array(eps), source))
                    draw(target, ret_angles, ax, color[idx], label='Target Final EE_ratio:{0}'.format(EE_ratio))
        else:
            print "Mode: {0}".format(MODE)
            for idx, EE_ratio in enumerate([0,0.5,1]):
                ret = retarget(source,source_angles,target,target_initial_angles,target_bounds,\
                    EE_ratio=EE_ratio, mode=MODE)
                ret_angles = ret[0].tolist()
                ret_ep = forwardKinematics(target,ret_angles)
                print "EE_ratio: {0} Final Cost: {1}".format(EE_ratio, \
                    calculate_rJoints_with_scaled_chain(np.array(ret_ep), target, np.array(eps), source))
                draw(target, ret_angles, ax, color[idx], label='Target Final EE_ratio:{0}'.format(EE_ratio))


        print "Initial Endpoints:"
        orig_ep = forwardKinematics(target,target_initial_angles)
        print orig_ep
        print "Retargeted Angles:"
        print ret_angles
        ret_ep = forwardKinematics(target,ret_angles)
        print "Retargeted Endpoints: "
        print ret_ep

        plt.show()



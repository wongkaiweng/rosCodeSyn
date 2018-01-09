# tests the retargeting solver functions.

import getpass
import unittest
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from PyKDL import *
from kdl_retargeter.functions import *
import numpy as np
import copy
import subprocess

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

import process_limits

import logging
import logging_config
urdf_logger = logging.getLogger("urdf_logger")

class TestMethods(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """ get_some_resource() is slow, to avoid calling it for each test use setUpClass()
            and store the result as class variable
        """
        super(TestMethods, cls).setUpClass()
        target_robot = URDF.from_xml_file('/home/{0}/ros_examples/config_files/urdf/ur5.urdf'.format(getpass.getuser()))
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


def find_robot_URDF(robot_name, version='indigo'):
    '''
    Find the path to the robot URDF
    '''
    USE_PRECOMPILED_URDFS  =True

    # path to xacros
    XACRO_dict = {'pr2':'/opt/ros/{0}/share/pr2_description/robots/pr2.urdf.xacro'.format(version),\
                  'youbot':'/opt/ros/{0}/share/youbot_description/robots/youbot.urdf.xacro'.format(version),\
                  'jaco':'/opt/ros/{0}/share/jaco_description/urdf/jaco_arm.urdf.xacro'.format(version),\
                  'kinova':'/home/{0}/ros_ws/src/kinova-ros/kinova_description/urdf/j2s6s300_standalone.xacro'.format(getpass.getuser()),\
                  'ur3':'/opt/ros/{0}/share/ur_description/urdf/ur3_robot.urdf.xacro'.format(version),\
                  'ur5':'/opt/ros/{0}/share/ur_description/urdf/ur5_robot.urdf.xacro'.format(version),\
                  'nao':'/opt/ros/{0}/share/nao_description/urdf/nao_robot_v3.urdf.xacro'.format(version),\
                  'kobuki':'/opt/ros/{0}/share/kobuki_description/urdf/kobuki.urdf.xacro'.format(version)}

    path_to_config_folder = '/home/{0}/ros_examples/config_files/urdf/'.format(getpass.getuser())
    urdf_dict = {'youbot':'youbot.urdf',\
                    'jaco':'jaco_arm.urdf',\
                    'kinova':'j2n6s300.urdf',\
                    'ur5':'ur5.urdf',\
                    'nao':'nao_robot_v3.urdf',\
                    'pepper':'pepper.urdf'}

    # check if robot exists
    if USE_PRECOMPILED_URDFS and robot_name in urdf_dict.keys():
        with open(path_to_config_folder+urdf_dict[robot_name], 'r') as f:
            urdf_string = f.read()
        f.closed
        return urdf_string

    elif robot_name in XACRO_dict.keys():

        # convert xacro to urdf
        p = subprocess.Popen("rosrun xacro xacro {0}}".format(XACRO_dict[robot_name]), shell=True, \
            stdout=subprocess.PIPE)
        urdf, stderr = p.communicate()
        #print urdf #str
        return urdf_string

    return None

# ____                             ____       _           _
#/ ___|  ___  _   _ _ __ ___ ___  |  _ \ ___ | |__   ___ | |_
#\___ \ / _ \| | | | '__/ __/ _ \ | |_) / _ \| '_ \ / _ \| __|
# ___) | (_) | |_| | | | (_|  __/ |  _ < (_) | |_) | (_) | |_
#|____/ \___/ \__,_|_|  \___\___| |_| \_\___/|_.__/ \___/ \__|


def load_urdf_string(filename):
    with open(filename, 'r') as f:
        urdf_string = f.read()
    f.closed
    return urdf_string

def find_links_from_joints(target_joint, robot_urdf_obj, base_link="", end_link=""):
    """
    Find base_link and end_link based on target_joint and
    original base_link and original_end_link
    Output:
    base_link: string
    end_link: string
    """
    # first find first joint
    if not (base_link or end_link):
        for link, children_list in robot_urdf_obj.child_map.iteritems():
            for (joint, next_link) in children_list:
                if joint == target_joint:
                    urdf_logger.debug("Find first joint: {0}".format(joint))
                    return link, next_link\

        urdf_logger.warning("No base_link or end_link found for joint: {0}".format(target_joint))
        return base_link, end_link

    else: # base_link and end_link is not empty
        new_base_link = recursive_map(target_joint, robot_urdf_obj.parent_map, base_link, map_type='parent')
        new_end_link = recursive_map(target_joint, robot_urdf_obj.child_map, end_link, map_type='child')

        if new_base_link: return new_base_link, end_link
        if new_end_link:  return base_link, new_end_link
        if not (new_base_link or new_end_link):
            urdf_logger.warning("Joint {0} is not found. Returning the same base_link and end_link".format(target_joint))
            return base_link, end_link


def recursive_map(target_joint, link_joint_map, start_link, map_type='child'):
    """
    This function recursively to find the link related to
    target joint starting with link_joint_map[start_link]
    Output:
    new_link: string or None
    """
    link = None
    if map_type == 'child':
        if start_link in link_joint_map.keys():
            for (joint, next_link) in link_joint_map[start_link]:
                if joint == target_joint:
                    return next_link
                else:
                    link = recursive_map(target_joint, link_joint_map, next_link, map_type)
        return link

    elif map_type == 'parent':
        if start_link in link_joint_map.keys():
            joint, next_link =  link_joint_map[start_link]
            if joint == target_joint:
                return next_link
            else:
                link = recursive_map(target_joint, link_joint_map, next_link, map_type)
        return link
    else:
        urdf_logger.warning('Map type is not "child" or "parent": {0}'.format(map_type))


def load_chain_from_URDF_string_and_joints(urdf_string, joint_list):
    """
    This function returns a kinmatic chain based on the URDF string given
    and the list of joints
    """

    robot = URDF.from_xml_string(urdf_string)

    # check if joint is in it if not then expand both in child or parent
    base_link, end_link = "", ""
    for joint in joint_list:
        base_link, end_link = find_links_from_joints(joint, robot, base_link, end_link)
        urdf_logger.debug("Current base_link:{0}, Current end_link:{1}".format(base_link, end_link))

    # return chain
    if base_link and end_link:
        tree = kdl_tree_from_urdf_model(robot)
        return tree.getChain(base_link, end_link)
    else:
        return None

# _____                    _     ____       _           _
#|_   _|_ _ _ __ __ _  ___| |_  |  _ \ ___ | |__   ___ | |_
#  | |/ _` | '__/ _` |/ _ \ __| | |_) / _ \| '_ \ / _ \| __|
#  | | (_| | | | (_| |  __/ |_  |  _ < (_) | |_) | (_) | |_
#  |_|\__,_|_|  \__, |\___|\__| |_| \_\___/|_.__/ \___/ \__|
#               |___/

def load_chain_from_URDF(filename):
    """
    This function loads the URDF file and return
    the longest arm chain, the bounds and the assumed init angle
    """
    with open(filename, 'r') as f:
        xml_string = f.read()
    f.closed

    # read URDF
    longest_chain, robot_bounds, robot_initial_angles = load_chain_from_URDF_string(xml_string)

    return longest_chain, robot_bounds, robot_initial_angles


def load_chain_from_URDF_string(urdf_string):
    """
    This function loads the URDF string and return
    the longest arm chain, the bounds and the assumed init angle
    """
    robot = URDF.from_xml_string(urdf_string)
    tree = kdl_tree_from_urdf_model(robot)

    base_link, end_link = "",""
    link_list = []

    # find base link and all other links
    for link in [link.name for link in robot.links]: #old:robot.link_map.keys():
        if all([x in link for x in ['base', 'link']]):
            #base_link = link
            link_list.append(link)
        elif 'link' in link and not any([x in link for x in ['finger','wheel', 'caster']]): #['finger', 'gripper','wheel', 'caster']])
            link_list.append(link)

    # get joint limits
    joint_limit_dict = process_limits.make_joint_limit_dic_from_robObj(robot)

    #print robot.joint_map.values()[0]
    # use root if we cannot find base link
    #if not base_link:
    #    base_link = robot.get_root()

    # Find the longest chain
    longest_chain_list = []
    base_link_list = []
    end_link_list  = []
    longest_chain_idx = 0
    longest_chain_nOfsegments = 0
    # Find all possible chains by iterating the link list
    for potential_base_link in link_list:
        longest_chain, end_link = find_longest_chain(potential_base_link, link_list, tree, joint_limit_dict)

        #urdf_logger.log(4,"base_link: {0}, end_link: {1}".format(potential_base_link, end_link))
        if longest_chain:
            longest_chain_list.append(longest_chain)
            base_link_list.append(potential_base_link)
            end_link_list.append(end_link)

            # track the longest segment
            if longest_chain_nOfsegments < longest_chain.getNrOfSegments():
                longest_chain_nOfsegments = longest_chain.getNrOfSegments()
                longest_chain_idx = len(longest_chain_list) - 1

    # save longest chain
    urdf_logger.debug('longest_chain from all possible chains: {0}'.format(longest_chain_nOfsegments))
    longest_chain = longest_chain_list[longest_chain_idx]
    base_link = base_link_list[longest_chain_idx]
    end_link = end_link_list[longest_chain_idx]

    urdf_logger.info("base_link:{0}, end_link:{1}, chain length:{2}".format(\
        base_link, end_link, longest_chain.getNrOfSegments()))


    #robot_bounds = [(-360,360), ...]
    robot_bounds = []
    robot_initial_angles = []

    # find bounds and init angles
    for idx in range(0,longest_chain.getNrOfSegments()):
        joint_name = longest_chain.getSegment(idx).getJoint().getName().encode('ascii','ignore')
        robot_bounds.append((joint_limit_dict[joint_name]['lower'], joint_limit_dict[joint_name]['upper']))
        init_angle = (joint_limit_dict[joint_name]['lower'] + joint_limit_dict[joint_name]['upper'])/2.0
        robot_initial_angles.append(init_angle)

    urdf_logger.debug('robot_bounds: {0}'.format(robot_bounds))
    urdf_logger.debug('robot_initial_angles: {0}'.format(robot_initial_angles))

    return longest_chain, robot_bounds, robot_initial_angles

def find_longest_chain(base_link, link_list, tree, joint_limit_dict):
    """
    Find the longest chain
    base_link: name of base link (string)
    link_list: list of possible links
    tree: kinematic tree of the robot
    joint_limit_dict: dictionary storing joints that have limits
    """
    longest_chain, end_link = None, None
    for link in link_list:
        chain = tree.getChain(base_link, link)

        valid_chain = True

        #check if all joints are valid
        for idx in range(0,chain.getNrOfSegments()):
            joint_name = chain.getSegment(idx).getJoint().getName().encode('ascii','ignore')
            urdf_logger.log(6, joint_name)
            if not joint_name in joint_limit_dict.keys():
                valid_chain = False
                urdf_logger.log(8, '-------> Not a valid chain <-------')
                urdf_logger.log(8, "Length: {0}, #ofJoints: {1}, Base link: {2} to {3}".format(\
                    chain.getNrOfSegments(), chain.getNrOfJoints(), base_link, link))
                urdf_logger.log(8, "Joint {0} not in joint_limit_dict".format(joint_name))
                break

        if valid_chain and (not longest_chain or \
            longest_chain.getNrOfSegments() < chain.getNrOfSegments()):
            longest_chain = chain
            end_link = link

    return longest_chain, end_link

def get_joints_from_chain(chain):
    #get all joints from chain
    joint_list = []
    for idx in range(0,chain.getNrOfSegments()):
        joint_list.append(chain.getSegment(idx).getJoint().getName().encode('ascii','ignore'))

    return joint_list

# ____      _                       _
#|  _ \ ___| |_ __ _ _ __ __ _  ___| |_
#| |_) / _ \ __/ _` | '__/ _` |/ _ \ __|
#|  _ <  __/ || (_| | | | (_| |  __/ |_
#|_| \_\___|\__\__,_|_|  \__, |\___|\__|
#                        |___/

def find_best_retarget(source,source_angles,target,target_initial_angles,target_bounds, mode='scale_by_unit_length'):
    """
    Find best retarget result by changing EE ratio: The bias to full arm versus just end
    effector
    Inputs:
    mode: 'original',"joints_only",'links','scale_by_length','scale_by_unit_length'
    """
    eps = forwardKinematics(source,source_angles)
    urdf_logger.info( "Retargeting Mode: {0}".format(mode))

    # track best case
    best_ret_angles = None
    best_cost = None
    best_EE_ratio = None
    all_ret_angles = []
    for idx, EE_ratio in enumerate([0,0.5,1]):
        ret = retarget(source,source_angles,target,target_initial_angles,target_bounds,\
            EE_ratio=EE_ratio, mode=mode)
        ret_angles = ret[0].tolist()
        ret_ep = forwardKinematics(target,ret_angles)
        cost = calculate_rJoints_with_scaled_chain(np.array(ret_ep), target, np.array(eps), source)
        all_ret_angles.append(ret_angles)
        urdf_logger.debug( "EE_ratio: {0} Cost: {1}".format(EE_ratio, cost))
        if not best_cost or best_cost > cost:
            best_cost = cost
            best_ret_angles = ret_angles
            best_EE_ratio = EE_ratio

    urdf_logger.info( "Best EE_ratio: {0} Best Cost: {1}".format(best_EE_ratio, best_cost))

    #urdf_logger.info("Initial Endpoints:")
    #urdf_logger.info(forwardKinematics(target,target_initial_angles))
    urdf_logger.info("Best Retargeted Angles: {0}".format(best_ret_angles))
    #urdf_logger.info("Retargeted Endpoints:")
    #urdf_logger.info(forwardKinematics(target,best_ret_angles))

    return best_ret_angles, best_EE_ratio, all_ret_angles

def plot_retarget(source,source_angles,target,target_initial_angles,target_bounds,all_ret_angles=[], best_EE_ratio=None):
    """
    This function plots the results of find_best_retarget function
    """
    # plot results
    fig = plt.figure()
    ax = p3.Axes3D(fig)
    ax.set_aspect('equal')
    draw(source, source_angles, ax,'b', label='Source Config')
    draw(target, target_initial_angles, ax, 'k', label='Target Init')
    color = ['g','c','r','y','o-']
    EE_ratio = [0,0.5,1]

    if not all_ret_angles:
        # find best ret angles
        best_ret_angles, best_EE_ratio, all_ret_angles = \
            find_best_retarget(source,source_angles,target,target_initial_angles,target_bounds)

    for idx, ret_angles in enumerate(all_ret_angles):
        draw(target, ret_angles, ax, color[idx], label='Target Final EE_ratio:{0}'.format(EE_ratio[idx]))

    def set_axes_equal(ax):
        '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
        cubes as cubes, etc..  This is one possible solution to Matplotlib's
        ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

        Input
          ax: a matplotlib axis, e.g., as output from plt.gca().

        From: https://stackoverflow.com/questions/13685386/matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to
        '''

        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()

        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)
        z_range = abs(z_limits[1] - z_limits[0])
        z_middle = np.mean(z_limits)

        # The plot bounding box is a sphere in the sense of the infinity
        # norm, hence I call half the max range the plot radius.
        plot_radius = 0.5*max([x_range, y_range, z_range])

        ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

    plt.title('EE_ratio: {0} for config: {1}'.format(best_EE_ratio, source_angles))
    set_axes_equal(ax)
    plt.show()

"""
def plot_retarget_with_ret_angles_list(source,source_angles,target,target_initial_angles, all_ret_angles):
    #This function takes in all_ret_angles list and plot the results
    # plot results
    fig = plt.figure()
    ax = p3.Axes3D(fig)
    ax.set_aspect('equal')
    draw(source, source_angles, ax,'b', label='Source Config')
    draw(target, target_initial_angles, ax, 'k', label='Target Init')
    color = ['g','c','r']
    EE_ratio = [0,0.5,1]
    for idx, ret_angles in enumerate(all_ret_angles):
        draw(target, ret_angles, ax, color[idx], label='Target Final EE_ratio:{0}'.format(EE_ratio[idx]))


    plt.show()
"""
def get_source_info(source_robot_name, source_joint_list):
    #####################
    # load source robot #
    #####################
    source_urdf_string = find_robot_URDF(source_robot_name)
    if source_urdf_string:
        source_chain = load_chain_from_URDF_string_and_joints(source_urdf_string, source_joint_list)
        return source_chain
    else:
        urdf_logger.warning("We cannot get a source URDF!")
        return None


def get_target_info(target_robot_name):
    #####################
    # load target robot #
    #####################
    # first get urdf_string
    target_urdf_string = find_robot_URDF(target_robot_name)
    if target_urdf_string:
        target_chain, target_bounds, target_initial_angles = load_chain_from_URDF_string(target_urdf_string)
        return target_chain, target_bounds, target_initial_angles
    else:
        urdf_logger.warning("We cannot get a target URDF!")
        return None, None, None

def kinamatic_retargeting(source_chain,source_angles,target_chain,target_initial_angles,target_bounds, mode='scale_by_unit_length'):
    ########################
    # kinmatic retargeting #
    ########################
    best_ret_angles, best_EE_ratio, all_ret_angles = \
        find_best_retarget(source_chain,source_angles,target_chain,target_initial_angles,target_bounds, mode=mode)
    plot_retarget(source_chain,source_angles,target_chain,target_initial_angles, target_bounds, all_ret_angles, best_EE_ratio)

    return best_ret_angles, best_EE_ratio, all_ret_angles


def find_ret_angles_from_source_joints(source_robot_name, target_robot_name, source_joint_list, source_angles, mode='scale_by_unit_length'):
    # get source and target
    source_chain = get_source_info(source_robot_name, source_joint_list)
    target_chain, target_bounds, target_initial_angles = get_target_info(target_robot_name)

    if not (source_chain and target_chain): return None, None

    # get all the joints of the target
    target_joints =  get_joints_from_chain(target_chain)
    urdf_logger.debug('target_joints: {0}'.format(target_joints))

    # kinematic retargeting
    best_ret_angles, best_EE_ratio, all_ret_angles = kinamatic_retargeting(source_chain,source_angles,target_chain,\
        target_initial_angles,target_bounds, mode=mode)

    return target_joints, best_ret_angles

"""
def plot_retarget(source,source_angles,target,target_initial_angles,target_bounds):
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
            urdf_logger.info("Mode: {0}".format(mode))
            for idx, EE_ratio in enumerate([0,0.5,1]):
                ret = retarget(source,source_angles,target,target_initial_angles,target_bounds,\
                    EE_ratio=EE_ratio, mode=mode)
                ret_angles = ret[0].tolist()
                ret_ep = forwardKinematics(target,ret_angles)
                urdf_logger.info( "EE_ratio: {0} Final Cost: {1}".format(EE_ratio, \
                    calculate_rJoints_with_scaled_chain(np.array(ret_ep), target, np.array(eps), source)))
                draw(target, ret_angles, ax, color[idx], label='Target Final EE_ratio:{0}'.format(EE_ratio))
    else:
        urdf_logger.info( "Mode: {0}".format(MODE))
        for idx, EE_ratio in enumerate([0,0.5,1]):
            ret = retarget(source,source_angles,target,target_initial_angles,target_bounds,\
                EE_ratio=EE_ratio, mode=MODE)
            ret_angles = ret[0].tolist()
            ret_ep = forwardKinematics(target,ret_angles)
            urdf_logger.info( "EE_ratio: {0} Final Cost: {1}".format(EE_ratio, \
                calculate_rJoints_with_scaled_chain(np.array(ret_ep), target, np.array(eps), source)))
            draw(target, ret_angles, ax, color[idx], label='Target Final EE_ratio:{0}'.format(EE_ratio))


    urdf_logger.info( "Initial Endpoints:")
    orig_ep = forwardKinematics(target,target_initial_angles)
    urdf_logger.info(orig_ep)
    urdf_logger.info("Retargeted Angles:")
    urdf_logger.info(ret_angles)
    ret_ep = forwardKinematics(target,ret_angles)
    urdf_logger.info("Retargeted Endpoints: ")
    urdf_logger.info(ret_ep)
"""


if __name__ == "__main__":

    TEST_CASE = False
    LINK_DEBUG = True

    if TEST_CASE:
        test_suite = unittest.TestLoader().loadTestsFromTestCase(TestMethods)
        unittest.TextTestRunner(verbosity=2).run(test_suite)
    elif LINK_DEBUG:

        source_robot_name = 'ur5'
        target_robot_name = 'kinova'
        source_joint_list = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', \
                             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        source_angles = [2.2,0,-1.57,0,0,0]
        target_joints, best_ret_angles = \
           find_ret_angles_from_source_joints(source_robot_name, target_robot_name, source_joint_list, source_angles)

        print target_joints, best_ret_angles
    else:
        UR5_testing = False
        JacoTesting = False
        youBotTesting = False
        mixTesting = True
        #  _   _ ____  ____  _        _   _ ____  ____         ____  ___   ___  ____
        # | | | |  _ \| ___|| |_ ___ | | | |  _ \| ___|       / ___|/ _ \ / _ \|  _ \
        # | | | | |_) |___ \| __/ _ \| | | | |_) |___ \ _____| |  _| | | | | | | | | |
        # | |_| |  _ < ___) | || (_) | |_| |  _ < ___) |_____| |_| | |_| | |_| | |_| |
        #  \___/|_| \_\____/ \__\___/ \___/|_| \_\____/       \____|\___/ \___/|____/

        urdf_logger.info('----- ur5 (from fn.)------')
        ur5_chain, ur5_bounds, ur5_initial_angles = load_chain_from_URDF('/home/{0}/ros_examples/config_files/urdf/ur5.urdf'.format(getpass.getuser()))
        urdf_logger.debug( "Forward Init: {0}".format(forwardKinematics(ur5_chain,ur5_initial_angles)))
        #print "Forward Final: {0}".format(forwardKinematics(ur5_chain,target_initial_angles))
        urdf_logger.info("UR5 Joints: {0}".format(ur5_chain.getNrOfJoints()))
        urdf_logger.info("UR5 Segments: {0}".format(ur5_chain.getNrOfSegments()))

        urdf_logger.info('----- ur5 (original)------')
        robot_ur5 = URDF.from_xml_file('/home/{0}/ros_examples/config_files/urdf/ur5.urdf'.format(getpass.getuser()))
        ur5_tree = kdl_tree_from_urdf_model(robot_ur5)
        ur5_base_link = 'base_link'
        ur5_end_link = 'ee_link'
        ur5_chain_orig = ur5_tree.getChain(ur5_base_link, ur5_end_link)
        urdf_logger.info("base_link: {0}, end_link: {1}".format(ur5_base_link, ur5_end_link))
        urdf_logger.info("UR5 Joints: {0}".format(ur5_chain_orig.getNrOfJoints()))
        urdf_logger.info("UR5 Segments: {0}".format(ur5_chain_orig.getNrOfSegments()))
        ur5_initial_angles_orig = [0,0,0,0,0,0]
        ur5_bounds_orig = [(-360,360),(-360,360),(-360,360),(-360,360),(-360,360),(-360,360)]

        if UR5_testing:
            # test ur fn to ur original
            urdf_logger.info("--Testing ur fn to ur original --")
            source = ur5_chain
            source_angles = [-0.27392274831050006, -0.8441796471848748, -0.6117867346251068, 2.1929307352068297, 3.267835461946445, 0.0]
            target = ur5_chain_orig
            target_initial_angles = ur5_initial_angles_orig
            target_bounds = ur5_bounds_orig
            plot_retarget(source,source_angles,target,target_initial_angles,target_bounds)

            # test ur original to ur fn
            urdf_logger.info("--Testing ur original to ur fn --")
            source = ur5_chain_orig
            source_angles = [-0.27392274831050006, -0.8441796471848748, -0.6117867346251068, 2.1929307352068297, 3.267835461946445, 0.0]
            target = ur5_chain
            target_initial_angles = ur5_initial_angles
            target_bounds = ur5_bounds
            plot_retarget(source,source_angles,target,target_initial_angles,target_bounds)

        ##################################################
        #      _               _____         _                        ____  ___   ___   ____
        #     | | __ _  ___ __|_   _|__     | | __ _  ___ ___        / ___|/ _ \ / _ \ |  _ \
        #  _  | |/ _` |/ __/ _ \| |/ _ \ _  | |/ _` |/ __/ _ \ _____| |  _| | | | | | || | | |
        # | |_| | (_| | (_| (_) | | (_) | |_| | (_| | (_| (_) |_____| |_| | |_| | |_| || |_| |
        #  \___/ \__,_|\___\___/|_|\___/ \___/ \__,_|\___\___/       \____|\___/ \___/ |____/

        urdf_logger.info('----- jaco (original)------')
        robot_jaco = URDF.from_xml_file('/home/{0}/ros_examples/config_files/urdf/jaco_arm.urdf'.format(getpass.getuser()))
        jaco_tree = kdl_tree_from_urdf_model(robot_jaco)
        jaco_base_link = 'jaco_link_base'
        jaco_end_link = 'jaco_link_hand'
        jaco_chain_orig = jaco_tree.getChain(jaco_base_link, jaco_end_link)
        jaco_initial_angles_orig = [0,0,0,0,0,0]
        jaco_bounds_orig = [(-360,360),(-360,360),(-360,360),(-360,360),(-360,360),(-360,360)]
        urdf_logger.info("base_link: {0}, end_link: {1}".format(jaco_base_link, jaco_end_link))
        urdf_logger.info("Jaco Joints: {0}".format(jaco_chain_orig.getNrOfJoints()))
        urdf_logger.info("Jaco Segments: {0}".format(jaco_chain_orig.getNrOfSegments()))

        urdf_logger.info('----- jaco (from fn.)------')
        jaco_chain, jaco_bounds, jaco_init_angles = load_chain_from_URDF('/home/{0}/ros_examples/config_files/urdf/jaco_arm.urdf'.format(getpass.getuser()))
        urdf_logger.debug("Forward Init: {0}".format(forwardKinematics(jaco_chain,jaco_init_angles)))
        #print "Forward Final: {0}".format(forwardKinematics(ur5_chain,target_initial_angles))
        urdf_logger.info("Jaco Joints: {0}".format(jaco_chain.getNrOfJoints()))
        urdf_logger.info("Jaco Segments: {0}".format(jaco_chain.getNrOfSegments()))

        if JacoTesting:
            # test jaco fn to jaco original
            urdf_logger.info("--Testing jaco fn to jaco original --")
            source = jaco_chain
            source_angles = [0.0,2.0,1.3,2.2,2.0,1.0]
            target = jaco_chain_orig
            target_initial_angles = jaco_initial_angles_orig
            target_bounds = jaco_bounds_orig
            plot_retarget(source,source_angles,target,target_initial_angles,target_bounds)

            # test jaco fn to jaco original
            urdf_logger.info("--Testing jaco original to jaco fn --")
            source = jaco_chain_orig
            source_angles = [0.0,2.0,1.3,2.2,2.0,1.0]
            target = jaco_chain
            target_initial_angles = jaco_init_angles
            target_bounds = jaco_bounds
            plot_retarget(source,source_angles,target,target_initial_angles,target_bounds)

        ###############################################
        #                   _           _   ____                    ____        _           ____  ___   ___   ____
        #  _   _  ___  _   _| |__   ___ | |_|___ \ _   _  ___  _   _| __ )  ___ | |_       / ___|/ _ \ / _ \ |  _ \
        # | | | |/ _ \| | | | '_ \ / _ \| __| __) | | | |/ _ \| | | |  _ \ / _ \| __|_____| |  _| | | | | | || | | |
        # | |_| | (_) | |_| | |_) | (_) | |_ / __/| |_| | (_) | |_| | |_) | (_) | |_ _____| |_| | |_| | |_| || |_| |
        #  \__, |\___/ \__,_|_.__/ \___/ \__|_____|\__, |\___/ \__,_|____/ \___/ \__|      \____|\___/ \___/ |____/
        #  |___/                                   |___/

        urdf_logger.info('----- youbot (original)------')
        robot_youbot = URDF.from_xml_file('/home/{0}/ros_examples/config_files/urdf/youbot.urdf'.format(getpass.getuser()))
        youbot_tree = kdl_tree_from_urdf_model(robot_youbot)
        youbot_tree_base_link = 'arm_link_0'
        youbot_tree_end_link = 'gripper_palm_link'
        youbot_chain_orig = youbot_tree.getChain(youbot_tree_base_link, youbot_tree_end_link)
        urdf_logger.info("base_link: {0}, end_link: {1}".format(youbot_tree_base_link, youbot_tree_end_link))
        urdf_logger.info("youbot_cahin Joints: {0}".format(youbot_chain_orig.getNrOfJoints()))
        urdf_logger.info("youbot_cahin Segments: {0}".format(youbot_chain_orig.getNrOfSegments()))
        youbot_initial_angles_orig = [0,0,0,0,0]
        youbot_bounds_orig = [(-360,360),(-360,360),(-360,360),(-360,360),(-360,360)] ## need to change

        urdf_logger.info('----- youbot (from fn.)------')
        youbot_chain, youbot_bounds, youbot_initial_angles = load_chain_from_URDF('/home/{0}/ros_examples/config_files/urdf/youbot.urdf'.format(getpass.getuser()))
        urdf_logger.debug("Forward Init: {0}".format(forwardKinematics(youbot_chain,youbot_initial_angles)))
        urdf_logger.info("youbot_cahin Joints: {0}".format(youbot_chain.getNrOfJoints()))
        urdf_logger.info("youbot_cahin Segments: {0}".format(youbot_chain.getNrOfSegments()))

        if youBotTesting:
            # test youbot fn to youbot original
            urdf_logger.info("--Testing youBot fn to youBot original --")
            source = youbot_chain
            source_angles = youbot_initial_angles
            target = youbot_chain_orig
            target_initial_angles = youbot_initial_angles_orig
            target_bounds = youbot_bounds_orig
            plot_retarget(source,source_angles,target,target_initial_angles,target_bounds)


            # test youBot original to youbot fn
            urdf_logger.info("--Testing youBot original to youbot fn --")
            source = youbot_chain_orig
            source_angles = youbot_initial_angles_orig
            target = youbot_chain
            target_initial_angles = youbot_initial_angles
            target_bounds = youbot_bounds
            plot_retarget(source,source_angles,target,target_initial_angles,target_bounds)

        if mixTesting:
            jaco_angles = [0.0,2.0,1.3,2.2,2.0,1.0]
            # jaco to UR5
            source = jaco_chain
            source_angles = jaco_angles
            target = ur5_chain
            target_initial_angles = ur5_initial_angles
            target_bounds = ur5_bounds
            plot_retarget(source,source_angles,target,target_initial_angles,target_bounds)


            # UR5 to youbot
            source = ur5_chain
            source_angles = jaco_angles #ur5_initial_angles
            target = youbot_chain
            target_initial_angles = youbot_initial_angles
            target_bounds = youbot_bounds
            plot_retarget(source,source_angles,target,target_initial_angles,target_bounds)

        #  _            _   _
        # | |_ ___  ___| |_(_)_ __   __ _
        # | __/ _ \/ __| __| | '_ \ / _` |
        # | ||  __/\__ \ |_| | | | | (_| |
        #  \__\___||___/\__|_|_| |_|\__, |
        #                           |___/


        #                        _
        #   __ _  ___   ___   __| |
        #  / _` |/ _ \ / _ \ / _` |
        # | (_| | (_) | (_) | (_| |
        #  \__, |\___/ \___/ \__,_|
        #  |___/

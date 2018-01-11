import getpass
import logging
import difflib
import os
import xml.etree.ElementTree as ET

import logging_config
srdf_logger = logging.getLogger("srdf_logger")

# True: automatically select the best match move group, the best tip_frame
# False: Prompt user with the sorted move group list, tip_frame
AUTO_MODE = False

def find_robot_SRDF(robot_name, version='indigo'):
    '''
    Find the path to the robot SRDF
    '''
    USE_LOCAL_SRDFS  =True

    # path to xacros
    srdf_opt_dict = {'pr2':'/opt/ros/{0}/share/pr2_moveit_config/config/pr2.srdf'.format(version),
                     'fetch':'/opt/ros/{0}/share/fetch_moveit_config/config/fetch.srdf'.format(version),
                     'jaco':'/opt/ros/{0}/share/jaco_moveit_config/config/jaco.srdf'.format(version),
                     'nao':'/opt/ros/{0}/share/nao_moveit_config/config/NaoH25V40.srdf'.format(version),
                     'pepper':'/opt/ros/{0}/share/pepper_moveit_config/config/JulietteY20MP.srdf'.format(version),
                     'kinova':'/home/{0}/ros_ws/src/kinova-ros/kinova_moveit/robot_configs/j2n6s300_moveit_config/config/j2n6s300.srdf'.format(getpass.getuser())}

    path_to_config_folder = os.path.dirname(os.path.abspath(__file__)) +'/../config_files/srdf/'
    srdf_local_dict = {'pr2':'pr2.srdf',
                       'fetch':'fetch.srdf',
                       'jaco':'jaco.srdf',
                       'nao':'nao.srdf',
                       'pepper':'pepper.srdf',
                       'kinova':'j2n6s300.srdf'}

    # check if robot exists
    if USE_LOCAL_SRDFS and robot_name in srdf_local_dict.keys():
        with open(path_to_config_folder+srdf_local_dict[robot_name], 'r') as f:
            srdf_string = f.read()
        f.closed
        return srdf_string

    elif robot_name in srdf_opt_dict.keys():

        with open(srdf_opt_dict[robot_name], 'r') as f:
            srdf_string = f.read()
        f.closed
        return srdf_string

    return None

def load_srdf_to_dict(srdf_string):
    # parse xml into xml tree
    root = ET.fromstring(srdf_string)

    srdf_dict = {}

    # get all moveit group
    for group in root.findall('group'):
        srdf_dict[group.attrib['name']] = {}

        for child in group:
            # chain from urdf
            if child.tag == 'chain':
                #{'base_link':'base_link_name' ,'tip_link': 'tip_link_name']}
                #srdf_logger.debug(child.attrib)
                srdf_dict[group.attrib['name']]['chain'] = child.attrib

            # joints in urdf
            elif child.tag == 'joint':
                if not 'joint' in srdf_dict[group.attrib['name']].keys():
                    srdf_dict[group.attrib['name']]['joint'] = []
                srdf_dict[group.attrib['name']]['joint'].append(child.attrib['name'])

            # moveit groups
            elif child.tag == 'group':
                if not 'group' in srdf_dict[group.attrib['name']].keys():
                    srdf_dict[group.attrib['name']]['group'] = []
                srdf_dict[group.attrib['name']]['group'].append(child.attrib['name'])

            # urdf links
            elif child.tag == 'link':
                if not 'link' in srdf_dict[group.attrib['name']].keys():
                    srdf_dict[group.attrib['name']]['link'] = []
                srdf_dict[group.attrib['name']]['link'].append(child.attrib['name'])

            else:
                srdf_logger.warning("Unknown type in srdf: {0}".format(child.tag))

    return srdf_dict


def find_new_group_name_in_srdf_dict(srdf_dict, orig_group_name):
    """
    This function finds a new group name based on an original_group_name
    and the target srdf_dict
    srdf_dict: dict of srdf info from target robot
    original_group_name: original group name (str)
    """

    group_name_list = srdf_dict.keys()
    sorted_closest_matches_list = difflib.get_close_matches(orig_group_name, group_name_list, len(group_name_list), 0)
    if AUTO_MODE or len(sorted_closest_matches_list) == 1:
        # return the best match
        return sorted_closest_matches_list[0]
    else:
        # prompt user to select
        group_name_idx = raw_input("Please select a group from the list. " +\
                                   "The original group was '{0}'. ".format(orig_group_name)+\
                                   "The list is sorted by the best match. "+\
                                   "Type the NUMBER (e.g.:1):\n{0}\n".format(\
                                   "\n".join([str(idx+1)+'. '+x for idx,x in enumerate(sorted_closest_matches_list)])))
        return sorted_closest_matches_list[int(group_name_idx)-1]


    srdf_logger.debug(closest_matches)

def get_tip_frame_list_from_group_name(srdf_dict, group_name):
    """
    This function gets the tip frame based on the
    group_name and srdf_dict given.
    srdf_dict: srdf info from target robot (dict)
    group_name: group name (str)
    orig_frame: name of the original frame
    """
    tip_frame_list = []

    group_struct = srdf_dict[group_name].keys()

    #srdf_logger.debug("srdf_dict[{0}]:{1}".format(group_name, srdf_dict[group_name]))
    if 'chain' in group_struct:
        return [srdf_dict[group_name]['chain']['tip_link']]
    elif 'group' in group_struct:
        for sub_group in srdf_dict[group_name]['group']:
            tip_frame_list.extend(get_tip_frame_list_from_group_name(srdf_dict, sub_group))
    elif 'joint' in group_struct:
        return [x.replace('joint','link') for x in srdf_dict[group_name]['joint']]
    elif 'link' in group_struct:
        return srdf_dict[group_name]['link']
    else:
        srdf_logger.warning("Not Implemented processing of field: {0} in srdf_dict.".format(group_struct))
        return []

    return tip_frame_list

def select_tip_frame_from_list(tip_frame_list, orig_frame):
    """
    This function selects a tip_frame from the list
    tip_frame_list: list of tip_frame
    orig_frame: name of the original frame (str)
    """

    sorted_matches_list = difflib.get_close_matches(orig_frame, tip_frame_list, len(tip_frame_list), 0)
    if AUTO_MODE or len(sorted_matches_list) == 1:
        # return the best match
        return sorted_matches_list[0]
    else:
        # prompt user to select
        tip_name_idx = raw_input("Please select a new tip frame from the list. " +\
                                   "The original frame was '{0}'. ".format(orig_frame)+\
                                   "The list is sorted by the best match. "+\
                                   "Type the NUMBER (e.g.:1):\n{0}\n".format(\
                                   "\n".join([str(idx+1)+'. '+x for idx,x in enumerate(sorted_matches_list)])))
        return sorted_matches_list[int(tip_name_idx)-1]





if __name__ == "__main__":
    target_srdf_string  = find_robot_SRDF('pr2', version='indigo')
    target_srdf_dict = load_srdf_to_dict(target_srdf_string)
    #srdf_logger.debug('target_srdf_dict:{0}'.format(target_srdf_dict))

    # find target group name
    target_group_name = find_new_group_name_in_srdf_dict(target_srdf_dict, "arm_with_torso")
    srdf_logger.debug('target_group_name:{0}'.format(target_group_name))

    # find target frame
    tip_frame_list =  get_tip_frame_list_from_group_name(target_srdf_dict, target_group_name)
    target_frame =  select_tip_frame_from_list(tip_frame_list, 'l_wrist_roll_link')
    srdf_logger.debug('target_frame:{0}'.format(target_frame))


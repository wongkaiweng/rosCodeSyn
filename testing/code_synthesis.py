import sys
import getpass
import ast
import logging

import prob_from_files
import parameters_in_file
import replacement_with_redbaron
import process_urdf

import logging_config
synthesis_logger = logging.getLogger("synthesis_logger")

def topic_replacement_by_distribution(file_path, red_obj):
    """
    This function takes in the folder with python codes
    and the redbaron_obj to modifiy and replace the topics of
    the original robot with the new robot.
    Inputs:
    file_path: directory of the code library with robot codes/examples
    red_obj: redbaron code object to replace
    Output:
    red_obj: modified red_obj
    """

    # channel_type_to_call_name_dict[channel_type] = call_name
    channel_type_to_call_name_dict = {"Publisher": 'rospy.Publisher',\
                                      "Subscriber": 'rospy.Subscriber',\
                                      "SimpleActionClient": 'actionlib.SimpleActionClient'}

    # replace all different types of channels
    for channel_type, call_name in channel_type_to_call_name_dict.iteritems():

        # changed approach.py in turtlebot
        target_topic_dict = prob_from_files.get_best_topic_match_for_all_msg_types(file_path, call_name=call_name)

        synthesis_logger.debug('For {call_name}: {target_topic_dict}'.format(call_name=call_name, target_topic_dict=target_topic_dict))
        #!! note the special string formating!
        for msg_type, topic in target_topic_dict.iteritems():
            target_topic_dict[msg_type] = '"'+topic+'"'

        # replacement operations
        replacement_with_redbaron.replace_topic(red_obj, channel_type, target_topic_dict)

    return red_obj

if __name__ == "__main__":
    TOPIC_REPLACEMENT = True
    PARAMETER_REPLACEMENT = True

    #if sys.platform == 'darwin': # Mac OS
    #    file_path = '/Users/{0}/Dropbox/ros_examples/turtlebot/processed/'.format(getpass.getuser())
    #else: # linux /windows?
    #    file_path = '/home/{0}/ros_examples/turtlebot/processed/'.format(getpass.getuser())
    #filename = "files/jackal_auto_drive.py"
    #filename = "files/wander.py"
    #filename = "files/move_robot_jaco.py"
    #filename = "files/set_velocity.py"
    #dest_file = "files/set_velocity_mod.py"

    # joint trajectory example (test_move - ur5 to jaco)
    filename = "files/test_move_ur5.py"
    dest_file = "files/code.py"
    source_robot_name = 'ur5'
    target_robot_name = 'kinova'
    if sys.platform == 'darwin': # Mac OS
        file_path = '/Users/{0}/Dropbox/ros_examples/jaco/semiprocessed/'.format(getpass.getuser())
    else: # linux /windows?
        file_path = '/home/{0}/ros_examples/jaco/semiprocessed/'.format(getpass.getuser())
    #  ____          _        ____              _   _               _
    # / ___|___   __| | ___  / ___| _   _ _ __ | |_| |__   ___  ___(_)___
    #| |   / _ \ / _` |/ _ \ \___ \| | | | '_ \| __| '_ \ / _ \/ __| / __|
    #| |__| (_) | (_| |  __/  ___) | |_| | | | | |_| | | |  __/\__ \ \__ \
    # \____\___/ \__,_|\___| |____/ \__, |_| |_|\__|_| |_|\___||___/_|___/
    #                               |___/

    # create object
    red_obj = replacement_with_redbaron.creat_redbaron_obj(filename)

    #######################
    ### Replace Topics ######
    #######################
    if TOPIC_REPLACEMENT:
        # replace topics by reading code libraries and find a distribution
        # replace with active topics
        red_obj = topic_replacement_by_distribution(file_path, red_obj)

    #######################
    ### Check Limits ######
    #######################
    if PARAMETER_REPLACEMENT:
        vel_limits = {'linear':{'x':{'lower': -0.2, 'upper':0.2}, \
                                'y':{'lower': -0.2, 'upper':0.2}, \
                                'z':{'lower': -0.2, 'upper':0.2}},\
                      'angular':{'x':{'lower': -0.2, 'upper':0.2}, \
                                 'y':{'lower': -0.2, 'upper':0.2}, \
                                 'z':{'lower': -0.2, 'upper':0.2}}}

        with open(filename) as f:
            ast_file = ast.parse(f.read())

        # _____          _     _
        #|_   _|_      _(_)___| |_
        #  | | \ \ /\ / / / __| __|
        #  | |  \ V  V /| \__ \ |_
        #  |_|   \_/\_/ |_|___/\__|

        rv = parameters_in_file.ROSParameterVisitor(['geometry_msgs.msg.Twist','Twist'], limits_dict=vel_limits)
        rv.visit(ast_file)

        if rv.out_of_bound_list:
            replacement_with_redbaron.replace_parameters(red_obj, rv.out_of_bound_list)

        #     _       _       _  _____           _           _
        #    | | ___ (_)_ __ | ||_   _| __ __ _ (_) ___  ___| |_ ___  _ __ _   _
        # _  | |/ _ \| | '_ \| __|| || '__/ _` || |/ _ \/ __| __/ _ \| '__| | | |
        #| |_| | (_) | | | | | |_ | || | | (_| || |  __/ (__| || (_) | |  | |_| |
        # \___/ \___/|_|_| |_|\__||_||_|  \__,_|/ |\___|\___|\__\___/|_|   \__, |
        #                                     |__/                         |___/

        rv = parameters_in_file.ROSParameterVisitor(['trajectory_msgs.msg.JointTrajectory','JointTrajectory'],\
                                                     limits_dict={}, msg_fields_dict={})
        rv.visit(ast_file)

        # Find new joints with kdl retargetor
        # joint_names
        # points, positions
        synthesis_logger.debug("All variables: {0}".format(rv.scopes[0]))
        synthesis_logger.debug("Parameters of interest: {0}".format(rv.msg_fields_dict))

        if 'joint_names' in rv.msg_fields_dict.keys():
            for joint_names in rv.msg_fields_dict['joint_names']:

                if 'points' in rv.msg_fields_dict.keys() and \
                   'positions' in rv.msg_fields_dict['points'].keys():

                    # get values of joint_names
                    if isinstance(joint_names,str):
                        joints_names_values = rv.scopes[0].find(joint_names)
                    else:
                        joints_names_values = joint_names

                    # if we have both joint_name fields and points/positions field
                    for positions in rv.msg_fields_dict['points']['positions']:

                        # get values of positions
                        if isinstance(positions,str):
                            positions_values = rv.scopes[0].find(positions)
                        else:
                            positions_values = positions

                        # ensure joints_names_values and positions_values are the same length
                        if len(joints_names_values) == len(positions_values):
                            synthesis_logger.info("joints_names_values:{0}".format(joints_names_values))
                            synthesis_logger.info("positions_values:{0}".format(positions_values))

                            for joint_list in joints_names_values:
                                for pos_list in positions_values:
                                    # do optimization with kdl_retargeter
                                    # E.g: joints_names_values:[['j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3',\
                                    #                       'j2n6s300_joint_4', 'j2n6s300_joint_5', 'j2n6s300_joint_6']]
                                    # E.g: positions_values:[[2.2, 2.3, 1.57, 0, 0, 0]]
                                    target_joints, best_ret_angles = process_urdf.find_ret_angles_from_source_joints(\
                                        source_robot_name, target_robot_name, joint_list, pos_list, mode='scale_by_length')

                                    # replace joint names
                                    #[attribute_list, limit_violation, limit, cur_value, var_name]
                                    replacement_list = [(['joint_names'], '--', \
                                                        target_joints, joint_list, joint_names)]
                                    replacement_with_redbaron.replace_parameters(red_obj, replacement_list)

                                    # replace joint values
                                    replacement_list = [(['points','positions'], '--', \
                                                        best_ret_angles, pos_list, positions)]
                                    replacement_with_redbaron.replace_parameters(red_obj, replacement_list)

        """
        #[attribute_list, limit_violation, limit, cur_value, var_name]
        replacement_list = [(['joint_names'], '--', \
                             ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', \
                             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], \
                             ['j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3', \
                             'j2n6s300_joint_4', 'j2n6s300_joint_5', 'j2n6s300_joint_6'], 'JOINT_NAMES')]

        replacement_with_redbaron.replace_parameters(red_obj, replacement_list)
        """

    # save to file
    replacement_with_redbaron.save_redbardon_obj_to_file(red_obj,dest_file)
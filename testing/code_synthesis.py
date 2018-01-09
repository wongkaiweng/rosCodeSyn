import sys
import getpass
import ast
import logging

import prob_from_files
import parameters_in_file
import replacement_with_redbaron
import process_urdf
import process_yaml
import process_limits

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

def convert_joint_traj_commands(red_obj, ast_file, source_robot_name, target_robot_name):
    """
    Convert source joint commands to target joint commands with
    kinematic retargeting
    red_obj: redbardon obj of source code
    ast_file, souce code parsed by ast
    source_robot_name: name of the original robot
    target_robot_name: name of the target robot
    """

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
                        synthesis_logger.info("================ Joint Trajectory Replacement ================")
                        synthesis_logger.info("joints_names_values:{0}".format(joints_names_values))
                        synthesis_logger.info("positions_values:{0}".format(positions_values))

                        for joint_list in joints_names_values:
                            for pos_list in positions_values:
                                # do optimization with kdl_retargeter
                                # E.g: joints_names_values:[['j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3',\
                                #                       'j2n6s300_joint_4', 'j2n6s300_joint_5', 'j2n6s300_joint_6']]
                                # E.g: positions_values:[[2.2, 2.3, 1.57, 0, 0, 0]]

                                # ++++++++++++++++++++++++++++++++++++++++ #
                                # ++++++++ MAKE IT MODULAR HERE ++++++++++ #
                                # ++++++++++++++++++++++++++++++++++++++++ #
                                target_joints, best_ret_angles = process_urdf.find_ret_angles_from_source_joints(\
                                    source_robot_name, target_robot_name, joint_list, pos_list, mode='scale_by_length')

                                # replace joint names
                                #[attribute_list, limit_violation, limit, cur_value, var_name, call_name]
                                replacement_list = [(['joint_names'], '--', \
                                                    target_joints, joint_list, joint_names, None)]
                                replacement_with_redbaron.replace_parameters(red_obj, replacement_list)

                                # replace joint values
                                replacement_list = [(['points','positions'], '--', \
                                                    best_ret_angles, pos_list, positions, None)]
                                replacement_with_redbaron.replace_parameters(red_obj, replacement_list)


def convert_velocity_commands(red_obj, ast_file, source_robot_name, target_robot_name):
    """
    convert velocity commands of source robot to target robot's with scaling
    red_obj: redbardon obj of source code
    ast_file, souce code parsed by ast
    source_robot_name: name of the original robot
    target_robot_name: name of the target robot
    """
    vel_type_idx = {'linear':0, 'angular':1}
    vel_dir_idx = {'x':0,'y':1,'z':2}

    # load velocity limits
    source_vel_lim_dict =  process_limits.make_vel_limit_dic_from_yaml_text(process_yaml.find_robot_YAML(source_robot_name))
    target_vel_lim_dict =  process_limits.make_vel_limit_dic_from_yaml_text(process_yaml.find_robot_YAML(target_robot_name))

    # TODO: replace hard-coded list with our source_vel_limit_dict

    # find parameters
    rv = parameters_in_file.ROSParameterVisitor(['geometry_msgs.msg.Twist','Twist'], limits_dict=source_vel_lim_dict)
    rv.visit(ast_file)

    synthesis_logger.debug("All variables: {0}".format(rv.scopes[0]))
    synthesis_logger.debug("Parameters of interest: {0}".format(rv.msg_fields_dict))

    # scale velocity
    # e.g.: rv.scopes[0]: {'turn': [0, 0], 'vel': [0, 0.3], 'target': [1.0]}
    # e.g.:rv.msg_fields_dict: {'linear': {'x': ['vel']}, 'angular': {'z': ['turn']}}
    for vel_type, vel_dir_dict in rv.msg_fields_dict.iteritems():
        for vel_dir, field_list in vel_dir_dict.iteritems():
            for param in field_list:
                # get values of param
                if isinstance(param,str):
                    param_value_list = rv.scopes[0].find(param)
                else:
                    param_value_list = [param]

                # check and scale
                for param_value in param_value_list:
                    scaled_value, scale_result = process_yaml.scale_velocity_command(\
                                [vel_type, vel_dir], param_value, source_vel_lim_dict, target_vel_lim_dict)

                    if scale_result:
                        # replace twist values
                        # [attribute_list, limit_violation, limit, cur_value, var_name]

                        # get call function history (hard-coded for now, looking for soln)
                        replacement_list = []
                        if isinstance(param, str):
                            # parameter string
                            replacement_list.append([([vel_type, vel_dir], '--', \
                                                scaled_value, param_value, param, None)])
                        else:
                            # function instantiation
                            replacement_list.append([([vel_type, vel_dir], '--', \
                                                scaled_value, param_value, param, \
                                                ['Twist', [vel_type_idx[vel_type], 'Vector3'], vel_dir_idx[vel_dir]])])
                            # assignment to e.g: a.b.c = 1
                            for var_name in rv.var_names_list:
                                replacement_list.append([([vel_type, vel_dir], '--', \
                                                          scaled_value, param_value, var_name, None)])

                        for replacement in replacement_list:
                            replacement_with_redbaron.replace_parameters(red_obj, replacement)



if __name__ == "__main__":
    WANDER_EXAMPLE = False
    JACKAL_CONTROLLER_EXAMPLE = False
    UR5_TO_JACO_EXAMPLE = False
    WAVE_EXAMPLE = True

    TOPIC_REPLACEMENT = True
    PARAMETER_REPLACEMENT = True

    # _____                           _
    #| ____|_  ____ _ _ __ ___  _ __ | | ___  ___
    #|  _| \ \/ / _` | '_ ` _ \| '_ \| |/ _ \/ __|
    #| |___ >  < (_| | | | | | | |_) | |  __/\__ \
    #|_____/_/\_\__,_|_| |_| |_| .__/|_|\___||___/
    #                          |_|

    # Others
    #filename = "files/jackal_auto_drive.py"
    #filename = "files/move_robot_jaco.py"
    #filename = "files/set_velocity.py"

    if WANDER_EXAMPLE:
        # (turtlebot to jackal)
        filename = '/home/{0}/ros_examples/Examples/turtlebot_to_jackal (wander)/wander_turtlebot.py'.format(getpass.getuser())
        dest_file = '/home/{0}/ros_examples/Examples/turtlebot_to_jackal (wander)/code_generated_wander_jackal.py'.format(getpass.getuser())
        source_robot_name = 'turtlebot'
        target_robot_name = 'jackal'
        if sys.platform == 'darwin': # Mac OS
            file_path = '/Users/{0}/Dropbox/ros_examples/Jackal/semiprocessed/'.format(getpass.getuser())
        else: # linux /windows?
            file_path = '/home/{0}/ros_examples/Jackal/semiprocessed/'.format(getpass.getuser())

    elif JACKAL_CONTROLLER_EXAMPLE:
        # (jackal to turtlebot)
        filename = '/home/{0}/ros_examples/Examples/jackal_to_turtlebot (controller)/jackal_controller.py'.format(getpass.getuser())
        dest_file = '/home/{0}/ros_examples/Examples/jackal_to_turtlebot (controller)/code_generated_turtlebot_controller.py'.format(getpass.getuser())
        source_robot_name = 'jackal'
        target_robot_name = 'turtlebot'
        if sys.platform == 'darwin': # Mac OS
            file_path = '/Users/{0}/Dropbox/ros_examples/turtlebot/processed/'.format(getpass.getuser())
        else: # linux /windows?
            file_path = '/home/{0}/ros_examples/turtlebot/processed/'.format(getpass.getuser())

    elif UR5_TO_JACO_EXAMPLE:
        # joint trajectory example (test_move - ur5 to jaco)
        filename = "/home/{0}/ros_examples/Examples/ur5_to_jaco (test_move)/test_move_ur5.py".format(getpass.getuser())
        dest_file = "/home/{0}/ros_examples/Examples/ur5_to_jaco (test_move)/code_generated_test_move_jaco.py".format(getpass.getuser())
        source_robot_name = 'ur5'
        target_robot_name = 'kinova'
        if sys.platform == 'darwin': # Mac OS
            file_path = '/Users/{0}/Dropbox/ros_examples/jaco/semiprocessed/'.format(getpass.getuser())
        else: # linux /windows?
            file_path = '/home/{0}/ros_examples/jaco/semiprocessed/'.format(getpass.getuser())

    elif WAVE_EXAMPLE:
        # wave example (pepper to nao)
        filename = "/home/{0}/ros_examples/Examples/pepper_to_nao (wave)/wave_pepper.py".format(getpass.getuser())
        dest_file = "/home/{0}/ros_examples/Examples/pepper_to_nao (wave)/code_generated_wave_nao.py".format(getpass.getuser())
        source_robot_name = 'pepper'
        target_robot_name = 'nao'
        if sys.platform == 'darwin': # Mac OS
            file_path = '/Users/{0}/Dropbox/ros_examples/nao/'.format(getpass.getuser())
        else: # linux /windows?
            file_path = '/home/{0}/ros_examples/nao/'.format(getpass.getuser())

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
        synthesis_logger.info(" _____           _       ____            _                                     _   ")
        synthesis_logger.info("|_   _|__  _ __ (_) ___  |  _ \ ___ _ __ | | __ _  ___ ___ _ __ ___   ___ _ __ | |_ ")
        synthesis_logger.info("  | |/ _ \| '_ \| |/ __| | |_) / _ \ '_ \| |/ _` |/ __/ _ \ '_ ` _ \ / _ \ '_ \| __|")
        synthesis_logger.info("  | | (_) | |_) | | (__  |  _ <  __/ |_) | | (_| | (_|  __/ | | | | |  __/ | | | |_ ")
        synthesis_logger.info("  |_|\___/| .__/|_|\___| |_| \_\___| .__/|_|\__,_|\___\___|_| |_| |_|\___|_| |_|\__|")
        synthesis_logger.info("          |_|                      |_|                                              ")

        red_obj = topic_replacement_by_distribution(file_path, red_obj)

    #######################
    ### Check Limits ######
    #######################
    if PARAMETER_REPLACEMENT:
        synthesis_logger.info(" ____                   ____            _                                     _   ")
        synthesis_logger.info("|  _ \ __ _ _ __ __  _ |  _ \ ___ _ __ | | __ _  ___ ___ _ __ ___   ___ _ __ | |_ ")
        synthesis_logger.info("| |_) / _` | '__/ _` | | |_) / _ \ '_ \| |/ _` |/ __/ _ \ '_ ` _ \ / _ \ '_ \| __|")
        synthesis_logger.info("|  __/ (_| | | | (_| | |  _ <  __/ |_) | | (_| | (_|  __/ | | | | |  __/ | | | |_ ")
        synthesis_logger.info("|_|   \__,_|_|  \__,_| |_| \_\___| .__/|_|\__,_|\___\___|_| |_| |_|\___|_| |_|\__|")
        synthesis_logger.info("                                 |_|                                              ")

        with open(filename) as f:
            ast_file = ast.parse(f.read())

        # _____          _     _
        #|_   _|_      _(_)___| |_
        #  | | \ \ /\ / / / __| __|
        #  | |  \ V  V /| \__ \ |_
        #  |_|   \_/\_/ |_|___/\__|

        convert_velocity_commands(red_obj, ast_file, source_robot_name, target_robot_name)

        #     _       _       _  _____           _           _
        #    | | ___ (_)_ __ | ||_   _| __ __ _ (_) ___  ___| |_ ___  _ __ _   _
        # _  | |/ _ \| | '_ \| __|| || '__/ _` || |/ _ \/ __| __/ _ \| '__| | | |
        #| |_| | (_) | | | | | |_ | || | | (_| || |  __/ (__| || (_) | |  | |_| |
        # \___/ \___/|_|_| |_|\__||_||_|  \__,_|/ |\___|\___|\__\___/|_|   \__, |
        #                                     |__/                         |___/

        convert_joint_traj_commands(red_obj, ast_file, source_robot_name, target_robot_name)

    # save to file
    replacement_with_redbaron.save_redbardon_obj_to_file(red_obj,dest_file)
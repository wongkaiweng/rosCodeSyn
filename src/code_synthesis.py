import sys
import getpass
import ast
import logging
import os

import prob_from_files
import parameters_in_file
import moveit_in_file
import replacement_with_redbaron
import process_srdf
import process_urdf
import process_yaml
import process_limits

import logging_config
synthesis_logger = logging.getLogger("synthesis_logger")

# your codebase dir from param_config.py
import param_config
ROS_CODEBASES_DIR = param_config.ROS_CODEBASES_DIR

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


def moveit_replacement(red_obj, ast_file, source_robot_name, target_robot_name):
    # find all moveit occurances
    rv = moveit_in_file.ROSMoveItVisitor()
    rv.visit(ast_file)
    #print("scopes: All variables: {0}".format(rv.scopes[0]))
    #print("Parameters of interest: {0}".format(rv.msg_fields_dict))
    #print("MoveGroupInterface Instantiated: {0}".format(rv.interface_dict))
    #print("moveToPose frames by objs: {0}".format(rv.pose_cmd_frame_dict))

    # find suitable replacement
    # load target srdf
    target_srdf_string  = process_srdf.find_robot_SRDF(target_robot_name)

    if target_srdf_string:
        target_srdf_dict = process_srdf.load_srdf_to_dict(target_srdf_string)
        #srdf_logger.debug('target_srdf_dict:{0}'.format(target_srdf_dict))

        for move_group_obj, group_name in rv.interface_dict.iteritems():

            if rv.scopes[0].find(group_name):
                source_group_list  = rv.scopes[0].find(group_name)
            else:
                source_group_list  = [group_name]

            for source_group in source_group_list:

                if rv.scopes[0].find(rv.pose_cmd_frame_dict[move_group_obj]):
                    source_frame_list  = rv.scopes[0].find(rv.pose_cmd_frame_dict[move_group_obj])
                else:
                    source_frame_list  = [rv.pose_cmd_frame_dict[move_group_obj]]

                for source_frame in source_frame_list:
                    # find target group name
                    target_group = process_srdf.find_new_group_name_in_srdf_dict(target_srdf_dict, source_group)
                    synthesis_logger.debug('target_group:{0}'.format(target_group))

                    # find target frame
                    tip_frame_list =  process_srdf.get_tip_frame_list_from_group_name(target_srdf_dict, target_group)
                    target_frame =  process_srdf.select_tip_frame_from_list(tip_frame_list, source_frame)
                    synthesis_logger.debug('target_frame:{0}'.format(target_frame))

                    # replace values
                    # both interface and moveToPose
                    replacement_with_redbaron.replace_moveit_interface(red_obj, rv.interface_dict, {source_group: target_group}, rv.scopes)
                    replacement_with_redbaron.replace_moveit_moveToPose(red_obj, rv.pose_cmd_frame_dict, {source_frame:target_frame}, rv.scopes)
    else:
        synthesis_logger.warning("No SRDF file for target robot: {0}. Skipping MoveIt Replacement.".format(target_robot_name))


if __name__ == "__main__":
    # Example Selection
    WANDER_EXAMPLE = False
    JACKAL_CONTROLLER_EXAMPLE = False
    UR5_TO_JACO_EXAMPLE = False
    WAVE_EXAMPLE = False
    MOVEIT_WAVE_EXAMPLE = False
    MOVEIT_PATHPLANNING_EXAMPLE = False

    example_list = ['wander              - turtlebot to jackal', \
                    'keyboard control    - jackal to turtlebot',\
                    'test_move           - ur5 to jaco',\
                    'wave                - pepper to nao',\
                    'moveit wave         - fetch to pr2',\
                    'moveit pathplanning - fetch to pr2']

    # prompt user to select
    example_idx = raw_input("Please select an example from below. "+\
                               "Type the NUMBER (e.g.:1):\n{0}\n".format(\
                               "\n".join([str(idx+1)+'. '+x for idx,x in enumerate(example_list)])))

    if int(example_idx) == 1: WANDER_EXAMPLE = True
    if int(example_idx) == 2: JACKAL_CONTROLLER_EXAMPLE = True
    if int(example_idx) == 3: UR5_TO_JACO_EXAMPLE = True
    if int(example_idx) == 4: WAVE_EXAMPLE = True
    if int(example_idx) == 5: MOVEIT_WAVE_EXAMPLE = True
    if int(example_idx) == 6: MOVEIT_PATHPLANNING_EXAMPLE = True


    # Replacement Method Settings
    TOPIC_REPLACEMENT = True
    PARAMETER_REPLACEMENT = True
    MOVEIT_REPLACEMENT = True

    # _____                           _
    #| ____|_  ____ _ _ __ ___  _ __ | | ___  ___
    #|  _| \ \/ / _` | '_ ` _ \| '_ \| |/ _ \/ __|
    #| |___ >  < (_| | | | | | | |_) | |  __/\__ \
    #|_____/_/\_\__,_|_| |_| |_| .__/|_|\___||___/
    #                          |_|

    if WANDER_EXAMPLE:
        # (turtlebot to jackal)
        filename = os.path.dirname(os.path.abspath(__file__))+'/../examples/turtlebot_to_jackal (wander)/wander_turtlebot.py'
        dest_file = os.path.dirname(os.path.abspath(__file__))+'/../examples/turtlebot_to_jackal (wander)/code_generated_wander_jackal.py'
        source_robot_name = 'turtlebot'
        target_robot_name = 'jackal'
        file_path = ROS_CODEBASES_DIR+'/Jackal/semiprocessed/'

    elif JACKAL_CONTROLLER_EXAMPLE:
        # (jackal to turtlebot)
        filename = os.path.dirname(os.path.abspath(__file__))+'/../examples/jackal_to_turtlebot (controller)/jackal_controller.py'
        dest_file = os.path.dirname(os.path.abspath(__file__))+'/../examples/jackal_to_turtlebot (controller)/code_generated_turtlebot_controller.py'
        source_robot_name = 'jackal'
        target_robot_name = 'turtlebot'
        file_path = ROS_CODEBASES_DIR+'/turtlebot/processed/'


    elif UR5_TO_JACO_EXAMPLE:
        # joint trajectory example (test_move - ur5 to jaco)
        filename = os.path.dirname(os.path.abspath(__file__))+"/../examples/ur5_to_jaco (test_move)/test_move_ur5.py"
        dest_file = os.path.dirname(os.path.abspath(__file__))+"/../examples/ur5_to_jaco (test_move)/code_generated_test_move_jaco.py"
        source_robot_name = 'ur5'
        target_robot_name = 'kinova'
        file_path = ROS_CODEBASES_DIR+'/jaco/semiprocessed/'

    elif WAVE_EXAMPLE:
        # wave example (pepper to nao)
        filename = os.path.dirname(os.path.abspath(__file__))+"/../examples/pepper_to_nao (wave)/wave_pepper.py"
        dest_file = os.path.dirname(os.path.abspath(__file__))+"/../examples/pepper_to_nao (wave)/code_generated_wave_nao.py"
        source_robot_name = 'pepper'
        target_robot_name = 'nao'
        file_path = ROS_CODEBASES_DIR+'/nao/'

    elif MOVEIT_WAVE_EXAMPLE:
        # moveit example (fetch to pr2)
        filename = os.path.dirname(os.path.abspath(__file__))+"/../examples/fetch_to_pr2 (wave)/wave_fetch.py"
        dest_file = os.path.dirname(os.path.abspath(__file__))+"/../examples/fetch_to_pr2 (wave)/code_generated_wave_pr2.py"
        source_robot_name = 'fetch'
        target_robot_name = 'pr2'
        file_path = ROS_CODEBASES_DIR +'/pr2/semiprocessed/'
        #file_path = None

    elif MOVEIT_PATHPLANNING_EXAMPLE:
        # moveit example (fetch to pr2)
        filename = os.path.dirname(os.path.abspath(__file__))+"/../examples/fetch_to_pr2 (pathplanning)/pathplanning_fetch.py"
        dest_file = os.path.dirname(os.path.abspath(__file__))+"/../examples/fetch_to_pr2 (pathplanning)/code_generated_pathplanning_pr2.py"
        source_robot_name = 'fetch'
        target_robot_name = 'pr2'
        file_path = ROS_CODEBASES_DIR +'/pr2/semiprocessed/'
        #file_path = None

    #  ____          _        ____              _   _               _
    # / ___|___   __| | ___  / ___| _   _ _ __ | |_| |__   ___  ___(_)___
    #| |   / _ \ / _` |/ _ \ \___ \| | | | '_ \| __| '_ \ / _ \/ __| / __|
    #| |__| (_) | (_| |  __/  ___) | |_| | | | | |_| | | |  __/\__ \ \__ \
    # \____\___/ \__,_|\___| |____/ \__, |_| |_|\__|_| |_|\___||___/_|___/
    #                               |___/

    # create object
    red_obj = replacement_with_redbaron.creat_redbaron_obj(filename)

    with open(filename) as f:
        ast_file = ast.parse(f.read())
    f.closed

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


    ####################################
    ### Replace interfaces in moveit ###
    ####################################
    if MOVEIT_REPLACEMENT:
        synthesis_logger.info(" __  __                ___ _   ____            _                                      _   ")
        synthesis_logger.info("|  \/  | _____   _____|_ _| |_|  _ \ ___ _ __ | | __ _  ___ ___  _ __ ___   ___ _ __ | |_ ")
        synthesis_logger.info("| |\/| |/ _ \ \ / / _ \| || __| |_) / _ \ '_ \| |/ _` |/ __/ _ \| '_ ` _ \ / _ \ '_ \| __|")
        synthesis_logger.info("| |  | | (_) \ V /  __/| || |_|  _ <  __/ |_) | | (_| | (_|  __/| | | | | |  __/ | | | |_ ")
        synthesis_logger.info("|_|  |_|\___/ \_/ \___|___|\__|_| \_\___| .__/|_|\__,_|\___\___||_| |_| |_|\___|_| |_|\__|")
        synthesis_logger.info("                                        |_|                                              ")

        moveit_replacement(red_obj, ast_file, source_robot_name, target_robot_name)

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
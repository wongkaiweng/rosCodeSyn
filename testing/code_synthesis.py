import sys
import getpass
import ast

import prob_from_files
import parameters_in_file
import replacement_with_redbaron

TOPIC_REPLACEMENT = True
PARAMETER_REPLACEMENT = True

if __name__ == "__main__":
    if sys.platform == 'darwin': # Mac OS
        file_path = '/Users/{0}/Dropbox/ros_examples/turtlebot/processed/'.format(getpass.getuser())
    else: # linux /windows?
        file_path = '/home/{0}/ros_examples/turtlebot/processed/'.format(getpass.getuser())
    filename = "files/jackal_auto_drive.py"
    filename = "files/wander.py"
    dest_file = "files/code.py"
    #filename = "files/set_velocity.py"
    #dest_file = "files/set_velocity_mod.py"

    # channel_type_to_call_name_dict[channel_type] = call_name
    channel_type_to_call_name_dict = {"Publisher": 'rospy.Publisher',\
                                      "Subscriber": 'rospy.Subscriber',\
                                      "SimpleActionClient": 'actionlib.SimpleActionClient'}

    # create object
    red_obj = replacement_with_redbaron.creat_redbaron_obj(filename)

    #######################
    ### Replace Topics ######
    #######################
    if TOPIC_REPLACEMENT:
        # replace all different types of channels
        for channel_type, call_name in channel_type_to_call_name_dict.iteritems():

            # changed approach.py in turtlebot
            target_topic_dict = prob_from_files.get_best_topic_match_for_all_msg_types(file_path, call_name=call_name)

            #!! note the special string formating!
            for msg_type, topic in target_topic_dict.iteritems():
                target_topic_dict[msg_type] = '"'+topic+'"'

            # replacement operations
            replacement_with_redbaron.replace_topic(red_obj, channel_type, target_topic_dict)

        # save to file
        replacement_with_redbaron.save_redbardon_obj_to_file(red_obj,dest_file)

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
        rv = parameters_in_file.ROSParameterVisitor(['geometry_msgs.msg.Twist','Twist'], limits_dict=vel_limits)
        rv.visit(ast_file)

        replacement_with_redbaron.replace_parameters(red_obj, rv.out_of_bound_list)

        # save to file
        replacement_with_redbaron.save_redbardon_obj_to_file(red_obj,dest_file)
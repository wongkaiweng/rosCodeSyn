import sys
import getpass

import files_to_prob
import replacement_with_redbaron



if __name__ == "__main__":
    if sys.platform == 'darwin': # Mac OS
        file_path = '/Users/{0}/Dropbox/ros_examples/turtlebot/processed/'.format(getpass.getuser())
    else: # linux /windows?
        file_path = '/home/{0}/ros_examples/turtlebot/processed/'.format(getpass.getuser())
    filename = "files/jackal_auto_drive.py"
    dest_file = "files/code.py"

    # channel_type_to_call_name_dict[channel_type] = call_name
    channel_type_to_call_name_dict = {"Publisher": 'rospy.Publisher',\
                                      "Subscriber": 'rospy.Subscriber',\
                                      "SimpleActionClient": 'actionlib.SimpleActionClient'}

    # create object
    red_obj = replacement_with_redbaron.creat_redbaron_obj(filename)

    # replace all different types of channels
    for channel_type, call_name in channel_type_to_call_name_dict.iteritems():

        # changed approach.py in turtlebot
        target_topic_dict = files_to_prob.get_best_topic_match_for_all_msg_types(file_path, call_name=call_name)

        #!! note the special string formating!
        for msg_type, topic in target_topic_dict.iteritems():
            target_topic_dict[msg_type] = '"'+topic+'"'

        # replacement operations
        replacement_with_redbaron.replace_topic(red_obj, channel_type, target_topic_dict)

    # save to file
    replacement_with_redbaron.save_redbardon_obj_to_file(red_obj,dest_file)



import argparse
import os
import pandas as pd
import sys
import getpass
import logging
import re

import topics_in_file
import parameters_in_file
import logging_config
test_logger = logging.getLogger("test_logger")

def retrieve_topic_names(file_path, msg_type, remove_slash=True):
    """
    Retrieve topic names based on a msg type
    file_path: path to python files (not recursive)
    msg_type: type of msg
    remove_slash: remove leading slash of topics if true
    """
    topic_name_list = []
    for root, dirs, files in os.walk(file_path):
        # traverse all directories
        for directory in dirs:
            topic_name_list.extend(retrieve_topic_names(os.path.join(root, directory), msg_type))

        # find all files
        for file in files:
            if file.endswith(".py"):
                #find msg_type in file
                file_topic_name_list = topics_in_file.get_topics_in_file(os.path.join(root, file), msg_type)

                # remove leading slash
                if remove_slash:
                    file_topic_name_list = [re.sub("^/|/$", "", s) for s in file_topic_name_list]

                topic_name_list.extend(file_topic_name_list)

    return topic_name_list

def retrieve_parameters(file_path, msg_type):
    """
    Retrieve parameters based on a msg type
    file_path: path to python files (not recursive)
    msg_type: type of msg
    """
    msg_fields = {}
    for root, dirs, files in os.walk(file_path):
        # traverse all directories
        for directory in dirs:
            topic_name_list.extend(retrieve_topic_names(os.path.join(root, directory), msg_type))

        # find all files
        for file in files:
            if file.endswith(".py"):
                #find parameters with msg_type
                msg_fields = parameters_in_file.get_parameters_in_file(os.path.join(root, file), msg_type)

    return msg_fields

def find_topic_name_distribution(name_list, col_name='name'):
    """
    Find distribution of a msg type and return a dict
    name_list: list containing topic names
    col_name: name of column. can be anything, e.g: publisher_topic, subscriber_topic
    """
    # use panda to process data
    df = pd.DataFrame(list(name_list), columns=[col_name,])

    # find out distribution for this msg type
    # print df[col_name].value_counts().to_dict()
    prob_df =  df.groupby(col_name).size() / len(df)
    test_logger.debug(df.groupby(col_name).size())
    test_logger.debug(prob_df)

    return prob_df.to_dict()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="find topics with ast")
    if sys.platform == 'darwin': # Mac OS
        file_path = '/Users/wongkaiweng/Dropbox/ros_examples/UR5/processed/'
    else: # linux /windows?
        file_path = '/home/{0}/ros_examples/turtlebot/processed/'.format(getpass.getuser())
    parser.add_argument('--file_path', type=str, help='Specify directory of files.', nargs='?', const=file_path, default=file_path)
    parser.add_argument('--msg_type', type=str, help='Specify message type', action='append', nargs='?', default=[])

    args, unknown = parser.parse_known_args()
    if not args.msg_type:
      args.msg_type = ['geometry_msgs.msg.Twist','Twist']

    topic_name_list = retrieve_topic_names(args.file_path, args.msg_type)

    msg_fields = retrieve_parameters(args.file_path, args.msg_type)
    print "============\nParameters\n============"
    print "Twist Data:{0}".format(msg_fields)

    print args.msg_type
    print "==================\nTopic Distribution\n=================="
    prob_dict = find_topic_name_distribution(topic_name_list)
    print prob_dict
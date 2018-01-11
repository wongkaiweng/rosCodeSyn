import re
import os
import matplotlib.pyplot as plt
import pandas as pd
import sys
import getpass

# your codebase dir from param_config.py
import param_config
ROS_CODEBASES_DIR = param_config.ROS_CODEBASES_DIR

re_subscriber = re.compile(r'rospy\.Subscriber\([\'|\"](?P<topic>[\w_/]+)[\'|\"]\s?\,\s?(?P<msgType>[\w_.]+)')  # a pattern for subscriber
re_publisher = re.compile(r'rospy\.Publisher\([\'|\"](?P<topic>[\w_/]+)[\'|\"]\s?\,\s?(?P<msgType>[\w_.]+)')  # a pattern for publisher
re_act_client = re.compile(r'actionlib\.SimpleActionClient\([\'|\"](?P<action_topic>[\w_/]+)[\'|\"]\s?\,\s?(?P<msgType>[\w_.]+)')

def retrieve_sub_and_pub_topic_msg_pair(file_path):
    sub_pub_dict =  retrieve_name_msg_pair(file_path, [re_subscriber, re_publisher])
    return sub_pub_dict[re_subscriber], sub_pub_dict[re_publisher]

def retrieve_sub_topic_msg_pair(file_path):
    return retrieve_name_msg_pair(file_path, [re_subscriber])[re_subscriber]

def retrieve_pub_topic_msg_pair(file_path):
    return retrieve_name_msg_pair(file_path, [re_publisher])[re_publisher]

def retrieve_action_client_msg_pair(file_path):
    return retrieve_name_msg_pair(file_path, [re_act_client])[re_act_client]


def retrieve_name_msg_pair(file_path, re_compile_obj_list):
    """
    Retrieve pair of name and msg based on re_compile
    file_path: path to python files (not recursive)
    re_compile_obj_list: list of re compile objects to reserach for
    """
    line_to_append, line_to_check = "",""
    pair_dict = {k:[] for k in re_compile_obj_list}
    for file in os.listdir(file_path):
        #file = 'draw_a_square.py'
        if file.endswith(".py"):
            #print file
            with open(os.path.join(file_path, file), "r") as fp:
                for line in fp:
                    # ignore comments
                    if line.strip().startswith('#'):
                        continue

                    # concatenate line when necessary
                    if line.strip().endswith('\\'):
                        line_to_append += line.strip()[:-len('\\')]
                    else:
                        line_to_check = line_to_append+line.strip()
                        line_to_append = ""

                    if line_to_check:
                        #print line_to_check
                        # find pair
                        for re_compile_obj in re_compile_obj_list:
                            group_name_msg = re.search(re_compile_obj, line_to_check)
                            if group_name_msg:
                                pair_dict[re_compile_obj].append(group_name_msg.groups())

            fp.closed

    return pair_dict


def find_distribution(name_msg_pair_list, target_msg_type_list, col_name='name'):
    """
    Find distribution of a msg type
    name_msg_pair_list: pair list containing (name, msg_type)
    target_msg_type_list: list of the type of msgs to extract (usually two: the long form: geometry_msgs.msgs.Twist, and the short form:Twist)
    col_name: name of column. can be anything, e.g: publisher_topic, subscriber_topic
    """
    # use panda to process data
    df = pd.DataFrame(list(name_msg_pair_list), columns=[col_name,'msg_type'])
    #print pub_df

    # find out distribution for twist topic
    name_select = pd.Series(target_msg_type_list)
    target_msg_type_df = df[df.msg_type.isin(name_select)]
    #print twist_df.describe()
    print target_msg_type_df[col_name].value_counts()

    return target_msg_type_df

def plot_distribution(df, target_msg_type, col_name='name'):
    df[col_name].value_counts().plot(kind='barh')
    plt.title('Number of appearances in for {0} messages'.format(target_msg_type))
    plt.xlabel('Frequency')
    plt.show()



def test_pub_and_sub():
    # with '/' at the end
    file_path = ROS_CODEBASES_DIR+"/turtlebot/processed/"

    print "----------------\nTurtlebot\n----------------"

    sublisher_pair_list, publisher_pair_list =  retrieve_sub_and_pub_topic_msg_pair(file_path)
    #publisher_pair_list = retrieve_pub_topic_msg_pair(file_path)
    #sublisher_pair_list = retrieve_sub_topic_msg_pair(file_path)

    twist_df = find_distribution(publisher_pair_list, ['Twist', 'geometry_msgs.msg.Twist'], 'publisher_topic') # twist topic
    find_distribution(sublisher_pair_list, ['LaserScan', 'sensor_msgs.msg.LaserScan'], 'subscriber_topic') # scan topic

    #plot_distribution(twist_df, 'Twist', 'publisher_topic')




def test_actionlib():
    file_path = ROS_CODEBASES_DIR+"/UR5/semiprocessed/"

    print "--------------------\nUniversial Robots UR5\n--------------------"

    action_client_pair_list = retrieve_action_client_msg_pair(file_path)

    df = find_distribution(action_client_pair_list, ['FollowJointTrajectoryAction', 'control_msgs.msg.FollowJointTrajectoryAction'], 'action_client')


if __name__ == "__main__":
    test_pub_and_sub()
    test_actionlib()




import logging

import rospy
import rostopic
import subprocess
import re

import logging_config
channel_logger = logging.getLogger("channel_logger")


def find_channel_name(channel_type, msg_type):
    """
    This function takes in channel_type and perform
    the correct operation to retrieve channel_name
    Input:
    channel_type: 'Publisher', 'Subscriber' or 'SimpleActionClient'
    msg_type: 'a.b.c' or 'c', e.g: tf2_msgs.msg.TFMessage
    Output:
    channel_name: topic name or action name
    """

    if msg_type.count('.') == 2:
        # process msg_type from a.b.c to a/c
        new_msg_type = "/".join(map(msg_type.split('.').__getitem__, [0,2]))

        # find channel_name
        if channel_type in ['Publisher', 'Subscriber']:
            return format_topic_name(find_topic_with_type(new_msg_type))
        elif channel_type == 'SimpleActionClient':
            return format_topic_name(find_action_with_type(new_msg_type))
        else:
            channel_logger.warning('Channel type Invalid: {0}'.format(channel_type))

    else:
        # if it's only 'c'
        return format_topic_name(find_channel_name_from_short_type(channel_type, msg_type))

def format_topic_name(topic_name):
    topic_name = re.sub("^/|/$", "", topic_name)
    return "'" + topic_name + "'"


def get_full_topic_list():
    # convert xacro to urdf
    p = subprocess.Popen("rostopic list", shell=True, stdout=subprocess.PIPE)
    topic_list_string, stderr = p.communicate()
    return topic_list_string.split('\n')


def find_channel_name_from_short_type(channel_type, msg_type):
    # get full list of topics
    topic_list = get_full_topic_list()

    for topic_name in topic_list:

        # get topic type
        topic_type,_,_ = rostopic.get_topic_type(topic_name)

        # checking only the second part
        short_type = topic_type.split('/')[1]
        if msg_type == short_type:
            return topic_name
        elif msg_type+'Goal' == short_type and\
             channel_type == 'SimpleActionClient':
            return topic_name.replace('/goal','')

    return None


def find_topic_with_type(topic_type):
    """
    topic_type format: module/type, e.g:'tf2_msgs/TFMessage'
    """
    topic_name_list = rostopic.find_by_type(topic_type)
    if topic_name_list:
        return topic_name_list[0] # only returns the first element
    else:
        return None


def find_action_with_type(action_type):
    """
    topic_type format: module/type, e.g:'control_msgs/FollowJointTrajectoryAction'
    """
    action_goal_list = rostopic.find_by_type(action_type+'Goal')
    if action_goal_list:
        return action_goal_list[0].replace('/goal','')
    else:
        return None


def find_service_with_type():
    return NotImplemented


if __name__ == "__main__":
    channel_logger.debug('---Getting None:---')
    channel_logger.debug(find_topic_with_type('tf2_msgs/TFMessag'))
    channel_logger.debug(find_action_with_type('control_msgs/FollowJointTrajectoryActin'))
    channel_logger.debug('---Getting a channel:---')
    channel_logger.debug(find_topic_with_type('tf2_msgs/TFMessage'))
    channel_logger.debug(find_action_with_type('control_msgs/FollowJointTrajectoryAction'))
    channel_logger.debug(find_service_with_type())

    channel_logger.debug('---Getting name from .format:---')
    channel_logger.debug(find_channel_name('Publisher', 'tf2_msgs.msg.TFMessage'))
    channel_logger.debug(find_channel_name('SimpleActionClient', 'control_msgs.msg.FollowJointTrajectoryAction'))

    channel_logger.debug('---Getting name from short msg type:---')
    channel_logger.debug(find_channel_name_from_short_type('SimpleActionClient', 'FollowJointTrajectoryAction'))
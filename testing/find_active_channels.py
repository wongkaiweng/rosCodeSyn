import logging

import rospy
import rostopic
import subprocess
import re

import logging_config
channel_logger = logging.getLogger("channel_logger")


def find_channel_name(orig_topic_name, channel_type, msg_type):
    """
    This function takes in channel_type and perform
    the correct operation to retrieve channel_name
    Input:
    orig_topic_name: name of the original topic
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
        topic_name_list = find_channel_name_from_short_type(channel_type, msg_type)

        # handle situations where are there multiple topic with the same type
        if not len(topic_name_list):
            return None
        elif len(topic_name_list) == 1:
            return format_topic_name(topic_name_list[0])
        else:
            # prompt user to select
            topic_name_idx = raw_input("Please select a topic for '{0}' from the list.\n".format(msg_type)+\
                                       "The original topic was '{0}'. ".format(orig_topic_name)+\
                                       "Type the NUMBER (e.g.:1):\n{0}\n".format(\
                                       "\n".join([str(idx+1)+'. '+x for idx,x in enumerate(topic_name_list)])))
            return format_topic_name(topic_name_list[int(topic_name_idx)-1])

def format_topic_name(topic_name):
    if topic_name:
        topic_name = re.sub("^/|/$", "", topic_name)
        return "'" + topic_name + "'"
    else:
        return None

def get_full_topic_list():
    # convert xacro to urdf
    p = subprocess.Popen("rostopic list", shell=True, stdout=subprocess.PIPE)
    topic_list_string, stderr = p.communicate()
    return topic_list_string.split('\n')


def find_channel_name_from_short_type(channel_type, msg_type):
    # get full list of topics
    topic_list = get_full_topic_list()
    topic_list.remove('')

    topic_name_list = []
    for topic_name in topic_list:
        # get topic type
        topic_type,_,_ = rostopic.get_topic_type(topic_name)

        # checking only the second part
        short_type = topic_type.split('/')[1]
        if msg_type == short_type:
            topic_name_list.append(topic_name)

        elif msg_type+'Goal' == short_type and\
                channel_type == 'SimpleActionClient':
            topic_name_list.append(topic_name.replace('/goal',''))

    # return list for user to choose
    return topic_name_list


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
    channel_logger.debug(find_channel_name('test','Publisher', 'tf2_msgs.msg.TFMessage'))
    channel_logger.debug(find_channel_name('test', 'SimpleActionClient', 'control_msgs.msg.FollowJointTrajectoryAction'))
    channel_logger.debug(find_channel_name('test', 'SimpleActionClient', 'FollowJointTrajectoryAction'))

    channel_logger.debug('---Getting name from short msg type:---')
    channel_logger.debug(find_channel_name_from_short_type('SimpleActionClient', 'FollowJointTrajectoryAction'))
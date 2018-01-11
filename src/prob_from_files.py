import argparse
import os
import pandas as pd
import sys
import logging
import re
import itertools
import unittest

import topics_in_file
import parameters_in_file
import logging_config
probs_logger = logging.getLogger("probs_logger")

# your codebase dir from param_config.py
import param_config
ROS_CODEBASES_DIR = param_config.ROS_CODEBASES_DIR

def retrieve_topic_names(file_path, msg_type, remove_slash=True, call_name='rospy.Publisher'):
    if isinstance(msg_type, list):
        probs_logger.warning("msg_type now is not a list. Using only the first element...")
        msg_type = msg_type[0]
    topic_name_dict = retrieve_all_topics(file_path, remove_slash, call_name)
    return retrieve_topic_name_from_topic_dict(topic_name_dict, msg_type)

def retrieve_topic_names_from_topic_dict(topic_name_dict, msg_type_list):
    """
    CHANGED: now handles aggregation in topics_in_file.get_topics_in_file
             Do not use this function anymore.
    """
    probs_logger.warning("This function is obselete. Use 'retrieve_topic_name_from_topic_dict' instead")
    # return only the msg_types of interest
    topics_for_msg_type_list = [topic_name_dict[msg_type] for msg_type in msg_type_list if msg_type in topic_name_dict.keys()]
    return [i for i in itertools.chain.from_iterable(topics_for_msg_type_list)]

def retrieve_topic_name_from_topic_dict(topic_name_dict, msg_type):
    return topic_name_dict[msg_type]

def retrieve_all_topics(file_path, remove_slash=True, call_name='rospy.Publisher'):
    """
    Retrieve topic names based on a msg type
    file_path: path to python files (not recursive)
    msg_type: type of msg
    remove_slash: remove leading slash of topics if true
    """
    topic_name_dict = {}

    if file_path:
        if not os.path.isdir(file_path):
            probs_logger.log(4, "{0} is not directory!".format(file_path))

        for root, dirs, files in os.walk(file_path):
            # traverse all directories
            for directory in dirs:
                update_dict_value_list(topic_name_dict, retrieve_all_topics(os.path.join(root, directory), remove_slash, call_name))

            # find all files
            for file in files:

                if file.endswith(".py"):
                    # get all topics in file
                    file_topic_name_dict = topics_in_file.get_topics_in_file(os.path.join(root, file), call_name)

                    # remove leading slash
                    if remove_slash:
                        for key, item_list in file_topic_name_dict.iteritems():
                            file_topic_name_dict[key] = [re.sub("^/|/$", "", s) for s in item_list]

                    # combine dicts and store new fields
                    update_dict_value_list(topic_name_dict, file_topic_name_dict)
    else:
        probs_logger.warning('File Path: {0} is not valid! Not retrieving topics.'.format(file_path))

    return topic_name_dict

def update_dict_value_list(target_dict, update_dict):
    """
    update target_dict with update_dict, knowing their values are lists
    (extending lists)
    """
    for key in update_dict.keys():
        if key in target_dict.keys():
            if not isinstance(target_dict[key], list):
                update_dict_value_list(target_dict[key],update_dict[key])
            else:
                target_dict[key].extend(update_dict[key])
        else:
            target_dict[key] = update_dict[key]

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
    probs_logger.log(8, "Count:{0}".format(df.groupby(col_name).size()))
    probs_logger.log(8, "Prob:{0}".format(prob_df))
    probs_logger.log(8, '------------------')

    return prob_df.to_dict()


def get_best_topic_match_for_all_msg_types(file_path, call_name='rospy.Publisher'):
    target_topic_dict = {}

    topic_name_dict = retrieve_all_topics(file_path, call_name=call_name)

    #** now long.topic.name and short.topic.name are grouped in topics_in_file.py
    for topic, topic_name_list in topic_name_dict.iteritems():

        # find distribution
        prob_dict = find_topic_name_distribution(topic_name_list, topic)

        # find the largest value and the key associated with it
        sorted_prob_dict_values = sorted(prob_dict.values())
        for topic_name, prob_value in prob_dict.iteritems():
            if prob_value == sorted_prob_dict_values[-1]:
                target_topic_dict[topic] = topic_name

    return target_topic_dict


def findDiff(d1, d2, path=""):
    """
    From https://stackoverflow.com/questions/27265939/comparing-python-dictionaries-and-nested-dictionaries
    Check if two dictionaries are different from each other
    """
    diff = False
    for k in d1.keys():
        if not d2.has_key(k):
            print path, ":"
            print k + " as key not in d2", "\n"
            diff = True
        else:
            if type(d1[k]) is dict:
                if path == "":
                    path = k
                else:
                    path = path + "->" + k
                findDiff(d1[k],d2[k], path)
            else:
                if sorted(d1[k]) != sorted(d2[k]):
                    print path, ":"
                    print " - ", k," : ", d1[k]
                    print " + ", k," : ", d2[k]
                    diff = True

    return diff


class TestMethods(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """ get_some_resource() is slow, to avoid calling it for each test use setUpClass()
            and store the result as class variable
        """
        super(TestMethods, cls).setUpClass()
        cls.file_path = ROS_CODEBASES_DIR+'/turtlebot/processed/'
        cls.msg_type_list = ['geometry_msgs.msg.Twist','Twist']

        cls.topic_name_dict = retrieve_all_topics(cls.file_path)
        cls.expected_topic_name_dict =  {\
        'geometry_msgs.msg.Twist': ['cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi',\
                                     'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', \
                                     'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', \
                                     'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', \
                                     'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', \
                                     'cmd_vel_mux/input/teleop', 'cmd_vel_mux/input/teleop', 'cmd_vel_mux/input/teleop', \
                                     'cmd_vel_mux/input/teleop', 'cmd_vel_mux/input/teleop', 'cmd_vel_mux/input/teleop', \
                                     'mobile_base/commands/velocity', 'mobile_base/commands/velocity', 'mobile_base/commands/velocity'],\
        'geometry_msgs.msg.PoseStamped': ['move_base_simple/goal', 'move_base_simple/goal', 'move_base_simple/goal', \
                                         'move_base_simple/goal'], \
        'DigitalOutput': ['mobile_base/commands/digital_output', 'mobile_base/commands/digital_output'], \
        'PoseStamped': ['move_base_simple/goal', 'move_base_simple/goal', 'move_base_simple/goal', 'move_base_simple/goal'], \
        'std_msgs.msg.String': ['range_data', 'range_data'], \
        'kobuki_msgs.msg.Sound': ['mobile_base/commands/sound', 'mobile_base/commands/sound', 'mobile_base/commands/sound'], \
        'kobuki_msgs.msg.DigitalOutput': ['mobile_base/commands/digital_output', 'mobile_base/commands/digital_output'], \
        'SoundRequest': ['robotsound', 'robotsound'], 'UInt8': ['command_input'], \
        'Twist': ['cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', \
                  'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', \
                  'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', \
                  'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/teleop', 'cmd_vel_mux/input/teleop', \
                  'cmd_vel_mux/input/teleop', 'cmd_vel_mux/input/teleop', 'cmd_vel_mux/input/teleop', 'cmd_vel_mux/input/teleop', \
                  'mobile_base/commands/velocity', 'mobile_base/commands/velocity', 'mobile_base/commands/velocity'], \
        'Sound': ['mobile_base/commands/sound', 'mobile_base/commands/sound', 'mobile_base/commands/sound'], \
        'Config': ['state_estimate', 'state_estimate'], \
        'std_msgs.msg.UInt8': ['command_input'], 'String': ['range_data', 'range_data']}

        cls.topic_name_list = retrieve_topic_name_from_topic_dict(cls.topic_name_dict, cls.msg_type_list[0])
        cls.expected_topic_name_list = ['cmd_vel_mux/input/navi', 'cmd_vel_mux/input/teleop', 'cmd_vel_mux/input/teleop', 'cmd_vel_mux/input/navi', \
            'cmd_vel_mux/input/teleop', 'cmd_vel_mux/input/teleop', 'mobile_base/commands/velocity', 'cmd_vel_mux/input/navi', \
            'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', \
            'cmd_vel_mux/input/teleop', 'cmd_vel_mux/input/teleop', 'mobile_base/commands/velocity', 'cmd_vel_mux/input/navi', \
            'cmd_vel_mux/input/navi', 'mobile_base/commands/velocity', 'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', \
            'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi']

        cls.prob_dict = find_topic_name_distribution(cls.topic_name_list)
        cls.expected_prob_dict = {'cmd_vel_mux/input/teleop': 0.2608695652173913, \
            'mobile_base/commands/velocity': 0.13043478260869565, \
            'cmd_vel_mux/input/navi': 0.60869565217391308}

        cls.msg_fields = parameters_in_file.retrieve_parameters(cls.file_path, cls.msg_type_list)
        cls.expected_msg_fields = {'linear': {'x': [0, 0.3, -0.1, 0, 0.0, 0.2, 0, 0.2, 0.2, 0.0, 0.0, 0.2, 0, -0.2, \
                                                0, 0.2, 0.2, 0.0, 0.0, -0.2, 0.2, 0.2, 0, -0.2, -0.2, 0, 0.2, 0, -0.2, \
                                                -0.2, 0, 0.2, 0.25, 0.25, 0.0, 0.0, -0.2, 0.0, 0.0, 0.2, 0.2, 0, 0.3]}, \
                                    'angular': {'z': [0, 0, 0, 0, 0, 0, 0.2, 0, -2, 0, 2, 0, 0, 0.2, 0.0, 0, 0, 0, -2, -2, \
                                                     2, 0, -2, 0, 2, 2, 0, 2, 1, 0.2, 0.0, 0, 0.3, -0.15, 0, 0]}}

        #cls.maxDiff = None

    def test_turtlebot_topic_dict_key_result(self):
        #probs_logger.debug(sorted(self.topic_name_dict.keys(), key=lambda x: x[::-1]))
        self.assertEqual(sorted(self.topic_name_dict.keys(), key=lambda x: x[::-1]), \
                         sorted(self.expected_topic_name_dict.keys(), key=lambda x: x[::-1]))


    def test_turtlebot_topic_dict_full_result(self):
        #probs_logger.debug(findDiff(self.expected_topic_name_dict, self.topic_name_dict))
        self.assertEqual(findDiff(self.expected_topic_name_dict, self.topic_name_dict), False)

    def test_turtlebot_twist_list_same_length_result(self):
        self.assertEqual(len(self.topic_name_list), len(self.expected_topic_name_list))

    def test_turtlebot_twist_list_sorted_result(self):
        self.assertEqual(sorted(self.topic_name_list), sorted(self.expected_topic_name_list))

    def test_turtlebot_twist_prob_result(self):
        self.assertEqual(self.prob_dict, self.expected_prob_dict)

    def test_turtlebot_twist_msg_fields_result(self):
        #probs_logger.debug(findDiff(self.msg_fields,self.expected_msg_fields))
        self.assertEqual(findDiff(self.msg_fields,self.expected_msg_fields), False)

    def test_update_dict_value_list(self):
        target_dict = {'A':{'a':{'aa':[1,2,3]}},'B':{'b':{'bb':[11,12,13]}}}
        update_dict = {'A':{'a':{'aa':[1,2,3],'ac':[1,2,3]}},'B':{'d':{'dc':[21,22,23]}}}
        update_dict_value_list(target_dict, update_dict)
        expected_target_dict = {'A': {'a': {'aa': [1, 2, 3, 1, 2, 3], 'ac': [1, 2, 3]}}, \
                                'B': {'b': {'bb': [11, 12, 13]}, 'd': {'dc': [21, 22, 23]}}}
        self.assertEqual(target_dict, expected_target_dict)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="find topics with ast")
    file_path = ROS_CODEBASES_DIR+'/turtlebot/processed/'

    parser.add_argument('--file_path', type=str, help='Specify directory of files.', nargs='?', const=file_path, default=file_path)
    parser.add_argument('--msg_type', type=str, help='Specify message type', action='append', nargs='?', default=[])
    parser.add_argument('--test', type=str, help='Specify True for unit test version. False otherwise', nargs='?', const='false', default='false')

    args, unknown = parser.parse_known_args()
    if not args.msg_type:
      args.msg_type = ['geometry_msgs.msg.Twist','Twist']

    if args.test.lower() == 'true':
        print "============================================\nTest Result: Comparison with previous version\n============================================"
        test_suite = unittest.TestLoader().loadTestsFromTestCase(TestMethods)
        unittest.TextTestRunner(verbosity=2).run(test_suite)
    else:
        call_name = 'rospy.Subscriber'
        topic_name_list = retrieve_topic_names(args.file_path, args.msg_type, call_name=call_name)
        msg_fields = parameters_in_file.retrieve_parameters(args.file_path, args.msg_type)
        print "File Path: {0}".format(file_path)
        print "============\nParameters\n============"
        print "Twist Data:{0}".format(msg_fields)

        print "msg_type: {0}".format(args.msg_type)
        print 'topic_name_list:{0}'.format(topic_name_list)
        print "==================\nTopic Distribution\n=================="
        prob_dict = find_topic_name_distribution(topic_name_list)
        print "Probability dictionary: {0}".format(prob_dict)


        # changed approach.py in turtlebot
        target_topic_dict = get_best_topic_match_for_all_msg_types(file_path, call_name=call_name)
        print "==================\nTarget Topic Dictionary\n=================="
        for key in sorted(target_topic_dict, key=lambda x: x[::-1]):
            print key + " : "+ target_topic_dict[key]


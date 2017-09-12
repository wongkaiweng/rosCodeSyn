import argparse
import os
import pandas as pd
import sys
import getpass
import logging
import re
import itertools
import unittest

import topics_in_file
import parameters_in_file
import logging_config
test_logger = logging.getLogger("test_logger")

def retrieve_topic_names(file_path, msg_type_list, remove_slash=True):
    topic_name_dict = retrieve_all_topics(file_path, remove_slash)
    return retrieve_topic_names_from_topic_dict(topic_name_dict, msg_type_list)

def retrieve_topic_names_from_topic_dict(topic_name_dict, msg_type_list):
    # return only the msg_types of interest
    topics_for_msg_type_list = [topic_name_dict[msg_type] for msg_type in msg_type_list if msg_type in topic_name_dict.keys()]
    return [i for i in itertools.chain.from_iterable(topics_for_msg_type_list)]

def retrieve_all_topics(file_path, remove_slash=True):
    """
    Retrieve topic names based on a msg type
    file_path: path to python files (not recursive)
    msg_type: type of msg
    remove_slash: remove leading slash of topics if true
    """
    topic_name_dict = {}
    for root, dirs, files in os.walk(file_path):
        # traverse all directories
        for directory in dirs:
            update_dict_value_list(topic_name_dict, retrieve_all_topics(os.path.join(root, directory), remove_slash))

        # find all files
        for file in files:
            if file.endswith(".py"):
                # get all topics in file
                file_topic_name_dict = topics_in_file.get_topics_in_file(os.path.join(root, file))

                # remove leading slash
                if remove_slash:
                    for key, item_list in file_topic_name_dict.iteritems():
                        file_topic_name_dict[key] = [re.sub("^/|/$", "", s) for s in item_list]

                # combine dicts and store new fields
                update_dict_value_list(topic_name_dict, file_topic_name_dict)

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

def retrieve_parameters(file_path, msg_type, msg_fields={}):
    """
    Retrieve parameters based on a msg type
    file_path: path to python files (not recursive)
    msg_type: type of msg
    """
    for root, dirs, files in os.walk(file_path):
        # traverse all directories
        for directory in dirs:
            ############# TODO: check that we retrieve all parameters
            msg_fields = retrieve_parameters(os.path.join(root, directory), msg_type, msg_fields)

        # find all files
        for file in files:
            if file.endswith(".py"):
                #find parameters with msg_type
                msg_fields = parameters_in_file.get_parameters_in_file(os.path.join(root, file), msg_type, msg_fields)

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


class TestMethods(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """ get_some_resource() is slow, to avoid calling it for each test use setUpClass()
            and store the result as class variable
        """
        super(TestMethods, cls).setUpClass()
        if sys.platform == 'darwin': # Mac OS
            cls.file_path = '/Users/wongkaiweng/Dropbox/ros_examples/turtlebot/processed/'
        else: # linux /windows?
            cls.file_path = '/home/{0}/ros_examples/turtlebot/processed/'.format(getpass.getuser())

        cls.msg_type_list = ['geometry_msgs.msg.Twist','Twist']

        cls.topic_name_dict = retrieve_all_topics(cls.file_path)
        cls.expected_topic_name_dict = {'Sound': ['mobile_base/commands/sound', 'mobile_base/commands/sound', 'mobile_base/commands/sound'], \
            'DigitalOutput': ['mobile_base/commands/digital_output', 'mobile_base/commands/digital_output'], \
            'PoseStamped': ['move_base_simple/goal', 'move_base_simple/goal', 'move_base_simple/goal', 'move_base_simple/goal'], \
            'String': ['range_data', 'range_data'], 'SoundRequest': ['robotsound', 'robotsound'], \
            'UInt8': ['command_input'], \
            'Twist': ['cmd_vel_mux/input/navi', 'cmd_vel_mux/input/teleop', 'cmd_vel_mux/input/teleop', 'cmd_vel_mux/input/navi', \
                'cmd_vel_mux/input/teleop', 'cmd_vel_mux/input/teleop', 'mobile_base/commands/velocity', 'cmd_vel_mux/input/navi', \
                'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', \
                'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/teleop', 'cmd_vel_mux/input/teleop', 'mobile_base/commands/velocity', \
                'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', 'mobile_base/commands/velocity', 'cmd_vel_mux/input/navi', \
                'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi', 'cmd_vel_mux/input/navi'], \
            'Config': ['state_estimate', 'state_estimate']}

        cls.topic_name_list = retrieve_topic_names_from_topic_dict(cls.topic_name_dict, cls.msg_type_list)
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

        cls.msg_fields = retrieve_parameters(args.file_path, args.msg_type)
        cls.expected_msg_fields = {'angular':{'z': [0, 0.3, -0.15, 0, 0.2, 0, 0, -2, -2, 2, 0, 2, 1, 0, 0.2, 0.0, 0, 0, -2, 0, 2, 0, 0, 0, 0]},\
            'linear':{'x': [0.0, 0.0, 0.2, 0.2, 0.0, 0.0, 0.2, 0, -0.2, -0.2, 0, 0.2, 0.25, 0.25, 0.2, 0.0, 0.0, -0.2, 0.2, \
                            0.2, 0.2, 0, 0, -0.2, 0, 0.2, -0.1, 0.2, 0, 0.0]}}

    def test_turtlebot_topic_dict_result(self):
        self.assertEqual(self.topic_name_dict, self.expected_topic_name_dict)

    def test_turtlebot_twist_list_result(self):
        self.assertEqual(self.topic_name_list, self.expected_topic_name_list)

    def test_turtlebot_twist_prob_result(self):
        self.assertEqual(self.prob_dict, self.expected_prob_dict)

    def test_turtlebot_twist_msg_fields_result(self):
        self.assertEqual(self.msg_fields, self.expected_msg_fields)

    def test_update_dict_value_list(self):
        target_dict = {'A':{'a':{'aa':[1,2,3]}},'B':{'b':{'bb':[11,12,13]}}}
        update_dict = {'A':{'a':{'aa':[1,2,3],'ac':[1,2,3]}},'B':{'d':{'dc':[21,22,23]}}}
        update_dict_value_list(target_dict, update_dict)
        expected_target_dict = {'A': {'a': {'aa': [1, 2, 3, 1, 2, 3], 'ac': [1, 2, 3]}}, \
                                'B': {'b': {'bb': [11, 12, 13]}, 'd': {'dc': [21, 22, 23]}}}
        self.assertEqual(target_dict, expected_target_dict)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="find topics with ast")
    if sys.platform == 'darwin': # Mac OS
        #file_path = '/Users/wongkaiweng/DropboWx/ros_examples/UR5/processed/'
        file_path = '/Users/wongkaiweng/Dropbox/ros_examples/turtlebot/processed/'

    else: # linux /windows?
        file_path = '/home/{0}/ros_examples/turtlebot/processed/'.format(getpass.getuser())
        #file_path = '/home/{0}/ros_examples/turtlebot/one_file_test/'.format(getpass.getuser())

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
        topic_name_list = retrieve_topic_names(args.file_path, args.msg_type)
        msg_fields = retrieve_parameters(args.file_path, args.msg_type)
        print "============\nParameters\n============"
        print "Twist Data:{0}".format(msg_fields)

        print "msg_type: {0}".format(args.msg_type)
        print 'topic_name_list:{0}'.format(topic_name_list)
        print "==================\nTopic Distribution\n=================="
        prob_dict = find_topic_name_distribution(topic_name_list)
        print "Probability dictionary: {0}".format(prob_dict)



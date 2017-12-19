import getpass
import logging
import math
import unittest

import process_limits

import logging_config
yaml_logger = logging.getLogger("yaml_logger")

def find_robot_YAML(robot_name, version='indigo'):
    '''
    Find the path to the robot YAML and return YAML string
    '''

    # path to YAML in ROS directory
    ROS_YAML_dict = {'jackal':'/opt/ros/{0}/share/jackal_control/config/control.yaml'.format(version),\
                     'turtlebot':'/opt/ros/{0}/share/turtlebot_bringup/param/defaults/smoother.yaml'.format(version),\
                     'kobuki':'/opt/ros/{0}/share/kobuki_keyop/param/keyop_smoother.yaml'.format(version)}

    path_to_config_folder = '/home/{0}/ros_examples/config_files/yaml/'.format(getpass.getuser())
    universial_yaml_dict = {'jackal':'jackal.yaml',\
                            'turtlebot':'turtlebot.yaml',\
                            'youbot':'youbot.yaml',\
                            'kobuki':'kobuki.yaml',\
                            'nao':'nao.yaml',\
                            'sphero':'sphero.yaml'}

    # check if robot exists
    if robot_name in universial_yaml_dict.keys():
        with open(path_to_config_folder+universial_yaml_dict[robot_name], 'r') as f:
            yaml_string = f.read()
        f.closed
        return yaml_string

    elif robot_name in ROS_YAML_dict.keys():
        yaml_logger.warning("Not Implemented. Use code in check_vel_limit")
        return None

    else:
        yaml_logger.warning("Cannot find robot:{0} in the list of yaml files available".format(robot_name))

    return None

def scale_velocity_command(field, command, source_limit_dict, target_limit_dict):
    """
    This function scales velocity based on the source_limit_dict and the
    target_limit_dict
    INPUTS:
    field: e.g: ['linear','x'], always in list form
    command: current comment, e.g.: 0.2
    source_limit_dict: vel_limit_dict of the source robot
    target_limit_dict: vel_limit_dict of the target robot
    OUTPUTS:
    scaled_command: scaled command
    status: True or False, True if scaled and False otherwise
    """

    # make sure the command is a float
    command = float(command)
    scaled_command = 0.0

    # check if fields are valid
    if len(field) != 2 or \
       field[0] not in ['linear', 'angular'] or \
       field[1] not in ['x', 'y', 'z']:
        yaml_logger.warning("Invalid field format: {0}".format(field))
        return command, False

    # get limit related to the command
    source_command_info = source_limit_dict[field[0]][field[1]]
    target_command_info = target_limit_dict[field[0]][field[1]]

    yaml_logger.log(8, "We are not currently doing any conversion on " +\
                        "differential drive to holonomic drive or vice versa")

    # check if the original command is not within source limits
    if command < source_command_info['lower']:
        yaml_logger.warning('Command: {0} is smaller than lower limit:{1}.'.format(\
            command, source_command_info['lower']) + ' Capped at lower limit.')
        command = float(source_command_info['lower'])

    elif source_command_info['upper'] < command:
        yaml_logger.warning('Command: {0} is bigger than lower limit:{1}.'.format(\
            command, source_command_info['upper']) + ' Capped at upper limit.')
        command = float(source_command_info['upper'])

    # check if the original command is inside deadband
    if source_command_info['deadband'] != 'None' and \
        abs(command) < abs(source_command_info['deadband']):
        yaml_logger.warning('Command: {0} is within the deadbands:{1}.'.format(\
            command, source_command_info['upper']) + ' Set to deadband.')
        command = math.copysign(1, command)*float(source_command_info['deadband'])


    # get source and target command difference
    if command:
        source_diff = float(source_command_info['upper'])
        target_diff = float(target_command_info['upper'])
        yaml_logger.debug('Scale from Upper: {0} to {1}'.format(\
                        source_command_info['upper'], target_command_info['upper']))
    else:
        source_diff = float(abs(source_command_info['lower']))
        target_diff = float(abs(target_command_info['lower']))
        yaml_logger.debug('Scale from Lower: {0} to {1}'.format(\
                        source_command_info['lower'], target_command_info['lower']))

    if source_command_info['deadband'] != 'None':
        source_diff -= float(source_command_info['deadband'])
        command -= float(source_command_info['deadband'])
        yaml_logger.debug('Scale with source deadband: {0}'.format(source_command_info['deadband']))

    if target_command_info['deadband'] != 'None':
        target_diff -= float(target_command_info['deadband'])
        scaled_command += float(target_command_info['deadband'])
        yaml_logger.debug('Scale with target deadband: {0}'.format(target_command_info['deadband']))

    # calcaulte new command
    # (U = Upper, same for lower, C - command, D - Deadband)
    # Dt added above
    # Cs - Ds  _   Ct - Dt
    # -------  _  --------
    # Us - Ds      Ut - Dt
    scaled_command += command/source_diff*target_diff
    yaml_logger.debug("Result: ({0}, True)".format(scaled_command))

    return scaled_command, True


class TestMethods(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """ get_some_resource() is slow, to avoid calling it for each test use setUpClass()
            and store the result as class variable
        """
        super(TestMethods, cls).setUpClass()
        cls.sphero_vel_lim_dict =  process_limits.make_vel_limit_dic_from_yaml_text(find_robot_YAML('sphero'))
        cls.turtlebot_vel_lim_dict =  process_limits.make_vel_limit_dic_from_yaml_text(find_robot_YAML('turtlebot'))

    def test_turtlebot_to_sphero_normal(self):
        self.assertEqual(scale_velocity_command(['linear', 'x'], 0.2, self.turtlebot_vel_lim_dict, self.sphero_vel_lim_dict), (42.5,True))

    def test_sphero_to_turtlebot_normal(self):
        self.assertEqual(scale_velocity_command(['linear', 'x'], 42.5, self.sphero_vel_lim_dict, self.turtlebot_vel_lim_dict), (0.2, True))

    def test_sphero_above_upper_limit_to_turtlbot(self):
        self.assertEqual(scale_velocity_command(['linear', 'x'], 90, self.sphero_vel_lim_dict, self.turtlebot_vel_lim_dict), (0.8, True))

    def test_sphero_below_deadband_to_turtlbot(self):
        self.assertEqual(scale_velocity_command(['linear', 'x'], 20, self.sphero_vel_lim_dict, self.turtlebot_vel_lim_dict), (0, True))



if __name__ == "__main__":
    UNIT_TEST = True

    if UNIT_TEST:
        test_suite = unittest.TestLoader().loadTestsFromTestCase(TestMethods)
        unittest.TextTestRunner(verbosity=2).run(test_suite)
    else:
        sphero_vel_lim_dict =  process_limits.make_vel_limit_dic_from_yaml_text(find_robot_YAML('sphero'))
        turtlebot_vel_lim_dict =  process_limits.make_vel_limit_dic_from_yaml_text(find_robot_YAML('turtlebot'))

        # turtlebot to sphero
        yaml_logger.debug("Turtlebot to Sphero")
        yaml_logger.debug(scale_velocity_command(['linear', 'x'], 0.2, turtlebot_vel_lim_dict, sphero_vel_lim_dict))

        # sphero to turtlebot
        yaml_logger.debug("Sphero to Turtlebot")
        yaml_logger.debug(scale_velocity_command(['linear', 'x'], 42.5, sphero_vel_lim_dict, turtlebot_vel_lim_dict))

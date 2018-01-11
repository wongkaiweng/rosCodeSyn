import getpass
import sys

if sys.platform == 'darwin': # Mac OS
    ROS_CODEBASES_DIR = '/Users/{0}/Dropbox/ros_examples'.format(getpass.getuser())
else: # linux /windows?
    ROS_CODEBASES_DIR = '/home/{0}/ros_examples'.format(getpass.getuser())
import sys
from urdf_parser_py.urdf import URDF
import argparse

import logging
import logging_config
limits_logger = logging.getLogger("limits_logger")

#extract the limits of all the robot joints from the URDF file into a dictionary
def make_dic_from_file(file):
	robot = URDF.from_xml_file(file)
	dic={}

	for joint in robot.joints:
		#add joints to dictionary dic with key=joint.name, value=joint.limit
		dic[joint.name]=joint.limit
	return dic

#extract the limits of all the robot joints from the robot object loaded from URDF file into a dictionary
def make_dic_from_robObj(robot):
	dic={}

	for joint in robot.joints:
		if joint.limit and (joint.limit.lower or joint.limit.upper):
			#add joints to dictionary dic with key=joint.name, value=joint.limit
			dic[joint.name]={'lower':joint.limit.lower, 'upper': joint.limit.upper}

		elif joint.joint_type == 'fixed':
			limits_logger.log(8, 'This is a fixed joint: {0}'.format(joint.name))
		else: # dummy value
			limits_logger.log(8, "Joint {0} is not a fixed point nor has lower or upper limits.".format(joint))
		#	limits_logger.warning("Adding dummy value for limits of joint: {0}".format(joint.name))
		#	dic[joint.name]={'lower':-6.28, 'upper': 6.28}

	return dic





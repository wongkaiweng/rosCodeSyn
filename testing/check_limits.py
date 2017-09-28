

# Import packages
import sys
from urdf_parser_py.urdf import URDF
import argparse

#extract the limits of all the robot joints from the URDF file into a dictionary
def make_dic(file):
	robot = URDF.from_xml_file(file)
	
	dic={}

	for joint in robot.joints:
		#add joints to dictionary dic with key=joint.name, value=joint.limit
		dic[joint.name]=joint.limit
	return dic

#check if the input command is within the upper and lower limits of the joint
#returns True if the command is within the limits and False if it is not
def check_limits(dic,comm,name):
	joint=dic[name] #get the joint from the dictionary

	comm=float(comm) #convert from string

	if comm>joint.lower and comm<joint.upper:
		return True
	else:
		return False


if __name__ == '__main__':
	#parse input arguments
	parser=argparse.ArgumentParser(description="find input parameters")
	parser.add_argument('--file',type=str,help='Specify the name of the URDF or YAML file for the robot',nargs='?')
	parser.add_argument('--comm',type=str,help='Specify the desired command to be sent to the robot',nargs='?')
	parser.add_argument('--name',type=str,help='Specify the name of the joint to which the command is to be sent',nargs='?')
	args=parser.parse_args()

	#example--sucess
	file="jaco_arm.urdf"
	comm="3"
	name="jaco_joint_base"
	print "Sending command of magnitude 3 to jaco_joint_base joint of jaco_arm."

	#create a dictionary for the robot
	dic=make_dic(file)
	#check the desired command
	check=check_limits(dic,comm,name)
	if check==True:
		print "RESULT: The command is within the limits."
	elif check==False:
		print "RESULT: The command cannot be sent to the given joint."

	#example--failure
	file="jaco_arm.urdf"
	comm="10"
	name="jaco_joint_base"
	print "Sending command of magnitude 10 to jaco_joint_base joint of jaco_arm."

	#create a dictionary for the robot
	dic=make_dic(file)
	#check the desired command
	check=check_limits(dic,comm,name)
	if check==True:
		print "RESULT: The command is within the limits."
	elif check==False:
		print "RESULT: The command cannot be sent to the given joint."




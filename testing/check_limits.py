

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
def check_limits(file,comm,name):
	dic=make_dic(file)
	joint=dic[name] #get the joint from the dictionary

	comm=float(comm) #convert from string

	if comm>joint.lower and comm<joint.upper:
		print "The command is within the limits."
		return True
	else:
		print "The command is not within the limits and cannot be sent to the joint." #update to include name of joint
		return False

def run_examples():
	#example--sucess with jaco_arm
	file="jaco_arm.urdf"
	comm="3" #point.positions
	name="jaco_joint_base" #joint.Cms.joint_names
	print "Sending command of magnitude 3 to jaco_joint_base joint of jaco_arm."
	check=check_limits(file,comm,name)
	

	#example--failure with jaco_arm
	file="jaco_arm.urdf"
	comm="10"
	name="jaco_joint_base"
	print "Sending command of magnitude 10 to jaco_joint_base joint of jaco_arm."
	check=check_limits(file,comm,name)

	#example--success with ur5
	file="ur5.urdf"
	comm="3"
	name="shoulder_pan_joint"
	print "Sending command of magnitude 3 to shoulder_pan_joint joint of ur5."
	check=check_limits(file,comm,name)


if __name__ == '__main__':
	#parse input arguments
	parser=argparse.ArgumentParser(description="find input parameters")
	parser.add_argument('file',type=str,help='Specify the name of the URDF or YAML file for the robot',nargs='?')
	parser.add_argument('comm',type=str,help='Specify the desired command to be sent to the robot',nargs='?')
	parser.add_argument('name',type=str,help='Specify the name of the joint to which the command is to be sent',nargs='?')
	args=parser.parse_args()

	check=check_limits(args.file,args.comm,args.name)

	#run_examples()

	




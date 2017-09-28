

# Import packages
import sys
from urdf_parser_py.urdf import URDF
import argparse

#parse input parameters
file=sys.argv[1] #URDF file with parameters of the robot
comm=sys.argv[2] #command to be checked
name=sys.argv[3] #name of joint command is being sent to

#extract the limits of all the robot joints from the URDF file into a dictionary
def make_dic(file):
	robot = URDF.from_xml_file(file)
	
	dic={}

	for joint in robot.joints:
		#add joints to dictionary dic with key=joint.name, value=joint.limit
		dic[joint.name]=joint.limit

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
	#parser=argparse.ArgumentParser(description="find input parameters")
	#parser.add_argument('--file',type=str,help='Specify the name of the URDF or YAML file for the robot',nargs='?')
	#parser.add_argument('--comm',type=str,help='Specify the desired command to be sent to the robot',nargs='?')
	#parser.add_argument('--name',type=str,help='Specify the name of the joint to which the command is to be sent',nargs='?')
	
	#create a dictionary for the robot--ADD if yaml,elif urdf
	dic=make_dic(file)
	#check the desired command
	check=check_limits(dic,comm,name)
	print check

	#add example

import rospy
import math
import threading
import actionlib
import naoqi_bridge_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg
try:
    # Python2
    import Tkinter as tk
except ImportError:
    # Python3
    import tkinter as tk

import std_msgs.msg


# Nao Launch
#roslaunch nao_bringup nao_full_py.launch nao_ip:=localhost:45972 roscore_ip:=localhost

# Pepper Launch (in Example folder)
#roslaunch pepper_full_py.launch nao_ip:=localhost:36161 roscore_ip:=localhost


class TkSimpleButton(object):
    def __init__(self, node_name, node_publish_topic, init_value=False):
        self.node_name = node_name
        self.init_value = init_value

        # start button
        self.create_button(node_name, init_value)

    def create_button(self, node_name, init_value):
        print ("{0} Init_value: {1}".format(node_name, init_value))
        background_color = "green" if init_value else "red"
        button_name = node_name+"-True" if init_value else node_name+"-false"
        self.button = tk.Button(text=button_name, width=20, command=self.toggle, \
                                activebackground=background_color, bg=background_color)
        self.button.pack()

    def toggle(self):
        '''
        use
        self.button.config('text')[-1]
        to get the present state of the toggle button
        '''
        if self.button.config('activebackground')[-1] == "green":
            self.button.config(text=self.node_name+'-False', activebackground="red", bg = "red")
            print ("Sensor {0} turned False.".format(self.node_name))
        else:
            self.button.config(text=self.node_name+'-True', activebackground="green", bg = "green")
            print ("Sensor {0} turned True.".format(self.node_name))

    def button_state(self):
        return True if self.button.config('activebackground')[-1] == "green" else False


class TkThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        title = 'Buttons'
        print ("Starting sensor {0} Tk button mainloop".format(title))
        self.root = tk.Tk()
        self.root.wm_title(title)

        # start button
        self.button_move_forward = TkSimpleButton('move_forward', '/button_move_forward', False)
        self.button_rotate = TkSimpleButton('move_rotate', '/button_rotate', False)
        self.button_hi = TkSimpleButton('move_hi', '/button_hi', False)

        # register shutdown hook
        rospy.on_shutdown(self.quit)


        self.root.mainloop()


    def quit(self):
        self.root.destroy()

# for pepper
JOINT_NAMES = ['RShoulderPitch', 'RShoulderRoll','RElbowYaw', 'RElbowRoll']
REST_POSE = [1.637, -0.150, 1.23, 0.513]
WAVE_LEFT = [-1.047, -0.154, 0.124, 0.314]
WAVE_RIGHT = [-1.047, -0.859, 0.124, 0.314]

action_client = None

def wave():
    g = naoqi_bridge_msgs.msg.JointTrajectoryGoal()
    g.trajectory = trajectory_msgs.msg.JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    # start with rest pos
    g.trajectory.points = [
            trajectory_msgs.msg.JointTrajectoryPoint(positions=REST_POSE, time_from_start=rospy.Duration(1.0)),
            trajectory_msgs.msg.JointTrajectoryPoint(positions=WAVE_LEFT, time_from_start=rospy.Duration(3.0)),
            trajectory_msgs.msg.JointTrajectoryPoint(positions=WAVE_RIGHT, time_from_start=rospy.Duration(4.0)),
            trajectory_msgs.msg.JointTrajectoryPoint(positions=WAVE_LEFT, time_from_start=rospy.Duration(6.0)),
            trajectory_msgs.msg.JointTrajectoryPoint(positions=WAVE_RIGHT, time_from_start=rospy.Duration(7.0)),
            trajectory_msgs.msg.JointTrajectoryPoint(positions=REST_POSE, time_from_start=rospy.Duration(8.0))]

    action_client.send_goal(g)
    try:
        action_client.wait_for_result()
    except KeyboardInterrupt:
        action_client.cancel_goal()
        raise


def rotate():
    vel_msg = geometry_msgs.msg.Twist()
    vel_msg.angular.z = 0.5
    vel_pub.publish(vel_msg)

def move_forward():
    vel_msg = geometry_msgs.msg.Twist()
    vel_msg.linear.x = 0.5
    vel_pub.publish(vel_msg)

def stop():
    vel_msg = geometry_msgs.msg.Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    vel_pub.publish(vel_msg)


class FuncThread(threading.Thread):
    def __init__(self, button, func):
        threading.Thread.__init__(self)
        self.button = button
        self.func = func
        possibles = globals().copy()
        possibles.update(locals())
        self.method = possibles.get(self.func)
        if not self.method:
             raise NotImplementedError("Method %s not implemented" % method_name)

        self.start()
        print 'Function {0} started!'.format(self.func)

    def run(self):
        while not rospy.is_shutdown():
            if self.button.button_state():
                self.method()
            rospy.sleep(0.1)

class FuncNotThread(threading.Thread):
    def __init__(self, button_list, func):
        threading.Thread.__init__(self)
        self.button_list = button_list
        self.func = func
        possibles = globals().copy()
        possibles.update(locals())
        self.method = possibles.get(self.func)
        if not self.method:
             raise NotImplementedError("Method %s not implemented" % method_name)

        self.start()
        print 'Function {0} started!'.format(self.func)

    def run(self):
        while not rospy.is_shutdown():
            if not any([x.button_state() for x in self.button_list]):
                self.method()
            rospy.sleep(0.1)


def main():
    global action_client
    global vel_pub
    try:
        rospy.init_node("test_wave", anonymous=True, disable_signals=True)

        action_client = actionlib.SimpleActionClient('/pepper_robot/pose/joint_trajectory', naoqi_bridge_msgs.msg.JointTrajectoryAction)
        vel_pub = rospy.Publisher("/cmd_vel", geometry_msgs.msg.Twist, queue_size=10)

        print "Waiting for server..."
        action_client.wait_for_server()
        print "Connected to server"

        thread = TkThread()
        thread.start()

        import time
        time.sleep(2)
        button_move_forward = thread.button_move_forward
        button_rotate  = thread.button_rotate
        button_hi = thread.button_hi

        # conditions
        func_rotate = FuncThread(button_rotate,'rotate')
        func_move_forward = FuncThread(button_move_forward,'move_forward')
        func_wave = FuncThread(button_hi,'wave')

        # else:
        func_stop = FuncNotThread([button_rotate,button_move_forward,button_hi],'stop')

        rospy.spin()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
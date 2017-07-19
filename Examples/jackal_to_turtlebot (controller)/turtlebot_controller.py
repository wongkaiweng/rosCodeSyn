#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

import sys,tty,termios


class _Getch:
    def __call__(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(3)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class Jackal(object): #self missing
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
        rospy.init_node('controller', anonymous=True)
        self.rate = rospy.Rate(10)

    def move(self, twist):
        counter = 0

        while not rospy.is_shutdown() and counter < 1 :
            counter += 1
            move_forward_twist = twist
            self.pub.publish(move_forward_twist)
            self.rate.sleep()

    def move_forward(self):
        self.move(Twist(Vector3(0.5, 0, 0), Vector3(0, 0 ,0)))

    def move_backward(self):
        self.move(Twist(Vector3(-0.5, 0, 0), Vector3(0, 0 ,0)))

    def turn_right(self):
        self.move(Twist(Vector3(0, 0, 0), Vector3(0, 0 ,-0.5)))

    def turn_left(self):
        self.move(Twist(Vector3(0, 0, 0), Vector3(0, 0 ,0.5)))


def get():
    inkey = _Getch()
    jackal = Jackal()
    while(1):
        k=inkey()
        if k!='':break
    if k=='\x1b[A':
        print "forward"
        jackal.move_forward()
    elif k=='\x1b[B':
        print "backward"
        jackal.move_backward()
    elif k=='\x1b[C':
        print "right"
        jackal.turn_right()
    elif k=='\x1b[D':
        print "left"
        jackal.turn_left()
    else:
        quit()


if __name__ == '__main__':
    while True:
        get()


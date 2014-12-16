from command import Command
from utils import *

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState

class Joint2Dof(Command):
    """ Control a 2 DoF joint"""
    
    def __init__(self, name, topic_name, x_range=None, y_range=None, clamp_below=0.01, additive=False):
        super(Joint2Dof, self).__init__(name)
        self.pub = rospy.Publisher(topic_name, JointState, queue_size=10)
        self.x_range = x_range or [-1, 1]
        self.y_range = y_range or [-1, 1]
        self.clamp_below = clamp_below
        self.additive = additive
        self.last_x = 0
        self.last_y = 0
        self.pub.publish(self.makemsg(0, 0))

    def makemsg(self, x, y):
        joint_state = JointState()
        if self.additive:
            x += self.last_x
            y += self.last_y
        v1 = from_normal_to_interval(self.x_range[0], self.x_range[1], x)
        v2 = from_normal_to_interval(self.y_range[0], self.y_range[1], y)
        joint_state.position = (v1, v2)
        return joint_state

    def oncallback(self, cmd_data):
        x, y = cmd_data['x'], cmd_data['y']
        x, y = clamp_center(x, self.clamp_below), clamp_center(y, self.clamp_below)
        msg = self.makemsg(x, y)
        self.last_x, self.last_y = msg.position
        self.pub.publish(msg)


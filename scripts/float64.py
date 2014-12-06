from command import Command
from utils import *

import rospy
from std_msgs.msg import Float64 as F64
from geometry_msgs.msg import Twist
from dynamixel_msgs.msg import JointState

class Float64(Command):
    """ Control a Float"""
    
    def __init__(self, name, topic_name, joint_state_topic_name,
            x_range, clamp_below=0.01, additive=False):
        super(Float64, self).__init__(name)
        self.enable_publish = False
        self.pub = rospy.Publisher(topic_name, F64, queue_size=10)
        self.sub = rospy.Subscriber(joint_state_topic_name, JointState, self.on_joint_update)
        self.x_last = 0
        self.x_range = x_range or [-1, 1]
        self.additive = additive
        self.clamp_below = clamp_below

    def makemsg(self, x):
        f = F64()
        if self.additive:
            x = x + self.x_last
        v1 = from_normal_to_interval(self.x_range[0], self.x_range[1], x)
        f.data = (v1)
        return f

    def on_joint_update(self, f):
        self.x_last = f.current_pos
        self.enable_publish = True

    def oncallback(self, cmd_data):
        x = cmd_data['x']
        x = clamp_center(x, self.clamp_below)
        msg = self.makemsg(x)
        if self.enable_publish:
            self.pub.publish(msg)


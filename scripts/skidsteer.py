from command import Command
from utils import *

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState

class SkidSteer(Command):
    """ Control a model with a SkidSteer plugin """
    
    def __init__(self, name, skid_steer_topic, speed=5, rot_multiplier=1, clamp_below=0.1):
        """ Args:
        speed: max speed
        rot_multiplier: rotation multiplier
        """
        super(SkidSteer, self).__init__(name)
        self.pub = rospy.Publisher(skid_steer_topic, Twist, queue_size=10)
        self.speed = speed
        self.rot_multiplier = rot_multiplier
        self.clamp_below = clamp_below

    def evaluate_speed_rot(self, x, y):
        rot = x*self.rot_multiplier
        speed = y*self.speed 
        return (speed, rot)


    def makemsg(self, speed, rot):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = rot
        return twist

    def oncallback(self, cmd_data):
        x, y = cmd_data['x'], cmd_data['y']
        x, y = clamp_center(x, self.clamp_below), clamp_center(y, self.clamp_below)
        speed, rot = self.evaluate_speed_rot(x, y)
        msg = self.makemsg(speed, rot)
        self.pub.publish(msg)

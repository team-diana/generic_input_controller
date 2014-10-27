#!/usr/bin/env python
import rospy
import yaml

from std_msgs.msg import String
from optparse import OptionParser
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState

def clamp_center(v, below=0.2):
    """ remove values around 0 """
    if abs(v) < below:
        return 0
    else:
        return v

def from_normal_to_interval(min_v, max_v, v):
    """ return a new value proportional to the interval (max_v - min_v) given value v"""
    return (max_v - min_v)/2 * v + (min_v + max_v)/2

class Command(object):
    def __init__(self, name):
        self.name = name

    def oncallback(self, joymsg):
        pass

class SkidSteer(Command):
    """ Control a model with a SkidSteer plugin """
    
    def __init__(self, name, x_axis_index, y_axis_index, skid_steer_topic, speed=5, rot_multiplier=1, clamp_below=0.1):
        """ Args:
        x_axis_index: index of the axis that will control the speed
        y_axis_index: index of the axis that will control the rotation
        speed: max speed
        rot_multiplier: rotation multiplier
        """
        super(SkidSteer, self).__init__(name)
        self.pub = rospy.Publisher(skid_steer_topic, Twist, queue_size=10)
        self.x_index = x_axis_index
        self.y_index = y_axis_index
        self.speed = speed
        self.rot_multiplier = rot_multiplier
        self.clamp_below = clamp_below

    def evaluate_speed_rot(self, x, y):
        speed = x*self.speed 
        rot = y*self.rot_multiplier
        return (speed, rot)


    def makemsg(self, speed, rot):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = rot
        return twist

    def oncallback(self, joymsg):
        x, y = joymsg.axes[self.x_index], joymsg.axes[self.y_index]
        x, y = clamp_center(x, self.clamp_below), clamp_center(y, self.clamp_below)
        speed, rot = self.evaluate_speed_rot(x, y)
        msg = self.makemsg(speed, rot)
        self.pub.publish(msg)

class Joint2Dof(Command):
    """ Control a 2 DoF joint"""
    
    def __init__(self, name, x_axis_index, y_axis_index, topic_name, x_range=None, y_range=None, clamp_below=0.01):
        """ Args:
        x_axis_index: index of the axis that will control the first revolute join
        y_axis_index: index of the axis that will control the second revolute join
        """
        super(Joint2Dof, self).__init__(name)
        self.pub = rospy.Publisher(topic_name, JointState, queue_size=10)
        self.x_index = x_axis_index
        self.y_index = y_axis_index
        self.x_range = x_range or [-1, 1]
        self.y_range = y_range or [-1, 1]
        self.clamp_below = clamp_below

    def makemsg(self, x, y):
        joint_state = JointState()
        v1 = from_normal_to_interval(self.x_range[0], self.x_range[1], x)
        v2 = from_normal_to_interval(self.y_range[0], self.y_range[1], y)
        joint_state.position = (v1, v2)
        return joint_state

    def oncallback(self, joymsg):
        x, y = joymsg.axes[self.x_index], joymsg.axes[self.y_index]
        x, y = clamp_center(x, self.clamp_below), clamp_center(y, self.clamp_below)
        msg = self.makemsg(x, y)
        self.pub.publish(msg)


class Controller():
    def __init__(self, commands):
        rospy.init_node('controller')
        rospy.Subscriber('joy', Joy, self.sixaxis_callback) 
        self.commands = commands

    def run(self):
        rospy.spin()

    def sixaxis_callback(self, joy):
        [com.oncallback(joy) for com in self.commands]

def read_commands(commands_file, commands_dict):
    with open(commands_file, "r") as f:
        y = yaml.load(f)
        commands = []
        for item in y.iteritems():
            name = item[0]
            values = item[1]
            cmd_cls = commands_dict[values['type']]
            values.pop('type')
            values['name'] = name
            cmd = cmd_cls(**values)
            commands.append(cmd)
        return commands
            

if __name__ == '__main__':
    commands_dict = { "skidsteer": SkidSteer, "joint2dof": Joint2Dof }
    try:
        command_file_name="./commands.yaml"
        commands = read_commands(command_file_name, commands_dict)
        controller = Controller(commands)
        controller.run()
    except rospy.ROSInterruptException:
        pass

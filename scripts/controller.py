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

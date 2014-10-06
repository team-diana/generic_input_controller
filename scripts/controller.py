#!/usr/bin/env python
import rospy
import yaml

from std_msgs.msg import String
from optparse import OptionParser
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def clamp_center(v):
    if abs(v) < 0.2:
        return 0
    else:
        return v

class Command(object):
    def __init__(self, name):
        self.name = name

    def oncallback(self, joymsg):
        pass

class SkidSteer(Command):
    """ Control a model with a SkidSteer plugin """
    
    def __init__(self, name, x_axis_index, y_axis_index, skid_steer_topic, speed=5, rot_multiplier=1):
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
        x, y = clamp_center(x), clamp_center(y)
        speed, rot = self.evaluate_speed_rot(x, y)
        msg = self.makemsg(speed, rot)
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
    commands_dict = { "skidsteer": SkidSteer }
    try:
        command_file_name="./commands.yaml"
        commands = read_commands(command_file_name, commands_dict)
        controller = Controller(commands)
        controller.run()
    except rospy.ROSInterruptException:
        pass

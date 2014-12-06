#!/usr/bin/env python

"""controller.py
A configurable node that can command many ros topics using different kind of inputs.

Usage:
  controller.py [--config=FILE_NAME] 

Options:
  -h --help     Show this screen.
  --config=FILE_NAME        Uses the configuration written in FILE_NAME (must be a .yaml file)
"""

from __future__ import print_function
try:
    from docopt import docopt
    import pygame
    from pygame.key import *
except ImportError, err:
    module_name = err.args[0].split()[-1]
    print("Some of the needed modules were missing (at least `%s')." % module_name)
    print("Please check that all dependencies are correctly installed.")
    print("To do so, it is suggested to use the command:")
    print("     $ sudo pip install docopt")
    exit()

import rospy
import yaml
import os
import sys

from command import Command
from skidsteer import SkidSteer
from float64 import Float64
from joint2dof import Joint2Dof

from commander import Commander
from joycommander import JoyJoystickCommander
from keyboardcommander import Keyboard2AxisCommander

from std_msgs.msg import String
from optparse import OptionParser
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState

class Controller():
    def __init__(self, commanders):
        rospy.init_node('controller')
        rospy.Subscriber('joy', Joy, self.joy_callback) 
        self.joy_commanders = list(filter(lambda c: c.source() == "joystick", commanders))
        self.keyboard_commanders = list(filter(lambda c: c.source() == "keyboard", commanders))

        pygame.init()

        size = width, height = 320, 240
        speed = [2, 2]
        black = 0, 0, 0
        screen = pygame.display.set_mode(size)

    def run(self):
        self.running = True
        while self.running:
            keys = pygame.key.get_pressed()
            if keys[pygame.K_ESCAPE] or keys[pygame.K_q]:
                print("\nExiting...\n")
                self.running = False
            else:
                [cmder.oncallback(keys) for cmder in self.keyboard_commanders]
            pygame.event.pump()

    def joy_callback(self, joy):
        [cmder.oncallback(joy) for cmder in self.joy_commanders]

def read_commanders(commands_file, commands_dict, commanders_dict):
    if not os.path.exists(commands_file):
        print("Error: {} does not exists".format(commands_file))
        sys.exit(-1)
    with open(commands_file, "r") as f:
        y = yaml.load(f)
        cmders = []
        for item in y.iteritems():
            name = item[0]
            values = item[1]
            cmd_cls = commands_dict[values['type']]
            values.pop('type')

            source = values['source']
            cmder_name = source.keys()[0]
            cmder_values = source.values()[0]
            cmder_type = cmder_values['type']
            print("instantiating " + str(cmder_type))
            cmder_cls = commanders_dict[cmder_type]
            cmder_values.pop('type')
            values.pop('source')

            values['name'] = name
            cmd = cmd_cls(**values)
            cmder = cmder_cls(**cmder_values)
            cmder.set_command(cmd)

            cmders.append(cmder)
        return cmders
            

if __name__ == '__main__':
    opts = docopt(__doc__)
    commands_dict = { "skidsteer": SkidSteer, "joint2dof": Joint2Dof, "float64": Float64 }
    commanders_dict = { "joyjoystick": JoyJoystickCommander, "keyboard2axis": Keyboard2AxisCommander }
    try:
        given_config_file = opts['--config'] 
        command_file_name = given_config_file or "./commands.yaml"
        print("reading configuration from: ", command_file_name)
        commanders = read_commanders(command_file_name, commands_dict, commanders_dict)
        controller = Controller(commanders)
        controller.run()
    except rospy.ROSInterruptException:
        pass

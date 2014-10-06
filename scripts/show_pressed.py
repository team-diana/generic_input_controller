#!/usr/bin/env python
import rospy
import sys, getopt
import datetime
import time
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from optparse import OptionParser

def clear_screen():
    print(chr(27) + "[2J")

class CommandsPrinter():
    def __init__(self):
        rospy.init_node('commands_printer')
        rospy.Subscriber('joy', Joy, self.sixaxisCallback) 

    def run(self):
        rospy.spin()

    def make_subarray(self, array, set_cards):
        subarrays = []
        tail = array
        for card in set_cards:
            if len(tail) < 1:
                break
            sub, tail = tail[0:card], tail[card:]
            subarrays.append(sub)
        if len(tail) > 0:
            subarrays.append(tail)
        return subarrays

    def print_set(self, array, set_cards, formatfun):
        subarrays = self.make_subarray(array, set_cards)
        i=0
        for sub in subarrays:
            msg = ""
            for value in sub:
                msg = msg + formatfun(value)
            print("Set %2d: %s" % (i, msg)) 
            i = i+1


    def sixaxisCallback(self, joy):
        axes_set_cards=[2, 2, 4, 4, 4, 4, 4, 4]
        button_set_cards=[4, 4, 4, 4, 1]
        clear_screen()
        print("sixaxis callback:")
        print("axis:")
        self.print_set(joy.axes, axes_set_cards, lambda x: ("  %4f" % x))
        print("buttons:")
        self.print_set(joy.buttons, button_set_cards, lambda x: ("  %d" % x))

if __name__ == '__main__':
    try:
        commandPrinter = CommandsPrinter()
        commandPrinter.run()
    except rospy.ROSInterruptException:
        pass

from commander import Commander
from sensor_msgs.msg import Joy, JointState
import pygame
from pygame import *

class RLUT(): 
    reverse_key_lut = None

    @staticmethod
    def build_reverse_key_lut():
        kv = [(pygame.key.name(k), k) for k in range(1, 500)]
        kv = filter(lambda x: x[0] != 'unknown key', kv)
        return dict(kv)

    @staticmethod
    def is_pressed(keys, ch):
        if RLUT.reverse_key_lut == None:
            RLUT.reverse_key_lut = RLUT.build_reverse_key_lut()
        return keys[RLUT.reverse_key_lut[ch]]
    
class Keyboard2AxisCommander(Commander):
    def __init__(self, x_axis, y_axis, x_axis_multiplier=1, y_axis_multiplier=1):
        super(Keyboard2AxisCommander, self).__init__()
        self.x_axis_keys = (x_axis[0], x_axis[1])
        self.y_axis_keys = (y_axis[0], y_axis[1])
        self.x_axis_multiplier=x_axis_multiplier
        self.y_axis_multiplier=y_axis_multiplier

    def source(self):
        return 'keyboard'

    def compare_key(self, axis, keys):
        if RLUT.is_pressed(keys, axis[0]):
            return -1
        elif RLUT.is_pressed(keys, axis[1]):
            return 1
        else: 
            return 0

    def oncallback(self, pressed_keys):
        x, y = (self.compare_key(self.x_axis_keys, pressed_keys), self.compare_key(self.y_axis_keys, pressed_keys))
        cmd_data = { 'x': x*self.x_axis_multiplier, 'y': y*self.y_axis_multiplier}
        self.update_command(cmd_data)

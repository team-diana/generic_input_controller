from commander import Commander
from sensor_msgs.msg import Joy, JointState

class JoyJoystickCommander(Commander):
    def __init__(self, x_axis_index, y_axis_index):
        super(JoyJoystickCommander, self).__init__()
        self.x_index = x_axis_index
        self.y_index = y_axis_index

    def source(self):
        return 'joystick'

    def oncallback(self, joymsg):
        x, y = joymsg.axes[self.x_index], joymsg.axes[self.y_index]
        cmd_data = { 'x': x, 'y': y }
        self.update_command(cmd_data)

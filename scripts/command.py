from sensor_msgs.msg import Joy, JointState

class Command(object):
    def __init__(self, name):
        self.name = name

    def source(self):
        raise Exception("source not defined")

    def oncallback(self, cmd_data):
        raise Exception("oncallback not defined")
        pass

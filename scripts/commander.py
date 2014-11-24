from command import Command

class Commander(object):
    def __init__(self):
        pass

    def set_command(self, command):
        self.command = command

    def update_command(self, cmd_data):
        self.command.oncallback(cmd_data)

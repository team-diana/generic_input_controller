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


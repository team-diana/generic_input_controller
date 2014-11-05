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

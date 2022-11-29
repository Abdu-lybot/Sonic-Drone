import rospy
from nav_msgs.msg import Odometry


class Singleton(object):
    _instance = None

    def __new__(class_, *args, **kwargs):
        if not isinstance(class_._instance, class_):
            class_._instance = object.__new__(class_, *args, **kwargs)
        return class_._instance


class Robot(Singleton):
    """
    Robot singleton
    """
    def __init__(self):
        # Singleton.__init__(self)
        rospy.Subscriber(rospy.get_namespace() + 'odom', Odometry, self.obtain_robot_position)
        self.robot_pose = list()

    def obtain_robot_position(self, msg):
        """
        Obtains the current robot position from the robot odom topic
        :return: Robot position. If no transform available return 0 position
        """
        position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                       msg.pose.pose.orientation.w]
        self.robot_pose = [position, orientation]

    def get_pose(self):
        return self.robot_pose

import rospy
from std_msgs.msg import String
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from aerial_manipulator.mobile_robot import MobileRobot

def send_odometry(robot: type, odom_publu: rospy.topics.Publisher, odom_message: type)->None:
    h = robot.get_internal_states()
    quat = robot.get_quaternio()

    return None
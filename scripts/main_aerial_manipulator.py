import rospy
from std_msgs.msg import String
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import scipy.io

# Global variables Linear velocities
vxd = 0.0
vyd = 0.0
vzd = 0.0
# Global Variables angular velocities
wxd = 0.0
wyd = 0.0
wzd = 0.0

def velocity_call_back(velocity_message):
    global vxd, vyd, vzd, wxd, wyd, wzd
    # Read the linear Velocities
    vxd = velocity_message.linear.x
    vyd = velocity_message.linear.y
    vzd = velocity_message.linear.z

    # Read desired angular velocities from node
    wxd = velocity_message.angular.x
    wyd = velocity_message.angular.y
    wzd = velocity_message.angular.z
    return None

# Simulation System
def main(publiser_odom):
    # Time definition
    ts = 0.05;
    t_final = 120;
    t = np.arange(0, t_final + ts, ts, dtype=np.double)

    # Frequency defintion
    hz = int(1/ts)
    rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("Aerial Manipulator Simulation")


if __name__ == '__main__':
    try:
        # Initialization Node
        rospy.init_node("aerial_manipulator_simulation",disable_signals=True, anonymous=True)

        # Publisher Info
        odomety_topic = "/aerial_manipulator/odom"
        odometry_publisher = rospy.Publisher(odomety_topic, Odometry, queue_size = 10)

        # Subscribe Info
        velocity_topic = "/aerial_manipulator/cmd_vel"
        velocity_subscriber = rospy.Subscriber(velocity_topic, Twist, velocity_call_back)

        main(odometry_publisher)

    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass
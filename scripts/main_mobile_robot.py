#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from aerial_manipulator.mobile_robot import MobileRobot
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
    rospy.loginfo_once("Mobile Robot Simulation")

    # Variables of the system
    a1 = 0.2
    L = [a1]

    # Identification Parameters
    chi = [0.3037, 0.2768, -0.0004018, 0.9835, 0.003818, 1.0725]

    # Initial Conditions
    x1 = 0.0
    y1 = 0.0
    theta_1 = 0*(np.pi/180)
    u = 0.0
    w = 0.0

    x1 = x1 + a1*np.cos(theta_1)
    y1 = y1 + a1*np.sin(theta_1)
    
    # Vector of states
    h = np.zeros((5, t.shape[0] + 1), dtype=np.double)
    h[0, 0] = x1
    h[1, 0] = y1
    h[2, 0] = theta_1
    h[3, 0] = u
    h[4, 0] = w

    # Robot defintion
    robot_1 = MobileRobot(L, h[:, 0], ts ,publiser_odom, chi)

    # Send Initial Values Communication
    robot_1.send_odometry()

    # Control Vector
    uc = np.zeros((2, t.shape[0]), dtype = np.double)
    uc[0, 0] = vxd
    uc[1, 0] = wzd


    # Simulation of the system
    for k in range(0, t.shape[0]):
        # Get time
        tic = rospy.get_time()

        # Get velocities in the communication chanel
        uc[0, k] = vxd
        uc[1, k] = wzd


       
        # Read Vaues Dynamics
        h[:, k+1] = robot_1.system(uc[:, k])
        robot_1.send_odometry()
        rospy.loginfo("Mobile Robot Simulation")

        # Time restriction Correct
        rate.sleep()
        toc = rospy.get_time()
        delta = toc - tic
    return None

if __name__ == '__main__':
    try:
        # Initialization Node
        rospy.init_node("mobile_robot_simulation",disable_signals=True, anonymous=True)

        # Publisher Info
        odomety_topic = "/mobile/odom"
        odometry_publisher = rospy.Publisher(odomety_topic, Odometry, queue_size = 10)

        # Subscribe Info
        velocity_topic = "/mobile/cmd_vel"
        velocity_subscriber = rospy.Subscriber(velocity_topic, Twist, velocity_call_back)

        main(odometry_publisher)

    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass
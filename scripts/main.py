#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from aerial_manipulator.mobile_robot import MobileRobot

# Simulation System
def main():
    # Time definition
    ts = 0.05;
    t_final = 60;
    t = np.arange(0, t_final + ts, ts, dtype=np.double)

    # Frequency defintion
    hz = int(1/ts)
    rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("Aerial Manipulator Simulation")

    # Variables of the system
    a1 = 0.2
    L = [a1]

    # Initial Conditions
    x1 = 0.0
    y1 = 0.0
    theta_1 = 0*(np.pi/180)

    x1 = x1 + a1*np.cos(theta_1)
    y1 = y1 + a1*np.sin(theta_1)
    
    # Vector of states
    h = np.zeros((3, t.shape[0] + 1), dtype=np.double)
    h[0, 0] = x1
    h[1, 0] = y1
    h[2, 0] = theta_1

    # Robot defintion
    robot_1 = MobileRobot(L, h[:, 0], ts)

    # Control Vector
    u = np.zeros((2, t.shape[0]), dtype = np.double)
    u[0, :] = 0.1
    u[1, :] = 0.1

    # Simulation of the system
    for k in range(0, t.shape[0]):
        # Get time
        tic = rospy.get_time()

        # Get system angles
        robot_1.get_rotation_matrix()
       
        # Read Vaues Dynamics
        h[:, k+1] = robot_1.system(u[:, k])

        # Time restriction Correct
        rate.sleep()
        toc = rospy.get_time()
        delta = toc - tic

if __name__ == '__main__':
    try:
        # Initialization Node
        rospy.init_node("DJI_Matrice600_aerial_manipulator",disable_signals=True, anonymous=True)

        # Publisher Info
        odomety_topic = "DJI_Matrice600/odom"
        odometry_message = Odometry
        odometry_publisher = rospy.Publisher(odomety_topic, Odometry, queue_size = 100)
        main()
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import scipy.io
from aerial_manipulator.aerial_manipulator_robot import AerialManipulatorRobot



# Global variables Linear velocities
vxd = 0.0
vyd = 0.0
vzd = 0.0
# Global Variables angular velocities
wxd = 0.0
wyd = 0.0
wzd = 0.0

# Global desired velocities joints
q1pd = 0.0
q2pd = 0.0
q3pd = 0.0

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
    ts = 0.1;
    t_final = 50;
    t = np.arange(0, t_final + ts, ts, dtype=np.double)

    # Frequency defintion
    hz = int(1/ts)
    rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("Aerial Manipulator Simulation")

    # Variables of the system
    l_2 = 0.092
    l_3 = 0.196
    g = 9.81
    L = [l_2, l_3, g]

    # Identification Parameters
    chi = [0.661572614024077, 0.169060601256173, 0.176106759253213, 0.00358336008248739, 0.00108259116812643, -0.00154112503045292, -0.000659822298657971, 33.6014054511568, 0.0742360120879960, 0.569427370141837, -0.264497052266231, 0.00309032010217130, 0.00672411315944624, -0.0206057382683352, 0.263024143591578, 0.0708653674792052, 2.56685427762986e-06, 11.8201949483608, -0.00179197350749983, 0.00312730734389077, 0.0296521572583440, 0.0109883478548300, 7.66001609460077e-05, 13.4468257356268, 0.0124545527379815, 0.491664311420148, 168.840550370009, -0.488896323007813, -0.853943687423747, -0.0745666702825102, -0.0656607511477870, -0.778288579347444]

    # Initial Conditions
    ul = 0.0
    um = 0.0
    un = 0.0
    w = 0.0
    q1p = 0.0
    q2p = 0.0
    q3p = 0.0
    q1 = 0.0
    q2 = 0.0
    q3 = 0.0

    # Vector of states of the system
    h = np.zeros((10, t.shape[0] + 1), dtype=np.double)
    h[0, 0] = ul
    h[1, 0] = um
    h[2, 0] = un
    h[3, 0] = w
    h[4, 0] = q1p
    h[5, 0] = q2p
    h[6, 0] = q3p
    h[7, 0] = q1
    h[8, 0] = q2
    h[9, 0] = q3

    # Definition Aerial Vehicle
    aerial_1 = AerialManipulatorRobot(L, h[:, 0], ts, publiser_odom, chi)

    # Control Variables
    uc = np.zeros((7, t.shape[0]), dtype = np.double)
    uc[0, 0] = vxd
    uc[1, 0] = vyd
    uc[2, 0] = vzd
    uc[3, 0] = wzd
    uc[4, 0] = q1pd
    uc[5, 0] = q2pd
    uc[6, 0] = q3pd

    # Simulation of the system
    for k in range(0, t.shape[0]):
        # Get time
        tic = rospy.get_time()

        # Get velocities through
        uc[0, k] = vxd
        uc[1, k] = vyd
        uc[2, k] = vzd
        uc[3, k] = wzd
        uc[4, k] = q1pd
        uc[5, k] = q2pd
        uc[6, k] = q3pd

        # System Dynamics
        h[:, k+1] = aerial_1.system(uc[:, k])
        rospy.loginfo("Aerial Manipulator Simulation")

        # Time restriction Correct
        rate.sleep()
        toc = rospy.get_time()
        delta = toc - tic

    return None

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
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
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

xd = 0.0
yd = 0.0
zd = 0.0

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

def trajectory_call_back(trajectory_message):
    # Read the linear Velocities
    global xd, yd, zd
    xd = trajectory_message.pose.position.x
    yd = trajectory_message.pose.position.y
    zd = trajectory_message.pose.position.z
    return None

def joints_call_back(joints_message):
    global q1pd, q2pd, q3pd
    joints_references = joints_message.data
    q1pd = joints_references[0]
    q2pd = joints_references[1]
    q3pd = joints_references[2]
    return None

# Simulation System
def main(publiser_odom, publisher_joint):

    # Time definition
    ts = 0.05;
    t_final = 300;
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
    chi = [0.767360879658187, 0.100392049558377, 0.154759038673317, 38.6081368718373, -0.251845093279128, 0.251162312249968, 0.000776423855577558, 0.00385727722890279, 0.0677648559474654, 0.600581359378254, -0.263735834925159, 28.6385256949156, 0.00991883372922247, -0.0164389624249138, 0.277904939603094, 0.0765941547426626, 0.0196359064621946, 0.0558688201369773, -0.400294847258480, 0.666204613050887, 0.0274410701828662, 3.10969308784518, -0.0645680025113339, 0.0649604268297644, 2.74795798072509, 0.496027789996623, 0.0177256588261969, -0.491939820125297, -0.866962882867046, -15.8401945765054, -13.3427202909208, -0.802106747335276]

    # Initial Conditions Odometry
    x1 = 0.0
    y1 = 0.0
    z1 = 2
    theta_1 = 0*(np.pi/180)

    # Initial conditions odometry vector
    x = np.zeros((4, t.shape[0] + 1), dtype=np.double)
    x[0, 0] = x1
    x[1, 0] = y1
    x[2, 0] = z1
    x[3, 0] = theta_1

    # Initial conditions system velocities
    ul = 0.0
    um = 0.0
    un = 0.0
    w = 0.0
    q1p = 0.0
    q2p = 0.0
    q3p = 0.0
    q1 = 45*(np.pi/180)
    q2 = 45*(np.pi/180)
    q3 = 0.0*(np.pi/180)


    # Initial conditions velocities vector
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
    aerial_1 = AerialManipulatorRobot(L, h[:, 0], x[:, 0], ts, publiser_odom, chi, publisher_joint)


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

        # Get velocities 
        uc[0, k] = vxd
        uc[1, k] = vyd
        uc[2, k] = vzd
        uc[3, k] = wzd
        uc[4, k] = q1pd
        uc[5, k] = q2pd
        uc[6, k] = q3pd

        # System Evolution
        h[:, k+1] = aerial_1.system(uc[:, k])
        x[:, k+1] = aerial_1.system_drone()
        aerial_1.send_odometry()
        aerial_1.send_joint()
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

        # Publisher Info
        joint_states_topic = "/aerial_manipulator/joints"
        joint_publisher = rospy.Publisher(joint_states_topic, JointState, queue_size=10)

        # Subscribe Info
        velocity_topic = "/aerial_manipulator/cmd_vel"
        velocity_subscriber = rospy.Subscriber(velocity_topic, Twist, velocity_call_back)

        trajectory_topic = "/aerial_manipulator/ref"
        trajectory_subscriber = rospy.Subscriber(trajectory_topic, PoseStamped, trajectory_call_back)

        # Subscribe Info joints
        joint_ref_topic = "/aerial_manipulator/joints_ref"
        velocity_subscriber = rospy.Subscriber(joint_ref_topic, Float64MultiArray, joints_call_back)
        main(odometry_publisher, joint_publisher)

    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass
import numpy as np
import rospy
from std_msgs.msg import String
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation

class MobileRobot:
    def __init__(self, L : list, x: np.ndarray, ts: float, odom_publi: rospy.topics.Publisher, parameters: list):
        super().__init__()
        # States desired point
        self.h = x
        # Sample time
        self.ts = ts
        # Variables robot
        self.a = L[0]
        # Identification Parameters
        self.chi = parameters

        # Angles Definition
        self.rpy = np.array([0, 0, self.h[2]], dtype=np.double)

        # Complete states Position and angles System
        self.H = np.array([self.h[0], self.h[1], 0, self.rpy[0], self.rpy[1], self.h[2]], dtype=np.double)
        self.Hp = np.array([self.h[3], 0, 0, 0, 0, self.h[4]], dtype=np.double)

        # Comunication Odometry
        self.odom_publisher = odom_publi

    def jacobian_system(self, x: np.ndarray) -> np.ndarray: 
        # Get internatl states
        qx = x[0]
        qy = x[1]
        q_theta = x[2]

        # Elements Jacobian Matrix
        J_11 = np.cos(q_theta)
        J_12 = -self.a*np.sin(q_theta)
        J_21 = np.sin(q_theta)
        J_22 = self.a*np.cos(q_theta)
        J_31 = 0
        J_32 = 1

        # Build Matrix
        J = np.array([[J_11, J_12],[J_21, J_22],[J_31, J_32]], dtype = np.double)
        return J
        
    def get_M_matrix(self, x:np.ndarray)-> np.ndarray:
        # Internal states of the system
        qx = x[0]
        qy = x[1]
        q_theta = x[2]
        qu = x[3]
        qw = x[4]
        
        # Dynamic parameters of the system
        chi_1 = self.chi[0]
        chi_2 = self.chi[1]
        chi_3 = self.chi[2]
        chi_4 = self.chi[3]
        chi_5 = self.chi[4]
        chi_6 = self.chi[5]

        # Create M matrix
        M_11 = chi_1
        M_12 = 0
        M_21 = 0
        M_22 = chi_2
        M = np.array([[M_11, M_12],[M_21, M_22]], dtype = np.double)
        return M

    def get_C_matrix(self, x: np.ndarray)->np.ndarray:
        # Internal states of the system
        qx = x[0]
        qy = x[1]
        q_theta = x[2]
        qu = x[3]
        qw = x[4]
        
        # Dynamic parameters of the system
        chi_1 = self.chi[0]
        chi_2 = self.chi[1]
        chi_3 = self.chi[2]
        chi_4 = self.chi[3]
        chi_5 = self.chi[4]
        chi_6 = self.chi[5]

        # Create C matrix of the system
        C_11 = chi_4
        C_12 = -chi_3*qw
        C_21 = chi_5*qw
        C_22 = chi_6

        C = np.array([[C_11, C_12],[C_21, C_22]], dtype = np.double)
        return C

    def f_model(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        # Get inertial Matrix
        M = self.get_M_matrix(x)
        M_1 = np.linalg.inv(M)
        
        # Get Centrifugal Matrix
        C = self.get_C_matrix(x)

        # Jacobian Matrix
        J = self.jacobian_system(x)

        # Get complete system
        n_states = x.shape[0]
        u_states = u.shape[0]

        A = np.zeros((n_states, n_states), dtype=np.double)
        A[0:3, 3:5] = J
        A[3:5, 3:5] = -M_1@C

        B = np.zeros((n_states, u_states), dtype=np.double)
        B[3:5, 0:2] = M_1


        # Evolution system
        xp = A@x+B@u
        return xp

    def f_model_1(self, x: np.ndarray, u:np.ndarray)->np.ndarray:
        # Internal states of the system
        qx = x[0]
        qy = x[1]
        q_theta = x[2]
        qu = x[3]
        qw = x[4]
        
        # Dynamic parameters of the system
        chi_1 = self.chi[0]
        chi_2 = self.chi[1]
        chi_3 = self.chi[2]
        chi_4 = self.chi[3]
        chi_5 = self.chi[4]
        chi_6 = self.chi[5]

        f11 = qu*np.cos(q_theta)-self.a*np.sin(q_theta)*qw
        f21 = qu*np.sin(q_theta)+self.a*np.cos(q_theta)*qw
        f31 = qw
        f41 = (chi_3/chi_1)*(qw**2) - (chi_4/chi_1)*qu
        f51 = -(chi_5/chi_2)*qu*qw - (chi_6/chi_2)*qw

        F = np.array([f11, f21, f31, f41, f51], dtype = np.double)
        G = np.array([[0.0, 0.0],[0.0, 0.0], [0.0, 0.0], [(1/chi_1), 0], [0, (1/chi_2)]], dtype = np.double)
        xp = F + G@u
        return xp
    
    def system(self, u: np.ndarray)->np.ndarray:
        # Get sample time
        Ts = self.ts

        # Get states of the system
        x = self.h

        # Runge Kuta 4
        k1 = self.f_model_1(x, u)
        k2 = self.f_model_1(x + (Ts/2)*k1, u)
        k3 = self.f_model_1(x + (Ts/2)*k2, u)
        k4 = self.f_model_1(x + Ts*k3, u)
        x = x + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4)

        # Update internal States
        self.h = x
        self.rpy = np.array([0.0, 0.0, self.h[2]], dtype=np.double)
        self.H = np.array([self.h[0], self.h[1], 0, self.rpy[0], self.rpy[1], self.h[2]], dtype=np.double)
        self.Hp = np.array([self.h[3], 0, 0, 0, 0, self.h[4]], dtype=np.double)
        return x.T

    def get_euler(self)->tuple:
        # Get angles of the system
        roll = self.rpy[0]
        pitch = self.rpy[1]
        yaw = self.rpy[2]

        return roll, pitch, yaw

    def get_rotation_matrix(self)->np.ndarray:
        # get angles of the system
        roll, pitch, yaw = self.get_euler()

        # Rotational Matrices
        rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

        # Get Matrices
        rt = rz @ ry @ rx
        rt_1 = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees = False)

        return rt_1.as_matrix()


    def get_quaternion(self)->np.ndarray:
        # get angles of the system
        roll, pitch, yaw = self.get_euler()

        # Rotational Matrices
        rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

        # Get Matrices
        rt = rz @ ry @ rx
        rt_1 = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees = False)

        return rt_1.as_quat()

    def get_internal_states(self)->np.ndarray:
        # Get the internal states of the system
        x = self.H
        xp = self.Hp
        return x, xp

    def send_odometry(self)->None:
        x, xp = self.get_internal_states()
        quat = self.get_quaternion()
        odom_message = Odometry()
        # Pose of the system 
        odom_message.pose.pose.position.x = x[0]
        odom_message.pose.pose.position.y = x[1]
        odom_message.pose.pose.position.z = x[2]

        odom_message.pose.pose.orientation.x = quat[0]
        odom_message.pose.pose.orientation.y = quat[1]
        odom_message.pose.pose.orientation.z = quat[2]
        odom_message.pose.pose.orientation.w = quat[3]

        # Velocity of the system
        odom_message.twist.twist.linear.x = xp[0]
        odom_message.twist.twist.linear.y = xp[1]
        odom_message.twist.twist.linear.z = xp[2]
        odom_message.twist.twist.angular.x = xp[3]
        odom_message.twist.twist.angular.y = xp[4]
        odom_message.twist.twist.angular.z = xp[5]

        self.odom_publisher.publish(odom_message)
        return None

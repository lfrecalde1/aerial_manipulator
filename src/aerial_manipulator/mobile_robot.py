import numpy as np
from scipy.spatial.transform import Rotation
class MobileRobot:
    def __init__(self, L : list, x: np.ndarray, ts: float):
        super().__init__()
        # States desired point
        self.h = x
        # Sample time
        self.ts = ts
        # Variables robot
        self.a = L[0]

        # Angles Definition
        self.rpy = np.array([0, 0, self.h[2]], dtype=np.double)
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

    def f_model(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        # Jacobian Matrix
        J = self.jacobian_system(x)

        # Evolution system
        xp = J@u
        return xp
    
    def system(self, u: np.ndarray)->np.ndarray:
        # Get sample time
        Ts = self.ts

        # Get states of the system
        x = self.h

        # Runge Kuta 4
        k1 = self.f_model(x, u)
        k2 = self.f_model(x + (Ts/2)*k1, u)
        k3 = self.f_model(x + (Ts/2)*k2, u)
        k4 = self.f_model(x + Ts*k3, u)
        x = x + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4)

        # Update internal States
        self.h = x
        self.rpy = np.array([0.0, 0.0, self.h[2]], dtype=np.double)
        return x.T

    def get_euler(self)->tuple:
        # Get angles of the system
        roll = self.rpy[0]
        pitch = self.rpy[1]
        yaw = self.rpy[2]

        return roll, pitch, yaw

    def get_rotation_matrix(self):
        # get angles of the system
        roll, pitch, yaw = self.get_euler()

        # Rotational Matrices
        rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

        # Get Matrices
        rt = rz @ ry @ rx
        rt_1 = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees = False)
        
        print(rt)
        print("--------------------------------------")
        print(rt_1.as_matrix())
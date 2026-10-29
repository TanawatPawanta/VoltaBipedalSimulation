from enum import Enum

import numpy as np
from pydrake.math import RollPitchYaw


class States:
    """Pose, twist and acceleration of one body (CoM, foot, ...) as rpy / xyz vectors."""

    def __init__(self,name:str):
        self.name = name
        # Pose
        self.orientation = np.zeros(3)          # roll, pitch, yaw 
        self.position = np.zeros(3)             # x, y, z
        self.position_m1 = np.zeros(3) 
        # Twist
        self.angular_velocity = np.zeros(3)     # wx, wy, wz
        self.linear_velocity = np.zeros(3)      # vx, vy, vz
        # Acceleration
        self.angular_acceleration = np.zeros(3) # alphax, alphay, alphaz
        self.linear_acceleration = np.zeros(3)  # ax, ay, az

    def GetLinearStateVector(self,acc:bool = False):
        if acc:
            state_vector = np.concatenate((self.position,
                                           self.linear_velocity,
                                           self.linear_acceleration), axis=0)
        else:
            state_vector = np.concatenate((self.position,
                                           self.linear_velocity,), axis=0)
        return state_vector
    
    def GetStateVector(self,acc:bool = False):
        if acc:
            state_vector = np.concatenate((self.orientation,
                                        self.position,
                                        self.angular_velocity,
                                        self.linear_velocity,
                                        self.angular_acceleration,
                                        self.linear_acceleration), axis=0)
        else:
            state_vector = np.concatenate((self.orientation,
                                        self.position,
                                        self.angular_velocity,
                                        self.linear_velocity), axis=0)
        return state_vector
    
    def UpdateStates(self,data):
        self.orientation = data[0:3]
        self.position = data[3:6]
        self.angular_velocity = data[6:9]
        self.linear_velocity = data[9:12]
        if len(data) > 12:
            self.angular_acceleration = data[12:15]
            self.linear_acceleration = data[15:18]

    def GetOrientationInRotationMatrix(self):
        rpy = RollPitchYaw(self.orientation)
        return rpy.ToRotationMatrix()
    
    def GetOrientationInQuaternion (self):
        rpy = RollPitchYaw(self.orientation)
        return rpy.ToRotationMatrix()

    def CalcLinearVelocity(self, dt):
        """Finite-difference linear velocity from the previous call's position."""
        linear_velocity = (self.position - self.position_m1) / dt
        self.position_m1 = self.position.copy()
        return linear_velocity


class WalkStates(Enum):
    """Phase of the offline-trajectory walking sequence."""
    init  = 0
    stand = 1
    walk = 2

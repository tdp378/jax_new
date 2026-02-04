import numpy as np
from enum import Enum

class State:
    def __init__(self):
        """
        Stores the current physical and behavioral state of Jax.
        """
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0.0
        self.height = -0.18 # Updated to match Jax default_z_ref in Config
        self.pitch = 0.0
        self.roll = 0.0
        self.joystick_control_active = 1
        self.behavior_state = BehaviorState.REST
        
        # Euler orientation [yaw, pitch, roll] typically from IMU
        self.euler_orientation = [0.0, 0.0, 0.0]
        self.ticks = 0
        
        # Foot locations: 3x4 matrix (x,y,z for each of the 4 legs)
        self.foot_locations = np.zeros((3, 4))
        self.rotated_foot_locations = np.zeros((3, 4))
        self.joint_angles = np.zeros((3, 4))
        
        self.speed_factor = 1.0
        self.currently_estopped = 0

class BehaviorState(Enum):
    """
    Finite State Machine (FSM) states for Jax's movement logic.
    """
    DEACTIVATED = -1
    REST = 0
    TROT = 1
    HOP = 2
    FINISHHOP = 3
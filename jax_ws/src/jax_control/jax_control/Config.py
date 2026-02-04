import numpy as np
from enum import Enum
import math as m

class Configuration:
    def __init__(self):
        # Rebranded Jax Identification
        self.robot_name = "Jax"
        
        ################# CONTROLLER VISUALS ##############
        # Default colors (previously imported, now hardcoded for independence)
        self.ps4_color = [0, 0, 255] # Blue
        self.ps4_deactivated_color = [255, 0, 0] # Red

        #################### COMMANDS ####################
        self.max_x_velocity = 1.2
        self.max_y_velocity = 0.5
        self.max_yaw_rate = 2.0
        self.max_pitch = 30.0 * np.pi / 180.0
        
        #################### MOVEMENT PARAMS ####################
        self.dt = 0.02 # 50Hz control loop
        self.z_time_constant = 0.02
        self.z_speed = 0.06  # maximum speed [m/s]
        self.pitch_deadband = 0.05
        self.pitch_time_constant = 0.25
        self.max_pitch_rate = 0.3
        self.roll_speed = 0.1  # maximum roll rate [rad/s]
        self.yaw_time_constant = 0.3
        self.max_stance_yaw = 1.2
        self.max_stance_yaw_rate = 1

        #################### STANCE ####################
        self.delta_x = 0.11
        self.delta_y = 0.08 
        self.x_shift = 0.0
        self.default_z_ref = -0.16

        # Leg Origins relative to body center
        self.LEG_ORIGINS = np.array(
            [
                [self.delta_x, self.delta_x, -self.delta_x, -self.delta_x],
                [self.delta_y, -self.delta_y, self.delta_y, -self.delta_y],
                [0, 0, 0, 0],
            ]
        )

        self.default_stance = np.array(
            [
                [
                    self.delta_x + self.x_shift,
                    self.delta_x + self.x_shift,
                    -self.delta_x + self.x_shift,
                    -self.delta_x + self.x_shift,
                ],
                [self.delta_y, -self.delta_y, self.delta_y, -self.delta_y],
                [0, 0, 0, 0],
            ]
        )

        #################### SWING & GAITS ####################
        self.alpha = 0.5
        self.beta = 0.5
        self.z_clearance = 0.05
        self.dt = 0.02
        self.num_phases = 4
        self.contact_phases = np.array(
            [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
        )
        self.overlap_time = 0.1
        self.swing_time = 0.15

        self.overlap_ticks = int(self.overlap_time / self.dt)
        self.swing_ticks = int(self.swing_time / self.dt)
        self.phase_ticks = np.array(
            [self.overlap_ticks, self.swing_ticks, self.overlap_ticks, self.swing_ticks]
        )
        self.phase_length = sum(self.phase_ticks)

        #################### JAX PHYSICAL DIMENSIONS ####################
        # Link lengths in meters
        self.L1 = 0.0  # Hip
        self.L2 = 0.13  # Upper leg
        self.L3 = 0.11  # Lower leg (Updated to 130mm as per your snippet)

class Leg_linkage:
    """Leg Linkage for hardware interfacing on Jax"""
    def __init__(self, configuration):
        self.a = 35.12 
        self.b = 37.6 
        self.c = 43 
        self.d = 35.23  
        self.e = 67.1 
        self.f = 130.0
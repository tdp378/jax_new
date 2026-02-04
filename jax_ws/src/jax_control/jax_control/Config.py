import numpy as np
import math as m

class Configuration:
    def __init__(self):
        self.robot_name = "Jax"
        
        # Controller Visuals
        self.ps4_color = [0, 0, 255]
        self.ps4_deactivated_color = [255, 0, 0]

        # Movement Parameters
        self.dt = 0.02
        self.max_x_velocity = 1.2
        self.max_y_velocity = 0.5
        self.max_yaw_rate = 2.0
        self.max_pitch = 30.0 * np.pi / 180.0
        
        # Stance (Units in Meters)
        self.delta_x = 0.11
        self.delta_y = 0.08 
        self.x_shift = 0.0
        self.default_z_ref = -0.16

        self.LEG_ORIGINS = np.array([
            [self.delta_x, self.delta_x, -self.delta_x, -self.delta_x],
            [self.delta_y, -self.delta_y, self.delta_y, -self.delta_y],
            [0, 0, 0, 0],
        ])

        # Physical Dimensions (Meters)
        self.L1 = 0.0   
        self.L2 = 0.13  
        self.L3 = 0.11  

import numpy as np
import math as m

class Leg_linkage:
    def __init__(self, configuration):
        # Meters
        self.a = 0.03512 
        self.b = 0.0376 
        self.c = 0.043 
        self.d = 0.03523  
        self.e = 0.0671 
        self.f = 0.130
        self.g = 0.03523
        self.h = 0.0376
        self.i = 0.043
        
        self.gamma = m.radians(0.0)
        self.EDC = m.radians(120.0) # Reset to 120
        self.lower_leg_bend_angle = m.radians(0.0)
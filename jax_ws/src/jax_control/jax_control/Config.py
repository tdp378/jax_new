import numpy as np
import math as m

class Configuration:
    def __init__(self):
        self.robot_name = "Jax"
        
        # --- ROS 2 Timing ---
        self.dt = 0.02  # 50Hz control loop
        
        # --- Gait Timing (Ticks) ---
        # These drive the phase logic in Gaits.py
        self.swing_ticks = 20
        self.stance_ticks = 20
        self.num_phases = 2
        self.phase_ticks = [self.stance_ticks, self.swing_ticks]
        self.phase_length = sum(self.phase_ticks)
        
        # Trotting Gait: Diagonal pairs move together
        # 1 = Stance, 0 = Swing
        # Order: [FL, FR, RL, RR]
        self.contact_phases = np.array([
            [1, 0],  # Front Left
            [0, 1],  # Front Right
            [0, 1],  # Rear Left
            [1, 0]   # Rear Right
        ])

        # --- Movement Limits & Gains ---
        self.z_speed = 0.05
        self.max_x_velocity = 1.0
        self.max_y_velocity = 0.4
        self.max_yaw_rate = 1.5
        
        # Raibert Heuristic Gains (for SwingLegController)
        self.alpha = 2.0  # Linear velocity compensation
        self.beta = 0.5  # Angular velocity compensation
        
        # Stance vertical compensation gain
        self.z_time_constant = 0.15 

        # --- Physical Dimensions (Meters) ---
        self.L1 = 0.04    # Hip offset (abduction) - non-zero recommended for stability
        self.L2 = 0.13     # Thigh length
        self.L3 = 0.138     # Knee/Shin length
        
        # --- Body Geometry ---
        self.delta_x = 0.11  # Distance from center to hip in X
        self.delta_y = 0.08  # Distance from center to hip in Y
        
        # Default Z height (Command and State should reference this)
        # -0.16 is a stable mid-range height for these link lengths
        self.default_z_ref = -0.14
        self.z_clearance = 0.02  # How high the foot lifts during swing

        # Leg origins relative to body center
        self.LEG_ORIGINS = np.array([
            [self.delta_x,  self.delta_x, -self.delta_x, -self.delta_x], # X
            [self.delta_y, -self.delta_y,  self.delta_y, -self.delta_y], # Y
            [0, 0, 0, 0],                                               # Z
        ])

        # Default neutral foot positions relative to body center
        self.default_stance = np.array([
            [self.delta_x,  self.delta_x, -self.delta_x, -self.delta_x],
            [self.delta_y, -self.delta_y,  self.delta_y, -self.delta_y],
            [self.default_z_ref, self.default_z_ref, self.default_z_ref, self.default_z_ref]
        ])

        # --- Controller Visuals ---
        self.ps4_color = [0, 0, 255]
        self.ps4_deactivated_color = [255, 0, 0]
import numpy as np
from transforms3d.euler import euler2mat

class SwingController:
    def __init__(self, config):
        """
        Calculates the trajectory of Jax's feet while they are in the air (swing phase).
        """
        self.config = config

    def raibert_touchdown_location(self, leg_index, command):
        """Calculates where the foot should land based on desired velocity and yaw."""
        # Standard stance duration is usually sum of overlap and swing, 
        # but here we use the specific stance_ticks defined in config.
        # Note: Ensure config.overlap_ticks * 2 is what you intend for stance_ticks
        stance_ticks = self.config.phase_length / 2 
        
        delta_p_2d = (
            self.config.alpha
            * stance_ticks
            * self.config.dt
            * command.horizontal_velocity
        )
        delta_p = np.array([delta_p_2d[0], delta_p_2d[1], 0])
        
        theta = (
            self.config.beta
            * stance_ticks
            * self.config.dt
            * command.yaw_rate
        )
        R = euler2mat(0, 0, theta)
        return R @ self.config.default_stance[:, leg_index] + delta_p

    def swing_height(self, swing_phase, triangular=True):
        """Calculates the vertical clearance (Z) of the foot during the swing."""
        swing_height_ = 0.0
        if triangular:
            if swing_phase < 0.5:
                swing_height_ = (swing_phase / 0.5) * self.config.z_clearance
            else:
                swing_height_ = self.config.z_clearance * (1.0 - (swing_phase - 0.5) / 0.5)
        return swing_height_

    def next_foot_location(self, swing_prop, leg_index, state, command):
        """Drives the leg using the swing percentage (0.0 to 1.0)"""
        # 1. Landing target based on teleop speed
        touchdown_location = self.raibert_touchdown_location(leg_index, command)
        
        # 2. Starting point (Neutral stance)
        start_location = self.config.default_stance[:, leg_index]
        
        # 3. Interpolate X and Y based on progress (swing_prop)
        new_location = (1.0 - swing_prop) * start_location + swing_prop * touchdown_location
        
        # 4. Add the vertical arch (Z)
        new_location[2] = self.config.default_z_ref + self.swing_height(swing_prop)
        
        return new_location

   
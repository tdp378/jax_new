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
        """Interpolates between the current foot location and the touchdown target."""
        assert 0 <= swing_prop <= 1
        
        foot_location = state.foot_locations[:, leg_index]
        swing_height_ = self.swing_height(swing_prop)
        touchdown_location = self.raibert_touchdown_location(leg_index, command)
        
        # Calculate time remaining in the swing phase
        time_left = self.config.dt * self.config.swing_ticks * (1.0 - swing_prop)
        
        # Linear interpolation for X and Y, and the calculated swing height for Z
        v = (touchdown_location - foot_location) / max(time_left, self.config.dt)
        delta_f = v * self.config.dt
        
        # We want the foot to move toward the touchdown point 
        # but follow the vertical 'arch' defined by swing_height
        incremented_location = foot_location + delta_f
        incremented_location[2] = self.config.default_z_ref + swing_height_

        return incremented_location
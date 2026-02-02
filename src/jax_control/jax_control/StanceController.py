import numpy as np
from transforms3d.euler import euler2mat

class StanceController:
    def __init__(self, config):
        """
        Controls the movement of Jax's feet while they are in the stance phase (on the ground).
        """
        self.config = config

    def position_delta(self, leg_index, state, command):
        """Calculate the difference between the next desired body location and the current body location.
        
        Returns
        -------
        (Numpy array (3), Numpy array (3, 3))
            (Position increment, rotation matrix increment)
        """
        # Current z-coordinate of the foot relative to the body
        z = state.foot_locations[2, leg_index]
        
        # Calculate desired velocity vector including vertical compensation
        v_xy = np.array(
            [
                -command.horizontal_velocity[0],
                -command.horizontal_velocity[1],
                1.0 / self.config.z_time_constant * (state.height - z),
            ]
        )
        
        delta_p = v_xy * self.config.dt
        delta_R = euler2mat(0, 0, -command.yaw_rate * self.config.dt)
        
        return (delta_p, delta_R)

    def next_foot_location(self, leg_index, state, command):
        """Calculates the next coordinate for a foot that is currently in stance."""
        foot_location = state.foot_locations[:, leg_index]
        (delta_p, delta_R) = self.position_delta(leg_index, state, command)
        
        # Apply the rotation and translation to the foot location
        incremented_location = delta_R @ foot_location + delta_p

        return incremented_location
import numpy as np

class Command:
    def __init__(self):
        """
        Stores movement commands for Jax.
        These values are typically updated by an input node (Controller/Joystick).
        """
        # Linear and Angular velocities
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0.0
        
        # Posture settings
        self.height = -0.18 # Matches Jax default_z_ref in Config
        self.pitch = 0.0
        self.roll = 0.0
        
        # State toggles
        self.joystick_control_active = 0
        self.trotting_active = 0

        # Movement offsets
        self.height_movement = 0
        self.roll_movement = 0
        
        # Events (Triggered on button presses)
        self.hop_event = False
        self.trot_event = False
        self.joystick_control_event = False
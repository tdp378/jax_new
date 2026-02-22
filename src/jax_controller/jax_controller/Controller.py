import numpy as np
from transforms3d.euler import euler2mat
from math import degrees

# Relative imports for ROS 2 package structure
from .Gaits import GaitController
from .StanceController import StanceController
from .SwingLegController import SwingController
from .State import BehaviorState, State

def clipped_first_order_filter(input_val, target, max_rate, time_constant):
    """Helper for REST mode yaw smoothing since Utilities.py was removed"""
    step = (target - input_val) / time_constant
    step = np.clip(step, -max_rate, max_rate)
    return step

class Controller:
    """Controller and planner object (ROS 2 Humble Version)"""

    def __init__(self, config, inverse_kinematics):
        self.config = config
        self.smoothed_yaw = 0.0  # for REST mode only
        self.inverse_kinematics = inverse_kinematics

        self.contact_modes = np.zeros(4)
        self.gait_controller = GaitController(self.config)
        self.swing_controller = SwingController(self.config)
        self.stance_controller = StanceController(self.config)

        # State transition mappings
        self.hop_transition_mapping = {
            BehaviorState.REST: BehaviorState.HOP, 
            BehaviorState.HOP: BehaviorState.FINISHHOP, 
            BehaviorState.FINISHHOP: BehaviorState.REST, 
            BehaviorState.TROT: BehaviorState.HOP
        }
        self.trot_transition_mapping = {
            BehaviorState.REST: BehaviorState.TROT, 
            BehaviorState.TROT: BehaviorState.REST, 
            BehaviorState.HOP: BehaviorState.TROT, 
            BehaviorState.FINISHHOP: BehaviorState.TROT
        }
        self.activate_transition_mapping = {
            BehaviorState.DEACTIVATED: BehaviorState.REST, 
            BehaviorState.REST: BehaviorState.DEACTIVATED
        }

    def step_gait(self, state, command):
        """Calculate the desired foot locations for the next timestep"""
        contact_modes = self.gait_controller.contacts(state.ticks)
        new_foot_locations = np.zeros((3, 4))
        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            if contact_mode == 1:
                new_location = self.stance_controller.next_foot_location(leg_index, state, command)
            else:
                swing_proportion = (
                    self.gait_controller.subphase_ticks(state.ticks) / self.config.swing_ticks
                )
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion,
                    leg_index,
                    state,
                    command
                )
            new_foot_locations[:, leg_index] = new_location
        return new_foot_locations, contact_modes

    def run(self, state, command):
        """Steps the controller forward one timestep"""
        previous_state = state.behavior_state

        ########## Update operating state based on command ######
        if command.joystick_control_event:
            state.behavior_state = self.activate_transition_mapping.get(state.behavior_state, state.behavior_state)
        elif command.trot_event:
            state.behavior_state = self.trot_transition_mapping.get(state.behavior_state, state.behavior_state)
        elif command.hop_event:
            state.behavior_state = self.hop_transition_mapping.get(state.behavior_state, state.behavior_state)

        if previous_state != state.behavior_state:
            print(f"State changed from {previous_state} to {state.behavior_state}")

        if state.behavior_state == BehaviorState.TROT:
            state.foot_locations, self.contact_modes = self.step_gait(state, command)

            # Apply the desired body rotation
            rotated_foot_locations = (
                euler2mat(command.roll, command.pitch, 0.0) @ state.foot_locations
            )

            # IMU Stabilization
            rotated_foot_locations = self.stabilise_with_IMU(rotated_foot_locations, state.euler_orientation)

            state.joint_angles = self.inverse_kinematics(rotated_foot_locations, self.config)
            state.rotated_foot_locations = rotated_foot_locations

        elif state.behavior_state == BehaviorState.REST:
            yaw_proportion = command.yaw_rate / self.config.max_yaw_rate
            
            # Smoothly transition yaw while standing
            self.smoothed_yaw += (
                self.config.dt
                * clipped_first_order_filter(
                    self.smoothed_yaw,
                    yaw_proportion * -self.config.max_stance_yaw,
                    self.config.max_stance_yaw_rate,
                    self.config.yaw_time_constant,
                )
            )

            # Set the foot locations to the default stance plus the standard height
            state.foot_locations = (
                self.config.default_stance
                + np.array([0, 0, command.height])[:, np.newaxis]
            )

            # Apply rotation and IMU stabilization
            rotated_foot_locations = (
                euler2mat(command.roll, command.pitch, self.smoothed_yaw) @ state.foot_locations
            )
            rotated_foot_locations = self.stabilise_with_IMU(rotated_foot_locations, state.euler_orientation)

            state.joint_angles = self.inverse_kinematics(rotated_foot_locations, self.config)
            state.rotated_foot_locations = rotated_foot_locations

        state.ticks += 1
        state.pitch = command.pitch
        state.roll = command.roll
        state.height = command.height

    def stabilise_with_IMU(self, foot_locations, orientation):
        """Applies roll and pitch compensation based on IMU data"""
        yaw, pitch, roll = orientation
        correction_factor = 0.5
        max_tilt = 0.4  # radians
        
        roll_compensation = correction_factor * np.clip(-roll, -max_tilt, max_tilt)
        pitch_compensation = correction_factor * np.clip(-pitch, -max_tilt, max_tilt)
        
        rmat = euler2mat(roll_compensation, pitch_compensation, 0)
        return rmat.T @ foot_locations

    def set_pose_to_default(self, state):
        state.foot_locations = (
            self.config.default_stance
            + np.array([0, 0, self.config.default_z_ref])[:, np.newaxis]
        )
        state.joint_angles = self.inverse_kinematics(state.foot_locations, self.config)
        return state.joint_angles
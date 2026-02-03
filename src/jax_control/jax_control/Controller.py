import rclpy
from rclpy.node import Node
import numpy as np
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from jax_control.Config import Configuration
from jax_control.Kinematics import leg_explicit_inverse_kinematics

class JaxBehaviorController(Node):
    def __init__(self):
        """
        SECTION 1: INITIALIZATION
        """
        super().__init__('jax_controller')
        self.config = Configuration()
        
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.timer = self.create_timer(self.config.dt, self.run_loop)
        self.start_time = time.time()
        self.last_cmd_time = time.time()
        
        # Motion States
        self.v_x, self.v_y, self.v_yaw, self.z_direction = 0.0, 0.0, 0.0, 0.0
        self.smooth_v_x = 0.0
        self.smooth_v_yaw = 0.0
        self.accel_limit = 0.6 

        # Height Management
        self.current_z_offset = 0.0
        self.target_z_offset = 0.0  # New: specifically for smooth transitions
        self.height_speed = 0.1     # m/s for smooth rising/sinking
        
        self.x_pos, self.y_pos, self.yaw_pos = 0.0, 0.0, 0.0
        self.is_sitting = False

    def cmd_callback(self, msg):
        """
        SECTION 2: INPUT HANDLING
        Updated to handle smooth 'Home' reset logic via the 'k' key.
        """
        self.v_x, self.v_y, self.v_yaw = msg.linear.x, msg.linear.y, msg.angular.z
        self.last_cmd_time = time.time()

        # Detection for the 'k' key (Zero velocity command)
        is_stop_cmd = all(abs(v) < 0.001 for v in [msg.linear.x, msg.linear.y, msg.angular.z, msg.linear.z])

        # Manual Height adjustment (Up/Down keys)
        if abs(msg.linear.z) > 0.01:
            self.z_direction = 1.0 if msg.linear.z > 0 else -1.0
            self.is_sitting = False 
        else:
            self.z_direction = 0.0

        # HOME RESET: If 'k' is pressed, target neutral pose
        if is_stop_cmd:
            self.is_sitting = False
            self.target_z_offset = 0.0  # Tell the loop to return to zero height
        
        # State transitions for Sitting
        if abs(self.v_x) < 0.01 and self.v_yaw < -0.3:
            self.is_sitting = True
        if abs(self.v_x) > 0.1:
            self.is_sitting = False

    def run_loop(self):
        """
        SECTION 3: MAIN PHYSICS & CONTROL LOOP
        """
        if (time.time() - self.last_cmd_time) > 0.5:
            self.v_x, self.v_y, self.v_yaw = 0.0, 0.0, 0.0

        dt = self.config.dt
        
        # --- SECTION 3A: VELOCITY RAMPING ---
        ramp_step = self.accel_limit * dt
        if self.smooth_v_x < self.v_x: self.smooth_v_x = min(self.smooth_v_x + ramp_step, self.v_x)
        elif self.smooth_v_x > self.v_x: self.smooth_v_x = max(self.smooth_v_x - ramp_step, self.v_x)

        if self.smooth_v_yaw < self.v_yaw: self.smooth_v_yaw = min(self.smooth_v_yaw + ramp_step * 2, self.v_yaw)
        elif self.smooth_v_yaw > self.v_yaw: self.smooth_v_yaw = max(self.smooth_v_yaw - ramp_step * 2, self.v_yaw)

        is_moving = abs(self.smooth_v_x) > 0.01 or abs(self.smooth_v_yaw) > 0.01
        
        # --- SECTION 3B: HEIGHT INTERPOLATION (The Smooth Sink/Rise) ---
        # If we are manually moving Z, update the target
        if self.z_direction != 0.0:
            self.target_z_offset = np.clip(self.target_z_offset + self.z_direction * self.height_speed * dt, -0.08, 0.06)
        
        # Always move current_z toward target_z smoothly
        z_diff = self.target_z_offset - self.current_z_offset
        if abs(z_diff) > 0.001:
            self.current_z_offset += np.sign(z_diff) * self.height_speed * dt

        # --- SECTION 3C: BODY POSE ---
        yaw_threshold = 0.08
        active_yaw = self.smooth_v_yaw if abs(self.smooth_v_yaw) > yaw_threshold else 0.0
        roll = active_yaw * -0.12 if is_moving else 0.0
        
        gait_speed = 6.0 if is_moving else 0.0
        t = (time.time() - self.start_time) * gait_speed
        rhythmic_sway = 0.003 * np.sin(t) if is_moving else 0.0
        body_shift_y = rhythmic_sway + (active_yaw * 0.02)

        # --- SECTION 3D: ODOMETRY ---
        self.x_pos += (self.smooth_v_x * np.cos(self.yaw_pos)) * dt
        self.yaw_pos += self.smooth_v_yaw * dt

        t_fs = TransformStamped()
        t_fs.header.stamp = self.get_clock().now().to_msg()
        t_fs.header.frame_id = 'odom'
        t_fs.child_frame_id = 'base_link'
        t_fs.transform.translation.x = self.x_pos
        t_fs.transform.translation.y = self.y_pos
        t_fs.transform.translation.z = self.current_z_offset + abs(self.config.default_z_ref)
        
        t_fs.transform.rotation.x = np.sin(roll/2) * np.cos(self.yaw_pos/2)
        t_fs.transform.rotation.y = np.sin(roll/2) * np.sin(self.yaw_pos/2)
        t_fs.transform.rotation.z = np.cos(roll/2) * np.sin(self.yaw_pos/2)
        t_fs.transform.rotation.w = np.cos(roll/2) * np.cos(self.yaw_pos/2)
        self.tf_broadcaster.sendTransform(t_fs)

        # --- SECTION 4: JOINT KINEMATICS ---
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = [
            'front_left_hip_joint', 'front_left_thigh_joint', 'front_left_knee_joint',
            'front_right_hip_joint', 'front_right_thigh_joint', 'front_right_knee_joint',
            'rear_left_hip_joint', 'rear_left_thigh_joint', 'rear_left_knee_joint',
            'rear_right_hip_joint', 'rear_right_thigh_joint', 'rear_right_knee_joint'
        ]
        
        positions = []
        V_OFFSET, THIGH_NUDGE, KNEE_NUDGE = 1.570795, -1.4, 0.0
        THIGH_GAIN, KNEE_GAIN = 1.3, 0.7   

        for i in range(4):
            # Target Z relative to the smoothed body height
            z_target = self.config.default_z_ref + self.current_z_offset
            
            # Special Sit Pose Override
            if self.is_sitting:
                z_target = (self.config.default_z_ref - 0.07) if i >= 2 else (self.config.default_z_ref - 0.01)

            phase_offset = 0 if i in [0, 3] else np.pi
            local_t = (t + phase_offset) % (2 * np.pi)
            x_target, y_target = 0.0, -body_shift_y 

            if is_moving:
                step_height, sx = 0.04, np.clip(-self.smooth_v_x * 0.12, -0.05, 0.05)
                if local_t < np.pi: # Swing
                    z_target += step_height * np.sin(local_t)
                    x_target += -sx * np.cos(local_t)
                else: # Stance
                    progress = (local_t - np.pi) / np.pi
                    x_target += sx * (1.0 - 2.0 * progress)
                
                x_target += (0.04 * active_yaw) if i in [0, 1] else -(0.04 * active_yaw)

            target = np.array([x_target, y_target, z_target])
            angles = leg_explicit_inverse_kinematics(target, i, self.config)
            positions.extend([
                float(angles[0]), 
                (float(angles[1]) * THIGH_GAIN) + THIGH_NUDGE, 
                ((float(angles[2]) - V_OFFSET) * KNEE_GAIN) + 0.5 + KNEE_NUDGE
            ])
            
        js_msg.position = positions
        self.js_pub.publish(js_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JaxBehaviorController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
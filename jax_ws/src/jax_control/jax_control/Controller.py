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
        super().__init__('jax_controller')
        self.config = Configuration()
        
        # ROS 2 Infrastructure
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timing & Loops
        self.timer = self.create_timer(self.config.dt, self.run_loop)
        self.start_time = time.time()
        self.last_cmd_time = time.time()
        
        # --- CLEAN STATE MANAGEMENT ---
        self.v_x, self.v_y, self.v_yaw, self.v_z = 0.0, 0.0, 0.0, 0.0
        self.smooth_v_x = 0.0
        self.smooth_v_yaw = 0.0
        self.accel_limit = 0.6 

        self.current_z_offset = 0.0
        self.target_z_offset = 0.0
        self.x_pos, self.y_pos, self.yaw_pos = 0.0, 0.0, 0.0
        
        self.is_sitting = False

    def cmd_callback(self, msg):
        """Unified command processing with priority handling."""
        self.last_cmd_time = time.time()
        
        # 1. Detect Home Command (All axes zero)
        is_home = all(abs(v) < 0.001 for v in [msg.linear.x, msg.linear.y, msg.angular.z, msg.linear.z])
        
        if is_home:
            self.reset_to_home()
            return

        # 2. Update Velocities
        self.v_x = msg.linear.x
        self.v_y = msg.linear.y
        self.v_yaw = msg.angular.z
        self.v_z = msg.linear.z

        # 3. Handle Sitting Signature (Period Key)
        if msg.linear.x == -1.0 and msg.angular.z == -1.0:
            self.is_sitting = True
        
        # 4. Auto-Stand Logic
        if abs(self.v_x) > 0.01 or abs(self.v_y) > 0.01:
            self.is_sitting = False

    def reset_to_home(self):
        """Resets Jax to his default stable stance."""
        self.v_x, self.v_y, self.v_yaw, self.v_z = 0.0, 0.0, 0.0, 0.0
        self.target_z_offset = 0.0
        self.is_sitting = False

    def run_loop(self):
        """The main execution loop. Now strictly for scheduling."""
        # Safety Timeout
        if (time.time() - self.last_cmd_time) > 0.5:
            self.v_x, self.v_y, self.v_yaw = 0.0, 0.0, 0.0

        # Update Internal states
        self.update_velocity_ramping()
        self.update_height()
        self.update_odometry()
        
        # Determine current movement status
        is_moving = (abs(self.smooth_v_x) > 0.01 or abs(self.smooth_v_yaw) > 0.01 or abs(self.v_y) > 0.01)
        
        # Calculate Body Transforms (Visuals/TF)
        self.broadcast_body_transform(is_moving)

        # Solve Kinematics and Publish
        self.publish_joint_states(is_moving)

    def update_velocity_ramping(self):
        step = self.accel_limit * self.config.dt
        self.smooth_v_x = np.clip(self.v_x, self.smooth_v_x - step, self.smooth_v_x + step)
        self.smooth_v_yaw = np.clip(self.v_yaw, self.smooth_v_yaw - step*2, self.smooth_v_yaw + step*2)

    def update_height(self):
        if abs(self.v_z) > 0.01:
            z_dir = 1.0 if self.v_z > 0 else -1.0
            self.target_z_offset = np.clip(self.target_z_offset + z_dir * self.config.z_speed * self.config.dt, -0.08, 0.06)
            self.is_sitting = False
        
        z_diff = self.target_z_offset - self.current_z_offset
        if abs(z_diff) > 0.001:
            self.current_z_offset += np.sign(z_diff) * self.config.z_speed * self.config.dt

    def update_odometry(self):
        dt = self.config.dt
        self.x_pos += (self.smooth_v_x * np.cos(self.yaw_pos) - self.v_y * np.sin(self.yaw_pos)) * dt
        self.y_pos += (self.smooth_v_x * np.sin(self.yaw_pos) + self.v_y * np.cos(self.yaw_pos)) * dt
        self.yaw_pos += self.smooth_v_yaw * dt

    def broadcast_body_transform(self, is_moving):
        # Roll based on yaw turn
        roll = self.smooth_v_yaw * -0.12 if is_moving else 0.0
        
        t_fs = TransformStamped()
        t_fs.header.stamp = self.get_clock().now().to_msg()
        t_fs.header.frame_id = 'odom'
        t_fs.child_frame_id = 'base_link'
        t_fs.transform.translation.x, t_fs.transform.translation.y = self.x_pos, self.y_pos
        t_fs.transform.translation.z = self.current_z_offset + abs(self.config.default_z_ref)
        
        # Quaternion math for Roll and Yaw
        t_fs.transform.rotation.x = np.sin(roll/2) * np.cos(self.yaw_pos/2)
        t_fs.transform.rotation.y = np.sin(roll/2) * np.sin(self.yaw_pos/2)
        t_fs.transform.rotation.z = np.cos(roll/2) * np.sin(self.yaw_pos/2)
        t_fs.transform.rotation.w = np.cos(roll/2) * np.cos(self.yaw_pos/2)
        self.tf_broadcaster.sendTransform(t_fs)

    def publish_joint_states(self, is_moving):
        """Calculates and publishes joint positions with URDF safety limits."""
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = [
            'front_left_hip_joint', 'front_left_thigh_joint', 'front_left_knee_joint',
            'front_right_hip_joint', 'front_right_thigh_joint', 'front_right_knee_joint',
            'rear_left_hip_joint', 'rear_left_thigh_joint', 'rear_left_knee_joint',
            'rear_right_hip_joint', 'rear_right_thigh_joint', 'rear_right_knee_joint'
        ]
        
        t = (time.time() - self.start_time) * 6.0
        body_shift_y = 0.003 * np.sin(t) if is_moving else 0.0
        
        positions = []
        # Verified Hardware Constants
        V_OFFSET, THIGH_NUDGE, KNEE_NUDGE = 1.570795, -1.4, 0.0
        THIGH_GAIN, KNEE_GAIN = 1.3, 0.7   

        for i in range(4):
            z_target = self.config.default_z_ref + self.current_z_offset
            x_target, y_target = 0.0, 0.0 
            
            # --- STATE 1: SITTING ---
            if self.is_sitting:
                if i >= 2: # Rear
                    z_target = self.config.default_z_ref + 0.075 
                    x_target = 0.03 
                else: # Front
                    z_target = self.config.default_z_ref - 0.015
                    x_target = -0.02

            # --- STATE 2: GAIT (WALKING) ---
            elif is_moving:
                phase_offset = 0 if i in [0, 3] else np.pi
                local_t = (t + phase_offset) % (2 * np.pi)
                step_height = 0.04
                sx = np.clip(-self.smooth_v_x * 0.12, -0.05, 0.05)
                sy = np.clip(-self.v_y * 0.12, -0.05, 0.05)

                if local_t < np.pi: # Swing
                    z_target += step_height * np.sin(local_t)
                    x_target += -sx * np.cos(local_t)
                    y_target += -sy * np.cos(local_t)
                else: # Stance
                    progress = (local_t - np.pi) / np.pi
                    x_target += sx * (1.0 - 2.0 * progress)
                    y_target += sy * (1.0 - 2.0 * progress)
                
            # Combine leg Y with body sway
            y_final = y_target - body_shift_y
            
            # Solve Inverse Kinematics
            target = np.array([x_target, y_final, z_target])
            angles = leg_explicit_inverse_kinematics(target, i, self.config)
            
            # 1. Raw Angles from IK
            theta_1, theta_2, theta_3 = angles
            
            # 2. Safety Clipping based on jax.urdf.xacro limits
            # Hip limits: lower="-0.6" upper="0.6" 
            theta_1 = np.clip(theta_1, -0.6, 0.6)
            
            # Thigh limits: lower="-1.2" upper="1.2" 
            theta_2 = np.clip(theta_2, -1.2, 1.2)
            
            # Knee limits: lower="0" upper="1.4" 
            theta_3 = np.clip(theta_3, 0.0, 1.4)

            # 3. Apply Hardware Gains and Offsets
            final_hip = float(theta_1)
            final_thigh = (float(theta_2) * THIGH_GAIN) + THIGH_NUDGE
            final_knee = ((float(theta_3) - V_OFFSET) * KNEE_GAIN) + 0.5 + KNEE_NUDGE
            
            positions.extend([final_hip, final_thigh, final_knee])
            
        js_msg.position = positions
        self.js_pub.publish(js_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JaxBehaviorController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
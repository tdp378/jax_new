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
        self.target_z_offset = 0.0
        self.height_speed = 0.1
        
        self.x_pos, self.y_pos, self.yaw_pos = 0.0, 0.0, 0.0
        self.is_sitting = False

    def cmd_callback(self, msg):
        self.v_x, self.v_y, self.v_yaw = msg.linear.x, msg.linear.y, msg.angular.z
        self.last_cmd_time = time.time()

        # 1. Check for the ACTUAL Home command (All 4 axes must be zero)
        is_home_cmd = all(abs(v) < 0.001 for v in [msg.linear.x, msg.linear.y, msg.angular.z, msg.linear.z])

        # 2. HEIGHT & SIT LOGIC
        if is_home_cmd:
            self.is_sitting = False
            self.target_z_offset = 0.0
            self.z_direction = 0.0
        elif abs(msg.linear.z) > 0.01:
            # This handles 't' and 'b' correctly now
            self.z_direction = 1.0 if msg.linear.z > 0 else -1.0
            self.is_sitting = False 
        else:
            self.z_direction = 0.0

        # 3. DEDICATED SIT FUNCTION (Period Key)
        if msg.linear.x == -1.0 and msg.angular.z == -1.0:
            self.is_sitting = True
        
        # 4. AUTO-STAND
        if abs(self.v_x) > 0.01 or abs(self.v_y) > 0.01:
            self.is_sitting = False
            
    def run_loop(self):
        if (time.time() - self.last_cmd_time) > 0.5:
            self.v_x, self.v_y, self.v_yaw = 0.0, 0.0, 0.0

        dt = self.config.dt
        
        # --- SECTION 3A: VELOCITY RAMPING ---
        ramp_step = self.accel_limit * dt
        if self.smooth_v_x < self.v_x: self.smooth_v_x = min(self.smooth_v_x + ramp_step, self.v_x)
        elif self.smooth_v_x > self.v_x: self.smooth_v_x = max(self.smooth_v_x - ramp_step, self.v_x)

        if self.smooth_v_yaw < self.v_yaw: self.smooth_v_yaw = min(self.smooth_v_yaw + ramp_step * 2, self.v_yaw)
        elif self.smooth_v_yaw > self.v_yaw: self.smooth_v_yaw = max(self.smooth_v_yaw - ramp_step * 2, self.v_yaw)

        # UPDATED: is_moving now includes v_y
        is_moving = abs(self.smooth_v_x) > 0.01 or abs(self.smooth_v_yaw) > 0.01 or abs(self.v_y) > 0.01
        
        # --- SECTION 3B: HEIGHT INTERPOLATION ---
        if self.z_direction != 0.0:
            self.target_z_offset = np.clip(self.target_z_offset + self.z_direction * self.height_speed * dt, -0.08, 0.06)
        
        z_diff = self.target_z_offset - self.current_z_offset
        if abs(z_diff) > 0.001:
            self.current_z_offset += np.sign(z_diff) * self.height_speed * dt

        # --- SECTION 3C: BODY POSE & GAIT SPEED ---
        yaw_threshold = 0.08
        active_yaw = self.smooth_v_yaw if abs(self.smooth_v_yaw) > yaw_threshold else 0.0
        roll = active_yaw * -0.12 if is_moving else 0.0
        
        gait_speed = 6.0 if is_moving else 0.0
        t = (time.time() - self.start_time) * gait_speed
        rhythmic_sway = 0.003 * np.sin(t) if is_moving else 0.0
        body_shift_y = rhythmic_sway 

        # --- SECTION 3D: ODOMETRY ---
        self.x_pos += (self.smooth_v_x * np.cos(self.yaw_pos)) * dt
        self.y_pos += (self.smooth_v_x * np.sin(self.yaw_pos)) * dt
        self.x_pos += (self.v_y * np.cos(self.yaw_pos + np.pi/2)) * dt
        self.y_pos += (self.v_y * np.sin(self.yaw_pos + np.pi/2)) * dt
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

        # --- SECTION 4: JOINT KINEMATICS (STABLE SIT) ---
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = [
            'front_left_hip_joint', 'front_left_thigh_joint', 'front_left_knee_joint',
            'front_right_hip_joint', 'front_right_thigh_joint', 'front_right_knee_joint',
            'rear_left_hip_joint', 'rear_left_thigh_joint', 'rear_left_knee_joint',
            'rear_right_hip_joint', 'rear_right_thigh_joint', 'rear_right_knee_joint'
        ]
        
        positions = []
        # THESE ARE YOUR WORKING CONSTANTS - DO NOT CHANGE
        V_OFFSET, THIGH_NUDGE, KNEE_NUDGE = 1.570795, -1.4, 0.0
        THIGH_GAIN, KNEE_GAIN = 1.3, 0.7   

        for i in range(4):
            # 1. Start with your default working height
            z_target = self.config.default_z_ref + self.current_z_offset
            x_target = 0.0
            y_target = 0.0 
            
            # 2. Refined Sitting Pose (Only if sitting is active)
            if self.is_sitting:
                if i >= 2: # REAR LEGS (Tuck them up toward the body)
                    # We add to z_ref because in your IK, a less negative number is "higher"
                    z_target = self.config.default_z_ref + 0.075 
                    x_target = 0.03 # Shift feet forward to brace
                else: # FRONT LEGS (Extend them slightly)
                    z_target = self.config.default_z_ref - 0.015
                    x_target = -0.02 # Brace forward

            # 3. Gait Math (Only if moving and NOT sitting)
            elif is_moving:
                phase_offset = 0 if i in [0, 3] else np.pi
                local_t = (t + phase_offset) % (2 * np.pi)
                step_height = 0.04
                sx = np.clip(-self.smooth_v_x * 0.12, -0.05, 0.05)
                sy = np.clip(-self.v_y * 0.12, -0.05, 0.05)

                if local_t < np.pi: # SWING
                    z_target += step_height * np.sin(local_t)
                    x_target += -sx * np.cos(local_t)
                    y_target += -sy * np.cos(local_t)
                else: # STANCE
                    progress = (local_t - np.pi) / np.pi
                    x_target += sx * (1.0 - 2.0 * progress)
                    y_target += sy * (1.0 - 2.0 * progress)
                
            # Combine gait/sit Y with the body sway
            y_final = y_target - body_shift_y

            target = np.array([x_target, y_final, z_target])
            angles = leg_explicit_inverse_kinematics(target, i, self.config)
            
            # Applying your specific gains that kept him out of the floor
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

if __name__ == '__main__':
    main()
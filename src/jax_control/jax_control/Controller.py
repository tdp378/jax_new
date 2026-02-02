import rclpy
from rclpy.node import Node
import numpy as np
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from jax_control.Config import Configuration
from jax_control.Kinematics import leg_explicit_inverse_kinematics

class JaxBehaviorController(Node):
    def __init__(self):
        super().__init__('jax_controller')
        self.config = Configuration()
        
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        
        self.timer = self.create_timer(self.config.dt, self.run_loop)
        self.start_time = time.time()
        self.last_cmd_time = time.time()
        
        # MOVEMENT STATE
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_yaw = 0.0
        self.z_direction = 0.0 
        
        # BEHAVIOR STATE
        self.current_z_offset = 0.0
        self.target_z_offset = 0.0
        self.height_increment = 0.08 
        self.is_sitting = False 

    def cmd_callback(self, msg):
        self.v_x = msg.linear.x
        self.v_y = msg.linear.y
        self.v_yaw = msg.angular.z
        self.last_cmd_time = time.time()

        # 1. MANUAL HEIGHT (T/B)
        if abs(msg.linear.z) > 0.01:
            self.z_direction = 1.0 if msg.linear.z > 0 else -1.0
            self.is_sitting = False 
            self.target_z_offset = self.current_z_offset 
        else:
            self.z_direction = 0.0

        # 2. TRIGGER LOGIC
        # If all axes are zero (K pressed)
        if all(abs(v) < 0.001 for v in [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z]):
            self.target_z_offset = 0.0
            self.is_sitting = False
            self.get_logger().info('Pose: STANDING')

        # If M is tapped (Backwards) but we aren't walking, trigger SIT
        elif msg.linear.x < -0.01 and msg.linear.x > -0.5 and abs(msg.linear.y) < 0.01:
            self.is_sitting = True
            self.target_z_offset = 0.0
            self.get_logger().info('Pose: SITTING')

        # If B is held, he will naturally crouch because target_z_offset follows current_z_offset

    def run_loop(self):
        if (time.time() - self.last_cmd_time) > 0.3:
            self.v_x = 0.0
            self.v_y = 0.0
            self.z_direction = 0.0

        # Height Interpolation
        if self.z_direction != 0.0:
            self.current_z_offset += self.z_direction * self.height_increment * self.config.dt
            self.current_z_offset = np.clip(self.current_z_offset, -0.08, 0.06)
            self.target_z_offset = self.current_z_offset
        else:
            diff = self.target_z_offset - self.current_z_offset
            if abs(diff) > 0.001:
                self.current_z_offset += np.sign(diff) * self.height_increment * self.config.dt

        # Disable Sitting if we start moving forward (I)
        if self.v_x > 0.1:
            self.is_sitting = False

        is_moving = abs(self.v_x) > 0.1 or abs(self.v_y) > 0.1
        gait_speed = 6.0 if is_moving else 0.0
        t = (time.time() - self.start_time) * gait_speed
        
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = [
            'front_left_hip_joint', 'front_left_thigh_joint', 'front_left_knee_joint',
            'front_right_hip_joint', 'front_right_thigh_joint', 'front_right_knee_joint',
            'rear_left_hip_joint', 'rear_left_thigh_joint', 'rear_left_knee_joint',
            'rear_right_hip_joint', 'rear_right_thigh_joint', 'rear_right_knee_joint'
        ]
        
        positions = []
        V_OFFSET = 1.570795
        THIGH_NUDGE = -1.4
        KNEE_NUDGE = 0.0
        THIGH_GAIN = 1.3  
        KNEE_GAIN = 0.7   

        for i in range(4):
            z_target = self.config.default_z_ref + self.current_z_offset
            
            # Sit logic: Front stands, Rear drops
            if self.is_sitting:
                if i >= 2: # Rear
                    z_target = self.config.default_z_ref - 0.075 
                else: # Front
                    z_target = self.config.default_z_ref - 0.01

            phase_offset = 0 if i in [0, 3] else np.pi
            local_t = (t + phase_offset) % (2 * np.pi)
            stride_x = np.clip(-self.v_x * 0.1, -0.04, 0.04) 
            stride_y = np.clip(self.v_y * 0.1, -0.03, 0.03)

            x_target = 0.0
            y_target = 0.0

            if is_moving:
                if local_t < np.pi: # SWING
                    z_target += 0.03 * np.sin(local_t)
                    x_target = -stride_x * np.cos(local_t)
                    y_target = -stride_y * np.cos(local_t)
                else: # STANCE
                    progress = (local_t - np.pi) / np.pi
                    x_target = stride_x * (1.0 - 2.0 * progress)
                    y_target = stride_y * (1.0 - 2.0 * progress)
                x_target += (0.03 * self.v_yaw) if i in [0, 1] else -(0.03 * self.v_yaw)

            target = np.array([x_target, y_target, z_target])
            angles = leg_explicit_inverse_kinematics(target, i, self.config)
            
            hip = float(angles[0])
            thigh = (float(angles[1]) * THIGH_GAIN) + THIGH_NUDGE 
            knee_angle_raw = float(angles[2]) - V_OFFSET
            knee = (knee_angle_raw * KNEE_GAIN) + 0.5 + KNEE_NUDGE
            
            positions.extend([hip, thigh, knee])
            
        js_msg.position = positions
        self.js_pub.publish(js_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JaxBehaviorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
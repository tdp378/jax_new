import rclpy
from rclpy.node import Node
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration
from math import atan2, asin

# Relative imports from your cleaned files
from .Config import Configuration
from .Kinematics import four_legs_inverse_kinematics
from .Controller import Controller
from .State import State, BehaviorState
from .Command import Command

def clipped_first_order_filter(input_val, target, max_rate, time_constant):
    """Replacement for missing Utilities.py function"""
    step = (target - input_val) / time_constant
    step = np.clip(step, -max_rate, max_rate)
    return step

def quat_to_euler(q):
    """Converts Gazebo IMU quaternion to [yaw, pitch, roll]"""
    # q is [x, y, z, w]
    x, y, z, w = q.x, q.y, q.z, q.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(t3, t4)
    return [yaw, pitch, roll]

class JaxBrain(Node):
    def __init__(self):
        super().__init__('jax_brain')
        
        # 1. Initialize logic
        self.config = Configuration()
        self.state = State()
        self.command = Command()
        # Use our local IK function
        self.controller = Controller(self.config, four_legs_inverse_kinematics)
        
        # 2. Publishers & Subscribers
        self.joint_pub = self.create_publisher(JointTrajectory, '/legs_controller/joint_trajectory', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_cb, 10)

        # 3. Main Loop (100Hz from your Config.dt)
        self.timer = self.create_timer(self.config.dt, self.timer_callback)
        self.get_logger().info("Jax Brain is standing by...")

    def cmd_vel_cb(self, msg):
        self.command.horizontal_velocity = np.array([msg.linear.x, msg.linear.y])
        self.command.yaw_rate = msg.angular.z
        
        # NEW: Adjust height based on linear.z input
        # We use a small increment (0.01) so he doesn't jump or collapse instantly
        if abs(msg.linear.z) > 0.01:
            self.command.height -= (msg.linear.z * 0.01)
            # Stay within reachable limits (0.16m to 0.24m deep)
            self.command.height = np.clip(self.command.height, -0.25, -0.15)
        
        # Switch behavior based on movement
        if np.linalg.norm(self.command.horizontal_velocity) > 0.05 or abs(self.command.yaw_rate) > 0.05:
            if self.state.behavior_state != BehaviorState.TROT:
                self.state.behavior_state = BehaviorState.TROT
                self.get_logger().info("Switching to TROT")
        else:
            if self.state.behavior_state != BehaviorState.REST:
                self.state.behavior_state = BehaviorState.REST
                self.get_logger().info(f"RESTing at height: {self.command.height:.2f}")

    def imu_cb(self, msg):
        self.state.euler_orientation = quat_to_euler(msg.orientation)

    def timer_callback(self):
        # Run the math step
        self.controller.run(self.state, self.command)
        
        # Build the trajectory message
        msg = JointTrajectory()
        # Order: FR, FL, RR, RL (Matches your IK matrix columns)
        msg.joint_names = [
            'front_right_hip_joint', 'front_right_thigh_joint', 'front_right_calf_joint',
            'front_left_hip_joint', 'front_left_thigh_joint', 'front_left_calf_joint',
            'rear_right_hip_joint', 'rear_right_thigh_joint', 'rear_right_calf_joint',
            'rear_left_hip_joint', 'rear_left_thigh_joint', 'rear_left_calf_joint'
        ]
        
        point = JointTrajectoryPoint()
        # Flatten the (3,4) IK matrix into a 1D list of 12 angles
        point.positions = self.state.joint_angles.T.flatten().tolist()
        point.time_from_start = Duration(sec=0, nanosec=int(self.config.dt * 1e9))
        
        msg.points.append(point)
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JaxBrain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
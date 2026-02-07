import rclpy
from rclpy.node import Node
import numpy as np
import time

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Internal Jax imports
from jax_control.Config import Configuration
from jax_control.Kinematics import leg_explicit_inverse_kinematics
from jax_control.State import State
from jax_control.Command import Command
from jax_control.Gaits import GaitController
from jax_control.SwingLegController import SwingController
from jax_control.StanceController import StanceController

class JaxBehaviorController(Node):
    def __init__(self):
        super().__init__('jax_controller')
        
        self.config = Configuration()
        self.state = State()
        self.command = Command()
        
        self.gait_controller = GaitController(self.config)
        self.swing_controller = SwingController(self.config)
        self.stance_controller = StanceController(self.config)
        
        # --- MISSING VARIABLES START HERE ---
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_yaw = 0.0
        # --- MISSING VARIABLES END HERE ---

        # Match QoS Reliability to Best Effort for robot_state_publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.js_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.timer = self.create_timer(self.config.dt, self.run_loop)
        self.last_cmd_time = time.time()
        
        # Initialize feet to default stance
        for i in range(4):
            self.state.foot_locations[:, i] = self.config.default_stance[:, i]
            
        self.get_logger().info("Jax Controller Initialized and Ready.")

    def cmd_callback(self, msg):
        self.last_cmd_time = time.time()
        # Ensure command values are updated correctly
        self.command.horizontal_velocity = np.array([msg.linear.x, msg.linear.y])
        self.command.yaw_rate = msg.angular.z
        
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.get_logger().info(f"Moving - X: {msg.linear.x:.2f}, Yaw: {msg.angular.z:.2f}", once=True)

    def run_loop(self):
        moving = np.linalg.norm(self.command.horizontal_velocity) > 0.01 or \
                 abs(self.command.yaw_rate) > 0.01
        
        if (time.time() - self.last_cmd_time) > 0.5:
            moving = False

        if moving:
            self.state.ticks += 1
            # Update the global position variables
            self.pos_x += float(self.command.horizontal_velocity[0] * self.config.dt)
            self.pos_y += float(self.command.horizontal_velocity[1] * self.config.dt)
            self.calculate_foot_trajectories()
        else:
            # Stand still logic
            for i in range(4):
                self.state.foot_locations[0, i] = self.config.LEG_ORIGINS[0, i]
                self.state.foot_locations[1, i] = self.config.LEG_ORIGINS[1, i]
                self.state.foot_locations[2, i] = self.config.default_z_ref
        
        self.broadcast_body_transform()
        self.publish_joint_states()

    def calculate_foot_trajectories(self):
        contact_modes = self.gait_controller.contacts(self.state.ticks)
        
        for i in range(4):
            if contact_modes[i] == 1: # STANCE
                new_pos = self.stance_controller.next_foot_location(i, self.state, self.command)
            else: # SWING
                # Calculate progress through swing (0.0 to 1.0)
                swing_prop = self.gait_controller.subphase_ticks(self.state.ticks) / self.config.swing_ticks
                new_pos = self.swing_controller.next_foot_location(swing_prop, i, self.state, self.command)
            
            self.state.foot_locations[:, i] = new_pos

    def broadcast_body_transform(self):
        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'        # Must match RViz "Fixed Frame"
            t.child_frame_id = 'base_link'    # Must match URDF root link
        
            # We use float() to ensure ROS doesn't complain about numpy types
            t.transform.translation.x = float(self.pos_x)
            t.transform.translation.y = float(self.pos_y)
            t.transform.translation.z = float(abs(self.config.default_z_ref))
        
          # For now, keep rotation neutral to avoid complex math errors
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.14
            t.transform.rotation.w = 1.0 

            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f"TF Broadcast Failed: {e}")

    def publish_joint_states(self):
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = [
            'front_left_hip_joint', 'front_left_thigh_joint', 'front_left_knee_joint',
            'front_right_hip_joint', 'front_right_thigh_joint', 'front_right_knee_joint',
            'rear_left_hip_joint', 'rear_left_thigh_joint', 'rear_left_knee_joint',
            'rear_right_hip_joint', 'rear_right_thigh_joint', 'rear_right_knee_joint'
        ]
        
        positions = []
        for i in range(4):
            # FIXED INDENTATION: Everything inside the loop to calculate for all 4 legs
            foot_pos_body = self.state.foot_locations[:, i]
            shoulder_pos = self.config.LEG_ORIGINS[:, i]
            
            # Convert to local shoulder coordinates for IK
            foot_pos_local = foot_pos_body - shoulder_pos
            
            angles = leg_explicit_inverse_kinematics(foot_pos_local, i, self.config)
            positions.extend([float(a) for a in angles])
        
        if len(positions) == 12:
            js_msg.position = positions
            self.js_pub.publish(js_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = JaxBehaviorController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from jax_interfaces.msg import JointSpace 

class SimBridge(Node):
    def __init__(self):
        super().__init__('sim_bridge')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription = self.create_subscription(
            JointSpace,
            'jax_joint_commands',
            self.listener_callback,
            10)
        
        # EXACT names derived from your RViz error sidebar
        # Note: We use 'knee_joint' or 'calf_joint' based on your URDF grep earlier
        self.joint_names = [
            'front_left_hip_joint', 'front_left_thigh_joint', 'front_left_knee_joint',
            'front_right_hip_joint', 'front_right_thigh_joint', 'front_right_knee_joint',
            'rear_left_hip_joint', 'rear_left_thigh_joint', 'rear_left_knee_joint',
            'rear_right_hip_joint', 'rear_right_thigh_joint', 'rear_right_knee_joint'
        ]

    def listener_callback(self, msg):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        
        # Extracting 3 joints per leg from your message
        fl = [msg.fl_leg.hip, msg.fl_leg.thigh, msg.fl_leg.calf]
        fr = [msg.fr_leg.hip, msg.fr_leg.thigh, msg.fr_leg.calf]
        rl = [msg.rl_leg.hip, msg.rl_leg.thigh, msg.rl_leg.calf]
        rr = [msg.rr_leg.hip, msg.rr_leg.thigh, msg.rr_leg.calf]
        
        # Combine in the sequence defined in self.joint_names
        joint_state.position = fl + fr + rl + rr
        
        self.publisher_.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = SimBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
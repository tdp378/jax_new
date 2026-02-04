import rclpy
from rclpy.node import Node
import numpy as np

# Import your custom messages and hardware logic
from jax_interfaces.msg import JointSpace
from jax_control.Config import Configuration, Leg_linkage
from jax_hardware.hardwareinterface import HardwareInterface

class JaxHardwareBridge(Node):
    def __init__(self):
        super().__init__('jax_hardware_bridge')
        
        # Initialize Config and Hardware Math
        self.config = Configuration()
        self.linkage = Leg_linkage(self.config)
        self.hw = HardwareInterface(self.linkage)
        
        # Subscribe to the JointSpace commands from your controller
        self.subscription = self.create_subscription(
            JointSpace,
            'jax_joint_commands', 
            self.joint_callback,
            10)
        
        self.get_logger().info('Jax Hardware Bridge has started. Awaiting JointSpace messages...')

    def joint_callback(self, msg):
        try:
            # We explicitly pull each named field from your Angle.msg
            # Columns are: 0:FR, 1:FL, 2:RR, 3:RL
            joint_angles = np.array([
                [msg.fr_leg.hip, msg.fl_leg.hip, msg.rr_leg.hip, msg.rl_leg.hip],
                [msg.fr_leg.thigh, msg.fl_leg.thigh, msg.rr_leg.thigh, msg.rl_leg.thigh],
                [msg.fr_leg.calf, msg.fl_leg.calf, msg.rr_leg.calf, msg.rl_leg.calf]
            ])
            
            # This sends the 3x4 matrix to your hardwareinterface.py math
            self.hw.set_actuator_postions(joint_angles)
            
            # Success logging
            fr_h = round(self.hw.servo_angles[0,0], 1)
            fr_t = round(self.hw.servo_angles[1,0], 1)
            fr_c = round(self.hw.servo_angles[2,0], 1)
            self.get_logger().info(f'FR Servos -> Hip: {fr_h}, Thigh: {fr_t}, Calf: {fr_c}')


        except Exception as e:
            self.get_logger().error(f'Failed to process joint angles: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = JaxHardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
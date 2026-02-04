import rclpy
from rclpy.node import Node
from jax_interfaces.msg import JointSpace, Angle
import math
import time

class BreatheTest(Node):
    def __init__(self):
        super().__init__('breathe_test')
        self.publisher_ = self.create_publisher(JointSpace, 'jax_joint_commands', 10)
        self.timer = self.create_timer(0.02, self.timer_callback) # 50Hz
        self.start_time = time.time()

    def timer_callback(self):
        elapsed = time.time() - self.start_time
        
        # Create a smooth sine wave (-0.3 to +0.3 radians)
        # Higher frequency (3.0) for a more noticeable 'breath'
        wave = 0.3 * math.sin(elapsed * 3.0) 
        
        msg = JointSpace()
        
        # Helper to fill leg data based on JAX URDF structure
        def set_leg(hip, thigh_osc, calf_osc):
            a = Angle()
            a.hip = hip
            
            # URDF OFFSET LOGIC:
            # 0.0 is horizontal (the "plank").
            # ~0.8 to 1.2 radians pushes the thigh down toward the floor.
            a.thigh = 0.0 + thigh_osc 
            
            # The calf needs to bend INWARD. 
            # In your URDF, a positive value usually bends the knee.
            # We use a large offset to create the standing 'V' shape.
            a.calf = 1.1 + calf_osc
            
            return a

        # To make a 'breathe' look real:
        # As the thigh moves forward (+wave), the calf should bend more (+wave)
        # to keep the foot moving mostly vertically.
        msg.fr_leg = set_leg(0.0, wave, wave)
        msg.fl_leg = set_leg(0.0, wave, wave)
        msg.rr_leg = set_leg(0.0, wave, wave)
        msg.rl_leg = set_leg(0.0, wave, wave)
        
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = BreatheTest()
    print("Sending 'Breathe' commands... Jax should now be standing and crouching.")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
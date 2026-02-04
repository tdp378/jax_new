import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
---------------------------
   JAX STANDARD TWIST GRID
---------------------------
Moving around:      Speed Control:
   u    i    o      w : increase linear speed
   j    k    l      x : decrease linear speed
   m    ,    .      e : increase angular speed
                    c : decrease angular speed

Jax Height:         Stop:
   t : Up           k or Space
   b : Down

(Note: 'i' is Forward. 'w' only changes speed settings!)
"""

class JaxStandardTeleop(Node):
    def __init__(self):
        super().__init__('jax_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Speed settings
        self.speed = 0.2
        self.turn = 1.0
        
        self.timer = self.create_timer(0.05, self.publish_command)
        print(msg)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_command(self):
        key = self.get_key()
        twist = Twist()

        # --- Speed Adjustments ---
        if key == 'w':
            self.speed *= 1.1
            print(f"\rCurrent linear speed: {self.speed:.2f}  ", end="")
        elif key == 'x':
            self.speed *= 0.9
            print(f"\rCurrent linear speed: {self.speed:.2f}  ", end="")
        elif key == 'e':
            self.turn *= 1.1
            print(f"\rCurrent angular speed: {self.turn:.2f} ", end="")
        elif key == 'c':
            self.turn *= 0.9
            print(f"\rCurrent angular speed: {self.turn:.2f} ", end="")

        # --- Movement Grid ---
        elif key == 'i': twist.linear.x = self.speed
        elif key == ',': twist.linear.x = -self.speed
        elif key == 'j': twist.angular.z = self.turn
        elif key == 'l': twist.angular.z = -self.turn
        elif key == 'u': twist.linear.x = self.speed; twist.angular.z = self.turn
        elif key == 'o': twist.linear.x = self.speed; twist.angular.z = -self.turn
        elif key == 'm': twist.linear.x = -self.speed; twist.angular.z = -self.turn
        elif key == '.': twist.linear.x = -self.speed; twist.angular.z = self.turn
        
        # --- Height (t/b) ---
        elif key == 't': twist.linear.z = 1.0
        elif key == 'b': twist.linear.z = -1.0
        
        # --- Stop ---
        elif key == 'k' or key == ' ':
            twist = Twist()

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JaxStandardTeleop()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        node.destroy_node(); rclpy.shutdown()
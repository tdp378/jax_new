import rclpy
from rclpy.node import Node
from jax_interfaces.msg import Command
import sys, select, termios, tty

msg = """
---------------------------
   Control Your Jax!
---------------------------
Moving around:      Adjust Stance:
   w                r : Increase Height
 a s d              f : Decrease Height
                    t : Pitch Up / g : Pitch Down
 Space : TROT ON/OFF  y : Roll Right / h : Roll Left

Current Command Status:
"""

class JaxTeleop(Node):
    def __init__(self):
        super().__init__('jax_teleop')
        self.publisher_ = self.create_publisher(Command, 'jax_commands', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Default State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.height = -0.18
        self.pitch = 0.0
        self.roll = 0.0
        self.trot = 0
        
        # Fast timer for publishing, slow keyboard check
        self.timer = self.create_timer(0.05, self.publish_command)
        print(msg)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        # Use a very short timeout so the loop doesn't hang
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_command(self):
        key = self.get_key()

        # Movement Logic
        if key == 'w': self.x = 0.1
        elif key == 's': self.x = -0.1
        elif key == 'a': self.th = 0.5
        elif key == 'd': self.th = -0.5
        
        # Stance Logic
        elif key == 'r': self.height -= 0.005
        elif key == 'f': self.height += 0.005
        elif key == 't': self.pitch += 0.02
        elif key == 'g': self.pitch -= 0.02
        elif key == 'y': self.roll += 0.02
        elif key == 'h': self.roll -= 0.02
        
        # Toggle Logic
        elif key == ' ': # Spacebar
            self.trot = 1 if self.trot == 0 else 0
            status = "ACTIVE" if self.trot else "OFF"
            print(f"\r>> Trot Mode: {status}   ", end="")
            
        elif key == '\x03': # CTRL-C
            self.x = 0.0; self.th = 0.0; self.trot = 0
            self.send_final_msg()
            rclpy.shutdown()
            return
        
        else:
            # Stop moving if no key is pressed (Zero the velocity)
            self.x = 0.0
            self.y = 0.0
            self.th = 0.0

        # Create and publish the message
        cmd = Command()
        cmd.horizontal_velocity = [float(self.x), float(self.y)]
        cmd.yaw_rate = float(self.th)
        cmd.height = float(self.height)
        cmd.pitch = float(self.pitch)
        cmd.roll = float(self.roll)
        cmd.trotting_active = int(self.trot)
        self.publisher_.publish(cmd)

    def send_final_msg(self):
        # Sends one last 'stop' command on exit
        cmd = Command()
        cmd.height = -0.18
        cmd.trotting_active = 0
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = JaxTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
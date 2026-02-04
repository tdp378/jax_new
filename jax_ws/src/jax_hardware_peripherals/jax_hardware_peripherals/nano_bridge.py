import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, ColorRGBA, Bool, Int32MultiArray
import serial

class NanoBridge(Node):
    def __init__(self):
        super().__init__('nano_bridge')
        
        # 1. Serial Setup (Hardwired to Pi GPIO pins)
        # Remember to disable the Serial Console in raspi-config first!
        try:
            self.ser = serial.Serial('/dev/serial0', 115200, timeout=0.1)
            self.get_logger().info("Successfully connected to Jax Peripheral Hub on /dev/serial0")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")

        # 2. Publishers (Data coming FROM Jax's body)
        self.batt_pub = self.create_publisher(Float64, '/jax/battery_v', 10)
        self.rail_pub = self.create_publisher(Float64, '/jax/rail_v', 10)
        self.cal_pub  = self.create_publisher(Float64, '/jax/imu_cal', 10)
        self.foot_pub = self.create_publisher(Int32MultiArray, '/jax/foot_sensors', 10)
        # Optional: Add publishers for Heading/Roll/Pitch here later if needed

        # 3. Subscribers (Commands going TO Jax's body)
        self.create_subscription(ColorRGBA, '/jax/eyes', self.eye_cb, 10)
        self.create_subscription(Bool, '/jax/estop', self.estop_cb, 10)
        self.create_subscription(Int32MultiArray, '/jax/servos', self.servo_cb, 10)
        
        # 4. Main Loop (Runs at 20Hz to match the Nano's output)
        self.create_timer(0.05, self.update)

    def update(self):
        if self.ser.in_waiting > 0:
            try:
                # Read line and clean it up
                raw_line = self.ser.readline().decode('utf-8').strip()
                data = raw_line.split(',')
                
                # Check if we got all 10 expected values:
                # [0:batt, 1:rail, 2:head, 3:roll, 4:pitch, 5:cal, 6:FL, 7:FR, 8:RL, 9:RR]
                if len(data) == 10:
                    # Publish Voltages
                    self.batt_pub.publish(Float64(data=float(data[0])))
                    self.rail_pub.publish(Float64(data=float(data[1])))
                    
                    # Publish Calibration Status (0-3)
                    self.cal_pub.publish(Float64(data=float(data[5])))
                    
                    # Publish Foot Sensors as an array
                    foot_msg = Int32MultiArray()
                    foot_msg.data = [int(data[6]), int(data[7]), int(data[8]), int(data[9])]
                    self.foot_pub.publish(foot_msg)
            except (ValueError, UnicodeDecodeError):
                # Ignore garbled serial data
                pass
            except Exception as e:
                self.get_logger().warn(f"Bridge Update Error: {e}")

    def eye_cb(self, msg):
        """Sends eye color to Nano in format <R,G,B>"""
        cmd = f"<{int(msg.r)},{int(msg.g)},{int(msg.b)}>\n"
        self.ser.write(cmd.encode())

    def estop_cb(self, msg):
        """Sends ! to Nano to cut PWM to all 16 channels"""
        if msg.data is True:
            self.ser.write(b"!\n")
            self.get_logger().warn("!!! EMERGENCY STOP SENT TO SERVO RAIL !!!")

    def servo_cb(self, msg):
        """Expects Int32MultiArray with 2 values: [channel, angle]"""
        if len(msg.data) == 2:
            cmd = f"[{msg.data[0]},{msg.data[1]}]\n"
            self.ser.write(cmd.encode())

def main(args=None):
    rclpy.init(args=args)
    node = NanoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Nano Bridge...")
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
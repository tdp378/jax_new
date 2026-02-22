import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from PIL import Image, ImageDraw, ImageFont
# Using the full package path to avoid ModuleNotFoundError
from jax_hardware_peripherals import LCD_1inch47

class JaxLCDNode(Node):
    def __init__(self):
        super().__init__('jax_lcd_node')
        
        # Initialize LCD
        self.disp = LCD_1inch47.LCD_1inch47()
        self.disp.Init()
        self.disp.clear()
        
        # State variables
        self.battery_v = 0.0
        self.rail_v = 0.0
        
        # Subscribers
        self.create_subscription(Float64, '/battery_voltage', self.batt_cb, 10)
        self.create_subscription(Float64, '/rail_voltage', self.rail_cb, 10)
        
        # 2Hz Timer for display refresh
        self.timer = self.create_timer(0.5, self.update_display)
        self.get_logger().info("Jax LCD Node Started - Waiting for Nano data...")

    def batt_cb(self, msg):
        self.battery_v = msg.data

    def rail_cb(self, msg):
        self.rail_v = msg.data

    def update_display(self):
        # Create image (Landscape layout)
        # Note: self.disp.height is the long side (320), width is short (172)
        image = Image.new("RGB", (self.disp.height, self.disp.width), "BLACK")
        draw = ImageDraw.Draw(image)
        
        # --- UI DRAWING ---
        # Header
        draw.rectangle([0, 0, 320, 30], fill="BLUE")
        draw.text((80, 5), "JAX SYSTEM MONITOR", fill="WHITE")
        
        # Battery Section
        batt_color = "GREEN" if self.battery_v > 14.5 else "RED"
        draw.text((20, 50), f"MAIN BATT:  {self.battery_v:.2f} V", fill=batt_color)
        
        # Rail Section
        rail_color = "YELLOW" if self.rail_v > 4.8 else "ORANGE"
        draw.text((20, 90), f"SERVO RAIL: {self.rail_v:.2f} V", fill=rail_color)
        
        # Status Footer
        status = "NOMINAL" if self.battery_v > 12.0 else "LOW POWER"
        draw.text((20, 140), f"STATUS: {status}", fill="CYAN")
        # ------------------
        
        # Rotate for the 1.47" screen alignment and show
        image = image.transpose(Image.ROTATE_270)
        self.disp.ShowImage(image)
        
        # Optional: Uncomment on PC to verify UI layout
        # image.save("/tmp/jax_lcd_debug.png")
        
        
        # Save to your home folder so you can look at it
        # image.save("jax_display_output.png")

def main():
    rclpy.init()
    node = JaxLCDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
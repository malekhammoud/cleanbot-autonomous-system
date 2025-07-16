import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Serial connection setup
        self.serial_port = serial.Serial(
            port='/dev/ttyACM0',  # You may need to change this
            baudrate=9600,
            timeout=1
        )
        time.sleep(2)  # Wait for serial connection to establish
        
        # ROS Subscription
        self.subscription = self.create_subscription(
            String,
            'robot_command',
            self.command_callback,
            10)
    
    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        try:
            self.serial_port.write((command+"\n").encode())
            self.get_logger().info(f'Sent to Pico: {command.strip()}')
            
            # Read response from Pico (optional)
            response = self.serial_port.readline().decode().strip()
            if response:
                self.get_logger().info(f'Pico response: {response}')
                    
        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')
    
    def __del__(self):
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

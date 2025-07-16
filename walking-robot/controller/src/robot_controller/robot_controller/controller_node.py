import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("RPi.GPIO not available - running in simulation mode")

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Initialize GPIO if available
        if GPIO_AVAILABLE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Stepper motor pins
            self.stepper_pins = [17, 18, 27, 22]
            for pin in self.stepper_pins:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, 0)
            
            # Stepper motor sequence
            self.stepper_sequence = [
                [1, 0, 0, 1],
                [1, 0, 0, 0],
                [1, 1, 0, 0],
                [0, 1, 0, 0],
                [0, 1, 1, 0],
                [0, 0, 1, 0],
                [0, 0, 1, 1],
                [0, 0, 0, 1]
            ]
            
            # Servo configuration - 8 servos as per input interface
            self.servo_pins = {
                'A': 4,   # Servo A (leg1_upper)
                'B': 5,   # Servo B (leg1_lower)
                'C': 6,   # Servo C (leg3_upper)
                'D': 13,  # Servo D (leg3_lower)
                'E': 19,  # Servo E (leg2_upper)
                'F': 26,  # Servo F (leg2_lower)
                'G': 21,  # Servo G (leg4_lower)
                'H': 20   # Servo H (leg4_upper)
            }
            
            # Initialize servo PWMs
            self.servos = {}
            for servo_name, pin in self.servo_pins.items():
                GPIO.setup(pin, GPIO.OUT)
                pwm = GPIO.PWM(pin, 50)  # 50Hz for servo
                pwm.start(0)
                self.servos[servo_name] = pwm
                
            self.get_logger().info("GPIO initialized for direct servo control")
        else:
            self.get_logger().warning("Running in simulation mode - no GPIO control")
        
        # ROS Subscription - listening to robot_command topic
        self.subscription = self.create_subscription(
            String,
            'robot_command',
            self.command_callback,
            10)
    
    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        try:
            # Handle different command types
            if command == 'L':
                self.rotate_stepper('left', 512)
            elif command == 'R':
                self.rotate_stepper('right', 512)
            elif command.startswith('S') and ':' in command:
                # Single servo command: S1:90
                parts = command.split(':')
                servo_num = int(parts[0][1:])
                angle = int(parts[1])
                servo_map = {1: 'A', 2: 'B', 3: 'C', 4: 'D', 5: 'E', 6: 'F', 7: 'G', 8: 'H'}
                if servo_num in servo_map:
                    self.set_servo_angle(servo_map[servo_num], angle)
            elif ';' in command:
                # Multi-servo command from GUI: "A 90;B 45;C 120;..."
                self.handle_multi_servo_command(command)
            else:
                self.get_logger().warning(f'Unknown command format: {command}')
                    
        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')
    
    def handle_multi_servo_command(self, command):
        """Handle multi-servo commands from GUI"""
        if not GPIO_AVAILABLE:
            self.get_logger().info(f'Simulation: would execute multi-servo command: {command}')
            return
            
        # Parse commands like "A 90;B 45;C 120;D 60;E 85;F 95;G 180;H 0;"
        servo_commands = command.strip().split(';')
        
        for servo_cmd in servo_commands:
            servo_cmd = servo_cmd.strip()
            if not servo_cmd:
                continue
                
            parts = servo_cmd.split(' ')
            if len(parts) == 2:
                servo_name = parts[0]
                try:
                    angle = int(parts[1])
                    if servo_name in self.servo_pins:
                        self.set_servo_angle(servo_name, angle)
                    else:
                        self.get_logger().warning(f'Unknown servo: {servo_name}')
                except ValueError:
                    self.get_logger().error(f'Invalid angle value: {parts[1]}')
    
    def set_servo_angle(self, servo_name, angle):
        """Set servo angle (0-180 degrees)"""
        if not GPIO_AVAILABLE:
            self.get_logger().info(f'Simulation: would set servo {servo_name} to {angle}°')
            return
            
        # Skip disabled servos (angle 999 is used to disable)
        if angle == 999:
            self.get_logger().info(f'Servo {servo_name} disabled')
            return
            
        # Clamp angle to valid range
        angle = max(0, min(180, angle))
        
        # Calculate duty cycle: 2.5% to 12.5% for 0° to 180°
        duty_cycle = (angle / 18.0) + 2.5
        
        if servo_name in self.servos:
            self.servos[servo_name].ChangeDutyCycle(duty_cycle)
            self.get_logger().info(f'Set servo {servo_name} to {angle}° (duty: {duty_cycle:.1f}%)')
        else:
            self.get_logger().error(f'Servo {servo_name} not found')
    
    def rotate_stepper(self, direction, steps):
        """Rotate stepper motor"""
        if not GPIO_AVAILABLE:
            self.get_logger().info(f'Simulation: would rotate stepper {direction} for {steps} steps')
            return
            
        self.get_logger().info(f'Rotating stepper {direction} for {steps} steps')
        
        for i in range(steps):
            for step in range(8):
                for pin_idx in range(4):
                    if direction == 'left':
                        GPIO.output(self.stepper_pins[pin_idx], self.stepper_sequence[step][pin_idx])
                    else:  # right
                        GPIO.output(self.stepper_pins[pin_idx], self.stepper_sequence[7-step][pin_idx])
                time.sleep(0.001)
    
    def __del__(self):
        if GPIO_AVAILABLE:
            # Stop all servos
            for servo in self.servos.values():
                servo.stop()
            # Clean up GPIO
            GPIO.cleanup()
            self.get_logger().info("GPIO cleaned up")

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

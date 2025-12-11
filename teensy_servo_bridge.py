#!/usr/bin/env python3
"""
ROS2 Teensy Servo Bridge
Subscribes to /servo_cmd topic and sends commands to Teensy via Serial

Node: teensy_servo_bridge
Subscribes to: /servo_cmd (String) - "left_angle right_angle"
Serial Output: Sends commands to Teensy 4.0 via USB
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class TeensyServoBridge(Node):
    def __init__(self):
        super().__init__('teensy_servo_bridge')
        
        # Declare parameters with defaults
        self.declare_parameter('teensy_port', '/dev/ttyACM1')  # Usually ACM1, Arduino is ACM0
        self.declare_parameter('teensy_baudrate', 115200)
        
        # Get parameters
        port = self.get_parameter('teensy_port').value
        baudrate = self.get_parameter('teensy_baudrate').value
        
        # Subscribe to servo commands
        self.servo_subscription = self.create_subscription(
            String,
            '/servo_cmd',
            self.servo_callback,
            10)
        
        # Initialize serial connection to Teensy
        try:
            self.teensy_serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=1
            )
            self.teensy_serial.flush()
            time.sleep(2)  # Wait for Teensy to initialize
            self.get_logger().info(f'Connected to Teensy on {port}')
            
            # Read startup message from Teensy
            time.sleep(0.5)
            while self.teensy_serial.in_waiting:
                startup_msg = self.teensy_serial.readline().decode('utf-8').strip()
                self.get_logger().info(f'Teensy: {startup_msg}')
        
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Teensy: {e}')
            self.teensy_serial = None
        
        # Create timer to read Teensy responses
        self.read_timer = self.create_timer(0.1, self.read_teensy)
        
        self.get_logger().info('Teensy Servo Bridge Started!')
    
    def servo_callback(self, msg):
        """Receive servo commands and forward to Teensy"""
        if self.teensy_serial is None:
            self.get_logger().warn('Teensy not connected!')
            return
        
        try:
            # Forward command to Teensy
            cmd = msg.data + '\n'
            self.teensy_serial.write(cmd.encode('utf-8'))
            
            # Optional: Log commands (can be noisy)
            # self.get_logger().debug(f'Sent to Teensy: {msg.data}')
        
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
    
    def read_teensy(self):
        """Read any messages from Teensy"""
        if self.teensy_serial is None:
            return
        
        try:
            while self.teensy_serial.in_waiting:
                line = self.teensy_serial.readline().decode('utf-8').strip()
                if line:
                    self.get_logger().info(f'Teensy: {line}')
        
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read error: {e}')
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        # Send stop command before closing (90 90 = both servos stop)
        if self.teensy_serial is not None:
            try:
                self.teensy_serial.write(b'94 94\n')  # Stop value
                time.sleep(0.1)
                self.teensy_serial.close()
            except:
                pass
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    bridge = TeensyServoBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

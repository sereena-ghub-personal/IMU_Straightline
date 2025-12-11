#!/usr/bin/env python3
"""
ROS2 Serial Reader for Arduino (IMU + Ultrasonic)
Reads both IMU orientation and ultrasonic distance data from Arduino

Node: arduino_serial_reader
Publishes:
  - /imu_data (String): "yaw pitch roll"
  - /ultrasonic_scan (String): "distance,angle"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class ArduinoSerialReader(Node):
    def __init__(self):
        super().__init__('arduino_serial_reader')
        
        # Declare parameters
        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        
        # Get parameters
        port = self.get_parameter('arduino_port').value
        baudrate = self.get_parameter('baudrate').value
        
        # Create publishers
        self.imu_publisher = self.create_publisher(String, '/imu_data', 10)
        self.ultrasonic_publisher = self.create_publisher(String, '/ultrasonic_scan', 10)
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=1
            )
            self.ser.flush()
            self.get_logger().info(f"Connected to Arduino on {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None
        
        # Create timer to read serial data (100Hz)
        if self.ser:
            timer_period = 0.01  # 10ms = 100Hz
            self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        if self.ser is not None:
            try:
                if self.ser.in_waiting:
                    # Read line from Arduino
                    line = self.ser.readline().decode('utf-8').rstrip()
                    
                    if line:
                        # Parse based on prefix
                        if line.startswith('YAW '):
                            # IMU data: "YAW yaw pitch roll"
                            data = line[4:]  # Remove "YAW " prefix
                            msg = String()
                            msg.data = data
                            self.imu_publisher.publish(msg)
                            # Optional debug logging (can be noisy)
                            # self.get_logger().debug(f'IMU: {data}')
                        
                        elif line.startswith('DIST '):
                            # Ultrasonic data: "DIST distance,angle"
                            data = line[5:]  # Remove "DIST " prefix
                            msg = String()
                            msg.data = data
                            self.ultrasonic_publisher.publish(msg)
                            self.get_logger().debug(f'Ultrasonic: {data}')
            
            except serial.SerialException:
                self.get_logger().error("Serial connection lost.")
                self.ser.close()
                self.ser = None

def main(args=None):
    rclpy.init(args=args)
    reader = ArduinoSerialReader()
    
    try:
        rclpy.spin(reader)
    except KeyboardInterrupt:
        pass
    finally:
        if reader.ser:
            reader.ser.close()
        reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

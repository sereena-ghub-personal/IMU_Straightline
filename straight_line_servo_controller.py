#!/usr/bin/env python3
"""
ROS2 Straight Line Controller for Continuous Rotation Servos
Subscribes to IMU data and publishes servo commands to maintain straight line

Node: straight_line_servo_controller
Subscribes to: /imu_data (String) - "yaw pitch roll"
Publishes to: /servo_cmd (String) - "left_angle right_angle"

For continuous rotation servos:
- 90 = stop
- 0-89 = rotate one direction (faster as value decreases)
- 91-180 = rotate other direction (faster as value increases)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math

class StraightLineServoController(Node):
    def __init__(self):
        super().__init__('straight_line_servo_controller')
        
        # Subscribe to IMU data
        self.imu_subscription = self.create_subscription(
            String,
            '/imu_data',
            self.imu_callback,
            10)
        
        # Publish servo commands
        self.servo_publisher = self.create_publisher(
            String,
            '/servo_cmd',
            10)
        
        # Control parameters
        self.initial_yaw = None
        self.current_yaw = 0.0
        self.is_moving = False
        
        # PID Controller parameters (tune these for your robot)
        self.Kp = 0.5   # Proportional gain (lower for servos - they're more sensitive)
        self.Ki = 0.05  # Integral gain
        self.Kd = 0.2   # Derivative gain
        
        self.integral_error = 0.0
        self.last_error = 0.0
        
        # Servo parameters
        self.servo_stop = 94        # Servo value for stop (adjust if needed)
        self.base_speed_offset = 10  # How much to offset from stop for movement
                                     # e.g., 94 - 10 = 84 for forward
        
        # Maximum correction (prevents extreme servo differences)
        self.max_correction = 15
        
        # Create timer for control loop (50Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('Straight Line Servo Controller Started!')
        self.get_logger().info('Commands:')
        self.get_logger().info('  ros2 service call /start_straight std_srvs/srv/Trigger')
        self.get_logger().info('  ros2 service call /stop_straight std_srvs/srv/Trigger')
        
        # Create services for starting/stopping
        from std_srvs.srv import Trigger
        self.start_service = self.create_service(
            Trigger,
            '/start_straight',
            self.start_callback)
        
        self.stop_service = self.create_service(
            Trigger,
            '/stop_straight',
            self.stop_callback)
    
    def imu_callback(self, msg):
        """Process incoming IMU data"""
        try:
            data = msg.data.split()
            if len(data) == 3:
                self.current_yaw = float(data[0])
                
                # Set initial yaw on first reading
                if self.initial_yaw is None:
                    self.initial_yaw = self.current_yaw
                    self.get_logger().info(f'Initial yaw set to: {self.initial_yaw:.2f}°')
        
        except ValueError:
            self.get_logger().warn('Invalid IMU data format')
    
    def normalize_angle(self, angle):
        """Normalize angle to [-180, 180] range"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def control_loop(self):
        """Main control loop - runs at 50Hz"""
        if not self.is_moving or self.initial_yaw is None:
            return
        
        # Calculate yaw error (how much we've drifted)
        yaw_error = self.normalize_angle(self.current_yaw - self.initial_yaw)
        
        # PID Control
        P = self.Kp * yaw_error
        
        self.integral_error += yaw_error * 0.02  # dt = 0.02s (50Hz)
        I = self.Ki * self.integral_error
        
        D = self.Kd * (yaw_error - self.last_error) / 0.02
        
        # Total correction
        correction = P + I + D
        
        # Limit correction
        correction = max(-self.max_correction, min(self.max_correction, correction))
        
        # Calculate servo angles
        # Base movement: both servos at (stop - offset) for forward
        # Correction: adjust each servo to turn
        
        # For continuous rotation servos:
        # Left servo: 94 - 10 = 84 (forward), 94 + 10 = 104 (backward)
        # Right servo: 94 - 10 = 84 (forward), 94 + 10 = 104 (backward)
        # BUT: servos are mounted opposite, so one needs reversed values
        
        # Base forward speed
        left_angle = self.servo_stop - self.base_speed_offset   # e.g., 84 (forward)
        right_angle = self.servo_stop + self.base_speed_offset  # e.g., 104 (forward, reversed mount)
        
        # Apply correction
        # If yaw_error > 0 (drifted right): slow down right servo
        # If yaw_error < 0 (drifted left): slow down left servo
        left_angle = left_angle - correction   # More negative = faster left
        right_angle = right_angle - correction  # Less positive = slower right
        
        # Ensure angles are within valid range [0, 180]
        left_angle = max(0, min(180, int(left_angle)))
        right_angle = max(0, min(180, int(right_angle)))
        
        # Publish servo command
        cmd_msg = String()
        cmd_msg.data = f"{left_angle} {right_angle}"
        self.servo_publisher.publish(cmd_msg)
        
        # Log status periodically
        if self.get_clock().now().nanoseconds % 1000000000 < 20000000:
            self.get_logger().info(
                f'Yaw: {self.current_yaw:.2f}° | Error: {yaw_error:.2f}° | '
                f'Servos: L={left_angle} R={right_angle}'
            )
        
        self.last_error = yaw_error
    
    def start_callback(self, request, response):
        """Start moving in straight line"""
        self.is_moving = True
        self.initial_yaw = self.current_yaw
        self.integral_error = 0.0
        self.last_error = 0.0
        
        self.get_logger().info(f'Starting straight line from yaw: {self.initial_yaw:.2f}°')
        response.success = True
        response.message = 'Started'
        return response
    
    def stop_callback(self, request, response):
        """Stop movement"""
        self.is_moving = False
        
        # Send stop command to servos
        cmd_msg = String()
        cmd_msg.data = f"{self.servo_stop} {self.servo_stop}"
        self.servo_publisher.publish(cmd_msg)
        
        self.get_logger().info('Stopped straight line movement')
        response.success = True
        response.message = 'Stopped'
        return response

def main(args=None):
    rclpy.init(args=args)
    controller = StraightLineServoController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop servos on shutdown
        cmd_msg = String()
        cmd_msg.data = f"{controller.servo_stop} {controller.servo_stop}"
        controller.servo_publisher.publish(cmd_msg)
        
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

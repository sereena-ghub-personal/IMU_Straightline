#!/usr/bin/env python3
"""
=============================================================================
STRAIGHT LINE CONTROLLER - TUNED FOR IMU CORRECTION
=============================================================================
Adjusted PID gains to handle left drift issue

Subscribes to: /imu_data (String) - "yaw pitch roll"
Publishes to: /servo_cmd (String) - "left_pwm right_pwm"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

class StraightLineController(Node):
    def __init__(self):
        super().__init__('straight_line_controller')
        
        # ============================================
        # SERVO PARAMETERS
        # ============================================
        
        # Neutral (STOP) values
        self.PWM_NEUTRAL_RIGHT = 94
        self.PWM_NEUTRAL_LEFT = 94
        
        # Forward base speeds - ADJUSTED for drift
        # Problem: Right wheel faster than left (drifting left)
        # Solution: Increase left base speed OR decrease right base speed
        
        self.BASE_RIGHT_FWD = 88   # Keep original
        self.BASE_LEFT_FWD = 100   # Increased from 97 to compensate
        
        # Alternative: Slow down right instead
        # self.BASE_RIGHT_FWD = 90   # Slower right
        # self.BASE_LEFT_FWD = 97    # Keep original
        
        # Speed limits
        self.SPEED_MIN = 83
        self.SPEED_MAX = 105
        
        # PID gains - INCREASED for stronger IMU correction
        # Your encoder-based code used 0.05 because encoders update fast
        # IMU updates slower, so we need stronger response
        self.Kp = 0.8   # Increased from 0.05 (much stronger correction)
        self.Ki = 0.1   # Integral to eliminate steady-state error
        self.Kd = 0.3   # Derivative to reduce oscillation
        
        # Maximum error limit
        self.MAX_ERROR = 80.0
        
        # ============================================
        # STATE VARIABLES
        # ============================================
        self.initial_yaw = None
        self.current_yaw = 0.0
        self.is_moving = False
        
        # PID state
        self.integral_error = 0.0
        self.last_error = 0.0
        
        # ============================================
        # ROS2 SETUP
        # ============================================
        
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
        
        # Services
        self.start_service = self.create_service(
            Trigger,
            '/start_straight',
            self.start_callback)
        
        self.stop_service = self.create_service(
            Trigger,
            '/stop_straight',
            self.stop_callback)
        
        # Control loop timer (50Hz for smoother control)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('='*60)
        self.get_logger().info('STRAIGHT LINE CONTROLLER - TUNED FOR DRIFT')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Servo Stop: R={self.PWM_NEUTRAL_RIGHT} L={self.PWM_NEUTRAL_LEFT}')
        self.get_logger().info(f'Forward Base: R={self.BASE_RIGHT_FWD} L={self.BASE_LEFT_FWD}')
        self.get_logger().info(f'PID Gains: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}')
        self.get_logger().info(f'Speed Range: [{self.SPEED_MIN}, {self.SPEED_MAX}]')
        self.get_logger().info('')
        self.get_logger().info('Note: Left base increased to 100 (was 97)')
        self.get_logger().info('      to compensate for right wheel being faster')
        self.get_logger().info('')
        self.get_logger().info('Commands:')
        self.get_logger().info('  Start: ros2 service call /start_straight std_srvs/srv/Trigger')
        self.get_logger().info('  Stop:  ros2 service call /stop_straight std_srvs/srv/Trigger')
        self.get_logger().info('='*60)
    
    def imu_callback(self, msg):
        """Process incoming IMU data"""
        try:
            data = msg.data.split()
            if len(data) >= 1:
                self.current_yaw = float(data[0])
                
                # Set initial yaw on first reading
                if self.initial_yaw is None:
                    self.initial_yaw = self.current_yaw
                    self.get_logger().info(f'Initial yaw recorded: {self.initial_yaw:.2f}°')
        
        except (ValueError, IndexError):
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
        
        # Calculate yaw error
        error = self.normalize_angle(self.current_yaw - self.initial_yaw)
        
        # Limit error
        if error > self.MAX_ERROR:
            error = self.MAX_ERROR
        elif error < -self.MAX_ERROR:
            error = -self.MAX_ERROR
        
        # PID Controller
        dt = 0.02  # 50Hz = 0.02 seconds
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral_error += error * dt
        # Anti-windup
        self.integral_error = max(-20, min(20, self.integral_error))
        I = self.Ki * self.integral_error
        
        # Derivative term
        D = self.Kd * (error - self.last_error) / dt
        
        # Total correction
        correction = P + I + D
        
        # Calculate servo PWM values
        # If error < 0 (drifting left): 
        #   - Need to turn right
        #   - Increase right speed (less negative correction)
        #   - Decrease left speed (more negative correction)
        
        speed_left = self.BASE_LEFT_FWD - int(correction)
        speed_right = self.BASE_RIGHT_FWD + int(correction)
        
        # Constrain to speed limits
        speed_left = max(self.SPEED_MIN, min(self.SPEED_MAX, speed_left))
        speed_right = max(self.SPEED_MIN, min(self.SPEED_MAX, speed_right))
        
        # Send command
        cmd_msg = String()
        cmd_msg.data = f"{speed_left} {speed_right}"
        self.servo_publisher.publish(cmd_msg)
        
        # Log status (every 0.5 seconds)
        if self.get_clock().now().nanoseconds % 500000000 < 20000000:
            self.get_logger().info(
                f'Yaw: {self.current_yaw:6.2f}° | '
                f'Target: {self.initial_yaw:6.2f}° | '
                f'Error: {error:+6.2f}° | '
                f'Corr: {correction:+5.1f} | '
                f'PWM: L={speed_left:3d} R={speed_right:3d}'
            )
        
        # Update for next iteration
        self.last_error = error
    
    def start_callback(self, request, response):
        """Start straight line movement"""
        if self.is_moving:
            response.success = False
            response.message = 'Already moving!'
            return response
        
        # Reset PID state
        self.initial_yaw = self.current_yaw
        self.integral_error = 0.0
        self.last_error = 0.0
        self.is_moving = True
        
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info(f'STARTING STRAIGHT LINE FROM YAW: {self.initial_yaw:.2f}°')
        self.get_logger().info('='*60)
        
        response.success = True
        response.message = f'Started from yaw: {self.initial_yaw:.2f}°'
        return response
    
    def stop_callback(self, request, response):
        """Stop movement"""
        if not self.is_moving:
            response.success = False
            response.message = 'Not moving!'
            return response
        
        self.is_moving = False
        
        # Send stop command
        stop_msg = String()
        stop_msg.data = f"{self.PWM_NEUTRAL_LEFT} {self.PWM_NEUTRAL_RIGHT}"
        self.servo_publisher.publish(stop_msg)
        
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('STOPPED')
        self.get_logger().info('='*60)
        
        response.success = True
        response.message = 'Stopped'
        return response

def main(args=None):
    rclpy.init(args=args)
    controller = StraightLineController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Emergency stop on shutdown
        stop_msg = String()
        stop_msg.data = f"{controller.PWM_NEUTRAL_LEFT} {controller.PWM_NEUTRAL_RIGHT}"
        controller.servo_publisher.publish(stop_msg)
        
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

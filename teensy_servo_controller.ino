/*
 * Teensy 4.0 - Continuous Rotation Servo Controller for ROS2
 * Controls two continuous rotation servos for differential drive robot
 * 
 * Hardware Connections (from your existing code):
 * - Left Servo Control:  Pin 5
 * - Right Servo Control: Pin 6
 * - Left Feedback:       Pin 7 (not used for continuous rotation)
 * - Right Feedback:      Pin 8 (not used for continuous rotation)
 * 
 * Communication:
 * - Receives commands via Serial from ROS2 bridge
 * - Format: "left_speed right_speed" where speed is servo angle (0-180)
 * - For continuous rotation servos:
 *   - 90 = stop
 *   - 0-89 = rotate one direction (speed increases as value decreases)
 *   - 91-180 = rotate other direction (speed increases as value increases)
 */

#include <Servo.h>

// ==========================================
// PIN DEFINITIONS (from your existing code)
// ==========================================

constexpr int pinFeedbackLeft = 7;   // Not used for continuous rotation
constexpr int pinFeedbackRight = 8;  // Not used for continuous rotation
constexpr int pinControlLeft = 5;    // Servo control pin
constexpr int pinControlRight = 6;   // Servo control pin

// ==========================================
// SERVO CALIBRATION VALUES
// ==========================================

// Adjust these based on your specific servos
// Standard continuous rotation servo values:
constexpr int SERVO_STOP = 94;           // Actual stop value (might need tuning)

// Forward motion (both servos need opposite values due to mounting)
constexpr int SERVO_FORWARD_RIGHT = 89;  // Right servo forward
constexpr int SERVO_FORWARD_LEFT = 98;   // Left servo forward (reversed)

// Backward motion
constexpr int SERVO_BACKWARD_RIGHT = 98; // Right servo backward
constexpr int SERVO_BACKWARD_LEFT = 89;  // Left servo backward (reversed)

// Speed ranges for proportional control
constexpr int MIN_SPEED = 5;   // Minimum speed offset from stop (e.g., 89 or 99)
constexpr int MAX_SPEED = 25;  // Maximum speed offset from stop (e.g., 69 or 119)

// ==========================================
// GLOBAL VARIABLES
// ==========================================

Servo servoRight;
Servo servoLeft;

int current_left_speed = 0;   // Current left servo angle (0-180)
int current_right_speed = 0;  // Current right servo angle (0-180)

unsigned long last_command_time = 0;
const unsigned long COMMAND_TIMEOUT = 1000;  // Stop if no command for 1 second

bool motors_enabled = true;
bool servos_attached = false;

// ==========================================

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  
  // Setup pins
  pinMode(pinFeedbackRight, INPUT);
  pinMode(pinFeedbackLeft, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Don't attach servos yet - will attach when needed
  stopMotors();
  
  // Startup indication
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  
  Serial.println("Teensy Servo Controller Ready!");
  Serial.println("Format: left_angle right_angle (0-180)");
  Serial.println("90 = stop, <90 = one direction, >90 = other direction");
}

void loop() {
  // Check for incoming commands
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    parseCommand(cmd);
    last_command_time = millis();
  }
  
  // Safety: Stop motors if no command received for timeout period
  if (millis() - last_command_time > COMMAND_TIMEOUT && motors_enabled) {
    stopMotors();
    Serial.println("Timeout - servos stopped");
  }
  
  // Update servo positions
  updateServos();
  
  // Status LED blink when moving
  if (abs(current_left_speed - SERVO_STOP) > 2 || 
      abs(current_right_speed - SERVO_STOP) > 2) {
    digitalWrite(LED_BUILTIN, (millis() / 250) % 2);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void parseCommand(String cmd) {
  /*
   * Parse servo command in format: "left_angle right_angle"
   * Example: "85 95" (left forward, right forward)
   * Example: "90 90" (both stop)
   */
  cmd.trim();
  
  int space_index = cmd.indexOf(' ');
  if (space_index == -1) {
    Serial.println("Error: Invalid command format");
    return;
  }
  
  String left_str = cmd.substring(0, space_index);
  String right_str = cmd.substring(space_index + 1);
  
  int left = left_str.toInt();
  int right = right_str.toInt();
  
  // Validate servo angles (0-180)
  if (left < 0 || left > 180 || right < 0 || right > 180) {
    Serial.println("Error: Angle out of range (0-180)");
    return;
  }
  
  current_left_speed = left;
  current_right_speed = right;
  
  // Debug output
  Serial.print("CMD: L=");
  Serial.print(current_left_speed);
  Serial.print(" R=");
  Serial.println(current_right_speed);
}

void updateServos() {
  /*
   * Update servo positions based on current speed values
   */
  
  // Attach servos if not already attached
  if (!servos_attached) {
    servoLeft.attach(pinControlLeft);
    servoRight.attach(pinControlRight);
    servos_attached = true;
  }
  
  // Write positions to servos
  servoLeft.write(current_left_speed);
  servoRight.write(current_right_speed);
}

void stopMotors() {
  /*
   * Stop both servos
   */
  current_left_speed = SERVO_STOP;
  current_right_speed = SERVO_STOP;
  
  if (!servos_attached) {
    servoLeft.attach(pinControlLeft);
    servoRight.attach(pinControlRight);
    servos_attached = true;
  }
  
  servoLeft.write(SERVO_STOP);
  servoRight.write(SERVO_STOP);
  
  delay(100);
  
  // Optionally detach to reduce jitter (comment out if causes issues)
  // servoLeft.detach();
  // servoRight.detach();
  // servos_attached = false;
}

// ==========================================
// HELPER FUNCTIONS FOR COMMON MOVEMENTS
// ==========================================

void moveForward() {
  current_left_speed = SERVO_FORWARD_LEFT;
  current_right_speed = SERVO_FORWARD_RIGHT;
}

void moveBackward() {
  current_left_speed = SERVO_BACKWARD_LEFT;
  current_right_speed = SERVO_BACKWARD_RIGHT;
}

void turnLeft() {
  // Left servo backward, right servo forward
  current_left_speed = SERVO_BACKWARD_LEFT;
  current_right_speed = SERVO_FORWARD_RIGHT;
}

void turnRight() {
  // Left servo forward, right servo backward
  current_left_speed = SERVO_FORWARD_LEFT;
  current_right_speed = SERVO_BACKWARD_RIGHT;
}

void setSpeed(int left_angle, int right_angle) {
  /*
   * Direct speed control
   * left_angle, right_angle: 0-180 (90 = stop)
   */
  current_left_speed = constrain(left_angle, 0, 180);
  current_right_speed = constrain(right_angle, 0, 180);
}

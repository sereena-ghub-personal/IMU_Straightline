/*
 * Arduino Nano 33 BLE - IMU + Ultrasonic Sensor Publisher
 * Combines calibrated IMU data with ultrasonic distance scanning
 * 
 * Hardware:
 * - Arduino Nano 33 BLE with LSM9DS1 IMU
 * - HC-SR04 Ultrasonic sensor on pins 2 (TRIG), 3 (ECHO)
 * - Servo motor on pin 4 for sensor scanning
 * 
 * Output Format:
 * Line 1: "YAW pitch roll" (IMU data)
 * Line 2: "DIST distance,angle" (Ultrasonic data)
 * 
 * Example:
 * YAW 285.37 0.5 0.2
 * DIST 45.3,90
 * YAW 285.42 0.5 0.2
 * DIST 23.5,0
 */

#include <Arduino_LSM9DS1.h>
#include <MadgwickAHRS.h>
#include <Servo.h>

// ==========================================
// PIN DEFINITIONS
// ==========================================

#define TRIG_PIN 2      // Ultrasonic trigger
#define ECHO_PIN 3      // Ultrasonic echo
#define SERVO_PIN 4     // Scanning servo

// ==========================================
// IMU CALIBRATION VALUES
// ==========================================

// Initialize the Madgwick Filter
Madgwick filter;

// 1. GYROSCOPE OFFSETS
float offset_gx = 1.26;
float offset_gy = 0.65;
float offset_gz = 1.59;

// 2. ACCELEROMETER CALIBRATION (Offset + Slope)
float accel_offset_x = -0.015127;
float accel_offset_y = -0.022152;
float accel_offset_z = -0.052882;

float accel_slope_x = 0.997415;
float accel_slope_y = 0.991539;
float accel_slope_z = 1.000259;

// 3. MAGNETOMETER CALIBRATION
float mag_off_x = 26.09;
float mag_off_y = 13.37;
float mag_off_z = 4.03;

float mag_slope_x = 1.20;
float mag_slope_y = 0.89;
float mag_slope_z = 0.96;

// ==========================================
// ULTRASONIC SCANNING SETUP
// ==========================================

Servo scanServo;

// Scanning angles: Front (90°), Right (0°), Left (180°)
int scanAngles[] = {90, 0, 180};
int currentAngleIndex = 0;
unsigned long lastScanTime = 0;
const int SCAN_INTERVAL = 400;  // Time between scans (ms)

// ==========================================

void setup() {
  Serial.begin(115200);  // Higher baud rate for faster communication
  
  // Initialize IMU
  if (!IMU.begin()) {
    // If IMU fails, blink LED rapidly
    pinMode(LED_BUILTIN, OUTPUT);
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(100);
    }
  }

  // Set accelerometer settings (must match calibration)
  IMU.setAccelFS(3);    // ±8g
  IMU.setAccelODR(5);   // 476Hz

  // Initialize Madgwick filter (50Hz update rate)
  filter.begin(50);
  
  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Initialize and position servo
  scanServo.attach(SERVO_PIN);
  scanServo.write(90);  // Start at front position
  
  delay(1000);  // Give sensors time to stabilize
}

void loop() {
  // --- 1. READ AND PUBLISH IMU DATA (High frequency - every loop) ---
  publishIMUData();
  
  // --- 2. SCAN AND PUBLISH ULTRASONIC DATA (Lower frequency - every 400ms) ---
  if (millis() - lastScanTime >= SCAN_INTERVAL) {
    publishUltrasonicData();
    lastScanTime = millis();
  }
  
  delay(20);  // 50Hz main loop rate
}

void publishIMUData() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, yaw;

  if (IMU.accelerationAvailable() && 
      IMU.gyroscopeAvailable() && 
      IMU.magneticFieldAvailable()) {
    
    // Read raw sensor data
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);

    // Apply accelerometer calibration (offset + slope)
    ax = (ax - accel_offset_x) / accel_slope_x;
    ay = (ay - accel_offset_y) / accel_slope_y;
    az = (az - accel_offset_z) / accel_slope_z;
    
    // Apply gyroscope calibration
    gx -= offset_gx;
    gy -= offset_gy;
    gz -= offset_gz;

    // Apply magnetometer calibration
    mx = (mx - mag_off_x) * mag_slope_x;
    my = (my - mag_off_y) * mag_slope_y;
    mz = (mz - mag_off_z) * mag_slope_z;

    // Update sensor fusion filter
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    // Get orientation angles
    yaw = filter.getYaw();
    pitch = filter.getPitch();
    roll = filter.getRoll();

    // Output IMU data with "YAW" prefix for identification
    Serial.print("YAW ");
    Serial.print(yaw, 2);
    Serial.print(" ");
    Serial.print(pitch, 2);
    Serial.print(" ");
    Serial.println(roll, 2);
  }
}

void publishUltrasonicData() {
  // Get current angle
  int angle = scanAngles[currentAngleIndex];
  
  // Move servo to angle
  scanServo.write(angle);
  delay(300);  // Wait for servo to stabilize
  
  // Measure distance
  float distance = getDistance();
  
  // Send data with "DIST" prefix for identification
  Serial.print("DIST ");
  Serial.print(distance, 1);
  Serial.print(",");
  Serial.println(angle);
  
  // Move to next angle for next scan
  currentAngleIndex = (currentAngleIndex + 1) % 3;
}

float getDistance() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read echo pulse (30ms timeout)
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  // Calculate distance in cm
  float distance = duration * 0.034 / 2.0;
  
  // Return 0 if out of range or error
  if (distance == 0 || distance > 400) {
    return 0;
  }
  
  return distance;
}

/*
 * Arduino Nano 33 BLE - Calibrated IMU for Straight Line Driving
 * Publishes: "yaw pitch roll" (space-separated, one line per reading)
 * 
 * IMPORTANT: Update calibration values (lines 18-40) with YOUR values!
 * 
 * Output Example:
 * 285.37 0.52 0.21
 * 285.42 0.53 0.22
 * 285.45 0.51 0.21
 */

#include <Arduino_LSM9DS1.h>
#include <MadgwickAHRS.h>

Madgwick filter;

// ==========================================
// YOUR CALIBRATION VALUES - UPDATE THESE!
// ==========================================

// Gyroscope offsets
float offset_gx = 1.26;
float offset_gy = 0.65;
float offset_gz = 1.59;

// Accelerometer calibration
float accel_offset_x = -0.015127;
float accel_offset_y = -0.022152;
float accel_offset_z = -0.052882;

float accel_slope_x = 0.997415;
float accel_slope_y = 0.991539;
float accel_slope_z = 1.000259;

// Magnetometer calibration
float mag_off_x = 26.09;
float mag_off_y = 13.37;
float mag_off_z = 4.03;

float mag_slope_x = 1.20;
float mag_slope_y = 0.89;
float mag_slope_z = 0.96;

// ==========================================

void setup() {
  Serial.begin(115200);
  
  // Initialize IMU
  if (!IMU.begin()) {
    // Blink LED if IMU fails
    pinMode(LED_BUILTIN, OUTPUT);
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(100);
    }
  }

  // NOTE: Arduino_LSM9DS1 library uses default settings
  // No setAccelFS() or setAccelODR() functions available
  // Library defaults are fine for this application

  // Initialize Madgwick filter
  filter.begin(50);  // 50Hz update rate
  
  delay(1000);  // Stabilization time
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, yaw;

  // Read sensors when all data is available
  if (IMU.accelerationAvailable() && 
      IMU.gyroscopeAvailable() && 
      IMU.magneticFieldAvailable()) {
    
    // Read raw data
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);

    // Apply accelerometer calibration
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

    // Update sensor fusion
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    // Get orientation
    yaw = filter.getYaw();
    pitch = filter.getPitch();
    roll = filter.getRoll();

    // CRITICAL: Output format MUST be "yaw pitch roll"
    // Space-separated, 2 decimal places, one line
    Serial.print(yaw, 2);
    Serial.print(" ");
    Serial.print(pitch, 2);
    Serial.print(" ");
    Serial.println(roll, 2);
  }
  
  delay(20);  // 50Hz update rate (1000ms / 50Hz = 20ms)
}

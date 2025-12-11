# 🔌 Complete Wiring Diagram and Setup Guide

## 📊 System Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                         ROBOT SYSTEM OVERVIEW                         │
└──────────────────────────────────────────────────────────────────────┘

┌─────────────────┐         ┌──────────────────────┐         ┌─────────────────┐
│  Arduino Nano   │   USB   │   Raspberry Pi 4     │   USB   │   Teensy 4.0    │
│    33 BLE       │◄───────►│    (ROS2 Humble)     │◄───────►│   Servo Motor   │
│                 │         │                      │         │   Controller    │
│  • IMU (LSM9DS1)│         │  • Straight Line     │         │                 │
│  • Ultrasonic   │         │    Controller        │         │  • Left Servo   │
│  • Servo Scanner│         │  • Serial Bridges    │         │  • Right Servo  │
└─────────────────┘         └──────────────────────┘         └─────────────────┘
         │                             │                              │
         ↓                             ↓                              ↓
   Yaw, Distance              PID Control Algo              Servo Angles (0-180)
   @ 50Hz                     Maintains Straight            90=stop, <90 & >90=move
```

---

## 🔧 Hardware Connections

### **Arduino Nano 33 BLE (Sensor Board)**

#### Built-in Sensors:
- **IMU (LSM9DS1)**: Built into Arduino Nano 33 BLE (no wiring needed)

#### External Sensors:
```
┌─────────────────────────────────────────┐
│        Arduino Nano 33 BLE              │
│                                         │
│  Pin 2 ──────► TRIG (Ultrasonic)       │
│  Pin 3 ──────► ECHO (Ultrasonic)       │
│  Pin 4 ──────► Servo Signal (Scanner)  │
│  5V    ──────► Ultrasonic VCC          │
│  GND   ──────► Ultrasonic GND          │
│  5V    ──────► Servo VCC (red wire)    │
│  GND   ──────► Servo GND (black wire)  │
│  USB   ──────► Raspberry Pi USB Port   │
└─────────────────────────────────────────┘
```

**HC-SR04 Ultrasonic Sensor:**
- VCC → Arduino 5V
- TRIG → Arduino Pin 2
- ECHO → Arduino Pin 3
- GND → Arduino GND

**Scanning Servo (for ultrasonic):**
- Red wire (VCC) → Arduino 5V
- Black/Brown wire (GND) → Arduino GND
- Orange/Yellow wire (Signal) → Arduino Pin 4

---

### **Teensy 4.0 (Motor Controller)**

#### Servo Motor Connections:
```
┌─────────────────────────────────────────┐
│           Teensy 4.0                    │
│                                         │
│  Pin 5 ──────► Left Servo Signal       │
│  Pin 6 ──────► Right Servo Signal      │
│  Pin 7 ──────► Left Feedback (unused)  │
│  Pin 8 ──────► Right Feedback (unused) │
│  VIN   ──────► (Not used)              │
│  GND   ──────► Common GND with Battery │
│  USB   ──────► Raspberry Pi USB Port   │
└─────────────────────────────────────────┘
```

**Left Continuous Rotation Servo:**
- Red wire (VCC) → Battery + (6V recommended)
- Black/Brown wire (GND) → Battery - AND Teensy GND
- Orange/Yellow wire (Signal) → Teensy Pin 5

**Right Continuous Rotation Servo:**
- Red wire (VCC) → Battery + (6V recommended)
- Black/Brown wire (GND) → Battery - AND Teensy GND
- Orange/Yellow wire (Signal) → Teensy Pin 6

**⚠️ IMPORTANT - Power Supply:**
- **DO NOT** power servos from Teensy 5V - they draw too much current
- Use separate battery pack (6V, 4x AA batteries recommended)
- **MUST** connect battery GND to Teensy GND (common ground)

```
Power Wiring Diagram:

Battery Pack (6V)
     │
     ├─── + ──→ Left Servo VCC (Red)
     │
     ├─── + ──→ Right Servo VCC (Red)
     │
     └─── - ──→ Left Servo GND (Black)
              │
              └─→ Right Servo GND (Black)
              │
              └─→ Teensy GND  ◄── COMMON GROUND!
```

---

### **Raspberry Pi 4 Connections**

```
USB Ports:
├── USB Port 1 → Arduino Nano 33 BLE (/dev/ttyACM0)
└── USB Port 2 → Teensy 4.0 (/dev/ttyACM1)

Power:
└── USB-C Power Supply (5V/3A minimum)
```

---

## 📋 Complete Parts List

### Required Components:

| Component | Quantity | Notes |
|-----------|----------|-------|
| Arduino Nano 33 BLE | 1 | With built-in LSM9DS1 IMU |
| Teensy 4.0 | 1 | Motor controller |
| Raspberry Pi 4 | 1 | 2GB+ RAM recommended |
| HC-SR04 Ultrasonic | 1 | Distance sensor |
| Micro Servo (9g) | 1 | For scanning ultrasonic |
| Continuous Rotation Servo | 2 | For wheel drive |
| Battery Pack (6V) | 1 | 4x AA batteries |
| USB Cables (Micro-USB) | 2 | For Arduino & Teensy |
| Jumper Wires | ~10 | For connections |
| Robot Chassis | 1 | To mount everything |

---

## 🚀 Software Setup

### Step 1: Upload Arduino Code

**File:** `arduino_imu_ultrasonic_combined.ino`

1. Open Arduino IDE
2. Select Board: **Arduino Nano 33 BLE**
3. Select Port: **/dev/ttyACM0**
4. **Update calibration values** (lines 31-55)
5. Upload

**Test:** Open Serial Monitor (115200 baud), should see:
```
YAW 285.37 0.5 0.2
DIST 45.3,90
YAW 285.42 0.5 0.2
DIST 23.5,0
```

---

### Step 2: Upload Teensy Code

**File:** `teensy_servo_controller.ino`

1. Open Arduino IDE with Teensyduino
2. Select Board: **Teensy 4.0**
3. Select Port: **/dev/ttyACM1**
4. **Calibrate servo stop values** (lines 24-33):
   ```cpp
   // Test to find your servo's actual stop value
   // Usually 90-94, might need adjustment
   constexpr int SERVO_STOP = 94;
   ```
5. Upload

**Test:** Open Serial Monitor (115200 baud), should see:
```
Teensy Servo Controller Ready!
```

**Manual Test:** Type in Serial Monitor:
```
90 90    (both servos stop)
85 95    (both forward - adjust if not moving)
95 85    (both backward)
```

---

### Step 3: Install ROS2 Nodes

```bash
cd ~/ros2_ws/src/robot_control/robot_control/

# Copy new Python files
cp arduino_serial_reader.py .
cp straight_line_servo_controller.py .
cp teensy_servo_bridge.py .

# Make executable
chmod +x arduino_serial_reader.py
chmod +x straight_line_servo_controller.py
chmod +x teensy_servo_bridge.py
```

**Update setup.py:**
```python
entry_points={
    'console_scripts': [
        'arduino_reader = robot_control.arduino_serial_reader:main',
        'straight_servo = robot_control.straight_line_servo_controller:main',
        'teensy_servo = robot_control.teensy_servo_bridge:main',
        'visualizer = robot_control.visualizer_node:main',
    ],
},
```

**Build:**
```bash
cd ~/ros2_ws
colcon build --packages-select robot_control
source install/setup.bash
```

---

## 🎮 Running the System

### Method 1: Manual Start (Terminal Per Node)

**Terminal 1 - Arduino Reader:**
```bash
ros2 run robot_control arduino_reader
```

**Terminal 2 - Straight Line Controller:**
```bash
ros2 run robot_control straight_servo
```

**Terminal 3 - Teensy Bridge:**
```bash
ros2 run robot_control teensy_servo
```

**Terminal 4 - Control:**
```bash
# Start moving straight
ros2 service call /start_straight std_srvs/srv/Trigger

# Monitor
ros2 topic echo /imu_data
ros2 topic echo /servo_cmd

# Stop
ros2 service call /stop_straight std_srvs/srv/Trigger
```

---

### Method 2: Launch File (All Together)

**Create:** `~/ros2_ws/src/robot_control/launch/straight_line_servo.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='arduino_reader',
            name='arduino_reader',
            parameters=[{'arduino_port': '/dev/ttyACM0'}]
        ),
        Node(
            package='robot_control',
            executable='straight_servo',
            name='straight_line_controller',
            parameters=[
                {'base_speed_offset': 10},  # Servo speed
                {'Kp': 0.5},
                {'Ki': 0.05},
                {'Kd': 0.2}
            ]
        ),
        Node(
            package='robot_control',
            executable='teensy_servo',
            name='teensy_bridge',
            parameters=[{'teensy_port': '/dev/ttyACM1'}]
        )
    ])
```

**Run:**
```bash
ros2 launch robot_control straight_line_servo.launch.py
```

---

## ⚙️ Calibration & Tuning

### 1. Find Servo Stop Value

Servos vary - yours might stop at 90, 92, 94, etc.

**Test Script:**
```bash
# Terminal 1: Start Teensy bridge
ros2 run robot_control teensy_servo

# Terminal 2: Test different values
ros2 topic pub /servo_cmd std_msgs/String "data: '90 90'" --once
# If servos move, try:
ros2 topic pub /servo_cmd std_msgs/String "data: '92 92'" --once
# Keep adjusting until servos stop
```

Update in code (line 24 of `teensy_servo_controller.ino`):
```cpp
constexpr int SERVO_STOP = 94;  // Your calibrated value
```

### 2. Test Forward Movement

```bash
# Both forward (adjust these values based on your stop value)
ros2 topic pub /servo_cmd std_msgs/String "data: '84 104'" --once
# If wrong direction, swap values or adjust
```

### 3. Tune PID Controller

Edit `straight_line_servo_controller.py` (lines 32-34):

**Start conservative:**
```python
self.Kp = 0.3
self.Ki = 0.02
self.Kd = 0.1
```

**Gradually increase if correction too slow.**

### 4. Adjust Base Speed

Edit `straight_line_servo_controller.py` (line 40):
```python
self.base_speed_offset = 10  # How far from stop value
```

- **Lower (5-8)**: Slower, more accurate
- **Higher (12-20)**: Faster, less accurate

---

## 🧪 Testing Procedure

### Test 1: Sensor Test
```bash
ros2 run robot_control arduino_reader
ros2 topic echo /imu_data        # Should see yaw values
ros2 topic echo /ultrasonic_scan # Should see distance,angle
```

### Test 2: Servo Test
```bash
ros2 run robot_control teensy_servo
ros2 topic pub /servo_cmd std_msgs/String "data: '94 94'" --once  # Stop
ros2 topic pub /servo_cmd std_msgs/String "data: '84 104'" --once # Forward
ros2 topic pub /servo_cmd std_msgs/String "data: '104 84'" --once # Backward
```

### Test 3: Straight Line Test
```bash
# Start all nodes
ros2 launch robot_control straight_line_servo.launch.py

# Start movement
ros2 service call /start_straight std_srvs/srv/Trigger

# Observe robot - should drive straight
# Gently push sideways - should self-correct

# Stop
ros2 service call /stop_straight std_srvs/srv/Trigger
```

---

## 🔍 Troubleshooting

### Problem: Servos don't stop at 90
**Solution:** Each servo is different. Calibrate using test script above.

### Problem: Robot turns instead of going straight
**Solution:** 
- Check servo mounting (both facing same direction?)
- Swap servo values in code if needed
- Verify `SERVO_FORWARD_LEFT` and `SERVO_FORWARD_RIGHT`

### Problem: Servos jitter or don't move
**Solution:**
- Check power supply (use 6V battery, not USB)
- Verify common ground between Teensy and battery
- Check servo connections (signal wires on correct pins)

### Problem: No serial connection
**Solution:**
```bash
# Check ports
ls /dev/ttyACM*

# Should see ACM0 and ACM1
# If not, check USB connections

# Add user to dialout group
sudo usermod -a -G dialout $USER
# Then reboot
```

### Problem: Robot oscillates (wiggles)
**Solution:**
- Reduce `Kp` value
- Increase `Kd` value
- Lower `base_speed_offset`

---

## 📊 Expected Performance

**Good Performance:**
- Yaw error: ±5° over 5 meters
- Lateral drift: <30cm
- Smooth motion

**Excellent Performance:**
- Yaw error: ±2° over 5 meters
- Lateral drift: <15cm
- No visible oscillation

---

## 🎉 Success Criteria

✅ Arduino publishes IMU data at 50Hz
✅ Ultrasonic scanner sweeps 0°, 90°, 180°
✅ Teensy responds to servo commands
✅ Robot drives straight for 5 meters
✅ Self-corrects when pushed sideways
✅ Smooth motion, no jerky movements

---

## 📞 Quick Reference

**USB Ports:**
- Arduino: `/dev/ttyACM0`
- Teensy: `/dev/ttyACM1`

**ROS2 Topics:**
- `/imu_data`: Yaw, pitch, roll
- `/ultrasonic_scan`: Distance, angle
- `/servo_cmd`: Left angle, right angle

**ROS2 Services:**
- `/start_straight`: Begin straight line driving
- `/stop_straight`: Stop movement

**Servo Values:**
- 90-94: Stop (calibrate yours)
- <90: One direction
- >90: Other direction

Good luck! 🚀

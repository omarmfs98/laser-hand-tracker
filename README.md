# ESP32 Pan-Tilt System with Joystick Control

This project controls two MG90S 180-degree servos using an analog joystick to create a Pan-Tilt system. The system includes automatic calibration, smooth movements, and **maintains the last position** without returning to center.

## Key Features

- **Maintains last position**: Servos don't return to center when you release the joystick
- **Automatic calibration**: Automatically detects joystick ranges
- **Configurable dead zone**: Prevents accidental movements
- **Smooth movements**: No vibrations or jerky movements
- **Laser pointer integration**: Built-in laser for pointing/tracking
- **Real-time serial monitoring**: Detailed status information

## Required Components

- ESP32 (any model)
- 2x MG90S Servos (180 degrees)
- 1x Analog joystick (GND, 3V, VRX, VRY, SW)
- Pan-Tilt mount for servos
- 5V power supply for servos
- Connection cables

## Connections

### Joystick
- **GND** → ESP32 GND
- **3V** → ESP32 3.3V
- **VRX** → GPIO 32 (ADC1_CH4)
- **VRY** → GPIO 33 (ADC1_CH5)
- **SW** → GPIO 25

### Servos
- **Pan Servo (Horizontal)**
  - Signal → GPIO 13
  - VCC → 5V (external power supply recommended)
  - GND → Common GND

- **Tilt Servo (Vertical)**
  - Signal → GPIO 14
  - VCC → 5V (external power supply recommended)
  - GND → Common GND

### Laser Module
- **Signal** → GPIO 26
- **VCC** → 5V
- **GND** → Common GND

## How It Works

### 1. **On Upload:**
```
=== PAN-TILT SYSTEM WITH JOYSTICK ===
Initializing...
Servos configured at center position (90°)
Laser automatically turned on
System using optimized calibration values!
System ready to use!
Move joystick to control servos
Press button to recalibrate if needed
================================================
```

### 2. **Calibration (5 seconds):**
```
=== JOYSTICK CALIBRATION ===
Move joystick in all directions for 5 seconds
Make sure to reach all extremes
=== CALIBRATION COMPLETED ===
X Axis: 100 - 3900 (Center: 2000)
Y Axis: 150 - 3850 (Center: 2000)
Position restored: Pan 90.0°, Tilt 90.0°
System ready to use!
================================================
```

## Adjustable Configuration

### Pins (modifiable in code)
```cpp
#define JOYSTICK_VRX_PIN 32  // Joystick X axis
#define JOYSTICK_VRY_PIN 33  // Joystick Y axis
#define JOYSTICK_SW_PIN  25  // Joystick button
#define SERVO_PAN_PIN    13  // Horizontal servo
#define SERVO_TILT_PIN   14  // Vertical servo
#define LASER_PIN        26  // Laser module
```

### Behavior Parameters
```cpp
#define JOYSTICK_DEADZONE 300     // Dead zone (0-500)
#define JOYSTICK_SMOOTHING 0.08   // Smoothing factor (0.0-1.0)
#define JOYSTICK_MIN_CHANGE 0.5   // Minimum change threshold
#define SERVO_MIN_ANGLE  0        // Minimum angle
#define SERVO_MAX_ANGLE  180      // Maximum angle
#define SERVO_CENTER_ANGLE 90     // Center position
```

## Usage Instructions

1. **Compile and upload the code** to ESP32
2. **Open Serial Monitor** (115200 baud)
3. **Press the joystick button** to calibrate
4. **Move the joystick** in all directions for 5 seconds
5. **Ready!** Now control the servos:
   - **X Axis**: Horizontal control (Pan) - Left/Right
   - **Y Axis**: Vertical control (Tilt) - Up/Down
   - **Release joystick**: Servos maintain their position

## Recommended Adjustments

### If movements are too slow:
```cpp
#define JOYSTICK_SMOOTHING 0.15  // Increase from 0.08 to 0.15
```

### If movements are too jerky:
```cpp
#define JOYSTICK_SMOOTHING 0.05  // Decrease from 0.08 to 0.05
```

### If joystick has too much drift:
```cpp
#define JOYSTICK_DEADZONE 400   // Increase from 300 to 400
```

### If joystick is too sensitive:
```cpp
#define JOYSTICK_DEADZONE 200   // Decrease from 300 to 200
```

## Important Notes

- **Servo Power**: MG90S servos can consume up to 500mA each. Use an external 5V power supply.
- **Calibration**: If movements are not precise, recalibrate the joystick by pressing the button.
- **Dead Zone**: The joystick must be in the dead zone to maintain position. If it moves slightly, servos will move.
- **Serial Monitor**: Keep the serial monitor open to see system status.

## Troubleshooting

### Servos don't move
- Check power supply connections
- Ensure signal pins are correct
- Verify calibration is complete

### Erratic movements
- Recalibrate the joystick
- Adjust dead zone
- Check for cable interference

### Movements too slow or fast
- Adjust smoothing factor (JOYSTICK_SMOOTHING)
- Modify update interval (UPDATE_INTERVAL)

### Servos return to center
- Verify dead zone is not too large
- Ensure joystick is well calibrated
- Check that `lastValidPanAngle` and `lastValidTiltAngle` variables are updating

## Technical Information

- **Update frequency**: 50Hz (20ms)
- **ADC resolution**: 12 bits (0-4095)
- **Servo range**: 0° - 180°
- **Center position**: 90°
- **Calibration time**: 5 seconds
- **Calibration samples**: ~500 samples

## License

Developed by Omar Florez
cv.omarflorez.me 
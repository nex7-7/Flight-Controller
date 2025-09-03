# Quadcopter Flight Controller

This is an Arduino-based flight controller for a quadcopter. The code is designed to read data from a radio receiver and an MPU6050 Inertial Measurement Unit (IMU) to control four motors via Electronic Speed Controllers (ESCs).

## Features

- **PID Control:** Implements PID controllers for stable flight, controlling Roll, Pitch, and Yaw. It uses a cascaded PID loop (angle and rate).
- **Kalman Filter:** A Kalman filter is used to fuse accelerometer and gyroscope data for accurate and smooth angle estimations (Roll and Pitch).
- **IBus Receiver Support:** Reads input from a FlySky IBus radio receiver for pilot commands.
- **MPU6050 IMU:** Interfaces with the MPU6050 to get accelerometer and gyroscope readings.
- **ESC Control:** Controls up to four ESCs using the Servo library.
- **Safety Features:**
    - **Failsafe:** Automatically cuts power or reduces throttle if the receiver connection is lost or the battery is critically low.
    - **Arming Check:** Ensures the throttle is low and a specific switch is engaged before arming the motors.
    - **Battery Monitor:** Continuously checks the battery voltage and triggers a failsafe if it drops below a critical level.
- **Debug Mode:** An optional debug mode can be enabled to print sensor values, PID outputs, and other data to the Serial monitor for tuning and troubleshooting.

## Hardware Requirements

- Arduino Board (e.g., Uno, Nano, or similar)
- MPU6050 Gyroscope & Accelerometer Module
- 4x Brushless Motors
- 4x Electronic Speed Controllers (ESCs)
- IBus-compatible Radio Receiver (e.g., FlySky FS-iA6B)
- LiPo Battery (Code is configured for ~3S, with a critical voltage of 11.4V)
- Quadcopter Frame

## Libraries Used

- `<IBusBM.h>`: For reading the IBus receiver.
- `<Servo.h>`: For controlling the ESCs.
- `<Wire.h>`: For I2C communication with the MPU6050.

## Pinout

- **ESCs:**
    - `ESC1`: Pin 3
    - `ESC2`: Pin 5
    - `ESC3`: Pin 6
    - `ESC4`: Pin 9
- **IBus Receiver:**
    - `Serial` port (RX pin of the Arduino)
- **MPU6050 (I2C):**
    - `SDA`: A4
    - `SCL`: A5
- **Onboard LED:** `Pin 13`
- **Debug Pin:** `Pin 2` (Connect to HIGH to enable debug mode)
- **Battery Voltage Sense:** `Pin A0`

## Setup & Operation

1.  **Wiring:** Connect all the components according to the pinout section.
2.  **Calibration:** Upon startup, the flight controller performs a calibration sequence for the MPU6050. Keep the drone on a flat, level surface during this time. The onboard LED will blink to indicate the process.
3.  **Arming:** To arm the drone, the following conditions must be met:
    - The throttle stick must be at its lowest position.
    - The arming switch on the transmitter (connected to Channel 7) must be in the 'ON' position.
4.  **Flying:** Once armed, the drone is ready to fly.
5.  **Disarming:** To disarm, flip the arming switch to the 'OFF' position.

## Code Overview

- `setup()`: Initializes serial communication, pins, ESCs, MPU6050, and performs initial calibrations and checks.
- `loop()`: The main control loop that runs at approximately 250Hz. It reads receiver and sensor data, runs the Kalman filter and PID calculations, and sends control signals to the ESCs.
- **Receiver Functions (`readReciever`, `inputConversion`):** Read and map the raw data from the IBus receiver into desired roll, pitch, yaw, and throttle commands.
- **MPU6050 Functions (`setupMPU`, `recordMPU`, `processMPU`):** Handle the initialization, data reading, and processing from the IMU.
- **Kalman Filter (`kalmanFilter`, `kalmanCtrl`):** Implements the Kalman filter for sensor fusion.
- **PID Control (`PIDfunction`, `PIDctrl`, `PIDreset`):** Contains the logic for the PID controllers.
- **ESC Control (`setupESC`, `ESCsignal`, `ESCctrl`):** Manages the signals sent to the motors.
- **Safety Functions (`setFailsafe`, `setPower`, `powerDown`, `batteryVoltage`):** Implement the various safety checks.

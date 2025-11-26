# Linefolowing-and-obsticale-avoidance-robot-arduino
# Arduino Robot: Unified Line Follower & Obstacle Avoider 

This repository contains the complete Arduino sketch for a mobile robot utilizing a **Finite State Machine (FSM)** approach to combine two core functionalities: precise line following using three Infrared (IR) sensors and dynamic obstacle avoidance using an ultrasonic sensor and a scanning servo.

The code is optimized for reliability and uses non-blocking logic where possible.

---

## ✨ Features

* **3-Sensor Line Following:** Uses dedicated IR sensors (Left, Center, Right) for accurate tracking and adjustment.
* **State Machine (FSM):** Implements a clean, non-blocking FSM logic to smoothly transition between `LINE_FOLLOW`, `SCAN`, `AVOID`, and `BACKUP` states.
* **Ultrasonic Obstacle Avoidance:** Employs a `NewPing` library and a scanning servo to dynamically check the path for obstacles.
* **AFMotor Shield Integration:** Directly configured to control four DC motors using the AFMotor library (pins 1, 2, 3, 4).
* **Startup Delay:** Includes a 5-second delay at startup (`START_DELAY_TIME`) for user safety and preparation.

## 🛠️ Prerequisites

To compile and run this sketch, you will need the Arduino IDE and the following libraries:

1.  **`NewPing`**: For the ultrasonic distance sensor.
2.  **`Servo`**: For controlling the scanning servo.
3.  **`AFMotor`**: For motor control (designed for the Adafruit Motor Shield V1/V2).

You can install these libraries through the Arduino Library Manager.

## 📌 Hardware and Pinout Configuration

The sketch is configured with the following specific pin assignments:

| Component | Pin Definition | Code Value |
| :--- | :--- | :--- |
| **IR Left Sensor** | `IR_LEFT` | `A5` |
| **IR Center Sensor** | `IR_CENTER` | `A4` |
| **IR Right Sensor** | `IR_RIGHT` | `A2` |
| **Ultrasonic Trigger** | `TRIGGER_PIN` | `A1` |
| **Ultrasonic Echo** | `ECHO_PIN` | `A0` |
| **Scanning Servo** | `SERVO_PIN` | `10` |

*Motor configuration is handled within the `AFMotor.h` library setup.*

## ⚙️ Key Constants and Settings

| Constant | Description | Value |
| :--- | :--- | :--- |
| `BASE_SPEED` | Standard motor speed during line following. | `80` |
| `PIVOT_SPEED` | Speed used during turning and pivot maneuvers. | `180` |
| `OBSTACLE_THRESHOLD` | Max distance (cm) to trigger obstacle avoidance. | `10` |
| `START_DELAY_TIME` | Delay in milliseconds before starting the loop. | `5000` |

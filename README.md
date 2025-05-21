# Simple Swerve Drive Base
Advantage Kit logged swerve drive base code, intended for PR events

Max linear speed is adjustable on dashboard when tunning mode boolean is set to true in Constants.java

## Simple swerve drive controls:
- Left Stick - Forward, Back, Left, Right
- Right Stick - Rotation
- A - Lock to 0 degrees
- X - Wheels to x pattern
- B - Reset Gyro to 0 degrees

## Hardware
- Drive Motors: SparkMax/Neo
- Angle Motors: SparkMax/Neo
- Absolute Encoders: CANCoders
- Gyro: Pigeon 2.0
- Input: XBox Controller on Port 0

## Setup - In DriveConstants.java:
1. Set CAN IDs for motors, CANCoders, and Pigeon in respective variables
2. Set track width (distance from center of left wheel to right wheel) and wheel base (distance from center of front wheel to back)
3. Set wheel translations from the center of the robot
4. Turn swerve wheels so that the gear on the side of the wheel is facing the left of the robot. Use a square against the chassis to face each wheel straight forward. In the CTRE Phoenix Tuner X software while connected to the robot: press the zero encoder button (it looks like a simon) and copy the value it zeroed to (with the sign) and paste it in respective 0 rotation slot
5. When you rotate each wheel counter clockwise both the angle encoder and the absolute encoder for the wheel should be increasing. If it is not flip the inverted boolean.
6. Tune PID, Feedforward, and Wheel Radius according to these insturctions: [https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template/#feedforward-characterization ](url)

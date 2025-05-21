// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public class DriveConstants {
        public static LoggedTunableNumber maxSpeedMetersPerSec = new LoggedTunableNumber("Max Teleop Linear Speed Meters/Sec", 6.0);
        public static final double maxAutoSpeedMetersPerSec = 6.0; 
        public static final double odometryFrequency = 100.0; // Hz
        public static final double trackWidth = Units.inchesToMeters(23.750);
        public static final double wheelBase = Units.inchesToMeters(23.750);
        public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
        public static final Translation2d[] moduleTranslations = new Translation2d[] {
                        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
                        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
                        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
                        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

        // Zeroed rotation values for each module, see setup instructions
        public static final double frontLeftZeroRotation = -0.435791;
        public static final double frontRightZeroRotation = 0.113037;
        public static final double backLeftZeroRotation = -0.183105;
        public static final double backRightZeroRotation = 0.041748;

        // Device CAN IDs
        public static final int pigeonCanId = 9;

        public static final int frontLeftDriveCanId = 5;
        public static final int backLeftDriveCanId = 3;
        public static final int frontRightDriveCanId = 7;
        public static final int backRightDriveCanId = 1;

        public static final int frontLeftTurnCanId = 6;
        public static final int backLeftTurnCanId = 4;
        public static final int frontRightTurnCanId = 8;
        public static final int backRightTurnCanId = 2;

        public static final int frontLeftAbsCanId = 11;
        public static final int backLeftAbsCanId = 14;
        public static final int frontRightAbsCanId = 12;
        public static final int backRightAbsCanId = 13;

        // Drive motor configuration
        public static final int driveMotorCurrentLimit = 50;
        public static final double wheelRadiusMeters = Units.inchesToMeters(4);
        public static final double driveMotorReduction = 6.75; // Mk4i L2
        public static final DCMotor driveGearbox = DCMotor.getNEO(1);

        // Drive encoder configuration
        public static final double driveEncoderPositionFactor = (2 * Math.PI) / driveMotorReduction; // Rotor Rotations ->
                                                                                                   // Wheel Radians
        public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM
                                                                                                            // ->
                                                                                                            // Wheel
                                                                                                            // Rad/Sec

        // Drive PID configuration
        public static final double driveKp = 0.02;
        public static final double driveKd = 0.0;
        public static final double driveKs = 0.0;
        public static final double driveKv = 0.1;
        public static final double driveSimP = 0.05;
        public static final double driveSimD = 0.0;
        public static final double driveSimKs = 0.0;
        public static final double driveSimKv = 0.0789;

        // Turn motor configuration
        public static final boolean frontLeftTurnInverted = true;
        public static final boolean frontRightTurnInverted = true;
        public static final boolean backLeftTurnInverted = true;
        public static final boolean backRightTurnInverted = true;

        public static final boolean frontLeftDriveInverted = true;
        public static final boolean frontRightDriveInverted = true;
        public static final boolean backLeftDriveInverted = true;
        public static final boolean backRightDriveInverted = true;

        public static final int turnMotorCurrentLimit = 40;
        public static final double turnMotorReduction = 150 / 7;
        public static final DCMotor turnGearbox = DCMotor.getNEO(1);

        // Turn encoder configuration
        public static final boolean turnEncoderInverted = true;
        public static final double turnEncoderPositionFactor = (2 * Math.PI) / turnMotorReduction; // Rotations ->
                                                                                                   // Radians
        public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0 / turnMotorReduction; // RPM ->
                                                                                                          // Rad/Sec

        // Abs Encoder Inversion
        public static SensorDirectionValue frontLeftAbsSensorDir = SensorDirectionValue.CounterClockwise_Positive;
        public static SensorDirectionValue frontRightAbsSensorDir = SensorDirectionValue.CounterClockwise_Positive;
        public static SensorDirectionValue backLeftAbsSensorDir = SensorDirectionValue.CounterClockwise_Positive;
        public static SensorDirectionValue backRightAbsSensorDir = SensorDirectionValue.CounterClockwise_Positive;

        // Turn PID configuration
        public static final double turnKp = 1.1;
        public static final double turnKd = 0.1;
        public static final double turnSimP = 8.0;
        public static final double turnSimD = 0.0;
        public static final double turnPIDMinInput = -Math.PI; // Radians
        public static final double turnPIDMaxInput = Math.PI; // Radians

        // PathPlanner configuration
        public static final double robotMassKg = 74.088;
        public static final double robotMOI = 6.883;
        public static final double wheelCOF = 1.2;
        public static final RobotConfig ppConfig = new RobotConfig(
                        robotMassKg,
                        robotMOI,
                        new ModuleConfig(
                                        wheelRadiusMeters,
                                        maxAutoSpeedMetersPerSec,
                                        wheelCOF,
                                        driveGearbox.withReduction(driveMotorReduction),
                                        driveMotorCurrentLimit,
                                        1),
                        moduleTranslations);
}

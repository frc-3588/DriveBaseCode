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

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max
 * turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOSpark implements ModuleIO {
        private final double zeroRotation;

        // Hardware objects
        private final SparkBase driveSpark;
        private final SparkBase turnSpark;
        private final RelativeEncoder driveEncoder;
        private final RelativeEncoder turnEncoder;

        private final CANcoder absoluteEncoder;

        // Closed loop controllers
        private final SparkClosedLoopController driveController;
        private final SparkClosedLoopController turnController;

        // Queue inputs from odometry thread
        private final Queue<Double> timestampQueue;
        private final Queue<Double> drivePositionQueue;
        private final Queue<Double> turnPositionQueue;

        // Connection debouncers
        private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
        private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

        public ModuleIOSpark(int module) {
                zeroRotation = switch (module) {
                        case 0 -> frontLeftZeroRotation;
                        case 1 -> frontRightZeroRotation;
                        case 2 -> backLeftZeroRotation;
                        case 3 -> backRightZeroRotation;
                        default -> 0;
                };
                driveSpark = new SparkMax(
                                switch (module) {
                                        case 0 -> frontLeftDriveCanId;
                                        case 1 -> frontRightDriveCanId;
                                        case 2 -> backLeftDriveCanId;
                                        case 3 -> backRightDriveCanId;
                                        default -> 0;
                                },
                                MotorType.kBrushless);
                turnSpark = new SparkMax(
                                switch (module) {
                                        case 0 -> frontLeftTurnCanId;
                                        case 1 -> frontRightTurnCanId;
                                        case 2 -> backLeftTurnCanId;
                                        case 3 -> backRightTurnCanId;
                                        default -> 0;
                                },
                                MotorType.kBrushless);
                absoluteEncoder = new CANcoder(
                                switch (module) {
                                        case 0 -> frontLeftAbsCanId;
                                        case 1 -> frontRightAbsCanId;
                                        case 2 -> backLeftAbsCanId;
                                        case 3 -> backRightAbsCanId;
                                        default -> 0;
                                });
                driveEncoder = driveSpark.getEncoder();
                turnEncoder = turnSpark.getEncoder();
                driveController = driveSpark.getClosedLoopController();
                turnController = turnSpark.getClosedLoopController();

                boolean driveInverted = switch (module) {
                        case 0 -> frontLeftDriveInverted;
                        case 1 -> frontRightDriveInverted;
                        case 2 -> backLeftDriveInverted;
                        case 3 -> backRightDriveInverted;
                        default -> false;
                };
                // Configure drive motor
                var driveConfig = new SparkMaxConfig();
                driveConfig
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(driveMotorCurrentLimit)
                                .voltageCompensation(12.0)
                                .inverted(driveInverted);
                driveConfig.encoder
                                .positionConversionFactor(driveEncoderPositionFactor)
                                .velocityConversionFactor(driveEncoderVelocityFactor)
                                .uvwMeasurementPeriod(10)
                                .uvwAverageDepth(2);
                driveConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(
                                                driveKp, 0.0,
                                                driveKd, 0.0);
                driveConfig.signals
                                .primaryEncoderPositionAlwaysOn(true)
                                .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                                .primaryEncoderVelocityAlwaysOn(true)
                                .primaryEncoderVelocityPeriodMs(20)
                                .appliedOutputPeriodMs(20)
                                .busVoltagePeriodMs(20)
                                .outputCurrentPeriodMs(20);
                tryUntilOk(
                                driveSpark,
                                5,
                                () -> driveSpark.configure(
                                                driveConfig, ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters));
                tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

                // Configure turn motor
                var turnConfig = new SparkMaxConfig();
                boolean turnInverted = switch (module) {
                        case 0 -> frontLeftTurnInverted;
                        case 1 -> frontRightTurnInverted;
                        case 2 -> backLeftTurnInverted;
                        case 3 -> backRightTurnInverted;
                        default -> false;
                };
                turnConfig
                                .inverted(turnInverted)
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(turnMotorCurrentLimit)
                                .voltageCompensation(12.0);
                turnConfig.encoder
                                .positionConversionFactor(turnEncoderPositionFactor)
                                .velocityConversionFactor(turnEncoderVelocityFactor);

                turnConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .positionWrappingEnabled(true)
                                .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
                                .pidf(turnKp, 0.0, turnKd, 0.0);
                turnConfig.signals
                                .primaryEncoderPositionAlwaysOn(true)
                                .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                                .primaryEncoderVelocityAlwaysOn(true)
                                .primaryEncoderVelocityPeriodMs(20)
                                .appliedOutputPeriodMs(20)
                                .busVoltagePeriodMs(20)
                                .outputCurrentPeriodMs(20);
                tryUntilOk(
                                turnSpark,
                                5,
                                () -> turnSpark.configure(
                                                turnConfig, ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters));

                // Set Turn Relative from CANcoder
                CANcoderConfigurator absConfig = absoluteEncoder.getConfigurator();
                // Encoder inversion
                SensorDirectionValue absSensorDir = (switch (module) {
                        case 0 -> frontLeftAbsSensorDir;
                        case 1 -> frontRightAbsSensorDir;
                        case 2 -> backLeftAbsSensorDir;
                        case 3 -> backRightAbsSensorDir;
                        default -> SensorDirectionValue.Clockwise_Positive;
                });

                // Turn CANCoder to [0, 1) rotation output mode and apply inversion
                MagnetSensorConfigs absMagnetConfig = new MagnetSensorConfigs()
                                .withAbsoluteSensorDiscontinuityPoint(0.5)
                                .withSensorDirection(absSensorDir)
                                .withMagnetOffset(zeroRotation);
                absConfig.apply(absMagnetConfig);

                // Set Spark Max internal encoder from abs at boot
                // Convert rotation from CANcoder to radians
                Rotation2d absBootPos = Rotation2d
                                .fromRotations(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
                tryUntilOk(turnSpark, 5, () -> turnEncoder.setPosition(absBootPos.getRadians()));
                System.out.println("TURN SPARK SET TO " + absBootPos.getRadians());

                // Create odometry queues
                timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
                drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark,
                                driveEncoder::getPosition);
                turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark,
                                turnEncoder::getPosition);
        }

        @Override
        public void updateInputs(ModuleIOInputs inputs) {
                // Update drive inputs
                sparkStickyFault = false;
                ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
                ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
                ifOk(
                                driveSpark,
                                new DoubleSupplier[] { driveSpark::getAppliedOutput, driveSpark::getBusVoltage },
                                (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
                ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
                inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);
                inputs.turnAbsoluteEncoderPosition = Rotation2d
                                .fromRotations(absoluteEncoder.getAbsolutePosition().getValueAsDouble());

                // Update turn inputs
                sparkStickyFault = false;
                ifOk(
                                turnSpark,
                                turnEncoder::getPosition,
                                (value) -> inputs.turnPosition = new Rotation2d(value));
                ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
                ifOk(
                                turnSpark,
                                new DoubleSupplier[] { turnSpark::getAppliedOutput, turnSpark::getBusVoltage },
                                (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
                ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
                inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

                // Update odometry inputs
                inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
                inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value)
                                .toArray();
                inputs.odometryTurnPositions = turnPositionQueue.stream()
                                .map((Double value) -> new Rotation2d(value))
                                .toArray(Rotation2d[]::new);
                timestampQueue.clear();
                drivePositionQueue.clear();
                turnPositionQueue.clear();
        }

        @Override
        public void resetFromAbsolute() {
                // Rotation2d absBootPos =
                // Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
                // tryUntilOk(turnSpark, 5, () ->
                // turnEncoder.setPosition(absBootPos.getRadians()));
        }

        @Override
        public void setDriveOpenLoop(double output) {
                driveSpark.setVoltage(output);
        }

        @Override
        public void setTurnOpenLoop(double output) {
                turnSpark.setVoltage(output);
        }

        @Override
        public void setDriveVelocity(double velocityRadPerSec) {
                double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
                driveController.setReference(
                                velocityRadPerSec,
                                ControlType.kVelocity,
                                ClosedLoopSlot.kSlot0,
                                ffVolts,
                                ArbFFUnits.kVoltage);
        }

        @Override
        public void setTurnPosition(Rotation2d rotation) {
                double setpoint = MathUtil.inputModulus(
                                rotation.getRadians(), turnPIDMinInput, turnPIDMaxInput);
                turnController.setReference(setpoint, ControlType.kPosition);
        }
}

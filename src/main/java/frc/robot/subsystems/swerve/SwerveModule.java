// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
        private final CANSparkMax drivingSparkMax;
        private final CANSparkMax steerSparkMax;

        private final RelativeEncoder drivingEncoder;
        private final AbsoluteEncoder turningEncoder;

        private final SparkMaxPIDController veloPIDController;
        private final SparkMaxPIDController anglePIDController;

        private double chassisAngularOffset = 0;
        private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

        public SwerveModule(int drivingCANId, int turningCANId, double newChassisAngularOffset, boolean invert) {
                drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
                steerSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

                // Factory reset, so we get the SPARKS MAX to a known state before configuring
                // them. This is useful in case a SPARK MAX is swapped out.
                drivingSparkMax.restoreFactoryDefaults();
                steerSparkMax.restoreFactoryDefaults();

                // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
                drivingEncoder = drivingSparkMax.getEncoder();
                turningEncoder = steerSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
                veloPIDController = drivingSparkMax.getPIDController();
                anglePIDController = steerSparkMax.getPIDController();
                veloPIDController.setFeedbackDevice(drivingEncoder);
                anglePIDController.setFeedbackDevice(turningEncoder);

                // Apply position and velocity conversion factors for the driving encoder. The
                // native units for position and velocity are rotations and RPM, respectively,
                // but we want meters and meters per second to use with WPILib's swerve APIs.
                drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
                drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

                // Apply position and velocity conversion factors for the turning encoder. We
                // want these in radians and radians per second to use with WPILib's swerve
                // APIs.
                turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
                turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

                // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of
                // the steering motor in the MAXSwerve Module.
                turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

                // Enable PID wrap around for the turning motor. This will allow the PID
                // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                // to 10 degrees will go through 0 rather than the other direction which is a
                // longer route.
                anglePIDController.setPositionPIDWrappingEnabled(true);
                anglePIDController
                                .setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
                anglePIDController
                                .setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

                // Set the PID gains for the driving motor. Note these are example gains, and
                // you
                // may need to tune them for your own robot!
                veloPIDController.setP(ModuleConstants.kDrivingP);
                veloPIDController.setI(ModuleConstants.kDrivingI);
                veloPIDController.setD(ModuleConstants.kDrivingD);
                veloPIDController.setFF(ModuleConstants.kDrivingFF);
                veloPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
                                ModuleConstants.kDrivingMaxOutput);

                // Set the PID gains for the turning motor. Note these are example gains, and
                // you
                // may need to tune them for your own robot!
                anglePIDController.setP(ModuleConstants.kTurningP);
                anglePIDController.setI(ModuleConstants.kTurningI);
                anglePIDController.setD(ModuleConstants.kTurningD);
                anglePIDController.setFF(ModuleConstants.kTurningFF);
                anglePIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
                                ModuleConstants.kTurningMaxOutput);

                anglePIDController.setIZone(16);

                drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
                steerSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
                drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
                steerSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

                // Save the SPARK MAX configurations. If a SPARK MAX browns out during
                // operation, it will maintain the above configurations.
                drivingSparkMax.burnFlash();
                steerSparkMax.burnFlash();

                chassisAngularOffset = newChassisAngularOffset;
                desiredState.angle = new Rotation2d(turningEncoder.getPosition());
                drivingEncoder.setPosition(0);
        }

        /**
         * Returns the current state of the module.
         *
         * @return The current state of the module.
         */
        public SwerveModuleState getState() {
                // Apply chassis angular offset to the encoder position to get the position
                // relative to the chassis.
                return new SwerveModuleState(drivingEncoder.getVelocity(),
                                new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
        }

        public double getModuleAngle(){
                return (new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset)).getDegrees();
        }

        /**
         * Returns the current position of the module.
         *
         * @return The current position of the module.
         */
        public SwerveModulePosition getPosition() {
                // Apply chassis angular offset to the encoder position to get the position
                // relative to the chassis.
                return new SwerveModulePosition(
                                drivingEncoder.getPosition(),
                                new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
        }

        public double getTranslationalVelocity(){
                return drivingEncoder.getVelocity();
        }

        public double get550Current(){
                return steerSparkMax.getOutputCurrent();
        }

        public double getNEOCurrent(){
                return drivingSparkMax.getOutputCurrent();
        }

        /**
         * Sets the desired state for the module.
         *
         * @param desiredState Desired state with speed and angle.
         */
        public void setDesiredState(SwerveModuleState desiredState) {
                // Apply chassis angular offset to the desired state.
                SwerveModuleState correctedDesiredState = new SwerveModuleState();
                correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
                correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

                // Optimize the reference state to avoid spinning further than 90 degrees.
                SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                                new Rotation2d(turningEncoder.getPosition()));

                double desiredSpeed = optimizedDesiredState.speedMetersPerSecond/DriveConstants.kMaxSpeedMetersPerSecond;

                // Command driving and turning SPARKS MAX towards their respective setpoints.
                drivingSparkMax.set(desiredSpeed);
                
                anglePIDController.setReference(optimizedDesiredState.angle.getRadians(),
                                CANSparkMax.ControlType.kPosition);

                // desiredState = desiredState;
        }

            public void setAutoSpeeds(SwerveModuleState desiredState) {
                // Apply chassis angular offset to the desired state.
                SwerveModuleState correctedDesiredState = new SwerveModuleState();
                correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
                correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

                // Optimize the reference state to avoid spinning further than 90 degrees.
                SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                                new Rotation2d(turningEncoder.getPosition()));

                // Command driving and turning SPARKS MAX towards their respective setpoints.
                veloPIDController.setReference(optimizedDesiredState.speedMetersPerSecond,
                                CANSparkMax.ControlType.kVelocity);
                
                anglePIDController.setReference(optimizedDesiredState.angle.getRadians(),
                                CANSparkMax.ControlType.kPosition);

                // desiredState = desiredState;
        }

        /** Zeroes all the SwerveModule encoders. */
        public void resetEncoders() {
                drivingEncoder.setPosition(0);
        }
}
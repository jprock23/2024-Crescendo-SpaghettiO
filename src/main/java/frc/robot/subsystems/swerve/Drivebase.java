package frc.robot.subsystems.swerve;

import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Constants.DriveConstants;

public class Drivebase extends SubsystemBase{
  private static Drivebase instance;

  public SwerveModule frontLeft;
  public SwerveModule backLeft;
  public SwerveModule frontRight;
  public SwerveModule backRight;

  private HolonomicPathFollowerConfig config;

  private static AHRS gyro;
  private double currentRotation = 0.0;

  SwerveDriveOdometry odometry;

  Field2d fieldmap = new Field2d();

  public Drivebase() {

    // Swerve modules

    frontLeft = new SwerveModule(Ports.frontLeftDrive, Ports.frontLeftSteer,
        DriveConstants.kFrontLeftChassisAngularOffset);
    backLeft = new SwerveModule(Ports.backLeftDrive, Ports.backLeftSteer, DriveConstants.kBackLeftChassisAngularOffset);

    frontRight = new SwerveModule(Ports.frontRightDrive, Ports.frontRightSteer,
        DriveConstants.kFrontRightChassisAngularOffset);
    backRight = new SwerveModule(Ports.backRightDrive, Ports.backRightSteer,
        DriveConstants.kBackRightChassisAngularOffset);

    gyro = new AHRS(SPI.Port.kMXP);

    gyro.setAngleAdjustment(90);
    gyro.zeroYaw();

    odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(-gyro.getAngle()), new SwerveModulePosition[] {
            frontLeft.getPosition(),
            backLeft.getPosition(),
            frontRight.getPosition(),
            backRight.getPosition()
        });

    config = new HolonomicPathFollowerConfig(new PIDConstants(.8, 0, 0),
        new PIDConstants(.43, 0, 0),
        2, Math.sqrt(Math.pow(DriveConstants.kTrackWidth / 2, 2) +
            Math.pow(DriveConstants.kWheelBase / 2, 2)),
        new ReplanningConfig());

    SmartDashboard.putData("FIELD", fieldmap);

    AutoBuilder.configureHolonomic(this::getPose, this::resetPose, this::getSpeeds, this::setChassisSpeed, config,
        shouldFlipPath(), this);

  }

  public void setFieldPose(final Pose2d pose) {
    this.resetOdometry(pose);
  }

  public void periodic() {
    SmartDashboard.putNumber("Gyro Angle:", gyro.getRotation2d().getDegrees());
    SmartDashboard.putNumber("X-coordinate", getPose().getX());
    SmartDashboard.putNumber("Y-coordinate", getPose().getY());

    // Update the odometry in the periodic block
    odometry.update(Rotation2d.fromDegrees(-gyro.getAngle()),
        new SwerveModulePosition[] { frontLeft.getPosition(), backLeft.getPosition(), frontRight.getPosition(),
            backRight.getPosition() });

    fieldmap.setRobotPose(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(),
        odometry.getPoseMeters().getRotation());
  }

  // Returns the currently-estimated pose of the robot
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  // Resets the odometry to the specified pose
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(-gyro.getAngle()),
        new SwerveModulePosition[] { frontLeft.getPosition(), backLeft.getPosition(), frontRight.getPosition(),
            backRight.getPosition() },
        pose);
  }

  public void resetOdometry() {
    odometry.resetPosition(
        Rotation2d.fromDegrees(-gyro.getAngle()),
        new SwerveModulePosition[] { frontLeft.getPosition(), backLeft.getPosition(), frontRight.getPosition(),
            backRight.getPosition() },
        getPose());
  }

  public void drive(double forward, double side, double rot, boolean fieldRelative) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    xSpeedCommanded = side;
    ySpeedCommanded = forward;
    currentRotation = rot;

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = currentRotation * DriveConstants.kMaxAngularSpeed;

    var chassisspeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
            Rotation2d.fromDegrees(-gyro.getAngle()))
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    setChassisSpeed(chassisspeeds);
  }

  public void setChassisSpeed(ChassisSpeeds input) {
    var speeds = ChassisSpeeds.discretize(input, 0.02);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState()
    };
  }

  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void resetPose(final Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
  }

  public void lockWheels() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  }

  // sets state for all modules
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    backLeft.setDesiredState(desiredStates[2]);
    frontRight.setDesiredState(desiredStates[1]);
    backRight.setDesiredState(desiredStates[3]);
  }

  // sets drive encoders to 0
  public void resetEncoders() {
    frontLeft.resetEncoders();
    backLeft.resetEncoders();
    frontRight.resetEncoders();
    backRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  // Returns the heading of the robot(=180 to 180)
  public double getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle()).getDegrees();
  }

  public BooleanSupplier shouldFlipPath() {
    return new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }
    };
  }

  // Returns the turn rate of the robot
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public static Drivebase getInstance() {
    if (instance == null) {
      instance = new Drivebase();
    }
    return instance;
  }

}
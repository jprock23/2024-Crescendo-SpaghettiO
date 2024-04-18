package frc.robot.subsystems.swerve;

import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Constants.DriveConstants;

public class Drivebase extends SubsystemBase {
  private static Drivebase instance;

  public enum DriveState {
    NORMAL(1),
    SLOW(0.5);

    public double driveSpeed;

    private DriveState(double driveSpeed) {
      this.driveSpeed = driveSpeed;
    }
  }

  private DriveState driveState = DriveState.NORMAL;

  public SwerveModule frontLeft;
  public SwerveModule backLeft;
  public SwerveModule frontRight;
  public SwerveModule backRight;

  private HolonomicPathFollowerConfig config;

  private static AHRS gyro;

  SwerveDriveOdometry odometry;

  Field2d fieldmap = new Field2d();

  private static SwerveDrivePoseEstimator poseEstimator;

  private ProfiledPIDController headingController;

  public double currHeading;

  public Drivebase() {

    // Swerve modules

    frontLeft = new SwerveModule(Ports.frontLeftDrive, Ports.frontLeftSteer,
        DriveConstants.kFrontLeftChassisAngularOffset, true);
    backLeft = new SwerveModule(Ports.backLeftDrive, Ports.backLeftSteer, DriveConstants.kBackLeftChassisAngularOffset,
        false);
    frontRight = new SwerveModule(Ports.frontRightDrive, Ports.frontRightSteer,
        DriveConstants.kFrontRightChassisAngularOffset, false);
    backRight = new SwerveModule(Ports.backRightDrive, Ports.backRightSteer,
        DriveConstants.kBackRightChassisAngularOffset, true);

    gyro = new AHRS(SPI.Port.kMXP);

    gyro.setAngleAdjustment(90);
    gyro.zeroYaw();

    poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(-gyro.getAngle()),
        getPositions(), new Pose2d());

    headingController = new ProfiledPIDController(4.5, 0, 0, new TrapezoidProfile.Constraints(0, 0), .02);
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    config = new HolonomicPathFollowerConfig(new PIDConstants(1.4, 0, 0),
        new PIDConstants(1.1, 0.0000, 0.0),
        // 0.12, 0.00001, 0.0
        5, Math.sqrt(Math.pow(DriveConstants.kTrackWidth / 2, 2) +
            Math.pow(DriveConstants.kWheelBase / 2, 2)),
        new ReplanningConfig());
    AutoBuilder.configureHolonomic(this::getPose, this::resetOdometry, this::getSpeeds, this::setAutoSpeeds, config,
        shouldFlipPath(), this);
  }

  public void setFieldPose(final Pose2d pose) {
    this.resetOdometry(pose);
  }

  public void periodic() {
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), Rotation2d.fromDegrees(getHeading()), getPositions());
  }

  // Returns the currently-estimated pose of the robot
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public static Pose2d getStaticPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // Resets the odometry to the specified pose
  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(Rotation2d.fromDegrees(-gyro.getAngle()), getPositions(), pose);
  }

  public void resetOdometry() {
    poseEstimator.resetPosition(Rotation2d.fromDegrees(-gyro.getAngle()), getPositions(), getPose());
  }

  public void drive(double forward, double side, double rot) {

    double xSpeedCommanded;
    double ySpeedCommanded;
    double rotationCommanded;

    xSpeedCommanded = side;
    ySpeedCommanded = forward;
    rotationCommanded = rot;

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond * driveState.driveSpeed;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond * driveState.driveSpeed;
    double rotDelivered = rotationCommanded * DriveConstants.kMaxAngularSpeed * driveState.driveSpeed;

    var chassisspeeds = ChassisSpeeds.fromRobotRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
        Rotation2d.fromDegrees(gyro.getAngle()));

    setChassisSpeed(chassisspeeds);
  }

  public void setChassisSpeed(ChassisSpeeds input) {
    var speeds = ChassisSpeeds.discretize(input, 0.02);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setAutoSpeeds(ChassisSpeeds input) {
    var speeds = ChassisSpeeds.discretize(input, 0.02);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    frontLeft.setAutoSpeeds(swerveModuleStates[0]);
    frontRight.setAutoSpeeds(swerveModuleStates[1]);
    backLeft.setAutoSpeeds(swerveModuleStates[2]);
    backRight.setAutoSpeeds(swerveModuleStates[3]);
  }

  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };
  }

  public double getTranslationalVelocity() {
    return backRight.getTranslationalVelocity();
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

  public void rotateTo(double x, double y, double goal) {
    double reqOmega = headingController.calculate(
        getPose().getRotation().getRadians(), Math.toRadians(goal));

    setChassisSpeed(new ChassisSpeeds(x, y, reqOmega));
  }

  public void holdHeading(double x, double y) {
    if (currHeading == -1) {
      currHeading = ((getHeading() + 90) % 360);
    }
    double reqOmega = headingController.calculate(
        getPose().getRotation().getRadians(), Math.toRadians(currHeading));

    setChassisSpeed(new ChassisSpeeds(x, y, reqOmega));
  }

  public void lockWheels() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  }

  // sets state for all modules
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
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
    return -gyro.getAngle();
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
    return gyro.getRate();
  }

  public double inputDeadband(double input) {
    return Math.abs(input) > .075 ? input : 0;
  }

  public double[] getConnections() {
    return new double[] {
        frontLeft.get550Current(), frontLeft.getNEOCurrent(),
        frontRight.get550Current(), frontRight.getNEOCurrent(),
        backLeft.get550Current(), backLeft.getNEOCurrent(),
        backRight.get550Current(), backRight.getNEOCurrent()
    };
  }

  public void printConnections() {
    SmartDashboard.putNumber("FrontLeft550", getConnections()[0]);
    SmartDashboard.putNumber("FrontLeftNEO", getConnections()[1]);

    SmartDashboard.putNumber("FrontRight550", getConnections()[2]);
    SmartDashboard.putNumber("FrontRightNEO", getConnections()[3]);

    SmartDashboard.putNumber("BackLeft550", getConnections()[4]);
    SmartDashboard.putNumber("BackLeftNEO", getConnections()[5]);

    SmartDashboard.putNumber("BackRight550", getConnections()[6]);
    SmartDashboard.putNumber("BackRightNEO", getConnections()[7]);
  }

  public void printTranslationalVelocities() {
    SmartDashboard.putNumber("Front Left TranslationalVelo", frontLeft.getTranslationalVelocity());
    SmartDashboard.putNumber("Front Right TranslationalVelo", frontRight.getTranslationalVelocity());
    SmartDashboard.putNumber("Back Left TranslationalVelo", backLeft.getTranslationalVelocity());
    SmartDashboard.putNumber("Back Right TranslationalVelo", backRight.getTranslationalVelocity());

  }

  public double getFLVelo() {
    return frontLeft.getTranslationalVelocity();
  }

  public double getFRVelo() {
    return frontRight.getTranslationalVelocity();
  }

  public double getBLVelo() {
    return backLeft.getTranslationalVelocity();
  }

  public double getBRVelo() {
    return backRight.getTranslationalVelocity();
  }

  public void setDriveState(DriveState state) {
    driveState = state;
  }

  public double[] getModuleRotations() {
    return new double[] {
        frontLeft.getModuleAngle(),
        frontRight.getModuleAngle(),
        backLeft.getModuleAngle(),
        backRight.getModuleAngle()
    };
  }

  public static Drivebase getInstance() {
    if (instance == null) {
      instance = new Drivebase();
    }
    return instance;
  }

}
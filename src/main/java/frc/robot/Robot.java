// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.launcher.*;
import frc.robot.subsystems.launcher.Launcher.LauncherPosition;
import frc.robot.subsystems.launcher.LauncherStates.LauncherControl;
import frc.robot.subsystems.launcher.LauncherStates.LauncherState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.swerve.Drivebase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  private Drivebase drivebase;
  private Climber climber;
  private Intake intake;

  private Launcher launcher;

  private static XboxController driver;
  private static XboxController operator;

  private double buttonPressed = 0.0;
  private double poseReached = 0.0;

  private Command m_autoSelected;
  private SendableChooser<Command> m_chooser;

  @Override
  public void robotInit() {
    drivebase = Drivebase.getInstance();
    launcher = Launcher.getInstance();
    intake = Intake.getInstance();
    climber = Climber.getInstance();

    driver = new XboxController(0);
    operator = new XboxController(1);
    drivebase.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0)));

    // m_chooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto choices", m_chooser);

    // CameraServer.startAutomaticCapture(0);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    drivebase.periodic();

    SmartDashboard.putNumber("Flipper Position", intake.getFlipperPosition());

    // SmartDashboard.putNumber("bigFlipper1", launcher.getPosition1());
    // SmartDashboard.putNumber("bigFlipper2", launcher.getPosition2());

    SmartDashboard.putNumber("Relative Launcher Position1", launcher.getLauncherPosition1());
    SmartDashboard.putNumber("Relative Launcher Position2", launcher.getLauncherPosition2());

    SmartDashboard.putNumber("Flipper Velocity", intake.getFlipperVelocity());
    SmartDashboard.putNumber("Flipper Velocity Setpoint", intake.getFlipperVelocitySetpoint());
    SmartDashboard.putString("Intake Position", intake.getIntakeState());

    SmartDashboard.putNumber("Pivot Velocity1", launcher.getPivotVelocity1());
    SmartDashboard.putNumber("Pivot Velocity2", launcher.getPivotVelocity2());
    SmartDashboard.putNumber("Pivot Acceleration1", launcher.getPivotAcceleration1(buttonPressed, poseReached));
    SmartDashboard.putNumber("Pivot Accleration2", launcher.getPivotAcceleration2(buttonPressed, poseReached));

    SmartDashboard.putNumber("Pivot Target Velocity", launcher.getPivotTargetVelocity());
    SmartDashboard.putNumber("Pivot Target Acceleration", launcher.getPivotTargetAcceleration());

    SmartDashboard.putString("Pivot Position", launcher.getPivotPosition());

    SmartDashboard.putNumber("Pivot Velocity Setpoint", launcher.getPivotVelocitySetPoint());
    SmartDashboard.putNumber("Pivot1 power", launcher.getReqPower1());

  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();

    if (m_autoSelected != null) {
      m_autoSelected.schedule();
    }

    // visionTables.putInfoOnDashboard();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // if (m_autoSelected != null) {
    // m_autoSelected.cancel();
    // }
  }

  @Override
  public void teleopPeriodic() {
    intake.periodic();
    launcher.periodic();

    boolean fieldRelative = true;

    // if(launcher.hasReachedPose(.001)){
    // poseReached = Timer.getFPGATimestamp();
    // }

    // if (driver.getLeftTriggerAxis() > 0.0){
    // launcher.setPivotState(LauncherPosition.TESTDOWN);
    // buttonPressed = Timer.getFPGATimestamp();
    // }
    // if (driver.getRightTriggerAxis() > 0.0){
    // launcher.setPivotState(LauncherPosition.TESTUP);
    // buttonPressed = Timer.getFPGATimestamp();
    // }

    /* DRIVE CONTROLS */

    // double ySpeed = driver.getLeftX();
    // double xSpeed = -driver.getLeftY();
    // double rot = driver.getRightX();

    double ySpeed = 0;
    double xSpeed = 0;
    double rot = 0;

    SmartDashboard.putNumber("Xspeed", xSpeed);
    SmartDashboard.putNumber("Yspeed", ySpeed);
    // SmartDashboard.putNumber("Vision yPose", visAlign.getY());
    SmartDashboard.putNumber("rot", rot);

    if (driver.getYButton()) {
      fieldRelative = !fieldRelative;
    }
    if (driver.getAButton()) {
      drivebase.lockWheels();
    } else {
      drivebase.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    /* INTAKE CONTROLS */

    // flipping intake

    boolean reverseLauncher = false;
    // rolling intake rollers
    if (operator.getYButton()) {
      intake.setIntakeState(IntakePosition.GROUND);
      intake.setRollerPower();
    } else if (operator.getXButton()) {
      intake.setReverseRollerPower();
      launcher.setReverse();
    } else if (operator.getAButton()) {
      intake.setRollerPower();
    } else if (operator.getRightTriggerAxis() >= .1) {
      launcher.setLauncherPower();
    } else {
      intake.setIntakeState(IntakePosition.HANDOFF);
      intake.setRollerOff();
      launcher.setLaunchZero();
    }

    // *CLIMBER CONTROLS */

    if (operator.getRightBumper()) {
      climber.setClimbingPower();
    } else if (operator.getLeftBumper()) {
      climber.reverseClimb();
    } else {
      climber.setClimberStop();
    }

    /* LAUNCHER CONTROLS */

    if (operator.getLeftStickButton()){
      launcher.setPivotState(LauncherPosition.AMP);
    } else if (operator.getRightStickButton()){
      launcher.setPivotState(LauncherPosition.HANDOFF);
    }

    // if (-operator.getRightY() > 0) {
    //   launcher.setLauncherAngle();
    // } else if (-operator.getRightY() < 0) {
    //   launcher.setReverseLauncherAngle();
    // } else {
    //   launcher.setAngleStop();
    // }

    // Launching notes

    // Flicking
    if (operator.getBButton()) {
      launcher.setFlickerOn();
    } else if (operator.getLeftTriggerAxis() > .1) {
      launcher.setFlickerReverse();
    } else {
      launcher.setFlickOff();
    }

    // launcher.setFlickerOn();

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}

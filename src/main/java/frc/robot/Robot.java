// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.swerve.Drivebase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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

  // private Launcher launcher;

  private static XboxController driver;
  private static XboxController operator;

  private Command m_autoSelected;
  private SendableChooser<Command> m_chooser;

  @Override
  public void robotInit() {
    drivebase = Drivebase.getInstance();
    // launcher = Launcher.getInstance();
    intake = Intake.getInstance();
    climber = Climber.getInstance();

    driver = new XboxController(0);
    operator = new XboxController(1);
    drivebase.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0)));

    m_chooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto choices", m_chooser);
    // m_chooser.addOption("DriveCommand", new PathPlannerAuto("TestAutov2"));
    // m_chooser.addOption("Straight Auto", new PathPlannerAuto("Straight"));
    SmartDashboard.putData("Auto choices", m_chooser);

    // CameraServer.startAutomaticCapture(0);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    drivebase.periodic();

    SmartDashboard.putNumber("Flipper Position", intake.getFlipperPosition());

    // SmartDashboard.putNumber("Relative Launcher Position1",
    // Launcher.getLauncherPosition1());
    // SmartDashboard.putNumber("Relative Launcher Position2",
    // Launcher.getLauncherPosition2());

    SmartDashboard.putString("Intake State", intake.getIntakeState());

    // SmartDashboard.putString("Pivot Position", launcher.getPivotPosition());

    SmartDashboard.putNumber("Start Time", intake.getStartTime());

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
    if (m_autoSelected != null) {
      m_autoSelected.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    intake.periodic();
    // launcher.periodic();

    boolean fieldRelative = true;

    /* DRIVE CONTROLS */

    double ySpeed = driver.getLeftX();
    double xSpeed = -driver.getLeftY();
    double rot = driver.getRightX();

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

    // rolling intake rollers
    if (operator.getYButton()) {
      // intake.setIntakeState(IntakePosition.GROUND);
      intake.setRollerPower();
    } else if (operator.getXButton()) {
      intake.setReverseRollerPower();
      // launcher.setReverse();
    } else if (operator.getAButton()) {
      intake.setRollerPower();
    } else if (operator.getRightTriggerAxis() >= .1) {
      // launcher.setLauncherPower();
    }
    // else if (operator.getRightTriggerAxis() >= .1) {
    // launcher.setLauncherPower();
    // intake.setRollerOff();
    // launcher.setLaunchZero();
    // }
    else {
      // intake.setIntakeState(IntakePosition.HANDOFF);
      intake.setRollerOff();
      // launcher.setLaunchZero();
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

    if (operator.getLeftStickButton()) {
      // launcher.setPivotState(LauncherPosition.HANDOFF);
      intake.setIntakeState(IntakePosition.HANDOFF);
      intake.resetStartTime();
    } else if (operator.getRightStickButton()) {
      // launcher.setPivotState(LauncherPosition.AMP);
      intake.setIntakeState(IntakePosition.GROUND);
    }

    // if (operator.getLeftStickButton()){
    // launcher.setPivotState(LauncherPosition.AMP);
    // } else if (operator.getRightStickButton()){
    // launcher.setPivotState(LauncherPosition.HANDOFF);
    // }

    // if (-operator.getRightY() > 0) {
    // launcher.setLauncherAngle();
    // } else if (-operator.getRightY() < 0) {
    // launcher.setReverseLauncherAngle();
    // } else {
    // launcher.setAngleStop();
    // }

    // Launching notes

    // Flicking
    if (operator.getBButton()) {
      // launcher.setFlickerOn();
    } else if (operator.getLeftTriggerAxis() > .1) {
      // launcher.setFlickerReverse();
    } else {
      // launcher.setFlickOff();
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

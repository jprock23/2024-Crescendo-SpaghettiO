// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auton.test.Boopbop;
import frc.robot.commands.TestHandoff;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.swerve.Drivebase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.SparkMaxRelativeEncoder;

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

  private Command m_autoSelected;
  private TestHandoff handoff;
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

    m_chooser = AutoBuilder.buildAutoChooser();
    m_chooser.addOption("Bop", new PathPlannerAuto("Bop"));
    m_chooser.addOption("Boopbop", new Boopbop());
    SmartDashboard.putData("Auto choices", m_chooser);

    handoff = new TestHandoff();

    // CameraServer.startAutomaticCapture(0);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    drivebase.periodic();

    launcher.launcherConnections();
    intake.intakeConnections();
    climber.climberConnections();

    launcher.printConnections();
    intake.printConnections();
    climber.printConnections();

    SmartDashboard.putNumber("Flipper Current", intake.getFlipperCurrent());
    SmartDashboard.putNumber("Pivot Current", launcher.getPivotCurrent());

    // intake.setRollerOff();
    // intake.setFlipperOff();

    // launcher.setFlickOff();
    // launcher.setAngleStop();
    // launcher.setLauncherOff();

    // climber.setClimberStop();

    SmartDashboard.putNumber("Roller Current", intake.getRollerCurrent());

    SmartDashboard.putString("intake state", intake.getIntakeState());

    SmartDashboard.putNumber("Flipper Position", intake.getFlipperPosition());

    SmartDashboard.putNumber(" Launcher Position", launcher.getPosition());

    SmartDashboard.putString("Intake State", intake.getIntakeState());

    SmartDashboard.putString("Launcher State", launcher.getLaunchState());

    SmartDashboard.putNumber("Velocity Goal", launcher.getVelocityGoal());
    SmartDashboard.putNumber("Current Velocity", launcher.getVelocity());

    SmartDashboard.putNumber("P", launcher.getConstants()[0]);
    SmartDashboard.putNumber("I", launcher.getConstants()[1]);
    SmartDashboard.putNumber("D", launcher.getConstants()[2]);
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
    intake.periodic();
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
    launcher.periodic();

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

    if (operator.getXButton()) {
      intake.setIntakeState(IntakePosition.GROUND);
    } else if (operator.getYButton()) {
      intake.setIntakeState(IntakePosition.HANDOFF);
    } else if (operator.getLeftBumper()) {
      intake.setIntakeState(IntakePosition.STOP);
    }

    // if(operator.getXButton()){
    // intake.setFlipperPower();
    // } else if(operator.getYButton()){
    // intake.setReverseFlipperPower();
    // } else {
    // intake.setFlipperOff();
    // }

    // if (operator.getRightBumper()) {
    //   intake.setRollerPower();
    // } else {
    //   intake.setRollerOff();
    // }

      if(operator.getRightBumper()){
        handoff.schedule();
      }

    if (operator.getRightTriggerAxis() > 0) {
      launcher.setLauncherOn();
    } else if (operator.getLeftTriggerAxis() > 0) {
      launcher.setReverseLauncherOn();
      launcher.setFlickerReverse();
    } 

    if (operator.getLeftStickButton()) {
      launcher.setFlickerOn();
    } else if (!(operator.getLeftTriggerAxis() > 0)) {
      launcher.setFlickOff();
    }

    // // rolling intake rollers
    // if (operator.getYButton()) {
    // // intake.setIntakeState(IntakePosition.GROUND);
    // intake.setRollerPower();
    // } else if (operator.getXButton()) {
    // intake.setReverseRollerPower();
    // // launcher.setReverse();
    // } else if (operator.getAButton()) {
    // intake.setRollerPower();
    // } else if (operator.getRightTriggerAxis() >= .1) {
    // // launcher.setLauncherPower();
    // }
    // // else if (operator.getRightTriggerAxis() >= .1) {
    // // launcher.setLauncherPower();
    // // intake.setRollerOff();
    // // launcher.setLaunchZero();
    // // }
    // else {
    // // intake.setIntakeState(IntakePosition.HANDOFF);
    // intake.setRollerOff();
    // // launcher.setLaunchZero();
    // }

    // *CLIMBER CONTROLS */

    if (driver.getRightBumper()) {
      climber.setClimbingPower();
    } else if (driver.getLeftBumper()) {
      climber.setReverseClimberPower();
    } else {
      climber.setClimberOff();
    }

    /* LAUNCHER CONTROLS */

    // if (-operator.getRightY() > 0) {
    //   launcher.setPivotPower();
    // } else if (-operator.getRightY() < 0) {
    //   launcher.setReversePivotPower();
    // } else {
    //   launcher.setPivotOff();
    // }

    if(operator.getAButton()){
    launcher.setPivotState(LauncherState.HANDOFF);
    } else if(operator.getBButton()){
    launcher.setPivotState(LauncherState.SPEAKER);
    }

    // Launching notes

    // Flicking
    // if (operator.getBButton()) {
    // launcher.setFlickerOn();
    // } else if (operator.getLeftTriggerAxis()>0) {
    // launcher.setFlickerReverse();
    // } else {
    // launcher.setFlickOff();
    // }
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

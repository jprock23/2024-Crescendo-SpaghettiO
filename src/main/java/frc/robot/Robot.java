// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.launcher.*;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
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
  // private Launcher launcher;
  private Drivebase drivebase;
  private Climber climber;
  private Intake intake;

  private Launcher launcher;

  // private AddressableLED lit;
  // private AddressableLEDBuffer litBuffer;

  // private VisionTablesListener visionTables;
  // private AutoAlign visAlign;

  private static XboxController driver;
  private static XboxController operator;

  private Command m_autoSelected;
  private SendableChooser<Command> m_chooser;

  @Override
  public void robotInit() {
    drivebase = Drivebase.getInstance();
    launcher = Launcher.getInstance();
    intake = Intake.getInstance();
    climber = Climber.getInstance();

    // lit = new AddressableLED(9);
    // litBuffer = new AddressableLEDBuffer(60);
    // lit.setLength(litBuffer.getLength());
    // lit.setData(litBuffer);
    // lit.start();

    driver = new XboxController(0);
    operator = new XboxController(1);
    drivebase.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0)));

    // visionTables = VisionTablesListener.getInstance();
    // visAlign = AutoAlign.getInstance();

    // m_chooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto choices", m_chooser);

    // CameraServer.startAutomaticCapture(0);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    drivebase.periodic();
    // SmartDashboard.putNumber("Flipper position", intake.getFlipperPosition());
    SmartDashboard.putNumber("Climber1Current", climber.getAmpClimber1());
    SmartDashboard.putNumber("Climber2Current", climber.getAmpClimber2());

    SmartDashboard.putString("Amit", "Amitball");

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

    boolean fieldRelative = true;

    /* DRIVE CONTROLS */

    double ySpeed = driver.getLeftX();
    double xSpeed = -driver.getLeftY();
    double rot = driver.getRightX();


    SmartDashboard.putNumber("Xspeed", xSpeed);
    SmartDashboard.putNumber("Yspeed", ySpeed);
    // SmartDashboard.putNumber("Vision yPose", visAlign.getY());
    SmartDashboard.putNumber("rot", rot);

    if (driver.getXButton()) {
      fieldRelative = !fieldRelative;
    }
    if (driver.getBButton()) {
      drivebase.lockWheels();
    } else {
      drivebase.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    // if (operator.getAButton()){
    // intake.setIntakeState(IntakePosition.GROUND);
    // } else if (operator.getBButton()){
    // intake.setIntakeState(IntakePosition.HANDOFF);
    // }
    // } else if (operator.getYButton()){
    // intake.setIntakeState(IntakePosition.RETRACTED);
    // }

    //flipping intake
    if(operator.getXButton()){
    intake.setFlipperPower();
    } else if(operator.getYButton()){
    intake.reverse();
    } else {
    intake.setFlipperOff();
    }

    //rolling intake rollers
    if (operator.getRightBumper()){
    intake.setRollerPower();
    } else if(operator.getLeftBumper()){
    intake.setRollerOff();
    }

    // *CLIMBER CONTROLS */

    // if (operator.getXButton()) {
    //   climber.setClimbingPower();
    // } else if (operator.getBButton()) {
    //   climber.reverseClimb();
    // } else {
    //   climber.setClimberStop();
    // }


    

    // // //Launcher angles
    // if(operator.getRightTriggerAxis() > 0.0){
    // launcher.setLauncherAngle(operator.getRightTriggerAxis(),
    // operator.getRightTriggerAxis());
    // }else if(operator.getLeftTriggerAxis()> 0.0){
    // launcher.setLauncherAngle(-operator.getLeftTriggerAxis(),
    // -operator.getLeftTriggerAxis());
    // }else{
    // launcher.setAngleStop();
    // }

    // // // //Launching notes
    // if(operator.getAButton()){
    // launcher.setLauncherPower();
    // }else if(operator.getLeftStickButton()){
    // launcher.increasePower();
    // }else if(operator.getRightStickButton()){
    // launcher.decreasePower();
    // }else if(operator.getXButton()){
    // launcher.setReverse();
    // }else{
    //    launcher.setLaunchZero();
    // }

    // // //Flicking

    // if (operator.getBButton()){
    //   launcher.setFlickerOn();
    // } else {
    //   launcher.setFlickOff();
    // }

    // // //angles to launch

    // if(operator.getRightBumper()){
    //   launcher.setAngle();
    // }else if(operator.getLeftBumper()){
    //   launcher.setReverseAngle();
    // }else {
    //   launcher.setZeroAngle();
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

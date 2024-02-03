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

    SmartDashboard.putNumber("Flipper Voltage", intake.getFlipperVoltage());
    SmartDashboard.putNumber("Flipper Power", intake.getFlipPower());
    SmartDashboard.putNumber("Flipper Position", intake.getFlipperPosition());
    SmartDashboard.putNumber("bigFlipper1", launcher.getPosition());
    SmartDashboard.putNumber("bigFlipper2", launcher.getPosition2());
   

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

    /*INTAKE CONTROLS */

    //flipping intake

    // if(-operator.getLeftY() >= .2){
    // intake.setFlipperPower();
    // } else if(-operator.getLeftY() <= -.2){
    // intake.reverseFlipper();
    // } else {
    // intake.setFlipperOff();
    // }

  
    // rolling intake rollers
    if (operator.getYButton()){
    intake.setRollerPower();
    intake.setIntakeState(IntakePosition.HANDOFF);
    }else if (operator.getXButton()){
      intake.setReverseRollerPower();
    }else{
      intake.setIntakeState(IntakePosition.GROUND);
    intake.setRollerOff();
    }

    // *CLIMBER CONTROLS */

    if (operator.getRightBumper()) {
      climber.setClimbingPower();
    } else if (operator.getLeftBumper()) {
      climber.reverseClimb();
    } else {
      climber.setClimberStop();
    }

    /*LAUNCHER CONTROLS*/

    // Launcher angles
    boolean reverseBigFlipper = false;
    if(operator.getRightStickButton()){
      reverseBigFlipper = !reverseBigFlipper;
    }

    // if(-operator.getRightY() > .2 && !reverseBigFlipper){
    //   launcher.setLauncherAngle();
    // } else if (-operator.getRightY() < .2){
    //   launcher.setReverseLauncherAngle();
    // } else{
     launcher.setAngleStop();
    // }

    

    //Launching notes
    boolean reverseLauncher = false;

     if(operator.getAButton()){
      reverseLauncher = !reverseLauncher;
    } 

    if(operator.getRightTriggerAxis() >= .1 && !reverseLauncher){
      launcher.setLauncherPower();
    } else if (operator.getRightTriggerAxis() >= .1 && reverseLauncher){
      launcher.setReverse();
    } else {
      launcher.setLaunchZero();
    } 

    //Flicking
   if (operator.getBButton()){
       launcher.setFlickerOn();
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

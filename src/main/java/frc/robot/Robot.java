// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.AutoAlign;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.VisionTablesListener;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.swerve.Drivebase;
import frc.robot.subsystems.Climbing;
import frc.robot.subsystems.Intake;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
    private Launcher launcher;
    private Drivebase drivebase;
    private Climbing climber;
    private Intake intake;
    private VisionTablesListener visionTables;
    private AutoAlign visAlign;
    private static TorqueLogiPro driver;
    private static XboxController operator;
    
    private boolean manual;

    private Command m_autoSelected;
    private SendableChooser<Command> m_chooser;
  
  @Override
  public void robotInit() {
    drivebase = Drivebase.getInstance();
    launcher = Launcher.getInstance();
    climber = Climbing.getInstance();
    intake = Intake.getInstance();
    
    driver = new TorqueLogiPro(0);
    operator = new XboxController(1);
    drivebase.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0)));

    visionTables = VisionTablesListener.getInstance();
    visAlign = AutoAlign.getInstance();

    m_chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  @Override
  public void robotPeriodic() {
      CommandScheduler.getInstance().run();
      drivebase.periodic();
      

      SmartDashboard.putBoolean("Arm Manual:", manual);
      visionTables.putInfoOnDashboard();
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
  

     if (m_autoSelected != null) {
      m_autoSelected.schedule();
     }

  
    visionTables.putInfoOnDashboard();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // if (m_autoSelected != null) {
    //   m_autoSelected.cancel();
    // }
  }

  @Override
  public void teleopPeriodic() {
      boolean fieldRelative = true;
      boolean climbingMode = false;

      /* Drive Controls */
      double 
      ySpeed = -driver.getRoll();
      double xSpeed = -driver.getPitch();

      //double ySpeed = driver.getLeftY();
      //double xSpeed = driver.getLeftX();
      //double rot = driver.getRightX();
      double rot = 0;

      SmartDashboard.putNumber("Xspeed", xSpeed);
      SmartDashboard.putNumber("Yspeed", ySpeed);
      SmartDashboard.putNumber("Vision yPose", visAlign.getY());
      SmartDashboard.putNumber("rot", rot);

      if (driver.getTrigger()) {
          rot = driver.getYaw();
      }
      if (driver.getButtonByIndex(10)) {
          fieldRelative = ! fieldRelative;
      }
      if (driver.getButtonByIndex(7)) {
          drivebase.lockWheels();
      }
      else if (driver.getButtonByIndex(2)) {
          //drivebase.drive(0, 0, visAlign.getRotSpeed(), fieldRelative);
          drivebase.drive(visAlign.getXSpeed(), visAlign.getYSpeed(), visAlign.getRotSpeed(), fieldRelative);
      } else {
          drivebase.drive(xSpeed, ySpeed, rot, fieldRelative);
      }

      //swap to climbing mode
      if(operator.getYButtonPressed())
        climbingMode = !climbingMode;
      
      //Controls for launcher for now
      if(operator.getAButton()) {
        launcher.setLauncherPower(1.0);
      } else if(operator.getBButton()) {
        launcher.setLauncherPower(0.0);
      } 
      
      //Controls for Intake
      intake.setIntakePowers(operator.getLeftTriggerAxis(), operator.getRightX());

      if(climbingMode) {
        //Controls for climbing for now
        climber.setClimberPower(operator.getLeftY(), operator.getRightX());

        // if(operator.getRightTriggerAxis() > 0.0){
        //     climber.setClimberPower(1.0,1.0);
        // } else if(operator.getLeftTriggerAxis()>0.0){
        //     climber.setClimberPower(-1.0,-1.0);
        // }else{
        //   climber.setClimberPower(0.0,0.0);
        // }
      }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutoAmp;
import frc.robot.commands.AutoSpeaker;
import frc.robot.commands.BreakBeamHandoff;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.IO.DigitalInputs;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;
import frc.robot.subsystems.led.LED;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.swerve.Drivebase;
import frc.robot.subsystems.vision.AutoAlign;
import frc.robot.subsystems.vision.VisionTablesListener;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
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
  private DigitalInputs digitalInputs;
  private AutoAlign autoAlign;
  private LED litty;
  private VisionTablesListener visTables;

  private static XboxController driver;
  private static XboxController operator;

  private Command m_autoSelected;

  private BreakBeamHandoff handoffCommand;
  private ShootCommand shootCommand;
  private AutoSpeaker autoSpeaker;
  private AutoAmp autoAmp;

  private SendableChooser<Command> m_chooser;

  @Override
  public void robotInit() {
    drivebase = Drivebase.getInstance();
    launcher = Launcher.getInstance();
    intake = Intake.getInstance();
    climber = Climber.getInstance();
    digitalInputs = DigitalInputs.getInstance();
    autoAlign = AutoAlign.getInstance();
    litty = LED.getInstance();
    visTables = VisionTablesListener.getInstance();

    driver = new XboxController(0);
    operator = new XboxController(1);
    drivebase.resetOdometry(new Pose2d(1.3, 5.56, new Rotation2d(0)));

    handoffCommand = new BreakBeamHandoff();
    shootCommand = new ShootCommand();
    autoSpeaker = new AutoSpeaker();
    autoAmp = new AutoAmp();

    NamedCommands.registerCommand("AutoAmp", autoAmp);
    NamedCommands.registerCommand("AutoSpeaker", autoSpeaker);
    NamedCommands.registerCommand("Handoff", handoffCommand);

    m_chooser = AutoBuilder.buildAutoChooser();
    m_chooser.addOption("Test1", new PathPlannerAuto("Test1"));
    m_chooser.addOption("Test2", new PathPlannerAuto("Test2"));
    m_chooser.addOption("Test3", new PathPlannerAuto("Test3"));
    m_chooser.addOption("BeepBoop", new PathPlannerAuto("BeepBoop"));

    SmartDashboard.putData("Auto choices", m_chooser);

    litty.setBlue();

    CameraServer.startAutomaticCapture(0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    drivebase.periodic();

    launcher.launcherConnections();
    intake.intakeConnections();
    climber.climberConnections();

    // launcher.printConnections();
    // intake.printConnections();
    // climber.printConnections();

    boolean[] DIO = digitalInputs.getInputs();

    SmartDashboard.putNumber("RIght Stick X", driver.getRightX());;

    SmartDashboard.putNumber("Gyro Angle:", drivebase.getHeading());
    SmartDashboard.putNumber("X-coordinate", drivebase.getPose().getX());
    SmartDashboard.putNumber("Y-coordinate", drivebase.getPose().getY());

    // SmartDashboard.putNumber("Flipper Current", intake.getFlipperCurrent());
    // SmartDashboard.putNumber("Pivot Current", launcher.getPivotCurrent());
    // SmartDashboard.putNumber("Roller Current", intake.getRollerCurrent());

    SmartDashboard.putNumber("Flipper Position", intake.getFlipperPosition());
    SmartDashboard.putNumber("Launcher Position", launcher.getPosition());

    // SmartDashboard.putString("Intake State", intake.getIntakeState());
    SmartDashboard.putBoolean("Breakbeam", launcher.getBreakBeam());

    // SmartDashboard.putString("Launcher State",
    // launcher.getLaunchState().toString());
    // SmartDashboard.putBoolean("Shoot Done", autoSpeaker.isFinished());

// visTables.putInfoOnDashboard();

    SmartDashboard.putNumber("Translational Velocity", drivebase.getTranslationalVelocity());
    SmartDashboard.putNumber("Angular Velocity", drivebase.getTurnRate());
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();

    drivebase.resetOdometry(new Pose2d(2.0, 1.0, new Rotation2d(0)));

    // drivebase.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(m_chooser.getSelected().getName()));

    if (m_autoSelected != null) {
      m_autoSelected.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    intake.updatePose();
    litty.setDisco();
  }

  @Override
  public void teleopInit() {
    if (m_autoSelected != null) {
      m_autoSelected.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    intake.updatePose();

    boolean fieldRelative = true;

    /* DRIVE CONTROLS */
    double ySpeed;
    double xSpeed;
    double rot;
    if (driver.getRightBumper()) {
      ySpeed = autoAlign.getXSpeed();
      // xSpeed = autoAlign.getYSpeed();
      // rot = autoAlign.getRotSpeed();
      xSpeed = 0;
      rot = 0;
    } else {
      ySpeed = driver.getLeftX();
      xSpeed = -driver.getLeftY();
      rot = -driver.getRightX();
    }

    if (operator.getBButton()) {
      litty.setBlue();
    }

    if (operator.getYButton()) {
      litty.setRed();
    }

    if (driver.getYButton()) {
      fieldRelative = !fieldRelative;
    }
    if (driver.getAButton()) {
      drivebase.lockWheels();
      drivebase.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(m_chooser.getSelected().getName()));
    } else {
      drivebase.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    /* INTAKE CONTROLS */

    if (operator.getRightBumper()) {
      handoffCommand.schedule();
    }

    if (operator.getPOV() == 0) {
      launcher.setLauncherState(LauncherState.SPEAKER);
    }
    if (operator.getPOV() == 90) {
      launcher.setLauncherState(LauncherState.AMP);
    }
    if (operator.getPOV() == 180) {
      launcher.setLauncherState(LauncherState.TRAP);
    }
    if (operator.getPOV() == 270) {
      launcher.setLauncherState(LauncherState.HOLD);
    }

    if (operator.getLeftBumper()) {
      intake.setIntakeState(IntakeState.STOP);
    }

    // if(operator.getYButton()){
    // intake.setIntakeState(IntakeState.HANDOFF);
    // }

    // if (operator.getXButton()) {
    // launcher.setLauncherState(LauncherState.AMP);
    // } else if (operator.getYButton()) {
    // launcher.setLauncherState(LauncherState.TRAP);
    // } else if (operator.getLeftBumper()) {
    // intake.setIntakeState(IntakePosition.STOP);
    // } else if (operator.getLeftStickButton()) {
    // intake.setIntakeState(IntakePosition.GROUND);
    // }
    //
    // if (operator.getAButton()) {
    // launcher.setLauncherState(LauncherState.HANDOFF);
    // } else if (operator.getBButton()) {
    // launcher.setLauncherState(LauncherState.SPEAKER);
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
    // launcher.setPivotPower();
    // } else if (-operator.getRightY() < 0) {
    // launcher.setReversePivotPower();
    // } else {
    // launcher.setPivotOff();
    // }

    if (operator.getRightTriggerAxis() > 0) {
      shootCommand.initialize();
      shootCommand.schedule();
    } else if (operator.getLeftTriggerAxis() > 0) {
      launcher.setLauncherOff();
      launcher.setFlickOff();
      intake.setRollerOff();
      shootCommand.cancel();
      handoffCommand.cancel();
      litty.setBlue();
    }

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

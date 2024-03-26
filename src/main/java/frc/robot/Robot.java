// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AmpCommand;
import frc.robot.commands.AutoAmp;
import frc.robot.commands.AutoHandoff;
import frc.robot.commands.AutoLeftShot;
import frc.robot.commands.AutoMidShot;
import frc.robot.commands.AutoRightShot;
import frc.robot.commands.AutoSpeaker;
import frc.robot.commands.BreakBeamHandoff;
import frc.robot.commands.Celebrate;
import frc.robot.commands.HandoffCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.IO.LED;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.subsystems.swerve.Drivebase;
import frc.robot.subsystems.swerve.Drivebase.DriveState;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class Robot extends LoggedRobot {

  private Drivebase drivebase;
  private Climber climber;
  private Intake intake;
  private Launcher launcher;
  private LED litty;
  // private VisionTablesListener visTables;

  private static XboxController driver;
  private static XboxController operator;

  private Command m_autoSelected;

  private BreakBeamHandoff handoffCommand;
  private ShootCommand shootCommand;
  private AutoSpeaker autoSpeaker;
  private AutoAmp autoAmp;
  private HandoffCommand currentSpikeHandoff;
  private AmpCommand ampCommand;

  private boolean useCurrentSpike;
  private boolean hasBrownedOut;

  private SendableChooser<Command> m_chooser;

     StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Pose", Pose3d.struct).publish();

  @Override
  public void robotInit() {
    drivebase = Drivebase.getInstance();
    launcher = Launcher.getInstance();
    intake = Intake.getInstance();
    climber = Climber.getInstance();
    litty = LED.getInstance();
    // visTables = VisionTablesListener.getInstance();

    driver = new XboxController(0);
    operator = new XboxController(1);
    // drivebase.resetOdometry(new Pose2d(1, 1, new Rotation2d(0)));

    handoffCommand = new BreakBeamHandoff();
    shootCommand = new ShootCommand();
    autoSpeaker = new AutoSpeaker();
    autoAmp = new AutoAmp();
    ampCommand = new AmpCommand();

    currentSpikeHandoff = new HandoffCommand();

    NamedCommands.registerCommand("AutoAmp", autoAmp);
    NamedCommands.registerCommand("AutoSpeaker", autoSpeaker);
    NamedCommands.registerCommand("Handoff", new AutoHandoff());
    NamedCommands.registerCommand("AutoLeftShot", new AutoLeftShot());
    NamedCommands.registerCommand("AutoRightShot", new AutoRightShot());
    NamedCommands.registerCommand("AutoMidShot", new AutoMidShot());
    NamedCommands.registerCommand("Celebrate", new Celebrate());

    m_chooser = AutoBuilder.buildAutoChooser();
    // m_chooser.addOption("Test1", new PathPlannerAuto("Test1"));
    // m_chooser.addOption("Test2", new PathPlannerAuto("Test2"));
    // m_chooser.addOption("Test3", new PathPlannerAuto("Test3"));
    // // m_chooser.addOption("BeepBoop", new PathPlannerAuto("BeepBoop"));
    // m_chooser.addOption("Test2 Left", new PathPlannerAuto("Test2 Left"));
    //     m_chooser.addOption("Test2 Right", new PathPlannerAuto("Test2 Right"));
    m_chooser.addOption("P2 4 Piece", new PathPlannerAuto("P2 4 Piece"));
    m_chooser.addOption("P3 4 Piece", new PathPlannerAuto("P3 4 Piece"));
    m_chooser.addOption("Handoff Test", new PathPlannerAuto("Handoff Test"));
    m_chooser.addOption("P1 4 Piece", new PathPlannerAuto("P1 4 Piece"));
        m_chooser.addOption("Q7", new PathPlannerAuto("Q7"));


    SmartDashboard.putData("Auto choices", m_chooser);

    useCurrentSpike = false;

    // CameraServer.startAutomaticCapture();

  }

  @Override

  public void robotPeriodic() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

if (isReal()) {
    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
} else {
    setUseTiming(false); // Run as fast as possible
    String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
}

// Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    CommandScheduler.getInstance().run();
    drivebase.periodic();

    // launcher.launcherConnections();
    // intake.intakeConnections();
    // climber.climberConnections();
  

    // launcher.printConnections();
    // intake.printConnections();
    // climber.printConnections();
    // drivebase.printConnections();

    // drivebase.printTranslationalVelocities();

    // visTables.putInfoOnDashboard();

    SmartDashboard.putNumber("Gyro Angle:", (drivebase.getHeading() + 90)%360);
    SmartDashboard.putNumber("X-coordinate", drivebase.getPose().getX());
    SmartDashboard.putNumber("Y-coordinate", drivebase.getPose().getY());

    SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());

    // SmartDashboard.putNumber("Flipper Current", intake.getFlipperCurrent());
    // SmartDashboard.putNumber("Pivot Current", launcher.getPivotCurrent());
    // SmartDashboard.putNumber("Roller Current", intake.getRollerCurrent());

    SmartDashboard.putNumber("Flipper Position", intake.getFlipperPosition());
    SmartDashboard.putNumber("Launcher Position", launcher.getPosition());

    SmartDashboard.putString("Intake State", intake.getIntakeState().toString());
    SmartDashboard.putString("Launcher State", launcher.getLaunchState().toString());

    SmartDashboard.putBoolean("Launcher Breakbeam", launcher.getBreakBeam());
    SmartDashboard.putBoolean("Intake Breakbeam", intake.getBreakBeam());

    SmartDashboard.putNumber("Translational Velocity", drivebase.getTranslationalVelocity());
    SmartDashboard.putNumber("Angular Velocity", drivebase.getTurnRate());

    SmartDashboard.putBoolean("Brownout", hasBrownedOut);

    if(launcher.hasBrownedOut()){
      hasBrownedOut = true;
    }
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();

    drivebase.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(m_chooser.getSelected().getName()));

    if (m_autoSelected != null) {
      m_autoSelected.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    intake.updatePose();
  }

  @Override
  public void teleopInit() {
    litty.setRed();
    if (m_autoSelected != null) {
      m_autoSelected.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    intake.updatePose();

    /* DRIVE CONTROLS */
    double ySpeed;
    double xSpeed;
    double rot;

    ySpeed = drivebase.inputDeadband(-driver.getLeftX());
    xSpeed = drivebase.inputDeadband(driver.getLeftY());
    rot = drivebase.inputDeadband(-driver.getRightX());

    if(operator.getAButton()){
      launcher.setSushiOn();
    } 
    if(operator.getYButton()){
      launcher.setSushiOff();
    }

    if (driver.getXButton()) {
      drivebase.lockWheels();
    } else {
      drivebase.drive(xSpeed, ySpeed, rot, true);
    }

    if(driver.getRightTriggerAxis() > 0){
      drivebase.setDriveState(DriveState.SLOW);
    } else if (!CommandScheduler.getInstance().isScheduled(autoAmp)){
      drivebase.setDriveState(DriveState.NORMAL);
    }

    /* INTAKE CONTROLS */

    if (driver.getAButton()) {
      drivebase.zeroHeading();
    }

  //   if(operator.getAButton()){
  //     launcher.setLauncherState(LauncherState.AUTORIGHTSHOT);
  //   }

  // if(operator.getYButton()){
  //   launcher.setLauncherState(LauncherState.AUTOLEFTSHOT);
  // }


    if (operator.getRightBumper() && !useCurrentSpike) {
      handoffCommand.schedule();
    } else if (operator.getRightBumper()) {
      currentSpikeHandoff.schedule();
    }

    if (operator.getBButton()) {
      launcher.eject();
      launcher.setFlickerOn();
    }

    if (operator.getLeftBumper()) {
      intake.setIntakeState(IntakeState.STOP);
      launcher.setLauncherState(LauncherState.HOVER);
      launcher.updatePose();
      launcher.setLauncherOff();
      launcher.setFlickOff();
      litty.setBlue();
    }

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

    if (operator.getPOV() == 0) {
      launcher.setLauncherState(LauncherState.SPEAKER);
    }
    if (operator.getPOV() == 90) {
      launcher.setLauncherState(LauncherState.AMP);
    }
    if (operator.getPOV() == 180) {
      launcher.setLauncherState(LauncherState.TOSS);
    }
    if (operator.getPOV() == 270) {
      launcher.setLauncherState(LauncherState.LONG);
    }

    if (operator.getXButton()) {
      intake.setReverseRollerPower();
      launcher.setFlickerReverse();
      launcher.setReverseLauncherOn();
    }

    // if(operator.getYButton()){
    //   launcher.setLauncherState(LauncherState.AUTOLEFTSHOT);
    // } 
    // if(operator.getAButton()){
    //   launcher.setLauncherState(LauncherState.AUTORIGHTSHOT);
    // }

    if (operator.getRightTriggerAxis() > 0) {
      if (launcher.getLaunchState() == LauncherState.AMP) {
        ampCommand.initialize();
        ampCommand.schedule();
        drivebase.setDriveState(DriveState.SLOW);
      } else {
        shootCommand.initialize();
        shootCommand.schedule();
      }
    } else if (operator.getLeftTriggerAxis() > 0) {
      launcher.setLauncherOff();
      launcher.setFlickOff();
      intake.setRollerOff();
      shootCommand.cancel();
      handoffCommand.cancel();
      // litty.setBlue();

    }

    if(autoAmp.isFinished()){
      driver.setRumble(RumbleType.kBothRumble, 0.5);
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
        drivebase = Drivebase.getInstance();
    launcher = Launcher.getInstance();
    intake = Intake.getInstance();
    climber = Climber.getInstance();
    litty = LED.getInstance();
    // visTables = VisionTablesListener.getInstance();

    driver = new XboxController(0);
    operator = new XboxController(1);
    // drivebase.resetOdometry(new Pose2d(1, 1, new Rotation2d(0)));

    handoffCommand = new BreakBeamHandoff();
    shootCommand = new ShootCommand();
    autoSpeaker = new AutoSpeaker();
    autoAmp = new AutoAmp();
    ampCommand = new AmpCommand();

    currentSpikeHandoff = new HandoffCommand();
    useCurrentSpike = false;

    drivebase.resetOdometry(new Pose2d(5, 5, new Rotation2d(0)));

    // CameraServer.startAutomaticCapture();

 
  }

  @Override
  public void simulationPeriodic() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

if (isReal()) {
    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
} else {
    setUseTiming(false); // Run as fast as possible
    String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
}

// Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

Logger.recordOutput("Pose", drivebase.getPose());
publisher.set(new Pose3d(5, 5, 1, new Rotation3d(0, 0, 0)));

    CommandScheduler.getInstance().run();
    drivebase.periodic();

   

    // launcher.launcherConnections();
    // intake.intakeConnections();
    // climber.climberConnections();
  

    // launcher.printConnections();
    // intake.printConnections();
    // climber.printConnections();
    // drivebase.printConnections();

    // drivebase.printTranslationalVelocities();

    // visTables.putInfoOnDashboard();

    SmartDashboard.putNumber("Gyro Angle:", (drivebase.getHeading() + 90)%360);
    SmartDashboard.putNumber("X-coordinate", drivebase.getPose().getX());
    SmartDashboard.putNumber("Y-coordinate", drivebase.getPose().getY());

    SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());

    // SmartDashboard.putNumber("Flipper Current", intake.getFlipperCurrent());
    // SmartDashboard.putNumber("Pivot Current", launcher.getPivotCurrent());
    // SmartDashboard.putNumber("Roller Current", intake.getRollerCurrent());

    SmartDashboard.putNumber("Flipper Position", intake.getFlipperPosition());
    SmartDashboard.putNumber("Launcher Position", launcher.getPosition());

    SmartDashboard.putString("Intake State", intake.getIntakeState().toString());
    SmartDashboard.putString("Launcher State", launcher.getLaunchState().toString());

    SmartDashboard.putBoolean("Launcher Breakbeam", launcher.getBreakBeam());
    SmartDashboard.putBoolean("Intake Breakbeam", intake.getBreakBeam());

    SmartDashboard.putNumber("Translational Velocity", drivebase.getTranslationalVelocity());
    SmartDashboard.putNumber("Angular Velocity", drivebase.getTurnRate());

    SmartDashboard.putBoolean("Brownout", hasBrownedOut);

    if(launcher.hasBrownedOut()){
      hasBrownedOut = true;
    }  

      double ySpeed = drivebase.inputDeadband(-driver.getLeftX());
    double xSpeed = drivebase.inputDeadband(driver.getLeftY());
    double rot = drivebase.inputDeadband(-driver.getRightX());

    if (driver.getXButton()) {
      drivebase.lockWheels();
    } else {
      drivebase.drive(xSpeed, ySpeed, rot, true);
    }

  
  }

    
}
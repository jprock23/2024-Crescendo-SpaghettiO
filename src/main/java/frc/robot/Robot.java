package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AltAmpCommand;
import frc.robot.commands.AltRevLauncher;
import frc.robot.commands.AmpCommand;
import frc.robot.commands.AutoHandoff;
import frc.robot.commands.AutoLeftShot;
import frc.robot.commands.AutoMidShot;
import frc.robot.commands.AutoPreload;
import frc.robot.commands.AutoRightShot;
import frc.robot.commands.AutoSpeaker;
import frc.robot.commands.BreakBeamHandoff;
import frc.robot.commands.Celebrate;
import frc.robot.commands.HandoffCommand;
import frc.robot.commands.RevLauncher;
import frc.robot.commands.RotationCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.IO.LED;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.swerve.Drivebase;
import frc.robot.subsystems.swerve.Drivebase.DriveState;

import org.littletonrobotics.junction.LoggedRobot;

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
  private HandoffCommand currentSpikeHandoff;
  private AmpCommand ampCommand;
  private AltAmpCommand altAmpCommand;

  private boolean useCurrentSpike;
  private boolean hasBrownedOut;

  private RotationCommand turn;

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
    ampCommand = new AmpCommand();
    altAmpCommand = new AltAmpCommand();

    turn = new RotationCommand(-25);

    currentSpikeHandoff = new HandoffCommand();

    NamedCommands.registerCommand("AutoSpeaker", autoSpeaker);
    NamedCommands.registerCommand("Handoff", new AutoHandoff());
    NamedCommands.registerCommand("AutoLeftShot", new AutoLeftShot());
    NamedCommands.registerCommand("AutoRightShot", new AutoRightShot());
    NamedCommands.registerCommand("AutoMidShot", new AutoMidShot());
    NamedCommands.registerCommand("Celebrate", new Celebrate());
    NamedCommands.registerCommand("RotateN155", new RotationCommand(-155));
    NamedCommands.registerCommand("RotateN25", new RotationCommand(-25));
    NamedCommands.registerCommand("Rotate220", new RotationCommand(220));
    NamedCommands.registerCommand("Rotate180", new RotationCommand(180));
    NamedCommands.registerCommand("RevLauncher", new RevLauncher());
    NamedCommands.registerCommand("AutoPreload", new AutoPreload());
    NamedCommands.registerCommand("AltRevLauncher", new AltRevLauncher());

    m_chooser = AutoBuilder.buildAutoChooser();
    // m_chooser.addOption("Test1", new PathPlannerAuto("Test1"));

    SmartDashboard.putData("Auto choices", m_chooser);

    useCurrentSpike = false;

    // CameraServer.startAutomaticCapture();

  }

  @Override

  public void robotPeriodic() {

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

    SmartDashboard.putNumber("Radians", drivebase.getPose().getRotation().getRadians());

    SmartDashboard.putNumber("Gyro Angle:", (drivebase.getHeading() + 90) % 360);
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

    if (launcher.hasBrownedOut()) {
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
    // launcher.updatePose();

    /* DRIVE CONTROLS */

    double ySpeed = drivebase.inputDeadband(-driver.getLeftX());
    double xSpeed = drivebase.inputDeadband(driver.getLeftY());
    double rot = drivebase.inputDeadband(-driver.getRightX());

    if (driver.getAButton()) {
      drivebase.currHeading = -1;
      drivebase.rotateTo(xSpeed, ySpeed, 180);
    } else if (driver.getBButton()) {
      drivebase.currHeading = -1;
      drivebase.rotateTo(xSpeed, ySpeed, 270);
    } else if (driver.getYButton()) {
      drivebase.currHeading = -1;
      drivebase.rotateTo(xSpeed, ySpeed, 0);
    } else if (driver.getXButton()) {
      drivebase.currHeading = -1;
      drivebase.rotateTo(xSpeed, ySpeed, 90);
    } else if (driver.getLeftTriggerAxis() > 0) {
      drivebase.holdHeading(xSpeed, ySpeed);
    } else {
      drivebase.currHeading = -1;
      drivebase.drive(xSpeed, ySpeed, rot);
    }

    if (driver.getPOV() == 0) {
      drivebase.zeroHeading();
    }

    if (operator.getAButton()) {
      intake.setIntakeState(IntakeState.GROUND);
    }

    if (driver.getRightTriggerAxis() > 0) {
      drivebase.setDriveState(DriveState.SLOW);
    } else if (!CommandScheduler.getInstance().isScheduled(ampCommand)) {
      drivebase.setDriveState(DriveState.NORMAL);
    }

    /* INTAKE CONTROLS */

    // if(operator.getAButton()){
    // launcher.setLauncherState(LauncherState.AUTORIGHTSHOT);
    // }

    // if(operator.getYButton()){
    // launcher.setLauncherState(LauncherState.AUTOMIDSHOT);
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

    if(operator.getYButton()){
      launcher.setLauncherState(LauncherState.ALTAMP);
    }

    if (operator.getXButton()) {
      intake.setReverseRollerPower();
      launcher.setFlickerReverse();
      launcher.setReverseLauncherOn();
    }

    if (operator.getRightTriggerAxis() > 0) {
      if (launcher.getLaunchState() == LauncherState.AMP) {
        ampCommand.initialize();
        ampCommand.schedule();
        drivebase.setDriveState(DriveState.SLOW);
      } else if(launcher.getLaunchState() == LauncherState.ALTAMP){
        altAmpCommand.initialize();
        altAmpCommand.schedule();
        drivebase.setDriveState(DriveState.SLOW);
      }
        else {
        shootCommand.initialize();
        shootCommand.schedule();
      }
    } else if (operator.getLeftTriggerAxis() > 0) {
      launcher.setLauncherOff();
      launcher.setFlickOff();
      intake.setRollerOff();
      shootCommand.cancel();
      ampCommand.cancel();
      handoffCommand.cancel();
      launcher.setSushiOff();
      // litty.setBlue();

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
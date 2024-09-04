package frc.robot;

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
import frc.robot.commands.FoldOutCommand;
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
import frc.robot.subsystems.launcher.Launcher.LeBronTeam;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.swerve.Drivebase;
import frc.robot.subsystems.swerve.Drivebase.DriveState;
//import frc.robot.subsystems.vision.VisionTablesListener;
import frc.robot.subsystems.vision.DualCamera;

import org.littletonrobotics.junction.LoggedRobot;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class Robot extends LoggedRobot {
  private DualCamera dualCamera;
  private Drivebase drivebase;
  private Climber climber;
  private Intake intake;
  private Launcher launcher;
  private LED litty;
  //private VisionTablesListener visTables;

  private static XboxController driver;
  private static XboxController operator;

  private Command m_autoSelected;

  private FoldOutCommand foldOutCommand;
  private BreakBeamHandoff handoffCommand;
  private ShootCommand shootCommand;
  private AutoSpeaker autoSpeaker;
  private HandoffCommand currentSpikeHandoff;
  private AmpCommand ampCommand;
  private AltAmpCommand altAmpCommand;


  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    dualCamera = DualCamera.getInstance();
    drivebase = Drivebase.getInstance();
    launcher = Launcher.getInstance();
    intake = Intake.getInstance();
    climber = Climber.getInstance();
    litty = LED.getInstance();
    //visTables = VisionTablesListener.getInstance();

    driver = new XboxController(0);
    operator = new XboxController(1);

    handoffCommand = new BreakBeamHandoff();
    shootCommand = new ShootCommand();
    autoSpeaker = new AutoSpeaker();
    ampCommand = new AmpCommand();
    altAmpCommand = new AltAmpCommand();
    foldOutCommand = new FoldOutCommand();
    // currentSpikeHandoff = new HandoffCommand();
    Pose2d defaultPose = new Pose2d(0, 0, new Rotation2d(0));

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

    m_chooser.addOption("P1 4L", new PathPlannerAuto("P1 4L"));

    m_chooser.addOption("P2 3L", new PathPlannerAuto("P2 3L"));
    m_chooser.addOption("P2 3ML", new PathPlannerAuto("P2 3ML"));
    m_chooser.addOption("P2 3MR", new PathPlannerAuto("P2 3MR"));
    m_chooser.addOption("P2 3R", new PathPlannerAuto("P2 3R"));
    m_chooser.addOption("P2 4L Mid", new PathPlannerAuto("P2 4L Mid"));
    m_chooser.addOption("P2 4L", new PathPlannerAuto("P2 4L"));
    m_chooser.addOption("P2 4R Mid", new PathPlannerAuto("P2 4R Mid"));
    m_chooser.addOption("P2 4R", new PathPlannerAuto("P2 4R"));
    m_chooser.addOption("Red P2 4R", new PathPlannerAuto("Red P2 4R"));
    m_chooser.addOption("Red P2 4R Mid", new PathPlannerAuto("Red P2 4R Mid"));
    m_chooser.addOption("Red P2 3MR Mid", new PathPlannerAuto("Red P2 3MR Mid"));
    m_chooser.addOption("Semi2", new PathPlannerAuto("Semi2"));
    m_chooser.addOption("VtestAuto", new PathPlannerAuto("VtestAuto"));
    m_chooser.addOption("Help", new PathPlannerAuto("Help"));
    m_chooser.addOption("Mid:ine", new PathPlannerAuto("MidLine"));
    
    m_chooser.addOption("FreakyMid", new PathPlannerAuto("FreakyMid"));

    m_chooser.addOption("P3 4R", new PathPlannerAuto("P3 4R"));

    m_chooser.addOption("P3UnderStage", new PathPlannerAuto("P3UnderStage"));
    SmartDashboard.putData("Auto choices", m_chooser);

  }

  @Override

  public void robotPeriodic() {
        Pose2d cameraPositionTele = dualCamera.calculateRobotPosition();
        PhotonPipelineResult result = dualCamera.getBackCameraResult(); 


       Pose2d posTele = drivebase.updateOdometry(cameraPositionTele, result);

        SmartDashboard.putNumber("Odometry X", posTele.getX());
        SmartDashboard.putNumber("Odometry Y", posTele.getY());





     if (dualCamera.isBackConnected()) {
            PhotonPipelineResult backResult = dualCamera.getBack();
            
            if (backResult.hasTargets()) {
                PhotonTrackedTarget target = backResult.getBestTarget();
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                double distance  = bestCameraToTarget.getTranslation().getNorm();
                SmartDashboard.putString("Back Camera Target", "Yes Targets");
                SmartDashboard.putNumber("Back to Target", distance);
                SmartDashboard.putNumber("Back Camera Target Yaw", target.getYaw());
                SmartDashboard.putNumber("Back Camera Target Pitch", target.getPitch());
                SmartDashboard.putNumber("Back Camera Target Area", target.getArea());
                SmartDashboard.putNumber("ID", target.getFiducialId());

            } else if(backResult.hasTargets() == false) {
                SmartDashboard.putString("Back Camera Target", "No Targets");
            }
        } 
        if (dualCamera.isFrontConnected()) {
            PhotonPipelineResult frontResult = dualCamera.getFront();
            
            if (frontResult.hasTargets()) {
                PhotonTrackedTarget target = frontResult.getBestTarget();
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                double distance  = bestCameraToTarget.getTranslation().getNorm();
                SmartDashboard.putString("Front Camera Target", "Yes Targets");
                SmartDashboard.putNumber("Front to Target", distance);
                SmartDashboard.putNumber("Front Camera Target Yaw", target.getYaw());
                SmartDashboard.putNumber("Front Camera Target Pitch", target.getPitch());
                SmartDashboard.putNumber("Front Camera Target Area", target.getArea());
                SmartDashboard.putNumber("ID", target.getFiducialId());

            } else if(frontResult.hasTargets() == false) {
                SmartDashboard.putString("Front Camera Target", "No Targets");
            }
        } 



      

      Pose2d cameraPosition = dualCamera.calculateRobotPosition();
      SmartDashboard.putNumber("Camera X Position", cameraPosition.getX());
      SmartDashboard.putNumber("Camera Y Position", cameraPosition.getY());
      SmartDashboard.putNumber("Camera Heading", cameraPosition.getRotation().getDegrees());


      


    CommandScheduler.getInstance().run();
    drivebase.periodic();

    //visTables.printDetects();
    SmartDashboard.putNumber("Gyro Angle:", (drivebase.getHeading() + 90) % 360);
    SmartDashboard.putNumber("X-coordinate", 16.579342 - drivebase.getPose().getX());
    SmartDashboard.putNumber("Y-coordinate", drivebase.getPose().getY());

    //SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());

    SmartDashboard.putNumber("Flipper Position", intake.getFlipperPosition());
    SmartDashboard.putNumber("Launcher Position", launcher.getPosition());

    SmartDashboard.putString("Intake State", intake.getIntakeState().toString());
    SmartDashboard.putString("Launcher State", launcher.getLaunchState().toString());

    SmartDashboard.putBoolean("Launcher Breakbeam", launcher.getBreakBeam());
    SmartDashboard.putBoolean("Intake Breakbeam", intake.getBreakBeam());

    SmartDashboard.putBoolean("Brownout", launcher.hasBrownedOut());

    //SmartDashboard.putNumber("Test Position", launcher.getTestPosition());
    SmartDashboard.putNumber("Speaker Position", launcher.getSpeakerPosition());



    SmartDashboard.putNumber("LeBron Position", launcher.getLeBronPostion());
    //SmartDashboard.putNumber("Lebron Current?", launcher.getLebronCurrent());
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
    Pose2d cameraPosition = dualCamera.calculateRobotPosition();
    
   
      PhotonPipelineResult result = dualCamera.getFrontCameraResult(); 
      Pose2d pose = drivebase.updateOdometry(cameraPosition, result);
     

    SmartDashboard.putNumber("Auto X", drivebase.getPose().getX());
    SmartDashboard.putNumber("Auto Y", drivebase.getPose().getY());
    SmartDashboard.putNumber("Odometry X", pose.getX());
    SmartDashboard.putNumber("Odometry Y", pose.getY());



    
  }

  @Override
  public void teleopInit() {

    litty.setBlue();
    if (m_autoSelected != null) {
      m_autoSelected.cancel();
    }

    Translation2d testxy = new Translation2d(16.57 - 14.7, 5.54);
    Rotation2d testRot = new Rotation2d(0);
    Pose2d test = new Pose2d(testxy, testRot);
    drivebase.resetOdometry(test);
  }

  @Override
  public void teleopPeriodic() {
    intake.updatePose();
   //launcher.updatePose();

    /* DRIVE CONTROLS */

    if (!ampCommand.isScheduled()) {
      launcher.moveLeBron();
    }

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
      // drivebase.holdHeading(xSpeed, ySpeed);
    } else {
      drivebase.currHeading = -1;
      drivebase.drive(xSpeed, ySpeed, rot);
    }
    // drivebase.lockWheels();

    

    if (driver.getPOV() == 0) {
      drivebase.zeroHeading();
    }
    if(driver.getPOV() == 180){
      litty.setRed();
    }
    // if (operator.getYButton()) {
    //   intake.setIntakeState(IntakeState.GROUND);
    // }

    if (driver.getRightTriggerAxis() > 0) {
      drivebase.setDriveState(DriveState.SLOW);
    } else if (!CommandScheduler.getInstance().isScheduled(ampCommand)) {
      drivebase.setDriveState(DriveState.NORMAL);
    }

    /* INTAKE CONTROLS */

    if (operator.getRightBumper()) {
      handoffCommand.schedule();
    }

    if (operator.getBButton()) {
      launcher.setLauncherState(LauncherState.TEST);
      launcher.eject();
      launcher.setFlickerPartial();
      litty.setRed();
    }

    if (operator.getLeftBumper()) {
      intake.setIntakeState(IntakeState.STOP); 
      launcher.setLauncherState(LauncherState.HOVER);
      launcher.setLeBronTeam(LeBronTeam.CAVS);
      launcher.updatePose();
      launcher.moveLeBron();
      launcher.setLauncherOff();
      launcher.setFlickOff();
      //litty.setBlue();
    }

    if (operator.getRightStickButtonPressed()) {
      launcher.increasePosition();
    }
    if(operator.getLeftStickButtonPressed()){
      launcher.decreasePosition();
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

    // if(driver.getRightStickButtonPressed()){
    //   launcher.setLauncherState(LauncherState.TEST);
    // }

    if (operator.getLeftStickButtonPressed()) {
      // launcher.increasePosition();
      launcher.increaseIncrement();
    } else if (operator.getRightStickButtonPressed()) {
      launcher.decreaseInrement();
      // launcher.decreasePosition();
    }

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

      // launcher.interpolateAngle();
      // launcher.lookUpPosition();

     if (operator.getAButton()) {
      foldOutCommand.schedule();
      intake.setIntakeState(IntakeState.GROUND);
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
      } else if (launcher.getLaunchState() == LauncherState.ALTAMP) {
        altAmpCommand.initialize();
        altAmpCommand.schedule();
        drivebase.setDriveState(DriveState.SLOW);
      } else {
        shootCommand.initialize();
        shootCommand.schedule();
      }
    } else if (operator.getLeftTriggerAxis() > 0) {
      foldOutCommand.cancel();
      launcher.setLauncherOff();
      launcher.setFlickOff();
      intake.setRollerOff();
      shootCommand.cancel();
      ampCommand.cancel();
      handoffCommand.cancel();
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
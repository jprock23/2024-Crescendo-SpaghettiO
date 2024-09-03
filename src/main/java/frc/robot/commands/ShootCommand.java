package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IO.LED;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;

public class ShootCommand extends Command {

  private Launcher launcher;
  private Intake intake;
  private LED litty;

  private boolean ended;

  private double startTime;
  private double windup = .25;
  private double duration = windup +.5;

  public ShootCommand() {
    launcher = Launcher.getInstance();
    intake = Intake.getInstance();
    litty = LED.getInstance();
  }

  @Override
  public void initialize() {
    ended = false;

    startTime = Timer.getFPGATimestamp();
    launcher.setLauncherOn();
    
    intake.setIntakeState(IntakeState.STOP);
    // if(launcher.getLaunchState() == LauncherState.INTERLOPE || launcher.getLaunchState() == LauncherState.TEST){
    //   launcher.lookUpPosition();
    // }
  }

  @Override
  public void execute() {
  launcher.updatePose();

      double elapsedTime = Timer.getFPGATimestamp() - startTime;

      if (elapsedTime > windup) {
        launcher.setFlickerOn();
      }

      if (elapsedTime > duration) {
        ended = true;
      }
  }

  @Override
  public void end(boolean interrupted) {
    launcher.setFlickOff();
    launcher.setLauncherOff();
    litty.setRed();
  }

  @Override
  public boolean isFinished() {
    return ended;
  }
}

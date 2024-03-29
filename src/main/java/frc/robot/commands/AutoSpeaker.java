package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;

public class AutoSpeaker extends Command {

  private Launcher launcher;

  private boolean ended;

  private double startTime;
  private double elapsedTime;
  private double windup = 0.2;
  private double duration = windup + .1;

  public AutoSpeaker() {
    launcher = Launcher.getInstance();
  }

  @Override
  public void initialize() {
    ended = false;

    launcher.setLauncherState(LauncherState.SPEAKER);
    startTime = -1;
    elapsedTime = 0;
    launcher.setLauncherOn();
  }

  @Override
  public void execute() {
    launcher.updatePose();

    if(launcher.hasReachedPose(1.2)){
    launcher.setLauncherOn();
    launcher.setFlickerOn();

      if (startTime == -1) {
        startTime = Timer.getFPGATimestamp();
      }
      elapsedTime = Timer.getFPGATimestamp() - startTime;

      if (elapsedTime > duration) {
        ended = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    launcher.setFlickOff();
    launcher.setLauncherOff();
    launcher.setLauncherState(LauncherState.HOVER);
    launcher.updatePose();  
  }

  @Override
  public boolean isFinished() {
    return ended;
  }
}

package frc.robot.commands;

import java.util.spi.TimeZoneNameProvider;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;

public class AutoSpeaker extends Command {

  private Launcher launcher;

  private boolean ended;

  private double startTime;
  private double elapsedTime;
  private double windup = 1.0;
  private double duration = windup + .5;

  private boolean beganShooting;

  public AutoSpeaker() {
    launcher = Launcher.getInstance();
  }

  @Override
  public void initialize() {
    ended = false;
    beganShooting = false;

    launcher.setLauncherState(LauncherState.SPEAKER);
    startTime = 0;
    elapsedTime = 0;
    launcher.setLauncherOn();
  }

  @Override
  public void execute() {
    launcher.updatePose();

    if(launcher.hasReachedPose(20.0)){
      if (!beganShooting) {
        startTime = Timer.getFPGATimestamp();
        beganShooting = true;
      }
      elapsedTime = Timer.getFPGATimestamp() - startTime;
      System.out.println("anshanshanshanshansh");

      if (elapsedTime > windup) {
        launcher.setFlickerOn();
      }

      if (elapsedTime > duration) {
        ended = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    launcher.setFlickOff();
    launcher.setLauncherOff();
  }

  @Override
  public boolean isFinished() {
    return ended;
  }
}

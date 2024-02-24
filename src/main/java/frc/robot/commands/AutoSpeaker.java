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
  // these are somewhat random numbers so change however you like
  private double windup = .25;
  private double duration = windup + .5;

  public AutoSpeaker() {
    launcher = Launcher.getInstance();
  }

  @Override
  public void initialize() {
    ended = false;

    launcher.setLauncherState(LauncherState.SPEAKER);
    startTime = Timer.getFPGATimestamp();
    launcher.setLauncherOn();
  }

  @Override
  public void execute() {

    if(launcher.hasReachedPose(20.0)){
      double elapsedTime = Timer.getFPGATimestamp() - startTime;
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
    // launcher.setFlickOff();
    launcher.setLauncherOff();
  }

  @Override
  public boolean isFinished() {
    return ended;
  }
}

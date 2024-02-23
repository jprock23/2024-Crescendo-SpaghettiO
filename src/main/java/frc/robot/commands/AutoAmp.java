package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;

public class AutoAmp extends Command {

  private Launcher launcher;

  private boolean ended;

  private double startTime;
  // these are somewhat random numbers so change however you like
  private double windup = .25;
  private double duration = .5;

  public AutoAmp() {
    launcher = Launcher.getInstance();
  }

  @Override
  public void initialize() {
    ended = false;
    
    launcher.setPivotState(LauncherState.AMP);
    startTime = Timer.getFPGATimestamp();
    launcher.setLauncherOn();
  }

  @Override
  public void execute() {
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
  }

  @Override
  public boolean isFinished() {
    return ended;
  }
}

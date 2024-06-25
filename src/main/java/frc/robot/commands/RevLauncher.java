package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;

public class RevLauncher extends Command {

  private Launcher launcher;

  private boolean ended;

  public RevLauncher() {
    launcher = Launcher.getInstance();
  }

  @Override
  public void initialize() {
    ended = false;

    launcher.setLauncherState(LauncherState.SPEAKER);
    launcher.updatePose();

    // startTime = -1;
    // elapsedTime = 0;
    launcher.setLauncherOn();
  }

  @Override
  public void execute() {

      // if (startTime == -1) {
      //   startTime = Timer.getFPGATimestamp();
      // }
      // elapsedTime = Timer.getFPGATimestamp() - startTime;

      // if (elapsedTime > windup) {
      //   ended = true;
      // }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return ended;
  }
}

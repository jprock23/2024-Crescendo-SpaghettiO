package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;

public class AltRevLauncher extends Command {

  private Launcher launcher;

  private boolean ended;
;

  public AltRevLauncher() {
    launcher = Launcher.getInstance();
  }

  @Override
  public void initialize() {
    ended = false;

    launcher.setLauncherState(LauncherState.AUTOLEFTSHOT);
        launcher.setLauncherOn();

    // launcher.setReverseLauncherOn();
    // launcher.setFlickerReverse();
    launcher.updatePose();
  }

  @Override
  public void execute() {

    // if(startTime == -1){
    //   startTime = Timer.getFPGATimestamp();
    // }

    // if(Timer.getFPGATimestamp() - startTime > .1){
    //   launcher.setLauncherOff();
    //   launcher.setFlickOff();
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

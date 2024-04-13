package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.Launcher;

public class AltAmpCommand extends Command {

  private Launcher launcher;

  private double starttime;

  private boolean ended;

  private double windup = .75;

  private double duration = windup + .75;

  public AltAmpCommand() {
  }

  @Override
  public void initialize() {
    launcher = Launcher.getInstance();

    starttime = -1;
    ended = false;

      launcher.updatePose();
      launcher.setLauncherOn();
  }

  @Override
  public void execute() {

      if(starttime == -1){
        starttime = Timer.getFPGATimestamp();
      }

      double elapsedtime = Timer.getFPGATimestamp() - starttime; 

      if (elapsedtime > windup) {
        launcher.setFlickerOn();
      }

      if(elapsedtime > duration){
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

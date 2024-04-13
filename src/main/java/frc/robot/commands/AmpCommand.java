package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LeBronTeam;

public class AmpCommand extends Command {

  private Launcher launcher;

  private double starttime;

  private boolean ended;

  private double windup = .75;

  private double duration = windup + 1;

  public AmpCommand() {
  }

  @Override
  public void initialize() {
    launcher = Launcher.getInstance();

    starttime = -1;
    ended = false;
    launcher.updatePose();
    launcher.eject();
  }

  @Override
  public void execute() {


      if(starttime == -1){
        starttime = Timer.getFPGATimestamp();
      }

      if(launcher.hasReachedPose(3.5)){
        launcher.setLeBronTeam(LeBronTeam.LAKERS);
        launcher.moveLeBron();
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
    launcher.setLeBronTeam(LeBronTeam.CAVS);
    launcher.moveLeBron();
  }

  @Override
  public boolean isFinished() {
    return ended;
  }
}

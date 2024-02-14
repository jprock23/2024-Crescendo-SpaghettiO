package frc.robot.auton.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class AutoIntake extends Command {

  private static Intake intake;
  private boolean ended = false;

  public AutoIntake() {
    intake = Intake.getInstance();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.setIntakeState(IntakePosition.GROUND);

    if(intake.hasReachedPose(.7)){
      ended = true;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return ended;
  }
}

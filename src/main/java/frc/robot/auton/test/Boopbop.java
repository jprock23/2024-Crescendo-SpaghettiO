package frc.robot.auton.test;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Boopbop extends SequentialCommandGroup {

  public Boopbop() {

    addCommands(
      new PathPlannerAuto("Bop"), new AutoIntake() 
    );
  }
}

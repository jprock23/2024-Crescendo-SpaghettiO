// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drivebase;

public class RotationCommand extends Command {

  private double goal;

  private Drivebase drivebase;

  private boolean ended = false;

  public RotationCommand(double goal) {
    this.goal = goal;

    ended = false;

    drivebase = Drivebase.getInstance();
  }


  public void initialize() {
  }

  @Override
  public void execute() {
    drivebase.rotateTo(goal);

    if(Math.abs(drivebase.getHeading() - goal) < 1.5){
      ended = true;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ended;
  }
}

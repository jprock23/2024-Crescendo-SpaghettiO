// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;

public class FoldOutCommand extends Command {
  /** Creates a new FoldOutCommand. */
  private boolean ended;
  private Launcher launcher;
  public FoldOutCommand() {
    launcher = Launcher.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    ended = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcher.setLauncherState(LauncherState.FIXER);
    launcher.updatePose();
    ended = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ended;
  }
}

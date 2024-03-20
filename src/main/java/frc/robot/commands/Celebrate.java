// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IO.LED;

public class Celebrate extends Command {

  private LED litty;

  private boolean ended;

  public Celebrate() {
    litty = LED.getInstance();
  }

  @Override
  public void initialize() {
    litty.setDisco();
  }

  @Override
  public void execute() {
    ended = true;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return ended;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.intake.Intake;

import frc.robot.subsystems.launcher.*;

public class Handoff extends Command {


  private Launcher launcher;
  private Intake intake;

  private double threshold = 40.0;

  private double startTime = 0.0;
  private double timeElapsed = 0.0;
  private double duration = 0.5;

  public Handoff() {
  }
  @Override
  public void initialize() {
    launcher = new Launcher();
    intake = new Intake();
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    launcher.periodic();

    intake.setRollerPower();

    if(intake.getRollerCurrent() > threshold){
        startTime = Timer.getFPGATimestamp();
        timeElapsed = Timer.getFPGATimestamp() - startTime;

        if(timeElapsed > duration){
            intake.setRollerOff();
        }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

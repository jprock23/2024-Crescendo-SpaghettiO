// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.Launcher;

public class AutoReverseLauncher extends Command {

  private Launcher launcher;

  private double starttime;

  private double duration = 1.0;

  private boolean ended;

  public AutoReverseLauncher() {

    launcher = Launcher.getInstance();
  }

  @Override
  public void initialize() {
    starttime = -1;

    ended = false;

    launcher.setFlickerReverse();
    launcher.setReverseLauncherOn();
  }

  @Override
  public void execute() {

    double elapsedTime = Timer.getFPGATimestamp() - starttime;

    if(elapsedTime > duration){
      ended = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    launcher.setLauncherOff();
    launcher.setFlickOff();
  }

  @Override
  public boolean isFinished() {
    return ended;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// DO NOT USE THIS IT IS OUTDATED
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.launcher.*;
import frc.robot.subsystems.launcher.Launcher.LauncherState;

public class HandoffCommand extends Command {

  private Launcher launcher;
  private Intake intake;

  private double threshold = 18.0;

  private double startTime = 0.0;
  private double timeElapsed = 0.0;
  private double duration = 1.15;

  private double startTime2 = 0.0;
  private double timeElapsed2 = 0.0;

  private boolean beganIntaking;
  private boolean hasRing;
  private boolean beganHandoff;
  private boolean ended;

  public HandoffCommand() {
    
    launcher = Launcher.getInstance();
    intake = Intake.getInstance();
  }

  @Override
  public void initialize() {

    startTime = 0;
    startTime2 = 0;

    intake.setIntakeState(IntakeState.GROUND);
    launcher.setLauncherState(LauncherState.HANDOFF);
    launcher.updatePose();

    beganIntaking = false;
    hasRing = false;
    beganHandoff = false;
    ended = false;
  }

  @Override
  public void execute() {
    intake.setRollerPower();

    if (intake.getRollerCurrent() > threshold) {
      if (!beganIntaking) {
        startTime = Timer.getFPGATimestamp();
        beganIntaking = true;
      }
      timeElapsed = Timer.getFPGATimestamp() - startTime;

      if (timeElapsed > duration) {
        
        intake.setRollerOff();
        hasRing = true;
      } 
    }

    if (hasRing) {
      intake.setIntakeState(IntakeState.HANDOFF);
      launcher.setReverseLauncherOn();
      launcher.setFlickerReverse();
    }
    SmartDashboard.putNumber("StartTime2", startTime2);
    SmartDashboard.putNumber("timeElapsed2", timeElapsed2);
    if (hasRing && intake.hasReachedPose(2.3)) {
      intake.setRollerPower();

      if (!beganHandoff) {
        startTime2 = Timer.getFPGATimestamp();
        beganHandoff = true;
      }

      timeElapsed2 = Timer.getFPGATimestamp() - startTime2;

      if (timeElapsed2 > .5) {
        ended = true;
      }

    }
  }

  @Override
  public void end(boolean interrupted) {
    launcher.setFlickOff();
    launcher.setLauncherOff();
    intake.setRollerOff();
  }

  @Override
  public boolean isFinished() {
    return ended;
  }
}

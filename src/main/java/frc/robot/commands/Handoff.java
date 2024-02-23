// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.launcher.*;
import frc.robot.subsystems.launcher.Launcher.LauncherState;

public class Handoff extends Command {

  private Launcher launcher;
  private Intake intake;

  private double threshold = 25.0;

  private double startTime = 0.0;
  private double timeElapsed = 0.0;
  private double duration = 1.0;

  private double startTime2 = 0.0;
  private double timeElapsed2 = 0.0;

  private boolean beganIntaking;
  private boolean hasRing;
  private boolean beganHandoff;
  private boolean ended;

  public Handoff() {
  }

  @Override
  public void initialize() {
    launcher = Launcher.getInstance();
    intake = Intake.getInstance();
    startTime = Timer.getFPGATimestamp();
    startTime2 = 0;

    intake.setIntakeState(IntakePosition.GROUND);
    launcher.setPivotState(LauncherState.HANDOFF);

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
      intake.setIntakeState(IntakePosition.HANDOFF);
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

      if (timeElapsed2 > .25) {
        System.out.println("ANSHANSHANSHANSHANSHANSH");
        launcher.setFlickOff();
        launcher.setLauncherOff();
        intake.setRollerOff();
        ended = true;
      }
      // if(ended){
      // end(true);
      // }
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

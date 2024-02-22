// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.launcher.*;
import frc.robot.subsystems.launcher.Launcher.LauncherState;

public class Handoff extends Command {

  private Launcher launcher;
  private Intake intake;

  private double threshold = 40.0;

  private double startTime = 0.0;
  private double timeElapsed = 0.0;
  private double duration = 0.25;

  private boolean beganIntaking;
  private boolean hasRing;
  private boolean beganHandoff;
  private boolean ended;

  public Handoff() {
  }

  @Override
  public void initialize() {
    launcher = new Launcher();
    intake = new Intake();
    startTime = Timer.getFPGATimestamp();

    beganIntaking = false;
    hasRing = false;
    beganHandoff = false;
    ended = false;
  }

  @Override
  public void execute() {
    launcher.periodic();
    intake.periodic();

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
      launcher.setPivotState(LauncherState.HANDOFF);
    }

    if (hasRing && intake.hasReachedPose(2.0) && launcher.hasReachedPose(2.0)) {
      launcher.setFlickerReverse();
      intake.setReverseRollerPower();

      if (!beganHandoff) {
        startTime = Timer.getFPGATimestamp();
        beganHandoff = true;
      }

      timeElapsed = Timer.getFPGATimestamp() - startTime;

      if (timeElapsed > duration) {
        launcher.setFlickOff();
        intake.setRollerOff();
        ended = true;
      }

    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return ended;
  }
}

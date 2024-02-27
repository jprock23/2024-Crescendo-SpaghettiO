// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.security.auth.login.AppConfigurationEntry.LoginModuleControlFlag;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;

public class BreakBeamHandoff extends Command {

  private Launcher launcher;
  private Intake intake;

  private double threshold = 18.0;

  private double startTime = 0.0;
  private double timeElapsed = 0.0;
  private double duration = 1.15;

  private double startTime2 = 0.0;
  private double timeElapsed2 = 0.0;

  private double startTime3 = 0.0;
  private double timeElapsed3 = 0.0;

  private double count;

  private boolean beganIntaking;
  private boolean hasRing;
  private boolean beganHandoff;
  private boolean launcherHasRing;
  private boolean ended;

  public BreakBeamHandoff() {
    launcher = Launcher.getInstance();
    intake = Intake.getInstance();
  }

  @Override
  public void initialize() {

    startTime = 0;
    startTime2 = 0;
    startTime3 = 0;

    count = 0;

    intake.setIntakeState(IntakeState.GROUND);
    launcher.setLauncherState(LauncherState.HANDOFF);
    launcher.updatePose();

    beganIntaking = false;
    hasRing = false;
    beganHandoff = false;
    launcherHasRing = false;
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

    if (hasRing && intake.hasReachedPose(2.3)) {
      intake.setRollerPower();

      if (!beganHandoff) {
        startTime2 = Timer.getFPGATimestamp();
        beganHandoff = true;
      }

      timeElapsed2 = Timer.getFPGATimestamp() - startTime2;

      if (timeElapsed2 > .25) {
        intake.setRollerOff();
      }

      if(!launcherHasRing && launcher.getBreakBeam()){
        launcherHasRing = true;
      }

      if(launcherHasRing && !launcher.getBreakBeam()){
        ended = true;
      }
  
    }
  }

  @Override
  public void end(boolean interrupted) {
    launcher.setFlickOff();
    intake.setRollerOff();
    launcher.setLauncherOff();
  }

  @Override
  public boolean isFinished() {
    return ended;
  }
}

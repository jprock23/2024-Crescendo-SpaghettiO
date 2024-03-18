// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IO.LED;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;

public class BreakBeamHandoff extends Command {

  private Launcher launcher;
  private Intake intake;
  private LED litty;

  private boolean launcherHasRing;
  private boolean ended;

  private double startTime;
  private double duration = 0.5;

  private boolean intakeHasRing;

  public BreakBeamHandoff() {
    launcher = Launcher.getInstance();
    intake = Intake.getInstance();
    litty = LED.getInstance();

        launcherHasRing = false;
    ended = false;

    intakeHasRing = false;
  }

  @Override
  public void initialize() {

    intake.setIntakeState(IntakeState.GROUND);
    launcher.setLauncherState(LauncherState.HANDOFF);
    launcher.updatePose();

    startTime = -1;


    launcherHasRing = false;
    ended = false;

    intakeHasRing = false;

    // litty.setRed();
  }

  @Override
  public void execute() {
    intake.setRollerPower();

    if (intake.getBreakBeam() && !intakeHasRing) {
      intake.setIntakeState(IntakeState.HANDOFF);
      intake.setRollerOff();
      launcher.setReverseLauncherOn();
      launcher.setFlickerReverse();
    }

    if (intake.getIntakeState() == IntakeState.HANDOFF && intake.hasReachedPose(3.0)) {
      intake.setRollerPower();

      if (!intake.getBreakBeam() && intakeHasRing) {
        intake.setRollerOff();
      }

      if (!launcherHasRing && launcher.getBreakBeam()) {
        launcherHasRing = true;
      }

      if (launcherHasRing && !launcher.getBreakBeam()) {
        launcher.setLauncherOff();
            intake.setRollerOff();


        if(startTime == -1){
          startTime = Timer.getFPGATimestamp();
        }

        if(Timer.getFPGATimestamp() - startTime > duration){
          ended = true;
        }
      }
    }
////////////////////////////////////////////
    // if (intake.getRollerCurrent() > threshold) {
    // if (!beganIntaking) {
    // startTime = Timer.getFPGATimestamp();
    // beganIntaking = true;
    // }
    // timeElapsed = Timer.getFPGATimestamp() - startTime;

    // if (timeElapsed > duration) {
    // intake.setRollerOff();
    // hasRing = true;
    // }
    // }

    // if (hasRing) {
    // intake.setIntakeState(IntakeState.HANDOFF);
    // launcher.setReverseLauncherOn();
    // launcher.setFlickerReverse();
    // }

    // if (hasRing && intake.hasReachedPose(3.0)) {
    // intake.setRollerPower();
    // System.out.print("anshanshasnh");

    // if (!beganHandoff) {
    // startTime2 = Timer.getFPGATimestamp();
    // beganHandoff = true;
    // }

    // timeElapsed2 = Timer.getFPGATimestamp() - startTime2;

    // if (timeElapsed2 > .25) {
    // intake.setRollerOff();
    // }

    // if(!launcherHasRing && launcher.getBreakBeam()){
    // launcherHasRing = true;
    // }

    // if(launcherHasRing && !launcher.getBreakBeam()){
    // ended = true;
    // }

    // }
  }

  @Override
  public void end(boolean interrupted) {
    launcher.setFlickOff();
    launcher.setLauncherState(LauncherState.HOVER);
    intake.setIntakeState(IntakeState.STOP);
    launcher.updatePose();
    // litty.setGreen();
  }

  @Override
  public boolean isFinished() {
    return ended;
  }
}

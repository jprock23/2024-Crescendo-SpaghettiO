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
  private double duration = .5;

  private boolean intakeGood;
  private double timestuff;


  public BreakBeamHandoff() {
    launcher = Launcher.getInstance();
    intake = Intake.getInstance();
    litty = LED.getInstance();

    launcherHasRing = false;
    ended = false;
    intakeGood = true;
    timestuff = -1;
  }

  @Override
  public void initialize() {

    intake.setIntakeState(IntakeState.GROUND);
    launcher.setLauncherState(LauncherState.HANDOFF);
    launcher.updatePose();
    
    startTime = -1;

    launcherHasRing = false;
    ended = false;

    intake.setRollerPower();

    litty.setRed();
  }

  @Override
  public void execute() {

    if (intake.getBreakBeam()) {
      intake.setIntakeState(IntakeState.HANDOFF);
      intake.setRollerOff();
      launcher.setReverseLauncherOn();
      launcher.setFlickerReverse();
      litty.setBlue();
    }

    if (intake.getIntakeState() == IntakeState.HANDOFF && intake.hasReachedPose(3.0)) {
      intake.setRollerPower();

      if (!launcherHasRing && launcher.getBreakBeam()) {
        launcherHasRing = true;
      } 

      if (launcherHasRing && !launcher.getBreakBeam()) {
        launcher.setLauncherOff();
        intake.setRollerOff();
        litty.setGreen();
        if (startTime == -1) {
          startTime = Timer.getFPGATimestamp();
        }
        if (Timer.getFPGATimestamp() - startTime > duration) {
          ended = true;
        } 
      } 
      // if(intakeGood && intake.getBreakBeam()){
      //     intakeGood = false;
      //     timestuff = Timer.getFPGATimestamp();
      //   }
      //   if(Timer.getFPGATimestamp() - timestuff > duration && intake.getBreakBeam() && !intakeGood){
      //     intake.setReverseRollerPower();
      //     Timer.delay(1.8);
      //       ended = true;
      //   }
        
    }
  }

  @Override
  public void end(boolean interrupted) {
    launcher.setFlickOff();
    launcher.setLauncherState(LauncherState.HOVER);
    intake.setIntakeState(IntakeState.HOLD);
    launcher.updatePose();
    intake.setRollerOff();
    launcher.setLauncherOff();
  }

  @Override
  public boolean isFinished() {
    return ended;
  }
}

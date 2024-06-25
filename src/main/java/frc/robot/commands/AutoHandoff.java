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

public class AutoHandoff extends Command {

  private Launcher launcher;
  private Intake intake;
  private LED litty;

  private boolean launcherHasRing;
  private boolean ended;

  private double timeout = 1;

  private double intakeStarttime;
  private double launcherStarttime;

  private boolean intakeHasRing;

  public AutoHandoff() {
    launcher = Launcher.getInstance();
    intake = Intake.getInstance();
    litty = LED.getInstance();

    launcherHasRing = false;
    ended = false;

    intakeStarttime = -1;
    launcherStarttime = -1;

    intakeHasRing = false;
  }

  @Override
  public void initialize() {

    intake.setIntakeState(IntakeState.GROUND);
    launcher.setLauncherState(LauncherState.HANDOFF);
    launcher.updatePose();

    intakeStarttime = -1;
    launcherStarttime = -1;

    launcherHasRing = false;
    ended = false;

    intakeHasRing = false;

    litty.setRed();
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

    if (intake.getBreakBeam() && intakeHasRing) {
      if (intakeStarttime == -1) {
        intakeStarttime = Timer.getFPGATimestamp();
      }
    }

    if (intake.getIntakeState() == IntakeState.HANDOFF && intake.hasReachedPose(3.0)) {
      intake.setRollerPower();

      if (!intake.getBreakBeam() && intakeHasRing) {
        intake.setRollerOff();
      }

      if (!launcherHasRing && launcher.getBreakBeam()) {
        launcherHasRing = true;
        intakeHasRing = false;
      }

      if (launcher.getBreakBeam() && launcherHasRing) {
        if (launcherStarttime == -1) {
          launcherStarttime = Timer.getFPGATimestamp();
        }
        if (Timer.getFPGATimestamp() - launcherStarttime > timeout) {
          launcher.eject();
        }
      }

      if (launcherHasRing && !launcher.getBreakBeam()) {
        intake.setRollerOff();

          ended = true;
      }
    }

  }

  @Override
  public void end(boolean interrupted) {
    launcher.setFlickOff();
    intake.setIntakeState(IntakeState.HOLD);
    intake.setRollerOff();
    litty.setGreen();
  }

  @Override
  public boolean isFinished() {
    return ended;
  }
}

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;

// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.intake.IntakeStates.*;

// import frc.robot.subsystems.launcher.*;
// import frc.robot.subsystems.launcher.LauncherStates.*;
// import frc.robot.subsystems.launcher.LauncherStates.LauncherVoltage;

// public class Handoff extends Command {


//   private Launcher launcher;
//   private Intake intake;

//   private double threshold;

//   private double startTime;
//   private double time;
//   private double duration = 0.5;

//   public Handoff() {
//   }
//   @Override
//   public void initialize() {
//     launcher = new Launcher();
//     intake = new Intake();
//     startTime = Timer.getFPGATimestamp();
//   }

//   @Override
//   public void execute() {
//     launcher.periodic();

//     time = Timer.getFPGATimestamp();

//     if (intake.getRollerCurrent() > threshold) {
//       if (time > startTime + duration){
//         intake.setIntakeState(IntakePosition.HANDOFF);
//       }

//       if (intake.hasReachedPose(9999)){
//         launcher.setLauncherState(LauncherState.HANDOFF);
//       }

//       if (intake.hasReachedPose(2) && launcher.hasReachedPose(2)) {
//         intake.setRollerPower(-1);
//         launcher.setLauncherVolts(LauncherVoltage.HANDOFF);
//       }
//     }


//   }

//   @Override
//   public void end(boolean interrupted) {}

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }

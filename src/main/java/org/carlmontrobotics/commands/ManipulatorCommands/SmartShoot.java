// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.ManipulatorCommands;

import static org.carlmontrobotics.Constants.IntakeC.*;
import static org.carlmontrobotics.Constants.OuttakeC.*;

import org.carlmontrobotics.subsystems.Intake;
import org.carlmontrobotics.subsystems.Outtake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartShoot extends Command {

  Outtake outtake;
  Intake intake;
  double goalRPM;
  /** Creates a new SmartShoot. */
  public SmartShoot(Outtake outtake, Intake intake, double RPM) {//FIXME smartshoot kills code
    // Use addRequirements() here to declare subsystem dependencies.
    goalRPM = RPM;
    this.outtake = outtake;
    this.intake = intake;
    addRequirements(outtake, intake);
   }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   outtake.spinOuttake(goalRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(outtake.atVelGoal(goalRPM, OUTTAKE_ESTIMATE_OFFSET)){
      outtake.spinOuttakeFeeder(OUTTAKE_FEEDER_VOLT_PERC);
      intake.spinConveyor(CONVEYOR_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    outtake.stopOuttake();  
    outtake.spinOuttakeFeeder(0);
    intake.stopConveyor();
  }

  // Returns true when the command should end.

  // @Override
  // public boolean isFinished() {
    // return timed && timer.hasElapsed(time);
  // }
}

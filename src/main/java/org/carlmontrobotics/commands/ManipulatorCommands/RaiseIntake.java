// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import org.carlmontrobotics.Constants.IntakeC;
import org.carlmontrobotics.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RaiseIntake extends Command {
  Intake intake;
  /** Creates a new RaiseIntake. */
  public RaiseIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.stopIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.intakeDown()){
      intake.raiseIntake(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntakeArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(intake.intakeUp()){
      return true;
    }
    else return false;
  }
}

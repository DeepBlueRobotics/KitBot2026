// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import org.carlmontrobotics.subsystems.ArmIntake;
import org.carlmontrobotics.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RaiseIntakeFullyUp extends Command {
  Intake intake;
  ArmIntake arm;

  /** Creates a new RaiseIntake. */
  public RaiseIntakeFullyUp(Intake intake, ArmIntake arm) {
    this.intake = intake;
    this.arm = arm;
    addRequirements(intake, arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.stop();
    arm.raiseIntakeFullyUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

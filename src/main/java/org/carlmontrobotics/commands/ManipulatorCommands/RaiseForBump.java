// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.ManipulatorCommands;

import org.carlmontrobotics.subsystems.ArmIntake;
import org.carlmontrobotics.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RaiseForBump extends Command {
  private final Intake intake;
  private final ArmIntake arm;
  private Command deploy;
  /** Creates a new RaiseForBump. */
  public RaiseForBump(Intake intake, ArmIntake arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.arm = arm;
    addRequirements(intake, arm);
    deploy = new DeployIntake(intake, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.raiseIntakeToBump();
    intake.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().schedule(deploy);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

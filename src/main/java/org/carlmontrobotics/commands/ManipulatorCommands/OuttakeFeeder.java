// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.Command;

import static org.carlmontrobotics.Constants.OuttakeC.OUTTAKE_FEEDER_VOLT_PERC;

import org.carlmontrobotics.subsystems.Outtake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OuttakeFeeder extends Command {
  /** Creates a new OuttakeFeeder. */
  Outtake feeder;

  public OuttakeFeeder(Outtake feeder) {
    this.feeder = feeder;
    addRequirements(feeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.spinOuttakeFeeder(OUTTAKE_FEEDER_VOLT_PERC);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.spinOuttakeFeeder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

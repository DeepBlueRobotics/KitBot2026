// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.ManipulatorCommands;

import static org.carlmontrobotics.Constants.IntakeC.*;

import org.carlmontrobotics.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunConveyor extends Command {
  Intake conveyor;
  Timer timer;
  boolean timed;
  double time;

  /** Creates a new RunConveyor. */
  public RunConveyor(Intake conveyor) {
    this.conveyor = conveyor;
    timer = new Timer();
    timed = false;
    addRequirements(conveyor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public RunConveyor(Intake conveyor, double time) {
    this.conveyor = conveyor;
    this.time = time;
    timer = new Timer();
    timed = true;
    addRequirements(conveyor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.spinConveyor(CONVEYOR_SPEED);
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.spinConveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timed && timer.hasElapsed(time);
  }
}
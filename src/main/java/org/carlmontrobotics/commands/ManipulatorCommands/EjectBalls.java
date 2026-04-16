// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.ManipulatorCommands;

import static org.carlmontrobotics.Constants.IntakeC.CONVEYOR_SPEED;
import static org.carlmontrobotics.Constants.IntakeC.INTAKE_SPEED;
import org.carlmontrobotics.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EjectBalls extends Command {
  private final Intake intake;
  /** Creates a new EjectBalls. */
  public EjectBalls(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.spinConveyor(-CONVEYOR_SPEED);
    intake.spinIntake(-INTAKE_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.spinConveyor(0);
    intake.spinIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

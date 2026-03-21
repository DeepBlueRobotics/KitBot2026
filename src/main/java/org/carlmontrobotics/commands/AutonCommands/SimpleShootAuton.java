// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.AutonCommands;

import static org.carlmontrobotics.Constants.IntakeC.CONVEYOR_SPEED;
import static org.carlmontrobotics.Constants.IntakeC.INTAKE_SPEED;
import static org.carlmontrobotics.Constants.OuttakeC.OUTTAKE_FEEDER_VOLT_PERC;
import static org.carlmontrobotics.Constants.OuttakeC.OUTTAKE_SHOOTING_RPM;

import org.carlmontrobotics.subsystems.Intake;
import org.carlmontrobotics.subsystems.Outtake;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimpleShootAuton extends Command {
  private final Outtake shooter;
  private final Intake intake;
  /** Creates a new SimpleShootAuton. */
  public SimpleShootAuton(Outtake shooter, Intake intake) {
    this.shooter = shooter;
    this.intake = intake;
    addRequirements(shooter, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.spinOuttake(OUTTAKE_SHOOTING_RPM);
    shooter.spinOuttakeFeeder(OUTTAKE_FEEDER_VOLT_PERC);
    intake.spinConveyor(CONVEYOR_SPEED);
    //intake.spinIntake(INTAKE_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopOuttake();
    shooter.spinOuttakeFeeder(0);
    intake.stopConveyor();
    //intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

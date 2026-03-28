// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.AutonCommands;

import static org.carlmontrobotics.Constants.IntakeC.CONVEYOR_SPEED;
import static org.carlmontrobotics.Constants.IntakeC.INTAKE_SPEED;
import static org.carlmontrobotics.Constants.OuttakeC.OUTTAKE_FEEDER_VOLT_PERC;
import static org.carlmontrobotics.Constants.OuttakeC.OUTTAKE_SHOOTING_RPM;

import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Intake;
import org.carlmontrobotics.subsystems.Outtake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimpleShootAuton extends Command {
  private final Drivetrain dt;
  private final Outtake shooter;
  private final Intake intake;
  private final Timer timer;
  /** Creates a new SimpleShootAuton. */
  public SimpleShootAuton(Drivetrain dt, Outtake shooter, Intake intake) {
    this.shooter = shooter;
    this.intake = intake;
    this.dt = dt;
    timer = new Timer();
    addRequirements(shooter, intake, dt);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() > 6.25) {
      dt.drive(0,0,1);
    }
    if (timer.get() > 5.25) {
      shooter.stopOuttake();
      shooter.spinOuttakeFeeder(0);
      intake.spinConveyor(0);
      intake.spinIntake(INTAKE_SPEED);
      dt.drive(3, 0, 0);
    }
    else if (timer.get() > 4.5) {
      shooter.stopOuttake();
      shooter.spinOuttakeFeeder(0);
      intake.spinConveyor(0);
      intake.spinIntake(INTAKE_SPEED);
      dt.drive(0, -2, 0);
    }
    else if (timer.get() > 0.5) {
      shooter.spinOuttakeFeeder(0.7);
      intake.spinConveyor(CONVEYOR_SPEED);
    }
    else {
      shooter.spinOuttake(OUTTAKE_SHOOTING_RPM);
      intake.spinIntake(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopOuttake();
    shooter.spinOuttakeFeeder(0);
    intake.stopConveyor();
    dt.stop();
    //intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

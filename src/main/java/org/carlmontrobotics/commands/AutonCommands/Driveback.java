// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.AutonCommands;

import org.carlmontrobotics.subsystems.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Driveback extends Command {

  private Drivetrain dt;
  double startDistance;
  double endDistance;

  /** Creates a new Driveback. */
  public Driveback(Drivetrain dt, double startDistance, double endDistance) {
    this.startDistance = startDistance;
    this.endDistance = endDistance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  private void travel(double speed,double start, double end) {
    double d = Math.abs(end - start);
    double p = 1 - Math.abs(dt.getPose().getTranslation().getX() - start) / d;
    double speed2 = MathUtil.clamp(speed * p, -0.2, -1); //forces robot to be moving even if its not close enough to goal
    dt.drive(speed2, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    travel(-1, startDistance, endDistance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return dt.getPose().getTranslation().getX() <= endDistance;
  }
}

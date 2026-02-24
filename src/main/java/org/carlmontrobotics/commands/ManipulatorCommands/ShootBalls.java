// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.ManipulatorCommands;

import static org.carlmontrobotics.Constants.OuttakeC.*;

import org.carlmontrobotics.subsystems.Outtake;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootBalls extends Command {
  Outtake outtake;
  Timer timer;
  boolean timed;
  double time;


  /** Creates a new ShootBalls. */
  public ShootBalls(Outtake outtake) {
    this.outtake = outtake;
    timer = new Timer();
    addRequirements(outtake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

    public ShootBalls(Outtake outtake, double time) {
    this.outtake = outtake;
    timer = new Timer();
    addRequirements(outtake);
    this.time = time;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    outtake.spinOuttake(OUTTAKE_FEEDER_VOLT_PERC);
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    outtake.stopOuttake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timed && timer.hasElapsed(time);
  }
}
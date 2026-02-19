// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.carlmontrobotics.subsystems.Outtake;
import org.carlmontrobotics.subsystems.Intake;
import org.carlmontrobotics.Constants.OuttakeC;
import org.carlmontrobototics.Constants.IntakeC;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartShoot extends Command {
  Timer timer;
  /** Creates a new SmartShoot. */
  public SmartShoot(Outtake outtake, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    timer = new Timer();
    addRequirements(outtake, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    outtake.spinOuttake(OuttakeC.SHOOT_VELOCITY);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
    if(timer.get() > 1){
      outtakeFeeder.set(feederSpeed);
      conveyor.set(CONVEYOR_SPEED)
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    outtake.stopOuttake();
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

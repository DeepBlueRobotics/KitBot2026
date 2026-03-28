
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.AlignCommands;

import static org.carlmontrobotics.Constants.LimeLightc.RIGHT_LL;


import edu.wpi.first.wpilibj.Timer;


import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Limelight;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToTag extends Command {
  private Drivetrain drivetrain;
  private Limelight limelight;
  private Timer timer = new Timer();
  /** 
   * @param drivetrain
   * @param limelight
   */
  public RotateToTag(Drivetrain drivetrain, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain = drivetrain, this.limelight = limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.seesTag(RIGHT_LL)) {
      drivetrain.drive(0, 0, -Units.degreesToRadians(limelight.getTX(RIGHT_LL))/10);
      SmartDashboard.putNumber("TX of limelight", limelight.getTX(RIGHT_LL));
      SmartDashboard.putNumber("abs of TX of limelight", Math.abs(limelight.getTX(RIGHT_LL)));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return (Math.abs(limelight.getTX(RIGHT_LL)) < 7 && limelight.seesTag(RIGHT_LL)) || (!limelight.seesTag(RIGHT_LL) && timer.get()>0.5);
    
  }
}

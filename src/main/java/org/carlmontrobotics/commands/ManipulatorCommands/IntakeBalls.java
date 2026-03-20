// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.ManipulatorCommands;

import static org.carlmontrobotics.Constants.IntakeC.*;

import org.carlmontrobotics.subsystems.Intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeBalls extends Command {
  Intake intake;
  XboxController manipulatorRumble;

  /** Creates a new IntakeBalls. */
  public IntakeBalls(Intake intake, XboxController manipulatorRumble) {
    this.intake = intake;
    this.manipulatorRumble = manipulatorRumble;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.spinIntake(INTAKE_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (checkIfJammed()) {
      SmartDashboard.putBoolean("IntakeJammed", true);
      manipulatorRumble.setRumble(RumbleType.kRightRumble, 0.5);
    }
    else {
      SmartDashboard.putBoolean("IntakeJammed", true);
      manipulatorRumble.setRumble(RumbleType.kRightRumble, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    manipulatorRumble.setRumble(RumbleType.kRightRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean checkIfJammed() {
    return intake.getIntakeVelocity() < 20;
  }
}

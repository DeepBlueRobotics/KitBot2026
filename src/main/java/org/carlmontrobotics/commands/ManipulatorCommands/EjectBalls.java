// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.ManipulatorCommands;

import static org.carlmontrobotics.Constants.ConveyorC.CONVEYOR_SPEED;
import static org.carlmontrobotics.Constants.IntakeC.NewIntakeC.ArmC.ARM_kDeployedAngle;
import static org.carlmontrobotics.Constants.IntakeC.NewIntakeC.RollerC.INTAKE_SPEED;

import org.carlmontrobotics.subsystems.ArmIntake;
import org.carlmontrobotics.subsystems.Conveyor;
import org.carlmontrobotics.subsystems.Intake;
import org.carlmontrobotics.subsystems.Outtake;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EjectBalls extends Command {
  private final Intake intake;
  private final Conveyor conveyor;
  private final ArmIntake arm;
  private final Outtake outtake;

  /** Creates a new EjectBalls. */
  public EjectBalls(Intake intake, Conveyor conveyor, ArmIntake arm, Outtake outtake) {
    this.intake = intake;
    this.conveyor = conveyor;
    this.arm = arm;
    this.outtake = outtake;
    addRequirements(intake, conveyor, arm, outtake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.setThrottle(-CONVEYOR_SPEED);
    intake.setRPM(-2*INTAKE_SPEED);
    arm.deploy();
    outtake.spinOuttakeFeeder(-0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.stop();
    intake.setRPM(INTAKE_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

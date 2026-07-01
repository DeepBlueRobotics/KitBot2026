// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.AutonCommands;

import static org.carlmontrobotics.Constants.IntakeC.NewIntakeC.RollerC.INTAKE_SPEED;

import org.carlmontrobotics.subsystems.ArmIntake;
import org.carlmontrobotics.subsystems.Conveyor;
import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Intake;
import org.carlmontrobotics.subsystems.Outtake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LeftNeutralAuto extends Command {
  private final Drivetrain dt;
  private final Outtake shooter;
  private final Intake intake;
  private final Conveyor conveyor;
  private final ArmIntake arm;
  private final Timer timer;

  private double endFastDrive = 2.4;
  private double endSlowDrive = 0.1;
  private double endRotation = 0.6;
  private double endStrafeDrive = 5;

  /** Creates a new SimpleShootAuton. */
  public LeftNeutralAuto(Drivetrain dt, Outtake shooter, Intake intake, Conveyor conveyor, ArmIntake arm) {
    this.shooter = shooter;
    this.intake = intake;
    this.dt = dt;
    this.conveyor = conveyor;
    this.arm = arm;
    timer = new Timer();
    addRequirements(shooter, intake, dt, arm, conveyor);
    // SmartDashboard.putNumber("startShoot", startShoot);
    // SmartDashboard.putNumber("endshoot", endshoot);
    // SmartDashboard.putNumber("endStrafe", endStrafe);
    // SmartDashboard.putNumber("endFastDrive", endFastDrive);
    // SmartDashboard.putNumber("endSlowDrive", endSlowDrive);
    // SmartDashboard.putNumber("endRotation", endRotation);
    // SmartDashboard.putNumber("endStrafeDrive", endStrafeDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetFieldOrientation();
    dt.setFieldOriented(true);
    timer.restart();
    arm.raiseIntakeFullyUp();
    intake.stop();
  //   startShoot = SmartDashboard.getNumber("startShoot", startShoot);
  //   endshoot = SmartDashboard.getNumber("endshoot", endshoot);
  //  endStrafe =  SmartDashboard.getNumber("endStrafe", endStrafe);
  //   endFastDrive = SmartDashboard.getNumber("endFastDrive", endFastDrive);
  //   endSlowDrive = SmartDashboard.getNumber("endSlowDrive", endSlowDrive);
  //   endRotation = SmartDashboard.getNumber("endRotation", endRotation);
  //   endStrafeDrive= SmartDashboard.getNumber("endStrafeDrive", endStrafeDrive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = timer.get();
    if (currentTime > endFastDrive+endSlowDrive+endRotation+endStrafeDrive) {
      dt.drive(0,0,0);
    }
    else if (currentTime > endFastDrive+endSlowDrive+endRotation) {
      dt.drive(0,0.8,0);
    }
    else if (currentTime > endFastDrive+endSlowDrive) {
      dt.drive(0,0,-3.4);
    }
    else if (currentTime > endFastDrive) {
      dt.drive(1.3,0,0);
      intake.setRPM(INTAKE_SPEED+500);
      arm.deployIntake();
    }
    else {
      dt.drive(3, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopOuttake();
    shooter.spinOuttakeFeeder(0);
    conveyor.stop();
    dt.stop();
    //intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

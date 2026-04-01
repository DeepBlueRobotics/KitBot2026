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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RightNeutralAuto extends Command {
  private final Drivetrain dt;
  private final Outtake shooter;
  private final Intake intake;
  private final Timer timer;

  private double startShoot = 0.5;
  private double endshoot = 4;
  private double endStrafe = 1;
  private double endFastDrive = 1.5;
  private double endSlowDrive = 2;
  private double endRotation = 0.5;
  private double endStrafeDrive = 3;

  /** Creates a new SimpleShootAuton. */
  public RightNeutralAuto(Drivetrain dt, Outtake shooter, Intake intake) {
    this.shooter = shooter;
    this.intake = intake;
    this.dt = dt;
    timer = new Timer();
    addRequirements(shooter, intake, dt);
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
      dt.drive(0,-1,0);
    }
    else if (currentTime > endFastDrive+endSlowDrive) {
      dt.drive(0,0,3.4);
    }
    else if (currentTime > endFastDrive) {
      dt.drive(1.3,0,0);
    }
    else {
      dt.drive(3, 0, 0);
      intake.spinIntake(INTAKE_SPEED);
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

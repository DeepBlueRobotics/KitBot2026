// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.ManipulatorCommands;

import static org.carlmontrobotics.Constants.IntakeC.*;
import static org.carlmontrobotics.Constants.OuttakeC.*;

import org.carlmontrobotics.subsystems.Intake;
import org.carlmontrobotics.subsystems.Outtake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartShoot extends Command {

  private final Outtake outtake;
  private final Intake intake;
  private final double goalRPM;
  private final Timer timer;

  private final boolean oscillate = true;
  private final boolean useIntake;
  private boolean backwards = false;
  private final boolean fasterSpinUp = true;

  /** Creates a new SmartShoot. */
  public SmartShoot(Outtake outtake, Intake intake, double RPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    goalRPM = Math.min(RPM, 5500);
    this.outtake = outtake;
    this.intake = intake;
    timer = new Timer();
    this.useIntake = false;
    addRequirements(outtake, intake);
   }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    outtake.spinOuttake(goalRPM);
    timer.restart();
    if (useIntake) {
      intake.spinIntake(INTAKE_SPEED);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (fasterSpinUp) {
      if (outtake.getOuttakeVelocity() < 2000) {
        outtake.spinOuttakeWithVoltage(1);
        SmartDashboard.putBoolean("Working", true);
      }
      else {
        outtake.spinOuttake(goalRPM);
        SmartDashboard.putBoolean("Working", false);
      }
    }

    if(outtake.atVelGoal(goalRPM, OUTTAKE_ESTIMATE_OFFSET)){
      outtake.spinOuttakeFeeder(OUTTAKE_FEEDER_VOLT_PERC);
      if (oscillate) {
        if (timer.get() > 5 && !backwards) {
        timer.restart();
        intake.spinConveyor(-0.2);
        backwards = true;
      }
      else if (timer.get() > 0.5 && backwards) {
        timer.restart();
        intake.spinConveyor(CONVEYOR_SPEED);
        backwards = false;
      } 
      }
      else {
        intake.spinConveyor(CONVEYOR_SPEED);
      } 
      outtake.spinOuttakeFeeder(OUTTAKE_FEEDER_VOLT_PERC);
      intake.spinConveyor(CONVEYOR_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    outtake.stopOuttake();  
    outtake.spinOuttakeFeeder(0);
    intake.stopConveyor();
    intake.stopIntake();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

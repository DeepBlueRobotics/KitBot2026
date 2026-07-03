// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.ManipulatorCommands;

import static org.carlmontrobotics.Constants.IntakeC.NewIntakeC.RollerC.*;
import static org.carlmontrobotics.Constants.ConveyorC.*;
import static org.carlmontrobotics.Constants.OuttakeC.*;

import org.carlmontrobotics.subsystems.ArmIntake;
import org.carlmontrobotics.subsystems.Conveyor;
import org.carlmontrobotics.subsystems.Intake;
import org.carlmontrobotics.subsystems.Outtake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartShoot extends Command {

  private final Outtake outtake;
  private final Intake intake;
  private final Conveyor conveyor;
  private final ArmIntake arm;
  private final double goalRPM;
  private final Timer conveyorTimer;
  private final Timer shootingTimer;

  private final boolean oscillate = true;
  private final boolean useIntake;
  private boolean backwards = true;
  private final boolean fasterSpinUp = true;

  /** Creates a new SmartShoot. */
  public SmartShoot(Outtake outtake, Intake intake, Conveyor conveyor, ArmIntake arm, double RPM) {
    goalRPM = RPM;
    this.outtake = outtake;
    this.intake = intake;
    this.conveyor = conveyor;
    this.arm = arm;
    conveyorTimer = new Timer();
    shootingTimer = new Timer();
    this.useIntake = false;
    addRequirements(outtake, intake, arm, conveyor);
   }

   public SmartShoot(Outtake outtake, Intake intake, Conveyor conveyor, ArmIntake arm, double RPM, boolean useIntake) {
    goalRPM = RPM;
    this.outtake = outtake;
    this.intake = intake;
    this.conveyor = conveyor;
    this.arm = arm;
    conveyorTimer = new Timer();
    shootingTimer = new Timer();
    this.useIntake = useIntake;
    addRequirements(outtake, intake, arm, conveyor);
   }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    outtake.setRPM(goalRPM);
    conveyorTimer.restart();
    shootingTimer.restart();
    if (useIntake) {
      arm.deploy();
      intake.setRPM(INTAKE_SPEED);
    }
    backwards = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (fasterSpinUp) {
      if (outtake.getOuttakeVelocity() < 2000) {
        outtake.spinOuttakeWithVoltage(1);
      }
      else {
        outtake.setRPM(goalRPM);
      }
    }

    if(outtake.atVelGoal(goalRPM, OUTTAKE_ESTIMATE_OFFSET)){
      outtake.spinOuttakeFeeder(OUTTAKE_FEEDER_VOLT_PERC);
      if (oscillate) {
        if (conveyorTimer.get() > 4 && !backwards) {
        conveyorTimer.restart();
        conveyor.setThrottle(0);
        backwards = true;
      }
      else if (conveyorTimer.get() > 0.3 && backwards) {
        conveyorTimer.restart();
        conveyor.setThrottle(CONVEYOR_SPEED);
        backwards = false;
      } 
      }
      else {
        conveyor.setThrottle(CONVEYOR_SPEED);
      } 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    outtake.stopOuttake();  
    outtake.spinOuttakeFeeder(0);
    conveyor.stop();
    intake.stop();
    conveyorTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

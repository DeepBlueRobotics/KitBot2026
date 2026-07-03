// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.AutonCommands;

import org.carlmontrobotics.subsystems.ArmIntake;
import org.carlmontrobotics.subsystems.Conveyor;
import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Intake;
import org.carlmontrobotics.subsystems.Outtake;

import static org.carlmontrobotics.Constants.IntakeC.NewIntakeC.RollerC.*;
import static org.carlmontrobotics.Constants.IntakeC.NewIntakeC.ArmC.*;
import static org.carlmontrobotics.Constants.ConveyorC.*;
import static org.carlmontrobotics.Constants.OuttakeC.*;
import static org.carlmontrobotics.Constants.EyeballAutoC.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterToLeftNeutralAuto extends SequentialCommandGroup {

  public CenterToLeftNeutralAuto(Drivetrain drivetrain, Outtake outtake, Intake intake, Conveyor conveyor, ArmIntake arm) {
    addCommands(
      new InstantCommand(() -> { // Intialize
            drivetrain.resetFieldOrientation();
            drivetrain.setFieldOriented(true);
            arm.collapse();
            intake.stop();
          }),
          new ParallelRaceGroup( //Shoot while pumping balls, wait for shooter to speed up
            new SequentialCommandGroup(
              new InstantCommand(() -> outtake.spinOuttakeWithVoltage(1)),
              new WaitUntilCommand(() -> outtake.getOuttakeVelocity() > OUTTAKE_SHOOTING_RPM - 250),
              new InstantCommand(() -> outtake.setRPM(OUTTAKE_SHOOTING_RPM)),
              new WaitUntilCommand(() -> outtake.atVelGoal(OUTTAKE_SHOOTING_RPM, OUTTAKE_ESTIMATE_OFFSET)),
              new InstantCommand(() -> outtake.spinOuttakeFeeder(OUTTAKE_FEEDER_VOLT_PERC)),
              new WaitCommand(endShoot)),
            new RepeatCommand( //Conveyor
              new SequentialCommandGroup(
                new InstantCommand(() -> conveyor.setThrottle(CONVEYOR_SPEED)),
                new WaitCommand(3),
                new InstantCommand(conveyor::stop),
                new WaitCommand(0.2)
              )
            )
          ),
          new InstantCommand(() -> { //Move towards bump
            outtake.stopOuttake();
            outtake.spinOuttakeFeeder(0);
            conveyor.stop();
            drivetrain.drive(-0.2, -2, 0);
          }),
          new WaitCommand(endStrafe),
          new InstantCommand(() -> drivetrain.drive(3, 0, 0)), //Go over bump quick!
          new ParallelCommandGroup( //Slow down as approaching balls, but before that lower down the intake
            new SequentialCommandGroup(
              new WaitCommand(endFastDrive),
              new InstantCommand(() -> {
                drivetrain.drive(2,0,0);
              }),
              new WaitCommand(endSlowDrive),
              new InstantCommand(() -> drivetrain.drive(2,0,-4)), //Turn 90* while edging a bit more forward getting extra depth into the middle
              new WaitCommand(endRotation),
              new InstantCommand(() -> drivetrain.drive(0,1,0)), //Go across filling up hopper
              new WaitCommand(endStrafeDrive),
              new InstantCommand(() -> { //Stop, keep intake on just incase :)
                outtake.stopOuttake();
                outtake.spinOuttakeFeeder(0);
                conveyor.stop();
                drivetrain.setX();
              })
            ),
            new SequentialCommandGroup(
              new WaitCommand(1.5),
              new InstantCommand(arm::deploy),
              new WaitUntilCommand(arm::isIntakeDown),
              new InstantCommand(() -> intake.setRPM(INTAKE_SPEED))
            )
          )
      );
  }
}

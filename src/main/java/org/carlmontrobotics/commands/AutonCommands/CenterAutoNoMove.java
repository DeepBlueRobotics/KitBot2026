// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.AutonCommands;

import org.carlmontrobotics.subsystems.ArmIntake;
import org.carlmontrobotics.subsystems.Conveyor;
import org.carlmontrobotics.subsystems.Intake;
import org.carlmontrobotics.subsystems.Outtake;

import static org.carlmontrobotics.Constants.IntakeC.NewIntakeC.RollerC.*;
import static org.carlmontrobotics.Constants.IntakeC.NewIntakeC.ArmC.*;
import static org.carlmontrobotics.Constants.ConveyorC.*;
import static org.carlmontrobotics.Constants.OuttakeC.*;
import static org.carlmontrobotics.Constants.EyeballAutoC.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterAutoNoMove extends SequentialCommandGroup {
  /** Creates a new CenterAutoNoMove. */
  public CenterAutoNoMove(Outtake outtake, Conveyor conveyor, ArmIntake arm, Intake intake) {
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup( //Shooter
              new InstantCommand(() -> outtake.spinOuttakeWithVoltage(1)),
              new WaitUntilCommand(() -> outtake.getOuttakeVelocity() > OUTTAKE_SHOOTING_RPM - 250),
              new InstantCommand(() -> outtake.setRPM(OUTTAKE_SHOOTING_RPM)),
              new WaitUntilCommand(() -> outtake.atVelGoal(OUTTAKE_SHOOTING_RPM, OUTTAKE_ESTIMATE_OFFSET)),
              new InstantCommand(() -> outtake.spinOuttakeFeeder(OUTTAKE_FEEDER_VOLT_PERC))
            ),

        new RepeatCommand( //Conveyor
            new SequentialCommandGroup(
                new InstantCommand(() -> conveyor.setThrottle(CONVEYOR_SPEED)),
                new WaitCommand(3),
                new InstantCommand(conveyor::stop),
                new WaitCommand(0.2)
              )
        ),
        new InstantCommand(() -> {
          intake.stop();
          arm.collapse();
        })
      ).withTimeout(10),
      new InstantCommand(() -> {
        conveyor.stop();
        outtake.stopOuttake();
        outtake.spinOuttakeFeeder(0);
      })
            
      );
  }
}

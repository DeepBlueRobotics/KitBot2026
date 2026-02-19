// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import org.carlmontrobotics.Constants.IntakeC;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;

public class Intake extends SubsystemBase {
  SparkFlex intake;
  SparkFlex conveyor;
  /** Creates a new Intake. */
  public Intake() {
    SparkFlex intake = MotorControllerFactory.createSparkFlex(IntakeC.INTAKE_ID);
    SparkFlex conveyor = MotorControllerFactory.createSparkFlex(IntakeC.CONVEYOR_ID);
  }

  public void spinIntake(double intakeSpeed) {
    intake.set(intakeSpeed);
  }

  public void spinConveyor(double conveyorSpeed) {
    conveyor.set(conveyorSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Speed", intake.get());
    SmartDashboard.putNumber("Conveyor Speed", conveyor.get());
  }
}

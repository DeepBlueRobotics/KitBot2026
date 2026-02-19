// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.Constants.IntakeC;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  SparkBase intakeMotor;
  SparkBase conveyorMotor;
  
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = MotorControllerFactory.createSpark(IntakeC.INTAKE_ID, MotorConfig.NEO_VORTEX);
    conveyorMotor = MotorControllerFactory.createSpark(IntakeC.CONVEYOR_ID, MotorConfig.NEO_VORTEX);
  }

  public void spinIntake(double intakeSpeed) {
    intakeMotor.set(intakeSpeed);
  }

  public void spinConveyor(double conveyorSpeed) {
    conveyorMotor.set(conveyorSpeed);
  }

  public void stopIntake(){
    intakeMotor.set(0);
  }

  public void stopConveyor(){
    conveyorMotor.set(0);
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("Intake Speed perc", () -> intakeMotor.get(), this::spinIntake);
    builder.addDoubleProperty("Conveyor Speed perc", () -> conveyorMotor.get(), this::spinConveyor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

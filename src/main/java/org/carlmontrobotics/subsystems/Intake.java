// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.Constants.IntakeC;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  SparkBase intakeMotor;
  SparkBase intakeFollowerMotor;
  SparkBase conveyorMotor;
  SparkClosedLoopController pidC;
  
  /** Creates a new Intake. */
  public Intake() {
    SparkBaseConfig conveyorConfig = MotorControllerFactory.sparkConfig(MotorConfig.NEO_VORTEX);
    SparkBaseConfig intakeConfig = MotorControllerFactory.sparkConfig(MotorConfig.NEO_VORTEX);
    conveyorConfig.inverted(true);
    intakeConfig.smartCurrentLimit(80)
                  .inverted(true)
                  .closedLoop.pid(0.0002, 0, 0);
    SparkBaseConfig intakeFollowerConfig = MotorControllerFactory.sparkConfig(MotorConfig.NEO_VORTEX);
    intakeFollowerConfig.apply(intakeConfig)
                        .follow(IntakeC.INTAKE_ID, true);
    intakeMotor = MotorControllerFactory.createSpark(IntakeC.INTAKE_ID, MotorConfig.NEO_VORTEX, intakeConfig);
    intakeFollowerMotor = MotorControllerFactory.createSpark(IntakeC.INTAKE_FOLLOWER_ID, MotorConfig.NEO_VORTEX, intakeFollowerConfig);
    conveyorMotor = MotorControllerFactory.createSpark(IntakeC.CONVEYOR_ID, MotorConfig.NEO_VORTEX, conveyorConfig);
    pidC = intakeMotor.getClosedLoopController();
  }

  public void spinIntake(double intakeSpeed) {
    pidC.setSetpoint(intakeSpeed, ControlType.kVelocity);
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
    builder.addDoubleProperty("Intake Speed rpm actual", () -> intakeMotor.getEncoder().getVelocity(), this::spinIntake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("rpm brr", intakeMotor.getEncoder().getVelocity());
  }

  /**
   * 
   * @return Number the RPM of the motor
   */
  public double getIntakeVelocity() {
    return intakeMotor.getEncoder().getVelocity();
  }

  /**
   * 
   * @return The motor controller's output current in Amps.
   */
  public double getIntakeCurrent() {
    return intakeMotor.getOutputCurrent();
  }
  /**
   * 
   * @return The motor controller's applied output duty cycle.

   */
  public double getIntakeAppliedOutput() {
    return intakeMotor.getAppliedOutput();
  }
}

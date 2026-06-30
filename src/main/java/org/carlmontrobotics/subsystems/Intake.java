// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.IntakeC.*;

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
    SparkBase intakeArmMotor;
    SparkBase conveyorMotor;
    SparkClosedLoopController pidC;
    SparkClosedLoopController pidArmC;
    
    /** Creates a new Intake. */
    public Intake() {
      SparkBaseConfig conveyorConfig = MotorControllerFactory.sparkConfig(MotorConfig.NEO_VORTEX);
      SparkBaseConfig intakeConfig = MotorControllerFactory.sparkConfig(MotorConfig.NEO_VORTEX);
      SparkBaseConfig intakeArmConfig = MotorControllerFactory.sparkConfig(MotorConfig.NEO_VORTEX);
      conveyorConfig.inverted(true);
      intakeConfig.smartCurrentLimit(80)
                    .inverted(true)
                    .closedLoop.pid(0.0002, 0, 0);
      SparkBaseConfig intakeFollowerConfig = MotorControllerFactory.sparkConfig(MotorConfig.NEO_VORTEX);
      intakeFollowerConfig.apply(intakeConfig)
                          .follow(IntakeC.INTAKE_ID, true);
      intakeArmConfig.smartCurrentLimit(80)
                      .inverted(false) //Don't know yet
                      .closedLoop.pid(0, 0, 0); //TODO
      intakeArmConfig.encoder
                     .positionConversionFactor(40) //converts to degs for ARM not motor 
                     .velocityConversionFactor(9/60); //converts to deg/s
      intakeMotor = MotorControllerFactory.createSpark(IntakeC.INTAKE_ID, MotorConfig.NEO_VORTEX, intakeConfig);
      intakeFollowerMotor = MotorControllerFactory.createSpark(IntakeC.INTAKE_FOLLOWER_ID, MotorConfig.NEO_VORTEX, intakeFollowerConfig);
      intakeArmMotor = MotorControllerFactory.createSpark(INTAKE_ARM_MOTOR_ID, MotorConfig.NEO_VORTEX, intakeArmConfig);
    conveyorMotor = MotorControllerFactory.createSpark(IntakeC.CONVEYOR_ID, MotorConfig.NEO_VORTEX, conveyorConfig);
    pidC = intakeMotor.getClosedLoopController();
    pidArmC = intakeArmMotor.getClosedLoopController();
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

  public void stopIntakeArm(){
    intakeArmMotor.set(0);
  }

  public void raiseIntake(Boolean pid){
    if(pid){
      pidArmC.setSetpoint(INTAKE_ARM_UP_POSITION, ControlType.kPosition);
    }
    else if(Math.abs(INTAKE_ARM_UP_POSITION - intakeArmMotor.getEncoder().getPosition()) > ARM_BIG_ESTIMATE_OFFSET){
      intakeArmMotor.set(0.75);
    }
    else if(Math.abs(INTAKE_ARM_UP_POSITION - intakeArmMotor.getEncoder().getPosition()) < ARM_BIG_ESTIMATE_OFFSET){
      intakeArmMotor.set(-0.35);
    }
    else if(Math.abs(INTAKE_ARM_UP_POSITION - intakeArmMotor.getEncoder().getPosition()) < ARM_SMALL_ESTIMATE_OFFSET){
      intakeArmMotor.set(0);
    }
  }

  public void deployIntake(Boolean pid){
    if(pid){
      pidArmC.setSetpoint(INTAKE_ARM_DOWN_POSITION, ControlType.kPosition);
    }

    else if(Math.abs(INTAKE_ARM_DOWN_POSITION - intakeArmMotor.getEncoder().getPosition()) > ARM_BIG_ESTIMATE_OFFSET){
      intakeArmMotor.set(0.75);
    }
    else if(Math.abs(INTAKE_ARM_DOWN_POSITION - intakeArmMotor.getEncoder().getPosition()) < ARM_BIG_ESTIMATE_OFFSET){
      intakeArmMotor.set(-0.75);
    }
    else if(Math.abs(INTAKE_ARM_DOWN_POSITION - intakeArmMotor.getEncoder().getPosition()) < ARM_SMALL_ESTIMATE_OFFSET){
      intakeArmMotor.set(0);
    }
  }

  public Boolean intakeDown(){
    if(Math.abs(INTAKE_ARM_UP_POSITION - intakeArmMotor.getEncoder().getPosition()) < ARM_SMALL_ESTIMATE_OFFSET){
      return true;
    }
    else return false;
  }

  public Boolean intakeUp(){
     if(Math.abs(INTAKE_ARM_UP_POSITION - intakeArmMotor.getEncoder().getPosition()) < ARM_SMALL_ESTIMATE_OFFSET){
      return true;
    }
    else return false;
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("Intake Speed perc", () -> intakeMotor.get(), this::spinIntake);
    builder.addDoubleProperty("Intake applied output", () -> intakeMotor.getAppliedOutput(), null);
    builder.addDoubleProperty("Intake follower applied output", () -> intakeMotor.getAppliedOutput(), null);
    builder.addDoubleProperty("Conveyor Speed perc", () -> conveyorMotor.get(), this::spinConveyor);
    builder.addDoubleProperty("Intake Speed rpm actual", () -> intakeMotor.getEncoder().getVelocity(), this::spinIntake);
    builder.addDoubleProperty("Intake ARM Position", () -> intakeArmMotor.getEncoder().getPosition(), null);
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

  public double getIntakeArmPosition(){
    return intakeArmMotor.getEncoder().getPosition();
  }
}

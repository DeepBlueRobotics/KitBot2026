// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;

import static org.carlmontrobotics.Constants.IntakeC.NewIntakeC.ArmC.*;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmIntake extends SubsystemBase {
  private SparkBase intakeArmMotor;
  private RelativeEncoder intakeArmEncoder;
  private SparkClosedLoopController armPID;
  private SparkBaseConfig intakeArmConfig;
  private boolean isBrake = true; //I JUST LOVE REV AND HOW THEIR METHODS ARE SO ACCESSIBLE and uS needing to use a boolean INSTEAD OF JUST BEING ABLE TO GET THE parameter

  /** Creates a new ArmIntake. */
  public ArmIntake() {
    intakeArmConfig = MotorControllerFactory.sparkConfig(MotorConfig.NEO_VORTEX);
    intakeArmConfig.smartCurrentLimit(40) //might wanna lower this even more
                    .inverted(true) //Don't know yet but I think its true
                    .idleMode(IdleMode.kBrake)
                    .closedLoop.pid(kP, kI, kD);
    intakeArmConfig.encoder
                    .positionConversionFactor(40) //converts to degs for ARM not motor 
                    .velocityConversionFactor(40.0/60); //converts to deg/s
    intakeArmMotor = MotorControllerFactory.createSpark(INTAKE_ARM_MOTOR_ID, MotorConfig.NEO_VORTEX, intakeArmConfig);
    intakeArmEncoder = intakeArmMotor.getEncoder();
    armPID = intakeArmMotor.getClosedLoopController();

    SmartDashboard.putData(this);

  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("Position (deg)", this::getIntakeArmPosition, null);
    builder.addBooleanProperty("is Deployed", this::isIntakeDown, null);
    builder.addBooleanProperty("is Collapsed", this::isIntakeFullyUp, null);
    builder.addBooleanProperty("Clearance for BUMP", this::isIntakeBumpUp, null);
    builder.addBooleanProperty("BrakeMode", () -> isBrake, this::changeIdleMode);
  }

  public void stopIntakeArm(){
    intakeArmMotor.set(0);
  }

  public void raiseIntakeFullyUp() {
    armPID.setSetpoint(ARM_kStowedAngle, ControlType.kPosition);
  }

  public void raiseIntakeToBump() {
    armPID.setSetpoint(ARM_kClearanceBumpAngle - 10, ControlType.kPosition); //-10 degrees to clear the level by 10 
  }

  public void deployIntake(){
    armPID.setSetpoint(ARM_kDeployedAngle, ControlType.kPosition);
  }

  public void setPosManual(double pos) {
    armPID.setSetpoint(pos, ControlType.kPosition);
  }

  public boolean isIntakeDown(){
    return Math.abs(ARM_kDeployedAngle - intakeArmEncoder.getPosition()) < ARM_SMALL_ESTIMATE_OFFSET;
  }

  public boolean isIntakeFullyUp(){
    return Math.abs(ARM_kStowedAngle - intakeArmEncoder.getPosition()) < ARM_SMALL_ESTIMATE_OFFSET;
  }

  public boolean isIntakeBumpUp() {
    return intakeArmEncoder.getPosition() < ARM_kClearanceBumpAngle; //Assuming that when fully collapsed is 0 and goes +
  }

  public double getIntakeArmPosition(){
    return intakeArmEncoder.getPosition();
  }

  public double getIntakeArmSetpoint() {
    return armPID.getSetpoint();
  }

  public boolean isIntakeAtPos() {
    return Math.abs(getIntakeArmSetpoint() - intakeArmEncoder.getPosition()) < ARM_SMALL_ESTIMATE_OFFSET;
  }

  //For in case something happened with motor and we just want it work as a passive intake
  private void changeIdleMode(boolean idleMode) {
    if (idleMode) {
      intakeArmConfig.idleMode(IdleMode.kBrake);
    }
    else {
      intakeArmConfig.idleMode(IdleMode.kCoast);
    }
    isBrake = idleMode;
    intakeArmMotor.configure(intakeArmConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}

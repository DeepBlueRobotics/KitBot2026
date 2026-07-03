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

import edu.wpi.first.math.MathUtil;
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
    builder.addBooleanProperty("is Collapsed", this::isIntakeCollapsed, null);
    builder.addBooleanProperty("Clearance for BUMP", this::isIntakeBumpUp, null);
    builder.addBooleanProperty("BrakeMode", () -> isBrake, this::setIdleMode);
  }

  /**
   * Cancels any setpoint set, stops the motor in place
   */
  public void stopIntakeArm(){
    armPID.setSetpoint(0, ControlType.kDutyCycle);
  }

  /**
   * Collapses arm fully inward
   */
  public void collapse() {
    armPID.setSetpoint(ARM_kStowedAngle, ControlType.kPosition);
  }

  /**
   * Raises arm 10 degrees above the clearance angle of 25 degrees
   */
  public void raiseIntakeToBump() {
    armPID.setSetpoint(ARM_kClearanceBumpAngle - 10, ControlType.kPosition); //-10 degrees to clear the level by 10 
  }

  /**
   * Deploys the arm
   */
  public void deploy(){
    armPID.setSetpoint(ARM_kDeployedAngle, ControlType.kPosition);
  }

  /**
   * Allows for setting the arm angle manually, is controlled between the limits
   * @param pos angle to which the arm is going
   */
  public void setPosManual(double pos) {
    armPID.setSetpoint(MathUtil.clamp(pos, ARM_kStowedAngle, ARM_kDeployedAngle), ControlType.kPosition);
  }

  /**
   * @return if intake is deployed
   */
  public boolean isIntakeDown(){
    return Math.abs(ARM_kDeployedAngle - intakeArmEncoder.getPosition()) < ARM_SMALL_ESTIMATE_OFFSET;
  }

  /**
   * @return if intake is stowed
   */
  public boolean isIntakeCollapsed(){
    return Math.abs(ARM_kStowedAngle - intakeArmEncoder.getPosition()) < ARM_SMALL_ESTIMATE_OFFSET;
  }

  /**
   * @return if intake is clearing the bump
   */
  public boolean isIntakeBumpUp() {
    return intakeArmEncoder.getPosition() < ARM_kClearanceBumpAngle; //Assuming that when fully collapsed is 0 and goes +
  }

  /**
   * @return current position of arm (deg)
   */
  public double getIntakeArmPosition(){
    return intakeArmEncoder.getPosition();
  }

  /**
   * @return desired position of arm (deg)
   */
  public double getIntakeArmSetpoint() {
    return armPID.getSetpoint();
  }

  /**
   * @return if intake is at desired position within tolerance
   */
  public boolean isIntakeAtPos() {
    return Math.abs(getIntakeArmSetpoint() - intakeArmEncoder.getPosition()) < ARM_SMALL_ESTIMATE_OFFSET;
  }

  /**
   * Sets idleMode of the arm.
   * Allows the intake be passive or motorized
   * @param idleMode true for BRAKE, false for COAST
   */
  private void setIdleMode(boolean idleMode) {
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

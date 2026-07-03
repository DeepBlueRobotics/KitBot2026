// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.IntakeC.NewIntakeC.RollerC.*;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.PeriodicStatus9;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private SparkBase intakeMotor;
    private SparkBase intakeFollowerMotor;
    private RelativeEncoder intakeEncoder;
    private SparkClosedLoopController intakePID;
    
    /** Creates a new Intake. */
    public Intake() {
      SparkBaseConfig intakeConfig = MotorControllerFactory.sparkConfig(MotorConfig.NEO_VORTEX);
      intakeConfig.smartCurrentLimit(smartCurrentLimit)
                    .inverted(true)
                    .closedLoop.pid(kP, kI, kD);
      SparkBaseConfig intakeFollowerConfig = MotorControllerFactory.sparkConfig(MotorConfig.NEO_VORTEX);
      intakeFollowerConfig.apply(intakeConfig)
                          .follow(INTAKE_ID, true);
      intakeMotor = MotorControllerFactory.createSpark(INTAKE_ID, MotorConfig.NEO_VORTEX, intakeConfig);
      intakeFollowerMotor = MotorControllerFactory.createSpark(INTAKE_FOLLOWER_ID, MotorConfig.NEO_VORTEX, intakeFollowerConfig);
      intakePID = intakeMotor.getClosedLoopController();
      intakeEncoder = intakeMotor.getEncoder();

      SmartDashboard.putData(this);
  }

  /**
   * Sets the speed of the intake using PID
   * @param intakeSpeed RPM of the roller
   */
  public void setRPM(double intakeSpeed) {
    intakePID.setSetpoint(intakeSpeed, ControlType.kVelocity);
  }

  /**
   * Stops the rollers
   */
  public void stop(){
    intakeMotor.set(0);
  }


  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("Input perc", () -> intakeMotor.get(), null);
    builder.addDoubleProperty("Applied output", this::getAppliedOutput, null);
    builder.addDoubleProperty("RPM actual", this::getVelocityRPM, null);
    builder.addDoubleProperty("RPM wanted", () -> intakePID.getSetpoint(), this::setRPM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * 
   * @return RPM of the rollers
   */
  public double getVelocityRPM() {
    return intakeEncoder.getVelocity();
  }

  /**
   * 
   * @return The motor controller's output current in Amps.
   */
  public double getCurrentAmps() {
    return intakeMotor.getOutputCurrent();
  }
  /**
   * 
   * @return The motor controller's applied output duty cycle.

   */
  public double getAppliedOutput() {
    return intakeMotor.getAppliedOutput();
  }

  /** 
   * Current RPM setpoint
  */
  public double getSetpoint() {
    return intakePID.getSetpoint();
  }

}

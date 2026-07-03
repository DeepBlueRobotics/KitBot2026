// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.OuttakeC.*;

import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {
  SparkBase outtakeMaster;
  SparkBase outtakeFollower;
  SparkBase outtakeFeeder;
  private SparkBaseConfig outtakeFollowerConfig;
  private SparkBaseConfig outtakeConfig;
  private SparkBaseConfig outtakeFeederConfig;
  private SparkClosedLoopController pidController;
  RelativeEncoder outtakeMasterEncoder;
  RelativeEncoder outtakeFeederEncoder;
  /** Creates a new Outtake. */
  public Outtake() {
    configureMotors();
    outtakeMaster = MotorControllerFactory.createSpark(OUTTAKE_ID, OUTTAKE_MASTER_MOTOR_CONFIG, outtakeConfig);
    outtakeFollower = MotorControllerFactory.createSpark(OUTTAKE_FOLLOWER_ID, OUTTAKE_FOLLOWER_MOTOR_CONFIG, outtakeFollowerConfig);
    outtakeFeeder = MotorControllerFactory.createSpark(OUTTAKE_FEEDER_ID, OUTTAKE_FEEDER_MOTOR_CONFIG, outtakeFeederConfig);

    pidController = outtakeMaster.getClosedLoopController();

    outtakeMasterEncoder = outtakeMaster.getEncoder();
    outtakeFeederEncoder = outtakeFeeder.getEncoder();
    SmartDashboard.putData(this);
   }

  private void configureMotors(){
    outtakeConfig = MotorControllerFactory.sparkConfig(OUTTAKE_MASTER_MOTOR_CONFIG);
    outtakeConfig.idleMode(IdleMode.kCoast)
                  .inverted(true)
                  .smartCurrentLimit(80)
                  .encoder.quadratureAverageDepth(2)
                          .quadratureMeasurementPeriod(8);
    outtakeConfig.closedLoop.pid(kP, kI, kD)
                            .feedForward.kV(kV);

    outtakeFollowerConfig = MotorControllerFactory.sparkConfig(OUTTAKE_FOLLOWER_MOTOR_CONFIG);
    outtakeFollowerConfig.apply(outtakeConfig)
                          .follow(OUTTAKE_ID, true);

    outtakeFeederConfig = MotorControllerFactory.sparkConfig(OUTTAKE_FEEDER_MOTOR_CONFIG);
    outtakeFeederConfig.inverted(true)
                        .encoder.velocityConversionFactor(1.0/60/16);
  }
  /**
   * 
   * @param input RPM
   */
  public void setRPM(double input) {
    pidController.setSetpoint(input, ControlType.kVelocity);
  }

  public void spinOuttakeWithVoltage(double input) {
    pidController.setSetpoint(input, ControlType.kDutyCycle);
  }

  public void spinOuttakeFeeder(double feederSpeed){
    outtakeFeeder.set(feederSpeed);
  }

  public void stopOuttake(){
    pidController.setSetpoint(0, ControlType.kDutyCycle);
  }

  public boolean atGoal(double erorrMargin){
    return Math.abs(pidController.getSetpoint() - outtakeMasterEncoder.getVelocity()) < erorrMargin;
    }

  public boolean atVelGoal(double goal, double erorrMargin) {
    return Math.abs(goal - outtakeMasterEncoder.getVelocity()) < erorrMargin;
  }

  public double getOuttakeVelocity() {
    return outtakeMasterEncoder.getVelocity();
  }
  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("Shooter Input percentage", () -> outtakeMaster.getAppliedOutput(), null);
    builder.addDoubleProperty("Outtake master Velocity", () -> outtakeMasterEncoder.getVelocity(), null);
    builder.addDoubleProperty("Outtake master setpoint", () -> pidController.getSetpoint(), this::setRPM);

    builder.addDoubleProperty("Feeder Input percentage", () -> outtakeFeeder.getAppliedOutput(), this::spinOuttakeFeeder);
    builder.addDoubleProperty("Feeder Velocity (balls/second)", this::feedingSpeed, null);
  }

  //Theoritical balls per second
  private double feedingSpeed() {
    return outtakeFeederEncoder.getVelocity()/6;//6 flaps? Test this out could be useful
  }

  @Override
  public void periodic() {}
    // This method will be called once per scheduler run
}
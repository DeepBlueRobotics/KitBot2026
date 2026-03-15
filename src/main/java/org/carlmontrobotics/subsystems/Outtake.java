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
  /** Creates a new Outtake. */
  public Outtake() {

    configureMotors();
    outtakeMaster = MotorControllerFactory.createSpark(OUTTAKE_ID, OUTTAKE_MASTER_MOTOR_CONFIG, outtakeConfig);
    outtakeFollower = MotorControllerFactory.createSpark(OUTTAKE_FOLLOWER_ID, OUTTAKE_FOLLOWER_MOTOR_CONFIG, outtakeFollowerConfig);
    outtakeFeeder = MotorControllerFactory.createSpark(OUTTAKE_FEEDER_ID, OUTTAKE_FEEDER_MOTOR_CONFIG, outtakeFeederConfig);

    pidController = outtakeMaster.getClosedLoopController();

    outtakeMasterEncoder = outtakeMaster.getEncoder();

   }

  public void configureMotors(){
    outtakeConfig = MotorControllerFactory.sparkConfig(OUTTAKE_MASTER_MOTOR_CONFIG);
    outtakeConfig.idleMode(IdleMode.kCoast)
                                    .closedLoop.pid(kP, kI, kD);

    outtakeFollowerConfig = MotorControllerFactory.sparkConfig(OUTTAKE_FOLLOWER_MOTOR_CONFIG);
    outtakeFollowerConfig.apply(outtakeConfig)
                          .inverted(true)
                          .follow(OUTTAKE_ID);

    outtakeFeederConfig = MotorControllerFactory.sparkConfig(OUTTAKE_FEEDER_MOTOR_CONFIG);
  }

  public void spinOuttake(double input) {
    pidController.setSetpoint(input, ControlType.kVelocity);
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

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("Outtake master Speed perc", () -> outtakeMaster.getAppliedOutput(), null);
    builder.addDoubleProperty("Outtake master Velocity", () -> outtakeMasterEncoder.getVelocity(), null);
    builder.addDoubleProperty("Outtake master setpoint", () -> pidController.getSetpoint(), this::spinOuttake);

    builder.addDoubleProperty("Outtake Feeder Speed perc", () -> outtakeFeeder.getAppliedOutput(), this::spinOuttakeFeeder);
  }
  @Override
  public void periodic() {}
    // This method will be called once per scheduler run
}
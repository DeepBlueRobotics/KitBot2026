// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.Constants.OuttakeC;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {
  SparkFlex outtake;
  SparkFlex outtakeFollower;
  SparkFlex outtakeFeeder;
  private SparkFlexConfig outtakeFollowerConfig;
  private SparkFlexConfig outtakeConfig;
  private SparkFlexConfig outtakeFeederConfig;
  private double kP;
  private double kI;
  private double kD;
  private SparkClosedLoopController pidController;
  /** Creates a new Outtake. */
  public Outtake(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;

    outtake = MotorControllerFactory.createSparkFlex(OuttakeC.OUTTAKE_ID);
    outtakeFollower = MotorControllerFactory.createSparkFlex(OuttakeC.OUTTAKE_FOLLOWER_ID);
    outtakeFeeder = MotorControllerFactory.createSparkFlex(OuttakeC.OUTTAKE_FEEDER_ID);

    pidController = outtake.getClosedLoopController();

    outtakeConfig = new SparkFlexConfig();
    outtakeConfig.idleMode(IdleMode.kCoast);
    outtakeConfig.closedLoop.pid(kP,kI,kD).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    outtake.configure(outtakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    outtakeFollowerConfig = new SparkFlexConfig();
    outtakeFollowerConfig.apply(outtakeConfig)
                          .follow(OuttakeC.OUTTAKE_ID);
    outtakeFollowerConfig.inverted(true); // may need to be false and swap outtake to true
    outtakeFollower.configure(outtakeFeederConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    outtakeFeederConfig = new SparkFlexConfig();
    outtakeFeederConfig.idleMode(IdleMode.kCoast);
    outtakeFeeder.configure(outtakeFeederConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void spinOuttake(double input, double feederSpeed) {
    pidController.setSetpoint(input, ControlType.kVelocity);
    outtakeFeeder.set(feederSpeed);
  }

  @Override
  public void periodic() {}
    // This method will be called once per scheduler run
}
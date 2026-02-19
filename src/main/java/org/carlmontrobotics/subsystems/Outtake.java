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
import static org.carlmontrobotics.Constants.OuttakeC;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {
  SparkFlex outtakeMaster;
  SparkFlex outtakeFollower;
  SparkFlex outtakeFeeder;
  private SparkFlexConfig outtakeFollowerConfig;
  private SparkFlexConfig outtakeConfig;
  private SparkFlexConfig outtakeFeederConfig;
  private SparkClosedLoopController pidController;
  /** Creates a new Outtake. */
  public Outtake() {

    outtakeMaster = MotorControllerFactory.createSparkFlex(OuttakeC.OUTTAKE_ID);
    outtakeFollower = MotorControllerFactory.createSparkFlex(OuttakeC.OUTTAKE_FOLLOWER_ID);
    outtakeFeeder = MotorControllerFactory.createSparkFlex(OuttakeC.OUTTAKE_FEEDER_ID);

    pidController = outtakeMaster.getClosedLoopController();

    outtakeConfig = new SparkFlexConfig();
    outtakeConfig.idleMode(IdleMode.kCoast);
    outtakeConfig.closedLoop.pid(OuttakeC.kP, OuttakeC.kI, OuttakeC.kD);
    outtakeMaster.configure(outtakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    outtakeFollowerConfig = new SparkFlexConfig();
    outtakeFollowerConfig.apply(outtakeConfig)
                          .follow(OuttakeC.OUTTAKE_ID, true); // may need to be false and swap outtake to true
    outtakeFollower.configure(outtakeFeederConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    outtakeFeederConfig = new SparkFlexConfig();
    outtakeFeeder.configure(outtakeFeederConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void spinOuttake(double input) {
    pidController.setSetpoint(input, ControlType.kVelocity);
  }

  public void spinOuttakeFeeder(double feederSpeed){
    outtakeFeeder.set(feederSpeed);
  }

  public void stopOuttake(){
    pidController.setSetpoint(0, ControlType.kDutyCycle);
    outtakeFeeder.set(0);
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("Outtake Speed perc", () -> outtakeMaster.getAppliedOutput(), null);
  }
  @Override
  public void periodic() {}
    // This method will be called once per scheduler run
}
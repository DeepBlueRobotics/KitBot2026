// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;

import static org.carlmontrobotics.Constants.ConveyorC.*;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
  private SparkBase conveyorMotor;
  private RelativeEncoder conveyorEncoder;


  /** Creates a new Conveyor. */
  public Conveyor() {
    SparkBaseConfig conveyorConfig = MotorControllerFactory.sparkConfig(MotorConfig.NEO_VORTEX);
    conveyorConfig.inverted(true);
    conveyorMotor = MotorControllerFactory.createSpark(CONVEYOR_ID, MotorConfig.NEO_VORTEX, conveyorConfig);
    conveyorEncoder = conveyorMotor.getEncoder();

    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("Input percent", () -> conveyorMotor.get(), this::setThrottle);
    builder.addDoubleProperty("Velocity", this::getVelocity, null);
  }

  public void setThrottle(double conveyorSpeed) {
    conveyorMotor.set(conveyorSpeed);
  }

  public void stop(){
    conveyorMotor.set(0);
  }

  public double getVelocity() {
    return conveyorEncoder.getVelocity();
  }

}

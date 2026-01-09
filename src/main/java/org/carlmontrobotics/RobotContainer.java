// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;


//199 files
import org.carlmontrobotics.subsystems.*;


import org.carlmontrobotics.commands.DriveCommands.TeleopDrive;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

//auton
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.auto.NamedCommands;

//controllers
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

//commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//control bindings
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//constats
import org.carlmontrobotics.Constants.OI;
import org.carlmontrobotics.Constants.OI.Driver;
import org.carlmontrobotics.Constants.OI.Manipulator;
import org.carlmontrobotics.Constants.Drivetrainc.Autoc;

//smartdashboard/elastic
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class RobotContainer {
    
    public final GenericHID driverController = new GenericHID(Driver.port);
    public final XboxController driverRumble = new XboxController(Driver.port); //For rumbling the controller
    public final GenericHID manipulatorController = new GenericHID(Manipulator.port);

    public final Limelight limelight = new Limelight();
    public final Drivetrain drivetrain =  new Drivetrain(limelight);


    private SendableChooser<Command> autoChooser = new SendableChooser<>();   
    public boolean alignOverride = true;
    public boolean autoScoring = true;

    public RobotContainer() {
        //#region AutoRegistration
        RegisterAutoCommands();
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser); 

        SmartDashboard.putBoolean("AlignOverride", alignOverride);
        SmartDashboard.putBoolean("AutoScoring", autoScoring);
        SmartDashboard.putBoolean("AlignOverride", true);
        //#endregion
        setDefaultCommands();
        setBindingsDriver();
        setBindingsManipulator();

        SmartDashboard.putBoolean("Baby Mode", Config.CONFIG.isBabyMode());
        // SmartDashboard.putData("Rotate Command",new RotateToTag(drivetrain, limelight));
        SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());
        SmartDashboard.putString("Location", DriverStation.getLocation().toString());
        SmartDashboard.putBoolean("Connected to FMS?", DriverStation.isFMSAttached());
        SmartDashboard.putString("Station", DriverStation.getAlliance().toString() + " " + DriverStation.getLocation().toString());

    }
   
    //#region ButtonBindings
    private void setBindingsDriver() {
        new JoystickButton(driverController, Driver.resetFieldOrientationButton)
            .onTrue(new InstantCommand(drivetrain::resetFieldOrientation));
        axisTrigger(driverController, Driver.RIGHT_TRIGGER_BUTTON, 0.2)
            .onTrue(new InstantCommand(()->drivetrain.setFieldOriented(false)))
            .onFalse(new InstantCommand(()->drivetrain.setFieldOriented(true)));

        axisTrigger(driverController, Driver.LEFT_TRIGGER_BUTTON, 0.2)
            .onTrue(new InstantCommand(() -> drivetrain.setExtraSpeedMult(.5)))//normal max turn is .5
            .onFalse(new InstantCommand(() -> drivetrain.setExtraSpeedMult(0)));        
    }

    private void setBindingsManipulator() {}
    //#endregion
    //#region AutoMaking
    private void RegisterAutoCommands() {}

    private void RegisterCustomAutos(){}
    //#endregion
    //#region DefualtCommands
  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(new TeleopDrive(
      drivetrain,
      () -> ProcessedAxisValue(driverController, Axis.kLeftY),//.06 drift purple, .10 drift black
      () -> ProcessedAxisValue(driverController, Axis.kLeftX),
      () -> ProcessedAxisValue(driverController, Axis.kRightX),
      () -> driverController.getRawButton(OI.Driver.slowDriveButton),
      manipulatorController,
      () -> SmartDashboard.getBoolean("Baby Mode", Config.CONFIG.isBabyMode())
      ));
  }
  //#endregion
  //#region getAutoCommand
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  //#endregion
  //#region HelpfulMethods
  //TODO: integrate these methods into lib199
  /**
   * Flips an axis' Y coordinates upside down if the select axis is a joystick axis and applies the deadband value to the joystick axis
   * 
   * @param hid The controller/plane joystick the axis is on
   * @param axis The processed axis
   * @return The processed value.
   */
  private double getStickValue(GenericHID hid, Axis axis) {
    double deadbandVal = MathUtil.applyDeadband(hid.getRawAxis(axis.value), Constants.OI.JOY_THRESH);
    return deadbandVal * (axis == Axis.kLeftY || axis == Axis.kRightY ? -1 : 1);
  }

  /**
   * Processes an input from the joystick into a value between -1 and 1, sinusoidally instead of linearly
   * 
   * @param value The value to be processed.
   * @return The processed value.
   */
  private double inputProcessing(double value) {
    double processedInput;
    // processedInput =
    // (((1-Math.cos(value*Math.PI))/2)*((1-Math.cos(value*Math.PI))/2))*(value/Math.abs(value));
    processedInput = Math.copySign(((1 - Math.cos(value * Math.PI)) / 2) * ((1 - Math.cos(value * Math.PI)) / 2),
        value);
    return processedInput;
  }
  /**
   * Combines both getStickValue and inputProcessing into a single function for processing joystick outputs
   * 
   * @param hid The controller/plane joystick the axis is on
   * @param axis The processed axis
   * @return The processed value.
   */
  private double ProcessedAxisValue(GenericHID hid, Axis axis){
    return inputProcessing(getStickValue(hid, axis));
  }
  
  private Trigger axisTrigger(GenericHID controller, Axis axis, double threshold) {
    return new Trigger((BooleanSupplier)(() -> Math.abs(getStickValue(controller, axis)) > threshold));
  }
  //#endregion
}

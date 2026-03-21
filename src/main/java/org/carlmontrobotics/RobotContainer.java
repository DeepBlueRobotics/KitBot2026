// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;


//199 files
import org.carlmontrobotics.subsystems.*;
import org.carlmontrobotics.commands.AutonCommands.SimpleShootAuton;
import org.carlmontrobotics.commands.DriveCommands.TeleopDrive;
import org.carlmontrobotics.commands.ManipulatorCommands.EjectBalls;
import org.carlmontrobotics.commands.ManipulatorCommands.IntakeBalls;
import org.carlmontrobotics.commands.ManipulatorCommands.RunConveyor;
import org.carlmontrobotics.commands.ManipulatorCommands.ShootBalls;
import org.carlmontrobotics.commands.ManipulatorCommands.SmartShoot;
import org.carlmontrobotics.commands.ManipulatorCommands.OuttakeFeeder;

import static org.carlmontrobotics.Constants.OuttakeC.OUTTAKE_SHOOTING_RPM;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

//constants
import org.carlmontrobotics.Constants.OI;
import org.carlmontrobotics.Constants.OI.Driver;
import org.carlmontrobotics.Constants.OI.Manipulator;
import org.carlmontrobotics.Constants.OuttakeC;
import org.carlmontrobotics.Constants.Drivetrainc.Autoc;

//smartdashboard/elastic
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class RobotContainer implements Sendable {
    
    public final GenericHID driverController = new GenericHID(Driver.port);
    public final GenericHID manipulatorController = new GenericHID(Manipulator.port);

    public final Limelight limelight = new Limelight();
    public final Drivetrain drivetrain =  new Drivetrain(limelight);

    public final Intake intake = new Intake();
    public final Outtake outtake = new Outtake();


    private SendableChooser<Command> autoChooser = new SendableChooser<>();   
    public boolean autoScoring = true;
    public static int intakeCounter;
    private boolean assumeAutoWin = false;
    private String gameData;

    public RobotContainer() {

      

        //#region AutoRegistration
        RegisterAutoCommands();
        //
        //
        
        
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser); 

        SmartDashboard.putBoolean("AutoScoring", autoScoring);
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
        SmartDashboard.putData("Hub", this);

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

    private void setBindingsManipulator() {
      new JoystickButton(manipulatorController, Manipulator.INTAKE_CONVEYOR_BUTTON)
      .whileTrue(new RunConveyor(intake)); //could be toggle mode instead
   // .whileTrue(new OuttakeFeeder(outtake)); //could be toggle mode instead
     new JoystickButton(manipulatorController, Manipulator.INTAKE_BUTTON)
      .whileTrue(new IntakeBalls(intake, manipulatorController));
      new JoystickButton(manipulatorController, Manipulator.OUTTAKE_BUTTON)
      .whileTrue(new ShootBalls(outtake));
      axisTrigger(manipulatorController, Manipulator.SMART_SHOOT_CLOSE_AXIS, OI.JOY_THRESH)
      .whileTrue(new SmartShoot(outtake, intake, OUTTAKE_SHOOTING_RPM, false));
      axisTrigger(manipulatorController, Manipulator.SMART_SHOOT_FAR_AXIS, OI.JOY_THRESH)
      .whileTrue(new SmartShoot(outtake, intake, 7000, true));
      new JoystickButton(manipulatorController, Manipulator.REPEL_BALLS)
      .whileTrue(new EjectBalls(intake));
    }
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

      intake.setDefaultCommand(new IntakeBalls(intake, manipulatorController));
    }
  //#endregion
  //#region getAutoCommand
  public Command getAutonomousCommand() {
    return new SmartShoot(outtake, intake, OUTTAKE_SHOOTING_RPM, false);
  }
  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    //Hub is technically on during disabled period
    if (!DriverStation.isTeleopEnabled()) {
      return true;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    gameData = DriverStation.getGameSpecificMessage();
    boolean shift1Active;
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
    if (gameData.isEmpty()) {
      shift1Active = !assumeAutoWin;
    }
    else {
      boolean redInactiveFirst = false;
      switch (gameData.charAt(0)) {
        case 'R' -> {
          redInactiveFirst = true;
          // Shift was is active for blue if red won auto, or red if blue won auto.
          shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
          };
        }
        case 'B' -> {
          redInactiveFirst = false; 
          // Shift was is active for blue if red won auto, or red if blue won auto.
          shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
          };
        }
        default -> {
          // If we have invalid game data, assume hub is active.
          shift1Active = !assumeAutoWin;
        }
      }
    }

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } 
    else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } 
    else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } 
    else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } 
    else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } 
    else {
      // End game, hub always active.
      return true;
    }
  }
  public double hubTimeLeft(){ //Gets time left until hub is active/inactive
    double matchTime = DriverStation.getMatchTime();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return -1;
    }
    if (DriverStation.isAutonomous()) {
      return assumeAutoWin ? matchTime + 10 : matchTime + 35;
    }
    gameData = DriverStation.getGameSpecificMessage();
    boolean shift1Active;
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
    if (gameData.isEmpty()) {
      shift1Active = !assumeAutoWin;
    }
    else {
      boolean redInactiveFirst = false;
      switch (gameData.charAt(0)) {
        case 'R' -> {
          redInactiveFirst = true;
          // Shift was is active for blue if red won auto, or red if blue won auto.
          shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
          };
        }
        case 'B' -> {
          redInactiveFirst = false; 
          // Shift was is active for blue if red won auto, or red if blue won auto.
          shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
          };
        }
        default -> {
          // If we have invalid game data, assume hub is active.
          shift1Active = !assumeAutoWin;
        }
      }
    }
    if (matchTime > 130) {
      // Transition shift, hub is active.
      return shift1Active ? matchTime - 105 : matchTime - 130;
    } 
    else if (matchTime > 105) {
      // Shift 1
      return matchTime - 105;
    } 
    else if (matchTime > 80) {
      // Shift 2
      return matchTime - 80;
    } 
    else if (matchTime > 55) {
      // Shift 3
      return matchTime - 55;
    } 
    else if (matchTime > 30) {
      // Shift 4
      return shift1Active ? matchTime - 30 : matchTime;
    } 
    else {
      // End game, hub always active.
      return matchTime;
    }
  }

  public boolean getAssumeAutoWin() {
    return assumeAutoWin;
  }

  public void setAssumeAutoWin(boolean assumption) {
    assumeAutoWin = assumption;
  }

  private int shiftNumber() {
    if (DriverStation.isAutonomous()) {
      return 0;
    }
    if (DriverStation.isTeleop()) {
      double matchTime = DriverStation.getMatchTime();
      if (matchTime > 130) {
      // Transition shift
      return 0;
      } 
      else if (matchTime > 105) {
        // Shift 1
        return 1;
      } 
      else if (matchTime > 80) {
        // Shift 2
        return 2;
      } 
      else if (matchTime > 55) {
        // Shift 3
        return 3;
      } 
      else if (matchTime > 30) {
        // Shift 4
        return 4;
      } 
      else {
        // End game
        return 5;
      }
    }
    return 0;
  }

  @Override
  public void initSendable(SendableBuilder builder){
    builder.addBooleanProperty("Hub Active (T/F)", this::isHubActive, null);
    builder.addDoubleProperty("Hub Time Left", this::hubTimeLeft, null);
    builder.addBooleanProperty("Assume Won Auto", this::getAssumeAutoWin, this::setAssumeAutoWin);
    builder.addDoubleProperty("Active shift", this::shiftNumber, null);
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
  
  private Trigger   axisTrigger(GenericHID controller, Axis axis, double threshold) {
    return new Trigger((BooleanSupplier)(() -> Math.abs(getStickValue(controller, axis)) > threshold));
  }
  //#endregion
}

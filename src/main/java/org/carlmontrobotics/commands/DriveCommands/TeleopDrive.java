package org.carlmontrobotics.commands.DriveCommands;

import static org.carlmontrobotics.Constants.Drivetrainc.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopDrive extends Command {

  //private static double robotPeriod = Robot.kDefaultPeriod;
  private final Drivetrain drivetrain;
  private DoubleSupplier fwd;
  private DoubleSupplier str;
  private DoubleSupplier rcw;
  private BooleanSupplier slow;
  private double currentForwardVel = 0;
  private double currentStrafeVel = 0;
  //private double prevTimestamp;
  GenericHID manipulatorController;
  BooleanSupplier babyModeSupplier;

  /**
   * Creates a new TeleopDrive.
   */
  public TeleopDrive(Drivetrain drivetrain, DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rcw,
      BooleanSupplier slow, GenericHID manipulatorController, BooleanSupplier babyModeSupplier) {
    addRequirements(this.drivetrain = drivetrain);
    this.fwd = fwd;
    this.str = str;
    this.rcw = rcw;
    this.slow = slow;
    this.manipulatorController = manipulatorController;
    this.babyModeSupplier = babyModeSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //prevTimestamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double currentTime = Timer.getFPGATimestamp();
    //robotPeriod = currentTime - prevTimestamp;
    if (!hasDriverInput()) {
      drivetrain.drive(0,0,0);
    }
    else {
      double[] speeds = getRequestedSpeeds();
      //prevTimestamp = currentTime;
      drivetrain.drive(speeds[0], speeds[1], speeds[2]);
    }
  }

  public double[] getRequestedSpeeds() {
    // Sets all values less than or equal to a very small value (determined by the
    // idle joystick state) to zero.
    // Used to make sure that the robot does not try to change its angle unless it
    // is moving,
    double forward = fwd.getAsDouble();
    double strafe = str.getAsDouble();
    double rotateClockwise = rcw.getAsDouble();
    forward *= maxForward;
    strafe *= maxStrafe;
    rotateClockwise *= maxRCW;

    double driveMultiplier = (slow.getAsBoolean() ? kSlowDriveSpeed : kNormalDriveSpeed);
    double rotationMultiplier = drivetrain.extraSpeedMult + (slow.getAsBoolean() ? kSlowDriveRotation : kNormalDriveRotation);
    
    if(babyModeSupplier.getAsBoolean()){
      driveMultiplier = kBabyDriveSpeed; 
      rotationMultiplier = kBabyDriveRotation;
    }

    forward *= driveMultiplier;
    strafe *= driveMultiplier;
    rotateClockwise *= rotationMultiplier;

    // Limit acceleration of the robot
    // double accelerationX = (forward - currentForwardVel) / robotPeriod;
    // double accelerationY = (strafe - currentStrafeVel) / robotPeriod;
    // double translationalAcceleration = Math.hypot(accelerationX, accelerationY);
    // SmartDashboard.putNumber("Translational Acceleration", translationalAcceleration);
    // if (translationalAcceleration > autoMaxAccelMps2 && false) {//DOES NOT RUN!!
    //   Translation2d limitedAccelerationVector = new Translation2d(autoMaxAccelMps2,
    //       Rotation2d.fromRadians(Math.atan2(accelerationY, accelerationX)));
    //   Translation2d limitedVelocityVector = limitedAccelerationVector.times(robotPeriod);
    //   currentForwardVel += limitedVelocityVector.getX();
    //   currentStrafeVel += limitedVelocityVector.getY();
    // } else {
    currentForwardVel = forward;
    currentStrafeVel = strafe;
    // }
    // SmartDashboard.putNumber("current velocity", Math.hypot(currentForwardVel, currentStrafeVel));

    return new double[] { currentForwardVel, currentStrafeVel, -rotateClockwise };
  }

  public boolean hasDriverInput() {
    return MathUtil.applyDeadband(fwd.getAsDouble(), Constants.OI.JOY_THRESH)!=0
        || MathUtil.applyDeadband(str.getAsDouble(), Constants.OI.JOY_THRESH)!=0
        || MathUtil.applyDeadband(rcw.getAsDouble(), Constants.OI.JOY_THRESH)!=0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
package org.carlmontrobotics.commands.DriveCommands;

import static org.carlmontrobotics.Constants.Drivetrainc.kBabyDriveRotation;
import static org.carlmontrobotics.Constants.Drivetrainc.kBabyDriveSpeed;
import static org.carlmontrobotics.Constants.Drivetrainc.kNormalDriveRotation;
import static org.carlmontrobotics.Constants.Drivetrainc.kNormalDriveSpeed;
import static org.carlmontrobotics.Constants.Drivetrainc.kSlowDriveRotation;
import static org.carlmontrobotics.Constants.Drivetrainc.kSlowDriveSpeed;
import static org.carlmontrobotics.Constants.Drivetrainc.maxForward;
import static org.carlmontrobotics.Constants.Drivetrainc.maxRCW;
import static org.carlmontrobotics.Constants.Drivetrainc.maxStrafe;
import static org.carlmontrobotics.Constants.ShootOnFlyc.*;
import static org.carlmontrobotics.Constants.OuttakeC.OUTTAKE_FEEDER_VOLT_PERC;


import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.Robot;
import org.carlmontrobotics.ShootOnFlyLib.HeadingAlignController;
import org.carlmontrobotics.ShootOnFlyLib.HexClosest;
import org.carlmontrobotics.ShootOnFlyLib.ShotCalculator;
import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Outtake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class BetterTeleopDrive extends Command {

  private static double robotPeriod = Robot.kDefaultPeriod;
  private final Drivetrain dt;
  private final Outtake outtake;
  private DoubleSupplier fwd;
  private DoubleSupplier str;
  private DoubleSupplier rcw;
  private BooleanSupplier slow;
  private double currentForwardVel = 0;
  private double currentStrafeVel = 0;
  private double prevTimestamp;
  private BooleanSupplier babyModeSupplier;
  private BooleanSupplier shootOnFly;

  private boolean isRed = false; 

  /**
   * Creates a new TeleopDrive.
   */
  public BetterTeleopDrive(Drivetrain drivetrain, Outtake outtake, DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rcw,
      BooleanSupplier slow, BooleanSupplier babyModeSupplier, BooleanSupplier shootOnFly) {
    addRequirements(dt = drivetrain, this.outtake = outtake);
    this.fwd = fwd;
    this.str = str;
    this.rcw = rcw;
    this.slow = slow;
    this.babyModeSupplier = babyModeSupplier;
    this.shootOnFly = shootOnFly;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putNumber("slow turn const", kSlowDriveRotation);
    // SmartDashboard.putNumber("slow speed const", kSlowDriveSpeed);
    // SmartDashboard.putNumber("normal turn const", kNormalDriveRotation);
    // SmartDashboard.putNumber("normal speed const", kNormalDriveSpeed);
    prevTimestamp = Timer.getFPGATimestamp();
    isRed = DriverStation.getAlliance()
      .map(a -> a == DriverStation.Alliance.Red)
      .orElse(false);   // default when unknown
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp();
    robotPeriod = currentTime - prevTimestamp;
    double[] speeds = getRequestedSpeeds();
    prevTimestamp = currentTime;
    if (shootOnFly.getAsBoolean()) {
      Pose2d currentPose2d = dt.getPose();
      double rotation = calculateRotationToAlign(currentPose2d);
      boolean alignedForShot = calculateAlignmentValid(currentPose2d);
      if (alignedForShot) {
        ShotCalculator.ShotResult shotResult = calculateShotVelocity();
        if (shotResult.impossible) {
          outtake.spinOuttake(passiveVelocity);
          outtake.spinOuttakeFeeder(0);
          dt.drive(speeds[0], speeds[1], rotation);     
        }
        else {
          double shotVelocity = shotResult.requiredRPM;
          outtake.spinOuttake(shotVelocity);
          outtake.spinOuttakeFeeder(OUTTAKE_FEEDER_VOLT_PERC);
        dt.drive(speeds[0], speeds[1], rotation);     
        } 
      }
      else {
        outtake.spinOuttake(passiveVelocity);
        outtake.spinOuttakeFeeder(0);
        dt.drive(speeds[0], speeds[1], rotation);     
      }
    }
    else {
      outtake.stopOuttake();
      outtake.spinOuttakeFeeder(0);
      dt.drive(speeds[0], speeds[1], speeds[2]);
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
    //boolean slow2 = slow.getAsBoolean(); gets not used ig
    forward *= maxForward;
    strafe *= maxStrafe;
    rotateClockwise *= maxRCW;
    double driveMultiplier = (slow.getAsBoolean() ? kSlowDriveSpeed : kNormalDriveSpeed);
    double rotationMultiplier = dt.extraSpeedMult + (slow.getAsBoolean() ? kSlowDriveRotation : kNormalDriveRotation);
    if(babyModeSupplier.getAsBoolean()){
      driveMultiplier = kBabyDriveSpeed; 
      rotationMultiplier = kBabyDriveRotation;
    }
    forward *= driveMultiplier;
    strafe *= driveMultiplier;
    rotateClockwise *= rotationMultiplier;
    currentForwardVel = forward;
    currentStrafeVel = strafe;
    return new double[] { currentForwardVel, currentStrafeVel, -rotateClockwise };
  }

  public boolean hasDriverInput() {
    return MathUtil.applyDeadband(fwd.getAsDouble(), Constants.OI.JOY_THRESH)!=0
        || MathUtil.applyDeadband(str.getAsDouble(), Constants.OI.JOY_THRESH)!=0
        || MathUtil.applyDeadband(rcw.getAsDouble(), Constants.OI.JOY_THRESH)!=0;
  }


  private double calculateRotationToAlign(Pose2d currentPose2d) {
    if (isRed) {
        return HeadingAlignController.calculateOmega(currentPose2d, centerOfRedGoal2d);
    }
    else {
        return HeadingAlignController.calculateOmega(currentPose2d, centerOfBlueGoal2d);
    }
  }

  private boolean calculateAlignmentValid(Pose2d currentPose2d) {
    if (isRed) {
        return HeadingAlignController.atGoal(currentPose2d, centerOfRedGoal2d);
    }
    else {
        return HeadingAlignController.atGoal(currentPose2d, centerOfBlueGoal2d);
    }
  }

  private ShotCalculator.ShotResult calculateShotVelocity() {
    Pose3d goalPose = isRed ? new Pose3d(11.916, 4.038, 1.8237877672, new Rotation3d()) : new Pose3d(4.618, 4.038, 1.8237877672, new Rotation3d());
    Pose2d current2dPose = dt.getPose();
    Pose3d current3dPose = new Pose3d(current2dPose);
    double[] velocityVectors = dt.getDrivetrainVelocity();
    double shooterVelocity = outtake.getVelocity();
    velocityVectors[0] += shooterVelocity * Math.cos(current2dPose.getRotation().getRadians());
    velocityVectors[1] += shooterVelocity * Math.sin(current2dPose.getRotation().getRadians());
    Translation3d path = goalPose.getTranslation().minus(current3dPose.getTranslation());
    Translation2d closestObstacle = HexClosest.closestVectorToHex(current2dPose, isRed);
    double[] obstacleDistances = {closestObstacle.getX(), closestObstacle.getY()};
    double[] obstacleHeights = {Units.inchesToMeters(72), Units.inchesToMeters(72)};
    ShotCalculator.ShotResult speed = ShotCalculator.calculateShot(path.getX(), path.getY(), path.getZ(), velocityVectors[0], velocityVectors[1], obstacleDistances, obstacleHeights);
    
    return speed;
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
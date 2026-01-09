
package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.LimeLightc.*;

import org.carlmontrobotics.lib199.vendorLibs.LimelightHelpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  

  // NEEDS TO SEE: Barge, Reef, Processor, Coral Dropoff
  public Limelight() {
    
    LimelightHelpers.SetFiducialIDFiltersOverride(sampleLL2, sampleLL2_VALID_IDS);
    LimelightHelpers.SetFiducialIDFiltersOverride(sampleLL1, sampleLL1_VALID_IDS);

    // SmartDashboard.putBoolean("sees coral station tag", seesTag(CORAL_LL));
    SmartDashboard.putBoolean("sees reef tag", seesTag(sampleLL1));
    // SmartDashboard.putNumber("distance to coral", getDistanceToApriltag(CORAL_LL, CORAL_MOUNT_ANGLE, Apriltagc.CORAL_HEIGHT_METERS, CORAL_LL_HEIGHT_FROM_GROUND_METERS));
    // SmartDashboard.putNumber("distance to reef", getDistanceToApriltag(REEF_LL, REEF_MOUNT_ANGLE, Apriltagc.REEF_HEIGHT_METERS, REEF_LL_HEIGHT_FROM_GROUND_METERS));
    SmartDashboard.putNumber("distance to reef mt2", getDistanceToApriltagMT2(sampleLL1));

    SmartDashboard.putNumber("strafe left", 0);
    SmartDashboard.putNumber("strafe right", 0);

    SmartDashboard.putNumber("limelight strafing kp", 1);


    SmartDashboard.putNumber("limelight forward kp", 6);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("sees coral station tag", seesTag(CORAL_LL));
    SmartDashboard.putBoolean("sees reef tag", seesTag(sampleLL1));
    // SmartDashboard.putNumber("distance to coral", getDistanceToApriltag(CORAL_LL, CORAL_MOUNT_ANGLE, Apriltagc.CORAL_HEIGHT_METERS, CORAL_LL_HEIGHT_FROM_GROUND_METERS));
    // SmartDashboard.putNumber("distance to reef", getDistanceToApriltag(REEF_LL, REEF_MOUNT_ANGLE, Apriltagc.REEF_HEIGHT_METERS, REEF_LL_HEIGHT_FROM_GROUND_METERS));
    SmartDashboard.putNumber("distance to reef megatag2", getDistanceToApriltagMT2(sampleLL1));

  }
  public static double getThor(String limelightName) {
      return LimelightHelpers.getT2DArray(limelightName)[15];
  }
  
  public static double getTvert(String limelightName) {
      return LimelightHelpers.getT2DArray(limelightName)[16];
  }

  public double getTYDeg(String limelightName) {
    return LimelightHelpers.getTY(limelightName);
  }

  // Distance accessors for field areas
  public double getDistanceToApriltag(String limelightName, double mountAngle, double tagHeight, double limelightHeight) {
    if (!seesTag(limelightName)) {
      return -1;
    }
    else {
      Rotation2d angleToGoal = Rotation2d.fromDegrees(mountAngle).plus(Rotation2d.fromDegrees(getTYDeg(limelightName)));
      double distance = (tagHeight - limelightHeight) / angleToGoal.getTan();
      return distance;
    }
  }

  public double getDistanceToApriltagMT2(String limelightName) { 
    Pose3d targetPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(limelightName);

    double x = targetPoseRobotSpace.getX();
    double z = targetPoseRobotSpace.getZ();

    return Math.hypot(x, z);
  }

  public double getDistanceToApriltag3D(String limelightName){ //This method is for if you need the actual distance to the tag(with height considered)
    Pose3d targetPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(limelightName);

    double x = targetPoseRobotSpace.getX();
    double z = targetPoseRobotSpace.getZ();
    double y = targetPoseRobotSpace.getY();

    return Math.sqrt(x * x + z * z + y * y); //calculates the actual 3D distance (with height component included) 
  }

  public double getAprilWidth (String name) {
    return getThor(name);
  }

  public double getAprilHeight (String name) {
    return getTvert(name);
  }

  public double getTX(String name) {
    return LimelightHelpers.getTX(name);
  }

  //TODO: TEST WHICH ONE IS MORE ACCURATE

  public double getRotateAngleRadMT2(String limelightName) {
    Pose3d targetPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(limelightName); // pose of the target
    
    double targetX = targetPoseRobotSpace.getX(); // the forward offset between the center of the
    // robot and target
    double targetZ = -targetPoseRobotSpace.getZ(); // the sideways offset

    double targetOffsetRads = MathUtil.inputModulus(Math.atan2(targetX, targetZ), -Math.PI, Math.PI);

    return targetOffsetRads;
  }

  

  public Pose2d getRobotPoseInField(String limelightName) {
    Pose2d robotPosFieldSpace = LimelightHelpers.getBotPose2d(limelightName);
    return robotPosFieldSpace;
  }


  public boolean seesTag(String limelightName) {
    return LimelightHelpers.getTV(limelightName);
  }


  public boolean seesTagId(int redId, int blueId) {
    if (seesTag(sampleLL1)) {
      if ((int) LimelightHelpers.getFiducialID(sampleLL1) == redId || (int) LimelightHelpers.getFiducialID(sampleLL1) == blueId) {
        return true;
      }
      else {
        return false;
      }
    }
    else {
      return false;
    }
  }
}
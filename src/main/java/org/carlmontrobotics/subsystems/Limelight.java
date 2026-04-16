
package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.LimeLightc.*;

import org.carlmontrobotics.lib199.vendorLibs.LimelightHelpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  

  // NEEDS TO SEE: Barge, Reef, Processor, Coral Dropoff
  public Limelight() {
    
    //LimelightHelpers.SetFiducialIDFiltersOverride(LEFT_LL, LL_GENERAL_VALID_IDS);
    LimelightHelpers.SetFiducialIDFiltersOverride(RIGHT_LL, LL_GENERAL_VALID_IDS);

  }

  @Override
  public void periodic() {

  }
  /**
   * Allows for smartShooting
   * @param shootingCrop boolean to use shooting crop or general
   */
  public void setCrop(boolean shootingCrop) {
    if (shootingCrop) {
      LimelightHelpers.setCropWindow(RIGHT_LL, LL_FRONT_SHOOTING_CROP[0], LL_FRONT_SHOOTING_CROP[1], LL_FRONT_SHOOTING_CROP[2], LL_FRONT_SHOOTING_CROP[3]);
      //LimelightHelpers.setCropWindow(LEFT_LL, LL_BACK_SHOOTING_CROP[0], LL_BACK_SHOOTING_CROP[1], LL_BACK_SHOOTING_CROP[2], LL_BACK_SHOOTING_CROP[3]);
    }
    else {
      LimelightHelpers.setCropWindow(RIGHT_LL, LL_FRONT_GENERAL_CROP[0], LL_FRONT_GENERAL_CROP[1], LL_FRONT_GENERAL_CROP[2], LL_FRONT_GENERAL_CROP[3]);
     //LimelightHelpers.setCropWindow(BACK_LL, LL_BACK_GENERAL_CROP[0], LL_BACK_GENERAL_CROP[1], LL_BACK_GENERAL_CROP[2], LL_BACK_GENERAL_CROP[3]);
    }
  }

  /**
   * Allows for faster speeds at certain times
   * @param redAlliance which alliance are you on
   * @param shootingFilter shooting or not
   */
  public void setIDFilters(boolean redAlliance, boolean shootingFilter) {
    if (redAlliance) {
      if (shootingFilter) {
        int[] frontFilter = difference(LL_FRONT_SHOOTING_VALID_IDS, LL_IDS_IGNORE_FOR_RED);
        //int[] backFilter = difference(LL_BACK_SHOOTING_VALID_IDS, LL_IDS_IGNORE_FOR_RED);
        LimelightHelpers.SetFiducialIDFiltersOverride(RIGHT_LL, frontFilter);
        //LimelightHelpers.SetFiducialIDFiltersOverride(LEFT_LL, backFilter);
      }
      else {
        int[] filter = difference(LL_GENERAL_VALID_IDS, LL_IDS_IGNORE_FOR_RED);
        LimelightHelpers.SetFiducialIDFiltersOverride(RIGHT_LL, filter);
        //LimelightHelpers.SetFiducialIDFiltersOverride(LEFT_LL, filter);
      }
    }
    else {
      if (shootingFilter) {
        int[] frontFilter = difference(LL_FRONT_SHOOTING_VALID_IDS, LL_IDS_IGNORE_FOR_BLUE);
        //int[] backFilter = difference(LL_BACK_SHOOTING_VALID_IDS, LL_IDS_IGNORE_FOR_BLUE);
        LimelightHelpers.SetFiducialIDFiltersOverride(RIGHT_LL, frontFilter);
        //LimelightHelpers.SetFiducialIDFiltersOverride(LEFT_LL, backFilter);
      }
      else {
        int[] filter = difference(LL_GENERAL_VALID_IDS, LL_IDS_IGNORE_FOR_BLUE);
        LimelightHelpers.SetFiducialIDFiltersOverride(RIGHT_LL, filter);
       // LimelightHelpers.SetFiducialIDFiltersOverride(LEFT_LL, filter);
      }
    }
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

  /**
   * Finds all the ints that are in the first array that are not in the second
   * @param first owner array
   * @param second subtractor
   * @return
   */
  public static int[] difference(int[] first, int[] second) {
    int[] temp = new int[first.length];
    int count = 0;

    for (int i = 0; i < first.length; i++) {
        boolean found = false;

        for (int j = 0; j < second.length; j++) {
            if (first[i] == second[j]) {
                found = true;
                break;
            }
        }

        if (!found) {
            temp[count] = first[i];
            count++;
        }
    }

    int[] result = new int[count];
    for (int i = 0; i < count; i++) {
        result[i] = temp[i];
    }

    return result;
}
}
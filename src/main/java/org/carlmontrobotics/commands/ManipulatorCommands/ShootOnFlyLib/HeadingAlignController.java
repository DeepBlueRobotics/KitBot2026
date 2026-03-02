package org.carlmontrobotics.commands.ManipulatorCommands.ShootOnFlyLib;

import static org.carlmontrobotics.Constants.ShootOnFlyc.*;

import edu.wpi.first.math.geometry.Pose2d;

public class HeadingAlignController {

    /**
     * Returns rotational speed (rad/s scaled by kP) needed to face target.
     *
     * @param currentPose robot pose (with rotation)
     * @param targetPose pose representing the point to face (rotation ignored)
     */
    public static double calculateOmega(Pose2d currentPose, Pose2d targetPose) {
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();

        double desiredAngle = Math.atan2(dy, dx);
        double currentAngle = currentPose.getRotation().getRadians();

        double error = wrapAngle(desiredAngle - currentAngle);

        return thetaAlignP * error;
    }

    /**
     * Returns true if robot is aligned within tolerance.
     */
    public static boolean atGoal(Pose2d currentPose, Pose2d targetPose) {
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();

        double desiredAngle = Math.atan2(dy, dx);
        double currentAngle = currentPose.getRotation().getRadians();

        double error = wrapAngle(desiredAngle - currentAngle);

        return Math.abs(error) <= toleranceRad;
    }

    /**
     * Wrap angle to [-pi, pi]
     */
    private static double wrapAngle(double angle) {
        angle %= Math.PI;
        if (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        return angle;
    }
}
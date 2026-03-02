package org.carlmontrobotics.commands.ManipulatorCommands.ShootOnFlyLib;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

public class HexClosest {
    private static final Pose2d[] hexVerticesBlue = {
        new Pose2d(Units.inchesToMeters(155.491863), Units.inchesToMeters(159.249827), new Rotation2d()), 
        new Pose2d(Units.inchesToMeters(169.870573), Units.inchesToMeters(180.359655), new Rotation2d()),
        new Pose2d(Units.inchesToMeters(193.838106), Units.inchesToMeters(180.228677), new Rotation2d()),
        new Pose2d(Units.inchesToMeters(205.837629), Units.inchesToMeters(159.444893), new Rotation2d()),
        new Pose2d(Units.inchesToMeters(193.967292), Units.inchesToMeters(138.622912), new Rotation2d()),
        new Pose2d(Units.inchesToMeters(169.661894), Units.inchesToMeters(138.558823), new Rotation2d())};
    private static final Pose2d[] hexVerticesRed = {
        new Pose2d(Units.inchesToMeters(443.907204), Units.inchesToMeters(159.093750), new Rotation2d()), 
        new Pose2d(Units.inchesToMeters(455.906727), Units.inchesToMeters(179.979820), new Rotation2d()),
        new Pose2d(Units.inchesToMeters(480.123542), Units.inchesToMeters(179.864588), new Rotation2d()),
        new Pose2d(Units.inchesToMeters(491.993879), Units.inchesToMeters(159.042607), new Rotation2d()),
        new Pose2d(Units.inchesToMeters(480.123542), Units.inchesToMeters(138.322912), new Rotation2d()),
        new Pose2d(Units.inchesToMeters(456.026823), Units.inchesToMeters(138.230132), new Rotation2d())};
    /**
     * Returns vector (dx, dy) from robot to closest point on hex boundary.
     * @param robot provides position of robot and direction
     * @param isRed is a boolean determining which hub is the user going for
     */
    public static Translation2d closestVectorToHex(Pose2d robot, boolean isRed) {
        Pose2d[] hexVertices;
        if (isRed) {
            hexVertices = hexVerticesRed;
        }
        else {
            hexVertices = hexVerticesBlue;
        }
        Translation2d p = robot.getTranslation();

        Translation2d bestPoint = null;
        double bestDistSq = Double.POSITIVE_INFINITY;

        for (int i = 0; i < 6; i++) {
            Translation2d a = hexVertices[i].getTranslation();
            Translation2d b = hexVertices[(i + 1) % 6].getTranslation();

            // ---- closest point on segment a→b ----
            Translation2d ab = b.minus(a);
            Translation2d ap = p.minus(a);

            double abLenSq = ab.getX()*ab.getX() + ab.getY()*ab.getY();
            Translation2d candidate;

            if (abLenSq == 0) {
                candidate = a;
            } else {
                double t = (ap.getX()*ab.getX() + ap.getY()*ab.getY()) / abLenSq;
                t = Math.max(0, Math.min(1, t));

                candidate = new Translation2d(
                        a.getX() + ab.getX() * t,
                        a.getY() + ab.getY() * t
                );
            }
            // --------------------------------------

            double dx = candidate.getX() - p.getX();
            double dy = candidate.getY() - p.getY();
            double distSq = dx*dx + dy*dy;

            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                bestPoint = candidate;
            }
        }

        return bestPoint.minus(p); // (x, y) components to hex
    }
}

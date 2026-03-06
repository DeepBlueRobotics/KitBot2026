package org.carlmontrobotics.ShootOnFlyLib;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import org.junit.jupiter.api.Test;

/**
 * JUnit 5 tests for {@link HexClosest}.
 *
 * <p>Blue hub approximate center: x ≈ 180 in, y ≈ 159 in
 * <br>Red hub approximate center: x ≈ 468 in, y ≈ 159 in
 * <br>Hex apothem (center to nearest edge): ≈ 18 in / 0.457 m
 */
class HexClosestTest {

    // ------------------------------------------------------------------
    // Directional sanity checks
    // ------------------------------------------------------------------

    /**
     * Robot far to the left of the blue hub → closest boundary point is to its
     * right, so the returned vector must have a positive X component.
     */
    @Test
    void blue_robotLeftOfHub_vectorPointsRight() {
        Pose2d robot = new Pose2d(0, Units.inchesToMeters(160), new Rotation2d());
        Translation2d vec = HexClosest.closestVectorToHex(robot, false);

        assertTrue(vec.getX() > 0, "Vector X should point right (+) toward blue hub");
    }

    /**
     * Robot far to the right of the red hub → closest boundary point is to its
     * left, so the returned vector must have a negative X component.
     */
    @Test
    void red_robotRightOfHub_vectorPointsLeft() {
        Pose2d robot = new Pose2d(Units.inchesToMeters(700), Units.inchesToMeters(160), new Rotation2d());
        Translation2d vec = HexClosest.closestVectorToHex(robot, true);

        assertTrue(vec.getX() < 0, "Vector X should point left (-) toward red hub");
    }

    /**
     * Robot directly below the blue hub center → closest boundary point is above
     * it, so the returned vector must have a positive Y component.
     */
    @Test
    void blue_robotBelowHub_vectorPointsUp() {
        Pose2d robot = new Pose2d(Units.inchesToMeters(180), 0, new Rotation2d());
        Translation2d vec = HexClosest.closestVectorToHex(robot, false);

        assertTrue(vec.getY() > 0, "Vector Y should point up (+) when robot is below hub");
    }

    // ------------------------------------------------------------------
    // Magnitude checks
    // ------------------------------------------------------------------

    /**
     * The returned vector must be non-zero for any robot not sitting exactly
     * on the hex boundary.
     */
    @Test
    void vectorNonZeroForDistantRobot() {
        Pose2d robot = new Pose2d(0, 0, new Rotation2d());
        Translation2d vec = HexClosest.closestVectorToHex(robot, false);

        double len = Math.hypot(vec.getX(), vec.getY());
        assertTrue(len > 1e-6, "Vector length must be > 0 for a robot not on the boundary");
    }

    /**
     * From the approximate center of the blue hub the vector magnitude should
     * equal the hub's apothem (≈ 0.457 m) within 5 cm, since the closest
     * boundary point is on the nearest edge directly outward from center.
     */
    @Test
    void blue_fromCenter_vectorLengthMatchesApothem() {
        Pose2d robot = new Pose2d(
            Units.inchesToMeters(180.0),
            Units.inchesToMeters(159.0),
            new Rotation2d()
        );

        Translation2d vec = HexClosest.closestVectorToHex(robot, false);
        double len = Math.hypot(vec.getX(), vec.getY());

        assertEquals(0.457, len, 0.05, "Vector length from hub center should ~= apothem (0.457 m)");
    }

    // ------------------------------------------------------------------
    // Blue vs. red distinction
    // ------------------------------------------------------------------

    /**
     * Querying from the same midfield position must yield different vectors for
     * blue vs. red, since the two hubs are on opposite sides of the field.
     */
    @Test
    void blueAndRedDifferForSameRobot() {
        Pose2d robot = new Pose2d(
            Units.inchesToMeters(300),
            Units.inchesToMeters(160),
            new Rotation2d()
        );

        Translation2d blue = HexClosest.closestVectorToHex(robot, false);
        Translation2d red  = HexClosest.closestVectorToHex(robot, true);

        assertNotEquals(blue.getX(), red.getX(), 1e-3,
            "Blue and red vectors must have different X components from midfield");
    }

    /**
     * From midfield, the blue vector should point left (-X) toward the blue hub
     * and the red vector should point right (+X) toward the red hub.
     */
    @Test
    void blue_midfieldVectorPointsLeft_red_midfieldVectorPointsRight() {
        Pose2d robot = new Pose2d(
            Units.inchesToMeters(323),
            Units.inchesToMeters(159),
            new Rotation2d()
        );

        Translation2d blue = HexClosest.closestVectorToHex(robot, false);
        Translation2d red  = HexClosest.closestVectorToHex(robot, true);

        assertTrue(blue.getX() < 0, "Blue vector should point left (-X) from midfield");
        assertTrue(red.getX()  > 0, "Red vector should point right (+X) from midfield");
    }

    // ------------------------------------------------------------------
    // Symmetry
    // ------------------------------------------------------------------

    /**
     * Robots placed symmetrically above and below the hub's horizontal midline
     * should be equidistant from the hex boundary, since the hex is symmetric
     * about that line.
     */
    @Test
    void blue_symmetricRobots_equalVectorMagnitudes() {
        double centerX = Units.inchesToMeters(180.0);
        double centerY = Units.inchesToMeters(159.0);
        double offset  = Units.inchesToMeters(50.0);

        Pose2d above = new Pose2d(centerX, centerY + offset, new Rotation2d());
        Pose2d below = new Pose2d(centerX, centerY - offset, new Rotation2d());

        double lenAbove = Math.hypot(
            HexClosest.closestVectorToHex(above, false).getX(),
            HexClosest.closestVectorToHex(above, false).getY());
        double lenBelow = Math.hypot(
            HexClosest.closestVectorToHex(below, false).getX(),
            HexClosest.closestVectorToHex(below, false).getY());

        assertEquals(lenAbove, lenBelow, 1e-6, "Symmetric robots must be equidistant from the boundary");
    }
}

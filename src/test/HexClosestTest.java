package org.team199.lib;

import static org.junit.Assert.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import org.junit.Test;

/**
 * JUnit 4 tests for {@link HexClosest}.
 *
 * <p>{@code closestVectorToHex} returns the (dx, dy) vector from the robot's
 * position to the nearest point on the target hub's hexagonal boundary. The
 * magnitude of this vector equals the robot's straight-line distance to the
 * hub edge.
 *
 * <p>Approximate hub centers used for reference in these tests:
 * <ul>
 *   <li>Blue hub: x ≈ 180 in, y ≈ 159 in</li>
 *   <li>Red  hub: x ≈ 468 in, y ≈ 159 in</li>
 * </ul>
 * Hex apothem (center to nearest edge): ≈ 18 in / 0.457 m.
 */
public class HexClosestTest {

    /**
     * Robot far to the left of the blue hub → the closest boundary point is
     * to the robot's right, so the returned vector must have a positive X component.
     */
    @Test
    public void blue_robotLeftOfHub_vectorPointsRight() {
        Pose2d robot = new Pose2d(0, Units.inchesToMeters(160), new Rotation2d());
        Translation2d vec = HexClosest.closestVectorToHex(robot, false);

        assertTrue("Vector X should be positive pointing right toward blue hub", vec.getX() > 0);
    }

    /**
     * Robot far to the right of the red hub → the closest boundary point is
     * to the robot's left, so the returned vector must have a negative X component.
     */
    @Test
    public void red_robotRightOfHub_vectorPointsLeft() {
        Pose2d robot = new Pose2d(Units.inchesToMeters(700), Units.inchesToMeters(160), new Rotation2d());
        Translation2d vec = HexClosest.closestVectorToHex(robot, true);

        assertTrue("Vector X should be negative pointing left toward red hub", vec.getX() < 0);
    }

    /**
     * Robot directly below the blue hub center → the closest boundary point is
     * directly above, so the returned vector must have a positive Y component.
     */
    @Test
    public void blue_robotBelowHub_vectorPointsUp() {
        Pose2d robot = new Pose2d(Units.inchesToMeters(180.0), 0, new Rotation2d());
        Translation2d vec = HexClosest.closestVectorToHex(robot, false);

        assertTrue("Vector Y should be positive pointing up toward blue hub", vec.getY() > 0);
    }

    /**
     * For any robot not sitting exactly on the hex boundary, the returned
     * vector must have non-zero magnitude.
     */
    @Test
    public void vectorNonZeroForDistantRobot() {
        Pose2d robot = new Pose2d(0, 0, new Rotation2d());
        Translation2d vec = HexClosest.closestVectorToHex(robot, false);

        double len = Math.hypot(vec.getX(), vec.getY());
        assertTrue("Vector length must be > 0 for a robot not on the boundary", len > 1e-6);
    }

    /**
     * From the approximate center of the blue hub the vector magnitude should
     * equal the hub's apothem (≈ 18 in / 0.457 m), within 5 cm.
     */
    @Test
    public void blue_fromCenter_vectorLengthMatchesApothem() {
        Pose2d robot = new Pose2d(
            Units.inchesToMeters(180.0),
            Units.inchesToMeters(159.0),
            new Rotation2d()
        );

        Translation2d vec = HexClosest.closestVectorToHex(robot, false);
        double len = Math.hypot(vec.getX(), vec.getY());

        assertEquals("Vector length from hub center should equal apothem (~0.457 m)",
            0.457, len, 0.05);
    }

    /**
     * Querying from the same midfield position must yield different X components
     * for blue vs. red, since the two hubs are on opposite sides of the field.
     */
    @Test
    public void blueAndRedDifferForSameRobot() {
        Pose2d robot = new Pose2d(
            Units.inchesToMeters(300),
            Units.inchesToMeters(160),
            new Rotation2d()
        );

        Translation2d blue = HexClosest.closestVectorToHex(robot, false);
        Translation2d red  = HexClosest.closestVectorToHex(robot, true);

        assertNotEquals("Blue and red vectors must differ in X from midfield",
            blue.getX(), red.getX(), 1e-3);
    }

    /**
     * From midfield, the blue vector must point left (−X) and the red vector
     * must point right (+X), matching their respective positions on the field.
     */
    @Test
    public void blue_midfieldVectorPointsLeft_red_midfieldVectorPointsRight() {
        Pose2d robot = new Pose2d(
            Units.inchesToMeters(323),
            Units.inchesToMeters(159),
            new Rotation2d()
        );

        Translation2d blue = HexClosest.closestVectorToHex(robot, false);
        Translation2d red  = HexClosest.closestVectorToHex(robot, true);

        assertTrue("Blue vector should point left (-X) from midfield",  blue.getX() < 0);
        assertTrue("Red vector should point right (+X) from midfield",  red.getX()  > 0);
    }

    /**
     * Robots placed symmetrically above and below the hub's horizontal midline
     * must be equidistant from the hex boundary, since the hex is symmetric
     * about that axis.
     */
    @Test
    public void blue_symmetricRobots_equalVectorMagnitudes() {
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

        assertEquals("Symmetric robots must be equidistant from the boundary",
            lenAbove, lenBelow, 1e-6);
    }
}

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
 * <p>Hub geometry (derived from actual vertex coordinates in {@code HexClosest}):
 * <ul>
 *   <li>Blue hub true center: x ≈ 181.44 in (4.609 m), y ≈ 159.41 in (4.049 m)</li>
 *   <li>Red hub true center:  x ≈ 468.03 in (11.888 m), y ≈ 159.04 in (4.040 m)</li>
 *   <li>Blue hub apothem (center to nearest edge): ≈ 20.82 in (0.529 m)</li>
 * </ul>
 *
 * <p><b>Note on symmetry:</b> the vertex coordinates are from field measurements
 * and are not perfectly symmetric — the top and bottom edges differ by ~0.065 in.
 * Tests avoid asserting exact symmetry.
 */
class HexClosestTest {

    // ------------------------------------------------------------------
    // Directional sanity checks
    // ------------------------------------------------------------------

    /**
     * Robot far to the left of the blue hub (x=0) → closest boundary point is
     * to its right, so the returned vector must have a positive X component.
     */
    @Test
    void blue_robotLeftOfHub_vectorPointsRight() {
        Pose2d robot = new Pose2d(0, Units.inchesToMeters(159), new Rotation2d());
        Translation2d vec = HexClosest.closestVectorToHex(robot, false);

        assertTrue(vec.getX() > 0, "Vector X should point right (+) toward blue hub");
    }

    /**
     * Robot far to the right of the red hub (x=700 in) → closest boundary point
     * is to its left, so the returned vector must have a negative X component.
     */
    @Test
    void red_robotRightOfHub_vectorPointsLeft() {
        Pose2d robot = new Pose2d(Units.inchesToMeters(700), Units.inchesToMeters(159), new Rotation2d());
        Translation2d vec = HexClosest.closestVectorToHex(robot, true);

        assertTrue(vec.getX() < 0, "Vector X should point left (-) toward red hub");
    }

    /**
     * Robot directly below the blue hub center (y=0) → closest boundary point
     * is above it, so the returned vector must have a positive Y component.
     */
    @Test
    void blue_robotBelowHub_vectorPointsUp() {
        Pose2d robot = new Pose2d(Units.inchesToMeters(181), 0, new Rotation2d());
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
     * From the true center of the blue hub the vector magnitude should equal the
     * hub's apothem. The true center is (181.44 in, 159.41 in) and the actual
     * apothem computed from the vertices is ≈ 0.529 m. Tolerance of 2 cm is used
     * to account for the slight irregularity in the measured vertex positions.
     */
    @Test
    void blue_fromCenter_vectorLengthMatchesApothem() {
        // True center computed from average of the six blue vertices
        Pose2d robot = new Pose2d(
            Units.inchesToMeters(181.444559),
            Units.inchesToMeters(159.410798),
            new Rotation2d()
        );

        Translation2d vec = HexClosest.closestVectorToHex(robot, false);
        double len = Math.hypot(vec.getX(), vec.getY());

        assertEquals(0.529, len, 0.02,
            "Vector length from true hub center should ~= apothem (0.529 m)");
    }

    /**
     * The vector magnitude from the center must be less than the distance to any
     * vertex (apothem < circumradius). This confirms the closest point is on an
     * edge, not a vertex.
     */
    @Test
    void blue_fromCenter_apothemLessThanCircumradius() {
        Pose2d center = new Pose2d(
            Units.inchesToMeters(181.444559),
            Units.inchesToMeters(159.410798),
            new Rotation2d()
        );

        // Distance from center to vertex 0 (a circumradius)
        double circumradius = Math.hypot(
            Units.inchesToMeters(155.491863) - center.getX(),
            Units.inchesToMeters(159.249827) - center.getY()
        );

        Translation2d vec = HexClosest.closestVectorToHex(center, false);
        double apothem = Math.hypot(vec.getX(), vec.getY());

        assertTrue(apothem < circumradius,
            "Apothem (edge distance) must be less than circumradius (vertex distance)");
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
            Units.inchesToMeters(323),
            Units.inchesToMeters(159),
            new Rotation2d()
        );

        Translation2d blue = HexClosest.closestVectorToHex(robot, false);
        Translation2d red  = HexClosest.closestVectorToHex(robot, true);

        assertNotEquals(blue.getX(), red.getX(), 1e-3,
            "Blue and red vectors must have different X components from midfield");
    }

    /**
     * From midfield (x ≈ 323 in), the blue hub is to the left and the red hub
     * is to the right, so the blue vector must point in -X and red in +X.
     */
    @Test
    void midfield_blueVectorPointsLeft_redVectorPointsRight() {
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
    // Consistency check
    // ------------------------------------------------------------------

    /**
     * The closer a robot is to the hub, the shorter the returned vector should be.
     * A robot at 100 in from center must have a smaller vector magnitude than
     * one at 200 in from center, both along the same axis.
     */
    @Test
    void blue_closerRobot_smallerVectorMagnitude() {
        double centerX = Units.inchesToMeters(181.444559);
        double centerY = Units.inchesToMeters(159.410798);

        // Both robots are directly to the left of the hub center, at different distances
        Pose2d closer  = new Pose2d(centerX - Units.inchesToMeters(100), centerY, new Rotation2d());
        Pose2d farther = new Pose2d(centerX - Units.inchesToMeters(200), centerY, new Rotation2d());

        double lenCloser  = Math.hypot(
            HexClosest.closestVectorToHex(closer,  false).getX(),
            HexClosest.closestVectorToHex(closer,  false).getY());
        double lenFarther = Math.hypot(
            HexClosest.closestVectorToHex(farther, false).getX(),
            HexClosest.closestVectorToHex(farther, false).getY());

        assertTrue(lenCloser < lenFarther,
            "A robot closer to the hub should have a shorter vector to the boundary");
    }
}

package org.carlmontrobotics.ShootOnFlyLib;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/**
 * JUnit 5 tests for {@link ShotCalculator}.
 *
 * <p>All expected values are derived from the real constants in
 * {@code Constants.ShootOnFlyc}:
 * <ul>
 *   <li>{@code launchAngleRad = 1.41656236619} (~81° — very steep arc)</li>
 *   <li>{@code tan(launchAngleRad) ≈ 6.432} — dominates the margin calculation</li>
 *   <li>{@code wheelRadiusMeters = 0.1016 m} (4-inch radius)</li>
 *   <li>{@code launchHeightMeters = 1.0 m}</li>
 *   <li>{@code clearanceSafety = 0.1651 m}</li>
 *   <li>{@code g = 9.81 m/s²}</li>
 * </ul>
 *
 * <p><b>Important:</b> because {@code launchAngleRad ≈ 81°}, the ball travels
 * nearly straight up and always arrives at the goal on its descending arc for
 * any practical horizontal range. {@link ShotCalculator.ShotResult#descendingAtGoal}
 * is therefore {@code true} for all valid shots.
 */
class ShotCalculatorTest {

    // ------------------------------------------------------------------
    // Basic validity
    // ------------------------------------------------------------------

    /**
     * Stationary robot, no obstacles, target at (dx=5, dy=0, dz=0).
     * margin = 5 * tan(1.417) - 0 ≈ 32.16 m — well above both thresholds.
     * Expects a finite positive RPM (~1194 RPM) with all warning flags false.
     */
    @Test
    void basicValidShot_finitePositiveRPM() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 0.0,
            0.0, 0.0,
            null, null
        );

        assertFalse(r.impossible,                     "Should not be impossible");
        assertFalse(r.nearImpossible,                 "Should not be near-impossible");
        assertTrue(r.requiredRPM > 0,                "RPM should be positive");
        assertFalse(Double.isInfinite(r.requiredRPM), "RPM should be finite");
        assertTrue(r.clearanceOK,                    "clearanceOK should be true with no obstacles");
    }

    /**
     * A diagonal target with {@code dx=3, dy=4} has the same horizontal range
     * (5 m) as a straight shot with {@code dx=5, dy=0}. Both should require
     * the same RPM since only horizontal range and height affect the physics.
     */
    @Test
    void diagonalTarget_sameRPMAsStraight() {
        ShotCalculator.ShotResult straight = ShotCalculator.calculateShot(5.0, 0.0, 0.0, 0.0, 0.0, null, null);
        ShotCalculator.ShotResult diagonal = ShotCalculator.calculateShot(3.0, 4.0, 0.0, 0.0, 0.0, null, null);

        assertEquals(straight.requiredRPM, diagonal.requiredRPM, 1e-6,
            "Diagonal shot must equal straight shot of same horizontal range");
    }

    /**
     * RPM must be strictly positive for every non-impossible shot across a
     * representative set of (dx, dy, dz) inputs.
     */
    @Test
    void validShots_rpmAlwaysPositive() {
        double[][] cases = {
            {3.0,  0.0,  0.0},
            {5.0,  5.0,  1.0},
            {10.0, 0.0, -1.0},
        };
        for (double[] c : cases) {
            ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
                c[0], c[1], c[2], 0, 0, null, null);
            if (!r.impossible) {
                assertTrue(r.requiredRPM > 0, "RPM must be positive for a valid shot");
            }
        }
    }

    // ------------------------------------------------------------------
    // Impossible / near-impossible
    // ------------------------------------------------------------------

    /**
     * When {@code dz >= x * tan(launchAngle)} the parabolic margin is zero or
     * negative, making the shot geometrically impossible.
     * At {@code launchAngleRad = 1.41656236619}, x=5:
     * {@code x * tan ≈ 32.1608}, so {@code dz=32.17} gives a negative margin.
     */
    @Test
    void marginNonPositive_impossible() {
        // 5 * tan(1.41656236619) = 32.1608..., so dz=32.17 gives margin < 0
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 32.17,
            0.0, 0.0,
            null, null
        );

        assertTrue(r.impossible,                     "margin<=0 should be marked impossible");
        assertTrue(Double.isInfinite(r.requiredRPM), "RPM should be infinite when impossible");
    }

    /**
     * When {@code dz > x * tan(launchAngle)} the margin is negative and the
     * target is above the peak of the arc — impossible.
     * x=5: any {@code dz > 32.16} gives negative margin.
     */
    @Test
    void marginNegative_impossible() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 33.16,
            0.0, 0.0,
            null, null
        );

        assertTrue(r.impossible, "Negative margin must be marked impossible");
    }

    /**
     * When {@code 0 < margin < 0.20} the shot is unreliable.
     * {@link ShotCalculator.ShotResult#nearImpossible} must be {@code true}
     * and {@link ShotCalculator.ShotResult#impossible} must remain {@code false}.
     * x=5, {@code dz = 32.06}: margin = 32.16 - 32.06 = 0.10 m.
     */
    @Test
    void tinyMargin_nearImpossible() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 32.06,
            0.0, 0.0,
            null, null
        );

        assertFalse(r.impossible,    "Should not be fully impossible");
        assertTrue(r.nearImpossible, "Should be near-impossible with margin = 0.10 m");
    }

    /**
     * {@code dz=0} at x=5 gives margin ≈ 32.16 m — far above the 0.20 m
     * threshold, so {@link ShotCalculator.ShotResult#nearImpossible} must
     * be {@code false}.
     */
    @Test
    void largeMargin_notNearImpossible() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 0.0,
            0.0, 0.0,
            null, null
        );

        assertFalse(r.nearImpossible, "Large margin (~32 m) must not be near-impossible");
    }

    // ------------------------------------------------------------------
    // Robot-motion compensation
    // ------------------------------------------------------------------

    /**
     * A robot moving toward the target in +X already contributes forward
     * velocity, so the flywheel adds less → lower RPM than a stationary robot.
     * Stationary: exitSpeed = vField ≈ 12.71 m/s (~1194 RPM).
     * Moving +1 m/s: exitSpeed = vField - 1 ≈ 11.71 m/s (~1100 RPM).
     */
    @Test
    void movingTowardTarget_lowerRPM() {
        ShotCalculator.ShotResult stat = ShotCalculator.calculateShot(5.0, 0.0, 0.0, 0.0, 0.0, null, null);
        ShotCalculator.ShotResult move = ShotCalculator.calculateShot(5.0, 0.0, 0.0, 1.0, 0.0, null, null);

        assertTrue(move.requiredRPM < stat.requiredRPM,
            "Moving toward target should require less RPM (~1100 vs ~1194)");
    }

    /**
     * A robot moving away from the target in -X opposes the shot, so the
     * flywheel must compensate → higher RPM than a stationary robot.
     * Moving -1 m/s: exitSpeed = vField + 1 ≈ 13.71 m/s (~1288 RPM).
     */
    @Test
    void movingAwayFromTarget_higherRPM() {
        ShotCalculator.ShotResult stat = ShotCalculator.calculateShot(5.0, 0.0, 0.0,  0.0, 0.0, null, null);
        ShotCalculator.ShotResult move = ShotCalculator.calculateShot(5.0, 0.0, 0.0, -1.0, 0.0, null, null);

        assertTrue(move.requiredRPM > stat.requiredRPM,
            "Moving away from target should require more RPM (~1288 vs ~1194)");
    }

    /**
     * Lateral robot motion (perpendicular to the shot direction) must still
     * produce a valid positive RPM — it changes the exit vector direction but
     * does not make the shot impossible.
     */
    @Test
    void lateralMotion_validRPM() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 0.0,
            0.0, 2.0,
            null, null
        );

        assertFalse(r.impossible,      "Lateral motion should not make the shot impossible");
        assertTrue(r.requiredRPM > 0, "Lateral motion should still give a positive RPM");
    }

    // ------------------------------------------------------------------
    // Descending at goal
    // ------------------------------------------------------------------

    /**
     * Due to the extremely steep launch angle ({@code launchAngleRad ≈ 81°}),
     * the ball travels nearly vertically and is always on its descending arc
     * when it arrives at any practical horizontal target.
     * {@link ShotCalculator.ShotResult#descendingAtGoal} must be {@code true}
     * for all valid shots.
     */
    @Test
    void allValidShots_descendingAtGoal() {
        double[][] cases = {
            {0.5,  0.0, 0.0},
            {5.0,  0.0, 0.0},
            {10.0, 0.0, 0.0},
        };
        for (double[] c : cases) {
            ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
                c[0], c[1], c[2], 0, 0, null, null);
            if (!r.impossible) {
                assertTrue(r.descendingAtGoal,
                    "Ball should always be descending at goal with ~81° launch angle (x=" + c[0] + ")");
            }
        }
    }

    // ------------------------------------------------------------------
    // Obstacle clearance
    // ------------------------------------------------------------------

    /**
     * When both obstacle arrays are {@code null},
     * {@link ShotCalculator.ShotResult#clearanceMargins} must be a non-null
     * empty array and {@link ShotCalculator.ShotResult#clearanceOK} must be
     * {@code true}.
     */
    @Test
    void nullObstacles_emptyMarginArrayAndClearanceOK() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 0.0,
            0.0, 0.0,
            null, null
        );

        assertNotNull(r.clearanceMargins,
            "clearanceMargins must never be null");
        assertEquals(0, r.clearanceMargins.length,
            "clearanceMargins must be empty for null obstacle arrays");
        assertTrue(r.clearanceOK,
            "clearanceOK must be true with no obstacles");
    }

    /**
     * A single low obstacle (0.5 m tall at s=1 m) is far below the trajectory
     * height at that distance (≈ 6.15 m above floor with {@code launchHeightMeters=1}).
     * clearanceOK must be {@code true} and the margin positive.
     */
    @Test
    void singleLowObstacle_clearanceOK() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 0.0,
            0.0, 0.0,
            new double[]{1.0},
            new double[]{0.5}
        );

        assertTrue(r.clearanceOK,
            "Obstacle at 0.5 m should pass clearance (trajectory ≈ 6.15 m at s=1)");
        assertTrue(r.clearanceMargins[0] > 0,
            "Clearance margin should be positive");
    }

    /**
     * A single obstacle taller than the trajectory height at its distance
     * must fail clearance. Trajectory at s=1 m is ≈ 6.15 m, so an obstacle
     * of 10 m definitely clips it.
     */
    @Test
    void singleTallObstacle_clearanceFails() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 0.0,
            0.0, 0.0,
            new double[]{1.0},
            new double[]{10.0}
        );

        assertFalse(r.clearanceOK,
            "Obstacle at 10 m should fail clearance (trajectory ≈ 6.15 m at s=1)");
        assertTrue(r.clearanceMargins[0] < 0,
            "Clearance margin should be negative");
    }

    /**
     * Multiple obstacles all well below the arc must all pass: every margin
     * positive, {@link ShotCalculator.ShotResult#clearanceOK} true, and the
     * {@code clearanceMargins} array length must match the obstacle count.
     */
    @Test
    void multipleObstacles_allClear() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            10.0, 0.0, 0.0,
            0.0, 0.0,
            new double[]{1.0, 3.0, 7.0},
            new double[]{0.5, 0.5, 0.5}
        );

        assertTrue(r.clearanceOK,
            "All low obstacles should pass clearance");
        assertEquals(3, r.clearanceMargins.length,
            "clearanceMargins length must equal obstacle count");
        for (double m : r.clearanceMargins) {
            assertTrue(m > 0, "Each individual margin should be positive");
        }
    }

    /**
     * When one of several obstacles is taller than the trajectory at its
     * distance, {@link ShotCalculator.ShotResult#clearanceOK} must be
     * {@code false} regardless of the other obstacles passing.
     */
    @Test
    void multipleObstacles_oneFails_clearanceNotOK() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            10.0, 0.0, 0.0,
            0.0, 0.0,
            new double[]{1.0, 3.0, 7.0},
            new double[]{0.5, 100.0, 0.5}
        );

        assertFalse(r.clearanceOK,
            "One tall obstacle should set clearanceOK = false");
    }
}

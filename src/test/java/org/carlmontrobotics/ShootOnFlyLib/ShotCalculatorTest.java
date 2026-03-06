package org.carlmontrobotics.ShootOnFlyLib;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/**
 * JUnit 5 tests for {@link ShotCalculator}.
 *
 * <p>Constants assumed from {@code Constants.ShootOnFlyc}:
 * <ul>
 *   <li>{@code launchAngleRad} – fixed launch angle (radians); tests assume 45°</li>
 *   <li>{@code wheelRadiusMeters} – flywheel radius used for RPM conversion</li>
 *   <li>{@code launchHeightMeters} – shooter height above field floor, applied to
 *       clearance calculations so obstacle heights and trajectory heights share
 *       the same reference frame</li>
 *   <li>{@code g} – gravitational acceleration (m/s²)</li>
 * </ul>
 */
class ShotCalculatorTest {

    // ------------------------------------------------------------------
    // Basic validity
    // ------------------------------------------------------------------

    /**
     * Stationary robot, no obstacles, target directly ahead at (5, 0, 0).
     * Expects a finite positive RPM with all warning and error flags false.
     */
    @Test
    void basicValidShot_finitePositiveRPM() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 0.0,
            0.0, 0.0,
            null, null
        );

        assertFalse(r.impossible,                    "Should not be impossible");
        assertFalse(r.nearImpossible,                "Should not be near-impossible");
        assertTrue(r.requiredRPM > 0,               "RPM should be positive");
        assertFalse(Double.isInfinite(r.requiredRPM), "RPM should be finite");
        assertTrue(r.clearanceOK,                   "clearanceOK should be true with no obstacles");
    }

    /**
     * A diagonal target with {@code dx=3, dy=4} has a horizontal range of 5 m,
     * identical to a straight shot with {@code dx=5, dy=0}. Both should require
     * the same RPM since only the horizontal range and height affect the physics.
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
     * When {@code dz} equals {@code x * tan(launchAngle)} the parabolic margin
     * is exactly 0, making the shot geometrically impossible.
     * At 45°: margin = 5 * tan(45°) − 5 = 0.
     */
    @Test
    void marginZero_impossible() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 5.0,
            0.0, 0.0,
            null, null
        );

        assertTrue(r.impossible,                    "margin=0 should be marked impossible");
        assertTrue(Double.isInfinite(r.requiredRPM), "RPM should be infinite when impossible");
    }

    /**
     * When {@code dz} exceeds {@code x * tan(launchAngle)} the margin is negative
     * and the shot is impossible — the target is above the peak of the arc.
     */
    @Test
    void marginNegative_impossible() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 10.0,
            0.0, 0.0,
            null, null
        );

        assertTrue(r.impossible, "Negative margin must be marked impossible");
    }

    /**
     * When the margin is positive but less than 0.20 m the shot is unreliable.
     * {@link ShotCalculator.ShotResult#nearImpossible} must be {@code true} and
     * {@link ShotCalculator.ShotResult#impossible} must remain {@code false}.
     * margin = 5 * tan(45°) − 4.85 = 0.15 m.
     */
    @Test
    void tinyMargin_nearImpossible() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 4.85,
            0.0, 0.0,
            null, null
        );

        assertFalse(r.impossible,    "Should not be fully impossible");
        assertTrue(r.nearImpossible, "Should be near-impossible");
    }

    /**
     * A large margin (5 m at 45° with dz=0) is well above the 0.20 m threshold,
     * so {@link ShotCalculator.ShotResult#nearImpossible} must be {@code false}.
     */
    @Test
    void largeMargin_notNearImpossible() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 0.0,
            0.0, 0.0,
            null, null
        );

        assertFalse(r.nearImpossible, "Large margin must not be near-impossible");
    }

    // ------------------------------------------------------------------
    // Robot-motion compensation
    // ------------------------------------------------------------------

    /**
     * A robot moving toward the target in the +X direction already contributes
     * forward velocity, so the flywheel needs to add less → lower RPM than
     * a stationary robot shooting the same target.
     */
    @Test
    void movingTowardTarget_lowerRPM() {
        ShotCalculator.ShotResult stat = ShotCalculator.calculateShot(5.0, 0.0, 0.0, 0.0, 0.0, null, null);
        ShotCalculator.ShotResult move = ShotCalculator.calculateShot(5.0, 0.0, 0.0, 1.0, 0.0, null, null);

        assertTrue(move.requiredRPM < stat.requiredRPM, "Moving toward target should require less RPM");
    }

    /**
     * A robot moving away from the target in the −X direction opposes the shot,
     * so the flywheel must compensate → higher RPM than a stationary robot.
     */
    @Test
    void movingAwayFromTarget_higherRPM() {
        ShotCalculator.ShotResult stat = ShotCalculator.calculateShot(5.0, 0.0, 0.0,  0.0, 0.0, null, null);
        ShotCalculator.ShotResult move = ShotCalculator.calculateShot(5.0, 0.0, 0.0, -1.0, 0.0, null, null);

        assertTrue(move.requiredRPM > stat.requiredRPM, "Moving away from target should require more RPM");
    }

    /**
     * Lateral robot motion (perpendicular to the shot direction) must still
     * produce a valid, positive RPM — the compensation simply changes the
     * exit vector direction rather than its overall feasibility.
     */
    @Test
    void lateralMotion_validRPM() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 0.0,
            0.0, 2.0,
            null, null
        );

        assertFalse(r.impossible,       "Lateral motion should not make the shot impossible");
        assertTrue(r.requiredRPM > 0,  "Lateral motion should still give a positive RPM");
    }

    // ------------------------------------------------------------------
    // Ascending / descending at goal
    // ------------------------------------------------------------------

    /**
     * At very short range (0.5 m, 45°) the ball has not yet reached the peak of
     * its arc when it arrives at the goal, so it must still be ascending.
     */
    @Test
    void shortRange_ascendingAtGoal() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            0.5, 0.0, 0.0,
            0.0, 0.0,
            null, null
        );

        assertFalse(r.descendingAtGoal, "Short-range shot should not be descending at goal");
    }

    /**
     * At long range (20 m, 45°) the ball is well past the apex of its arc when
     * it arrives at the goal, so it must be descending.
     */
    @Test
    void longRange_descendingAtGoal() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            20.0, 0.0, 0.0,
            0.0, 0.0,
            null, null
        );

        assertTrue(r.descendingAtGoal, "Long-range shot should be descending at goal");
    }

    // ------------------------------------------------------------------
    // Obstacle clearance
    // ------------------------------------------------------------------

    /**
     * When both obstacle arrays are {@code null}, {@link ShotCalculator.ShotResult#clearanceMargins}
     * must be a non-null empty array and {@link ShotCalculator.ShotResult#clearanceOK}
     * must be {@code true}.
     */
    @Test
    void nullObstacles_emptyMarginArrayAndClearanceOK() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 0.0,
            0.0, 0.0,
            null, null
        );

        assertNotNull(r.clearanceMargins,               "clearanceMargins must never be null");
        assertEquals(0, r.clearanceMargins.length,      "clearanceMargins must be empty for null obstacle arrays");
        assertTrue(r.clearanceOK,                       "clearanceOK must be true with no obstacles");
    }

    /**
     * A single obstacle well below the trajectory must pass clearance:
     * {@link ShotCalculator.ShotResult#clearanceOK} true and the margin positive.
     */
    @Test
    void singleLowObstacle_clearanceOK() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 0.0,
            0.0, 0.0,
            new double[]{2.5},
            new double[]{0.1}
        );

        assertTrue(r.clearanceOK,                  "Low obstacle should pass clearance");
        assertTrue(r.clearanceMargins[0] > 0,      "Clearance margin should be positive");
    }

    /**
     * A single impossibly tall obstacle (100 m) must fail clearance:
     * {@link ShotCalculator.ShotResult#clearanceOK} false and the margin negative.
     */
    @Test
    void singleTallObstacle_clearanceFails() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 0.0,
            0.0, 0.0,
            new double[]{2.5},
            new double[]{100.0}
        );

        assertFalse(r.clearanceOK,                 "Tall obstacle should fail clearance");
        assertTrue(r.clearanceMargins[0] < 0,      "Clearance margin should be negative");
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
            new double[]{2.0, 5.0, 8.0},
            new double[]{0.1, 0.1, 0.1}
        );

        assertTrue(r.clearanceOK,                                       "All low obstacles should pass");
        assertEquals(3, r.clearanceMargins.length,             "clearanceMargins length must equal obstacle count");
        for (double m : r.clearanceMargins) {
            assertTrue(m > 0, "Each individual margin should be positive");
        }
    }

    /**
     * When one of several obstacles is impossibly tall,
     * {@link ShotCalculator.ShotResult#clearanceOK} must be {@code false}
     * regardless of the other obstacles passing.
     */
    @Test
    void multipleObstacles_oneFails_clearanceNotOK() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            10.0, 0.0, 0.0,
            0.0, 0.0,
            new double[]{2.0, 5.0, 8.0},
            new double[]{0.1, 100.0, 0.1}
        );

        assertFalse(r.clearanceOK, "One tall obstacle should set clearanceOK = false");
    }
}

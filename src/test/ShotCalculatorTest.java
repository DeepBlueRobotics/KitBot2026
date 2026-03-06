package org.team199.lib;

import static org.junit.Assert.*;

import org.junit.Test;

/**
 * JUnit 4 tests for {@link ShotCalculator}.
 *
 * <p>Constants assumed from {@code Constants.ShootOnFlyc}:
 * <ul>
 *   <li>{@code launchAngleRad}    – fixed launch angle (rad); tests assume 45°</li>
 *   <li>{@code wheelRadiusMeters} – flywheel radius used for RPM conversion</li>
 *   <li>{@code launchHeightMeters}– shooter height above the field floor (m)</li>
 *   <li>{@code g}                 – gravitational acceleration (m/s²)</li>
 * </ul>
 *
 * <p>Obstacle clearance heights are measured from the field floor. The trajectory
 * height at each obstacle is offset by {@code launchHeightMeters} so both values
 * share the same reference frame.
 */
public class ShotCalculatorTest {

    // ------------------------------------------------------------------
    // Basic validity
    // ------------------------------------------------------------------

    /**
     * Stationary robot, no obstacles, target at (5, 0, 0).
     * Expects a finite positive RPM with all warning flags false.
     */
    @Test
    public void basicValidShot_finitePositiveRPM() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 0.0,
            0.0, 0.0,
            null, null
        );

        assertFalse("Should not be impossible",      r.impossible);
        assertFalse("Should not be near-impossible", r.nearImpossible);
        assertTrue("RPM should be positive",         r.requiredRPM > 0);
        assertFalse("RPM should be finite",          Double.isInfinite(r.requiredRPM));
        assertTrue("clearanceOK with no obstacles",  r.clearanceOK);
    }

    /**
     * A diagonal target (dx=3, dy=4) has the same horizontal range as a straight
     * target at (5, 0, 0) since {@code x = hypot(dx, dy) = 5}. Both should
     * produce identical RPM values.
     */
    @Test
    public void diagonalTarget_sameRPMAsStraight() {
        ShotCalculator.ShotResult straight = ShotCalculator.calculateShot(5.0, 0.0, 0.0, 0.0, 0.0, null, null);
        ShotCalculator.ShotResult diagonal = ShotCalculator.calculateShot(3.0, 4.0, 0.0, 0.0, 0.0, null, null);

        assertEquals("Diagonal and straight shots of equal range must require equal RPM",
            straight.requiredRPM, diagonal.requiredRPM, 1e-6);
    }

    /**
     * RPM must be strictly positive for every non-impossible shot across a range
     * of (dx, dy, dz) combinations.
     */
    @Test
    public void validShots_rpmAlwaysPositive() {
        double[][] cases = {
            {3.0,  0.0,  0.0},
            {5.0,  5.0,  1.0},
            {10.0, 0.0, -1.0},
        };
        for (double[] c : cases) {
            ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
                c[0], c[1], c[2], 0, 0, null, null);
            if (!r.impossible) {
                assertTrue("RPM must be positive for a valid shot", r.requiredRPM > 0);
            }
        }
    }

    // ------------------------------------------------------------------
    // Impossible / near-impossible
    // ------------------------------------------------------------------

    /**
     * When {@code dz == x * tan(launchAngle)}, the parabolic margin is exactly 0
     * and the shot must be marked impossible with RPM = ∞.
     * At 45°: margin = 5 * tan(45°) − 5 = 0.
     */
    @Test
    public void marginZero_impossible() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 5.0,
            0.0, 0.0,
            null, null
        );

        assertTrue("margin = 0 must be marked impossible",   r.impossible);
        assertTrue("RPM must be infinite when impossible",   Double.isInfinite(r.requiredRPM));
    }

    /**
     * When {@code dz > x * tan(launchAngle)}, the margin is negative and the
     * target is above the maximum arc height — must be marked impossible.
     */
    @Test
    public void marginNegative_impossible() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 10.0,
            0.0, 0.0,
            null, null
        );

        assertTrue("Negative margin must be marked impossible", r.impossible);
    }

    /**
     * When {@code 0 < margin < 0.20 m}, the shot is physically possible but
     * considered unreliable — {@code nearImpossible} must be set and
     * {@code impossible} must remain false.
     * margin = 5 * tan(45°) − 4.85 = 0.15 m.
     */
    @Test
    public void tinyMargin_nearImpossible() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 4.85,
            0.0, 0.0,
            null, null
        );

        assertFalse("Should not be fully impossible",  r.impossible);
        assertTrue("Should be near-impossible",        r.nearImpossible);
    }

    /**
     * When the margin is well above 0.20 m, {@code nearImpossible} must be false.
     * margin = 5 * tan(45°) − 0 = 5 m.
     */
    @Test
    public void largeMargin_notNearImpossible() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 0.0,
            0.0, 0.0,
            null, null
        );

        assertFalse("Large margin must not be near-impossible", r.nearImpossible);
    }

    // ------------------------------------------------------------------
    // Robot-motion compensation
    // ------------------------------------------------------------------

    /**
     * A robot moving toward the target (+X) already contributes velocity in the
     * shot direction, so the flywheel needs to add less — resulting in lower RPM
     * than a stationary robot.
     */
    @Test
    public void movingTowardTarget_lowerRPM() {
        ShotCalculator.ShotResult stat = ShotCalculator.calculateShot(5.0, 0.0, 0.0, 0.0, 0.0, null, null);
        ShotCalculator.ShotResult move = ShotCalculator.calculateShot(5.0, 0.0, 0.0, 1.0, 0.0, null, null);

        assertTrue("Moving toward target should require less RPM",
            move.requiredRPM < stat.requiredRPM);
    }

    /**
     * A robot moving away from the target (−X) opposes the required ball velocity,
     * so the flywheel must compensate — resulting in higher RPM than a stationary robot.
     */
    @Test
    public void movingAwayFromTarget_higherRPM() {
        ShotCalculator.ShotResult stat = ShotCalculator.calculateShot(5.0, 0.0, 0.0,  0.0, 0.0, null, null);
        ShotCalculator.ShotResult move = ShotCalculator.calculateShot(5.0, 0.0, 0.0, -1.0, 0.0, null, null);

        assertTrue("Moving away from target should require more RPM",
            move.requiredRPM > stat.requiredRPM);
    }

    /**
     * Lateral robot motion (perpendicular to the shot direction) still affects
     * the required exit velocity via vector subtraction and must produce a valid,
     * positive RPM.
     */
    @Test
    public void lateralMotion_validRPM() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 0.0,
            0.0, 2.0,
            null, null
        );

        assertFalse("Lateral motion should not make shot impossible", r.impossible);
        assertTrue("Lateral motion should still give positive RPM",   r.requiredRPM > 0);
    }

    // ------------------------------------------------------------------
    // Ascending / descending at goal
    // ------------------------------------------------------------------

    /**
     * At very short range (0.5 m) the ball has not yet reached the apex of its arc,
     * so {@code descendingAtGoal} must be false.
     */
    @Test
    public void shortRange_ascendingAtGoal() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            0.5, 0.0, 0.0,
            0.0, 0.0,
            null, null
        );

        assertFalse("Short-range shot should not be descending at goal", r.descendingAtGoal);
    }

    /**
     * At long range (20 m) the ball is well past the apex and on the descending
     * half of its arc when it reaches the goal, so {@code descendingAtGoal} must
     * be true.
     */
    @Test
    public void longRange_descendingAtGoal() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            20.0, 0.0, 0.0,
            0.0, 0.0,
            null, null
        );

        assertTrue("Long-range shot should be descending at goal", r.descendingAtGoal);
    }

    // ------------------------------------------------------------------
    // Obstacle clearance
    // ------------------------------------------------------------------

    /**
     * When both obstacle arrays are {@code null}, {@code clearanceMargins} must
     * be a non-null empty array and {@code clearanceOK} must be {@code true}.
     */
    @Test
    public void nullObstacles_emptyMarginArrayAndClearanceOK() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 0.0,
            0.0, 0.0,
            null, null
        );

        assertNotNull("clearanceMargins must never be null",                    r.clearanceMargins);
        assertEquals("clearanceMargins must be empty for null obstacle arrays", 0, r.clearanceMargins.length);
        assertTrue("clearanceOK must be true with no obstacles",                r.clearanceOK);
    }

    /**
     * A single obstacle well below the trajectory must produce a positive clearance
     * margin and leave {@code clearanceOK} true.
     */
    @Test
    public void singleLowObstacle_clearanceOK() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 0.0,
            0.0, 0.0,
            new double[]{2.5},
            new double[]{0.1}
        );

        assertTrue("Low obstacle should pass clearance",  r.clearanceOK);
        assertTrue("Clearance margin should be positive", r.clearanceMargins[0] > 0);
    }

    /**
     * A single obstacle taller than the entire arc must produce a negative clearance
     * margin and set {@code clearanceOK} to {@code false}.
     */
    @Test
    public void singleTallObstacle_clearanceFails() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            5.0, 0.0, 0.0,
            0.0, 0.0,
            new double[]{2.5},
            new double[]{100.0}
        );

        assertFalse("Tall obstacle should fail clearance",        r.clearanceOK);
        assertTrue("Clearance margin should be negative",         r.clearanceMargins[0] < 0);
    }

    /**
     * Multiple obstacles all below the arc must each have a positive margin,
     * {@code clearanceOK} must be {@code true}, and {@code clearanceMargins}
     * must have the same length as the obstacle arrays.
     */
    @Test
    public void multipleObstacles_allClear() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            10.0, 0.0, 0.0,
            0.0, 0.0,
            new double[]{2.0, 5.0, 8.0},
            new double[]{0.1, 0.1, 0.1}
        );

        assertTrue("All low obstacles should pass",                    r.clearanceOK);
        assertEquals("clearanceMargins length must equal obstacle count",
            3, r.clearanceMargins.length);
        for (double m : r.clearanceMargins) {
            assertTrue("Each individual margin should be positive", m > 0);
        }
    }

    /**
     * When any single obstacle fails clearance, {@code clearanceOK} must be
     * {@code false} even if all other obstacles pass.
     */
    @Test
    public void multipleObstacles_oneFails_clearanceNotOK() {
        ShotCalculator.ShotResult r = ShotCalculator.calculateShot(
            10.0, 0.0, 0.0,
            0.0, 0.0,
            new double[]{2.0, 5.0, 8.0},
            new double[]{0.1, 100.0, 0.1}
        );

        assertFalse("One tall obstacle should set clearanceOK = false", r.clearanceOK);
    }
}

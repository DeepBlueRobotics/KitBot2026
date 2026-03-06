package org.carlmontrobotics.ShootOnFlyLib;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.junit.jupiter.api.Test;

/**
 * JUnit 5 tests for {@link HeadingAlignController}.
 *
 * <p>Heading error is wrapped into (-π, π] via {@link edu.wpi.first.math.MathUtil#angleModulus},
 * so all edge cases around ±π are handled correctly.
 *
 * <p>Constants assumed from {@code Constants.ShootOnFlyc}:
 * <ul>
 *   <li>{@code thetaAlignP} – proportional gain (rad/s per rad)</li>
 *   <li>{@code toleranceRad} – alignment tolerance in radians</li>
 * </ul>
 */
class HeadingAlignControllerTest {

    /**
     * Robot at origin facing east (+X), target due north (+Y).
     * Desired angle = π/2, current = 0 → error = π/2.
     * Omega must be positive (CCW correction).
     */
    @Test
    void targetNorth_omegaPositive() {
        Pose2d current = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d target  = new Pose2d(0, 5, new Rotation2d(0));

        double omega = HeadingAlignController.calculateOmega(current, target);
        assertTrue(omega > 0, "Omega should be positive when target is to the left");
    }

    /**
     * Robot facing directly toward the target → error = 0 → omega = 0.
     */
    @Test
    void alreadyFacing_omegaNearZero() {
        Pose2d current = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d target  = new Pose2d(5, 0, new Rotation2d(0));

        double omega = HeadingAlignController.calculateOmega(current, target);
        assertEquals(0.0, omega, 1e-9, "Omega should be ~0 when already aligned");
    }

    /**
     * Robot facing south (-π/2), target directly south → error = 0 → omega = 0.
     */
    @Test
    void facingSouth_targetSouth_omegaNearZero() {
        Pose2d current = new Pose2d(0, 0, new Rotation2d(-Math.PI / 2));
        Pose2d target  = new Pose2d(0, -5, new Rotation2d(0));

        double omega = HeadingAlignController.calculateOmega(current, target);
        assertEquals(0.0, omega, 1e-9, "Omega should be ~0 when facing target south");
    }

    /**
     * Target is to the robot's right (south when facing east) → omega must be negative (CW correction).
     */
    @Test
    void targetToRight_omegaNegative() {
        Pose2d current = new Pose2d(0, 0, new Rotation2d(0));   // facing east
        Pose2d target  = new Pose2d(0, -5, new Rotation2d(0));  // target is south

        double omega = HeadingAlignController.calculateOmega(current, target);
        assertTrue(omega < 0, "Omega should be negative when target is to the right");
    }

    /**
     * Robot facing directly away from target (error = ±π).
     * {@link edu.wpi.first.math.MathUtil#angleModulus} correctly wraps this to ±π,
     * so omega must be non-zero.
     */
    @Test
    void facingDirectlyAway_omegaNonZero() {
        Pose2d current = new Pose2d(0, 0, new Rotation2d(Math.PI / 2)); // facing north
        Pose2d target  = new Pose2d(0, -10, new Rotation2d(0));         // target is south

        double omega = HeadingAlignController.calculateOmega(current, target);
        assertNotEquals(0.0, omega, 1e-9, "Omega must be non-zero when facing directly away from target");
    }

    /**
     * {@link HeadingAlignController#atGoal} returns {@code true} when the robot
     * is pointing directly at the target (error = 0).
     */
    @Test
    void atGoal_trueWhenAligned() {
        Pose2d current = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d target  = new Pose2d(10, 0, new Rotation2d(0));

        assertTrue(HeadingAlignController.atGoal(current, target));
    }

    /**
     * {@link HeadingAlignController#atGoal} returns {@code false} when the robot
     * is 90° off target.
     */
    @Test
    void atGoal_falseWhenFar() {
        Pose2d current = new Pose2d(0, 0, new Rotation2d(0));   // facing east
        Pose2d target  = new Pose2d(0, 10, new Rotation2d(0));  // target is north

        assertFalse(HeadingAlignController.atGoal(current, target));
    }

    /**
     * {@link HeadingAlignController#calculateOmega} and {@link HeadingAlignController#atGoal}
     * must be consistent: when omega is 0 the robot is aligned, so atGoal must return {@code true}.
     */
    @Test
    void omegaZeroImpliesAtGoal() {
        Pose2d current = new Pose2d(3, 4, new Rotation2d(0));
        Pose2d target  = new Pose2d(10, 4, new Rotation2d(0)); // due east

        double omega = HeadingAlignController.calculateOmega(current, target);
        boolean goal  = HeadingAlignController.atGoal(current, target);

        assertEquals(0.0, omega, 1e-9, "Omega should be 0 when aligned east");
        assertTrue(goal, "atGoal must agree when omega == 0");
    }
}

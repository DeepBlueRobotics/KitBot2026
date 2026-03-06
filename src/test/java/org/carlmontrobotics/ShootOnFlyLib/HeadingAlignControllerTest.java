package org.carlmontrobotics.ShootOnFlyLib;

import static org.junit.Assert.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.junit.Test;

/**
 * JUnit 4 tests for {@link HeadingAlignController}.
 *
 * <p>Heading error is wrapped into (-π, π] via {@link edu.wpi.first.math.MathUtil#angleModulus}.
 *
 * <p>Constants assumed from {@code Constants.ShootOnFlyc}:
 * <ul>
 *   <li>{@code thetaAlignP} – proportional gain (rad/s per rad)</li>
 *   <li>{@code toleranceRad} – alignment tolerance in radians</li>
 * </ul>
 */
public class HeadingAlignControllerTest {

    /**
     * Robot at origin facing east (+X), target due north (+Y).
     * Desired angle = π/2, current = 0 → error = π/2.
     * Omega must be positive (CCW correction).
     */
    @Test
    public void targetNorth_omegaPositive() {
        Pose2d current = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d target  = new Pose2d(0, 5, new Rotation2d(0));

        double omega = HeadingAlignController.calculateOmega(current, target);
        assertTrue("Omega should be positive when target is to the left", omega > 0);
    }

    /**
     * Robot facing directly toward the target → error = 0 → omega = 0.
     */
    @Test
    public void alreadyFacing_omegaNearZero() {
        Pose2d current = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d target  = new Pose2d(5, 0, new Rotation2d(0));

        double omega = HeadingAlignController.calculateOmega(current, target);
        assertEquals("Omega should be ~0 when already aligned", 0.0, omega, 1e-9);
    }

    /**
     * Robot facing south (-π/2), target directly south → error = 0 → omega = 0.
     */
    @Test
    public void facingSouth_targetSouth_omegaNearZero() {
        Pose2d current = new Pose2d(0, 0, new Rotation2d(-Math.PI / 2));
        Pose2d target  = new Pose2d(0, -5, new Rotation2d(0));

        double omega = HeadingAlignController.calculateOmega(current, target);
        assertEquals("Omega should be ~0 when facing target south", 0.0, omega, 1e-9);
    }

    /**
     * Target is to the robot's right (south when facing east) → omega must be
     * negative (CW correction).
     */
    @Test
    public void targetToRight_omegaNegative() {
        Pose2d current = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d target  = new Pose2d(0, -5, new Rotation2d(0));

        double omega = HeadingAlignController.calculateOmega(current, target);
        assertTrue("Omega should be negative when target is to the right", omega < 0);
    }

    /**
     * Robot facing north (π/2), target due south → raw error = -π.
     * {@code MathUtil.angleModulus} correctly wraps this to ±π rather than 0,
     * so omega must be non-zero.
     */
    @Test
    public void facingDirectlyAway_omegaNonZero() {
        Pose2d current = new Pose2d(0, 0, new Rotation2d(Math.PI / 2));
        Pose2d target  = new Pose2d(0, -10, new Rotation2d(0));

        double omega = HeadingAlignController.calculateOmega(current, target);
        assertNotEquals("Omega must be non-zero when facing directly away from target",
            0.0, omega, 1e-9);
    }

    /**
     * {@code atGoal} returns {@code true} when the robot is pointing directly at
     * the target (error = 0, well within {@code toleranceRad}).
     */
    @Test
    public void atGoal_trueWhenAligned() {
        Pose2d current = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d target  = new Pose2d(10, 0, new Rotation2d(0));

        assertTrue(HeadingAlignController.atGoal(current, target));
    }

    /**
     * {@code atGoal} returns {@code false} when the robot is 90° off target,
     * which far exceeds any reasonable {@code toleranceRad}.
     */
    @Test
    public void atGoal_falseWhenFar() {
        Pose2d current = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d target  = new Pose2d(0, 10, new Rotation2d(0));

        assertFalse(HeadingAlignController.atGoal(current, target));
    }

    /**
     * {@code calculateOmega} and {@code atGoal} must be consistent: when omega
     * is 0 (robot already aligned), {@code atGoal} must return {@code true}.
     */
    @Test
    public void omegaZeroImpliesAtGoal() {
        Pose2d current = new Pose2d(3, 4, new Rotation2d(0));
        Pose2d target  = new Pose2d(10, 4, new Rotation2d(0));

        double omega = HeadingAlignController.calculateOmega(current, target);
        boolean goal  = HeadingAlignController.atGoal(current, target);

        assertEquals("Omega should be 0 when aligned east", 0.0, omega, 1e-9);
        assertTrue("atGoal must agree when omega == 0", goal);
    }
}

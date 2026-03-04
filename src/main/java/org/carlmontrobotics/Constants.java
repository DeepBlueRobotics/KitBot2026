// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import org.carlmontrobotics.lib199.swerve.SwerveConfig;
import org.carlmontrobotics.lib199.MotorConfig;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.math.util.Units;
import static org.carlmontrobotics.Config.CONFIG;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    	public static final double g = 9.81; // meters per second squared

    // public static final class Drivetrain {
    //     public static final double MAX_SPEED_MPS = 2;
    // }
	public static final class OI {
		public static final class Driver {
			public static final int port = 0;

			public static final int slowDriveButton = Button.kLeftBumper.value;
			public static final int resetFieldOrientationButton = Button.kRightBumper.value;
			public static final Axis RIGHT_TRIGGER_BUTTON = Axis.kRightTrigger;
			public static final Axis LEFT_TRIGGER_BUTTON = Axis.kLeftTrigger;

			public static final int y = Button.kY.value;
			public static final int b = Button.kB.value;
			public static final int a = Button.kA.value;
			public static final int x = Button.kX.value;
		}

		public static final class Manipulator {
			public static final int port = 1;
			public static final int Y = Button.kY.value;
			public static final int INTAKE_CONVEYOR_BUTTON = Button.kA.value; //get real buttons later
            public static final int OUTTAKE_BUTTON = Button.kB.value; 
			public static final int SMART_SHOOT_BUTTON = Button.kX.value;
        }

		public static final double JOY_THRESH = 0.13;
		public static final double MIN_AXIS_TRIGGER_VALUE = 0.2;// woah, this is high.

	}

    //#region Drivetrain
	public static final class Drivetrainc {
		//general drivetrain constants
		public static final int driveFrontLeftPort = 1;
		public static final int driveFrontRightPort = 2;
		public static final int driveBackLeftPort = 3;
		public static final int driveBackRightPort = 4;

		public static final int turnFrontLeftPort = 11;
		public static final int turnFrontRightPort = 12;
		public static final int turnBackLeftPort = 13;
		public static final int turnBackRightPort = 14;
		//TODO: set hammerhead can coder ports the same as kitbot
		public static final int canCoderPortFL = CONFIG.isHammerHead() ? 0 : 1; 
		public static final int canCoderPortFR = CONFIG.isHammerHead() ? 1 : 2; 
		public static final int canCoderPortBL = CONFIG.isHammerHead() ? 3 : 3;
		public static final int canCoderPortBR = CONFIG.isHammerHead() ? 2 : 0; 

		// swerve config constants
		public static final double wheelBase = CONFIG.isHammerHead() ? Units.inchesToMeters(16.750003) : 
																		Units.inchesToMeters(16.750003);
		public static final double trackWidth = CONFIG.isHammerHead() ? Units.inchesToMeters(23.750000) : 
																		Units.inchesToMeters(23.750000);
		public static final double swerveRadius = Math.sqrt(Math.pow(wheelBase / 2, 2) + Math.pow(trackWidth / 2, 2));
		public static final double wheelDiameterMeters = Units.inchesToMeters(4.0) * 7.36 / 7.65;
		public static final double driveGearing = 6.75;
		public static final double mu = 1; /* 70/83.2; */ // coefficient of friction. less means less max acceleration.
		public static final double autoCentripetalAccel = mu * g * 2;
		public static final double[] kForwardVolts = CONFIG.isHammerHead() ? new double[] { 0.2,0.2,0.2,0.2 }: //kS
																				new double[] {0, 0, 0, 0}; 
		public static final double[] kForwardVels = CONFIG.isHammerHead() ? new double[] { 0,0,0,0 } : //kV
																				new double[] { 2.9875, 2.9875, 2.7323, 2.9264 };
		public static final double[] kForwardAccels = { 0, 0, 0, 0 };//{0.31958, 0.33557, 0.70264, 0.46644};    //{ 0, 0, 0, 0 };// volts per m/s^2
		public static final double[] kBackwardVolts = kForwardVolts;
		public static final double[] kBackwardVels = kForwardVels;
		public static final double[] kBackwardAccels = kForwardAccels;

		public static final double[] drivekP = {1, 1, 1, 1};
		public static final double[] drivekI = {0, 0, 0, 0};
		public static final double[] drivekD = CONFIG.isHammerHead()? new double[] { 0, 0, 0, 0 }:
																				new double[] { 0,0,0,0 };
		public static final double[] turnkP = CONFIG.isHammerHead() ? new double[] {50, 50, 50, 50} : 
																				new double[] {0,0,0,0};
		public static final double[] turnkI = {0, 0, 0, 0};
		public static final double[] turnkD = {0, 0, 0, 0};
		public static final double[] turnkS = {1, 1, 1, 1};
		public static final double[] turnkV = {0, 0, 0, 0};
		public static final double[] turnkA = {0, 0, 0, 0};
		public static final double[] turnZeroDeg = CONFIG.isHammerHead() ? new double[] { 85.7812, 85.0782, -96.9433, -162.9492 } : 
																				new double[] { 17.2266, -96.8555, -95.8008, 85.166 };
		public static final boolean[] driveInversion = CONFIG.isHammerHead() ? new boolean[] { true, false, true, false } :
																				new boolean[] { false, true, false, true };
		public static final boolean[] reversed = { false, false, false, false };
		public static final double driveModifier = 1;
		public static final boolean[] turnInversion = { true, true, true, true };

		public static final double turnGearing = 150.0 / 7;
		public static final double ROBOTMASS_KG = CONFIG.isHammerHead() ? 35.49159484 : 48.582;
		public static final double MOI = CONFIG.isHammerHead() ? 4.10872647 : 5.38619461; // moment of inertia, kg/m^2, Lzz in onshape
		public static final double NEOFreeSpeed = 5676 * (2 * Math.PI) / 60; // radians/s
		public static final double VortexFreeSpeed = 6784 * (2 * Math.PI) / 60; // radians/s
		// Angular speed to translational speed --> v = omega * r / gearing
		public static final double maxSpeed = (CONFIG.isVortexDrive() ? VortexFreeSpeed : NEOFreeSpeed) * (wheelDiameterMeters / 2.0) / driveGearing; // meter/s
		public static final double maxForward = maxSpeed;
		public static final double maxStrafe = maxSpeed;
		// seconds it takes to go from 0 to 12 volts(aka MAX)
		public static final double secsPer12Volts = 0.1;
		// maxRCW is the angular velocity of the robot.
		// Calculated by looking at one of the motors and treating it as a point mass
		// moving around in a circle.
		// Tangential speed of this point mass is maxSpeed and the radius of the circle
		// is sqrt((wheelBase/2)^2 + (trackWidth/2)^2)
		// Angular velocity = Tangential speed / radius
		public static final double maxRCW = maxSpeed / swerveRadius;

		public static final double autoMaxSpeedMps = 4;//0.6 * 4.4; // Meters / second
		public static final double autoMaxAccelMps2 = mu * g; // Meters / seconds^2
		public static final double autoMaxAmps = 40.0; 
		// The maximum acceleration the robot can achieve is equal to the coefficient of
		// static friction times the gravitational acceleration
		// a = mu * 9.8 m/s^2

		public static final boolean isGyroReversed = true;

		public static final double[] thetaPIDController = CONFIG.isHammerHead() ? new double[] { 0.10, 0.0, 0.001 }
		: new double[] {0.05, 0.0, 0.00};

		public static final SwerveConfig swerveConfig = new SwerveConfig(wheelDiameterMeters, driveGearing, mu,
		autoCentripetalAccel, kForwardVolts, kForwardVels, kForwardAccels, kBackwardVolts, kBackwardVels,
		kBackwardAccels, drivekP, drivekI, drivekD, turnkP, turnkI, turnkD, turnkS, turnkV, turnkA, turnZeroDeg,
		driveInversion, reversed, driveModifier, turnInversion);


		public static double kNormalDriveSpeed = 1; // Percent Multiplier	
		public static double kNormalDriveRotation = 0.5; // Percent Multiplier
		public static double kSlowDriveSpeed = 0.4; // Percent Multiplier
		public static double kSlowDriveRotation = 0.250; // Percent Multiplier

		public static double kBabyDriveSpeed = 0.3;
		public static double kBabyDriveRotation = 0.2;

		public static final double wheelTurnDriveSpeed = 0.0001; // Meters / Second ; A non-zero speed just used to
														// orient the wheels to the correct angle. This
														// should be very small to avoid actually moving the
														// robot.

		public static final double[] positionTolerance = { Units.inchesToMeters(.5), Units.inchesToMeters(.5), 5 }; // Meters,
																										// Meters,
																										// Degrees
		public static final double[] velocityTolerance = { Units.inchesToMeters(1), Units.inchesToMeters(1), 5 }; // Meters,
																													// Meters,
																													// Degrees/Second

		public static final double turnkP_avg = (turnkP[0] + turnkP[1] + turnkP[2] + turnkP[3]) / 4;
		public static final double turnIzone = .1;

		public static final double driveIzone = .1;
		public static final double COLLISION_ACCELERATION_THRESHOLD = 2; //The minimum acceleration that will trigger a collision detection, in m/s^2
		public static final class Autoc {
			public static final RobotConfig robotConfig = new RobotConfig(
					// Mass mass, kg
					ROBOTMASS_KG,
					// double Moment Oof Inertia, kg/mm
					MOI, // ==1
					// ModuleConfig moduleConfig,
					new ModuleConfig(
							// double wheelRadiusMeters,
							wheelDiameterMeters/2,
							// double maxDriveVelocityMPS,
							autoMaxSpeedMps,
							// double wheelCOF,
							mu,
							// DCMotor driveMotor,
							DCMotor.getNEO(1),
							// double driveGearing,
							driveGearing,
							// double driveCurrentLimit,
							autoMaxAmps,
							// int numMotors
							1),
					// Translation2d... moduleOffsets
					new Translation2d(wheelBase / 2, trackWidth / 2),
					new Translation2d(wheelBase / 2, -trackWidth / 2),
					new Translation2d(-wheelBase / 2, trackWidth / 2),
					new Translation2d(-wheelBase / 2, -trackWidth / 2));
			// public static final ReplanningConfig repConfig = new ReplanningConfig( /*
			// * put in
			// * Constants.Drivetrain.Auto
			// */
			// false, // replan at start of path if robot not at start of path?
			// false, // replan if total error surpasses total error/spike threshold?
			// 1.5, // total error threshold in meters that will cause the path to be
			// replanned
			// 0.8 // error spike threshold, in meters, that will cause the path to be
			// replanned
			// );
			public static final PathConstraints pathConstraints = new PathConstraints(3.5, 2.5, Math.PI-0.5, Math.PI-0.5); // The constraints for this path. If using a differential drivetrain, the
									// angular constraints have no effect.
		}
	}
	//#endregion
	public static class LimeLightc {
		public static final String sampleLL1 = "";
		public static final String sampleLL2 = ""; 

		public static final int[] sampleLL1_VALID_IDS = {1, 2, 12, 13};
		public static final int[] sampleLL2_VALID_IDS = {1, 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
	}
	//#region Manipulator
	public static final class IntakeC { // FIXME get real values fpr both
		public static final int INTAKE_ID = 1;
		public static final int CONVEYOR_ID = 1;
		public static final double INTAKE_SPEED = 0.1;
		public static final double CONVEYOR_SPEED = 0.5;
	}	
	public static final class OuttakeC { 
		public static final int OUTTAKE_ID = 14;
		public static final int OUTTAKE_FOLLOWER_ID = 6;
		public static final int OUTTAKE_FEEDER_ID = 7;
		public static final MotorConfig OUTTAKE_MASTER_MOTOR_CONFIG = MotorConfig.NEO_VORTEX;
		public static final MotorConfig OUTTAKE_FOLLOWER_MOTOR_CONFIG = MotorConfig.NEO_VORTEX;
		public static final MotorConfig OUTTAKE_FEEDER_MOTOR_CONFIG = MotorConfig.NEO_VORTEX;
		public static final double OUTTAKE_RPM = 60;
		public static final double kP = 0.01;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double OUTTAKE_FEEDER_VOLT_PERC = 0.7;
		public static final double OUTTAKE_ESTIMATE_OFFSET = 0; //+- range for atGoal
	}
}
//#endregion
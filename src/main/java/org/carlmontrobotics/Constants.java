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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

			public static final int SLOW_DRIVE_BUTTON = Button.kLeftBumper.value;
			public static final int RESET_FIELD_ORIENTATION_BUTTON = Button.kRightBumper.value;

			public static final int LOCK_WHEELS_BUTTON = Button.kX.value;

			public static final Axis RIGHT_TRIGGER_BUTTON = Axis.kRightTrigger;
			public static final Axis LEFT_TRIGGER_BUTTON = Axis.kLeftTrigger;

			public static final int y = Button.kY.value;
			public static final int b = Button.kB.value;
			public static final int a = Button.kA.value;
		}

		public static final class Manipulator {
			public static final int port = 1;
			public static final Axis SMART_SHOOT_CLOSE_AXIS = Axis.kRightTrigger;
			public static final Axis SMART_SHOOT_FAR_AXIS = Axis.kLeftTrigger;

			public static final int DEPLOY_INTAKE_POV = 180;
			public static final int COLLAPSE_INTAKE_POV = 0;

			public static final int RAISE_INTAKE_BUMP_TOGGLE_BUTTON = Button.kLeftBumper.value;
			public static final int PREPARE_SHOOTER_BUTTON = Button.kRightBumper.value;

			public static final int INTAKE_CONVEYOR_BUTTON = Button.kB.value;
			public static final int REPEL_BALLS_BUTTON = Button.kA.value;

			public static final int TESTING = Button.kX.value;


			public static final int OUTTAKE_FEEDER_BUTTON = Button.kY.value;
		}

		public static final double JOY_THRESH = 0.13;
		public static final double MIN_AXIS_TRIGGER_VALUE = 0.2;

	}

    //#region Drivetrain
	public static final class Drivetrainc {
		public static final double wheelBase = CONFIG.isHammerHead() ? Units.inchesToMeters(16.750003) : Units.inchesToMeters(20.5); 
		public static final double trackWidth = CONFIG.isHammerHead() ? Units.inchesToMeters(23.750000) : Units.inchesToMeters(20.5);
		// "swerveRadius" is the distance from the center of the robot to one of the modules
		public static final double swerveRadius = Math.sqrt(Math.pow(wheelBase / 2, 2) + Math.pow(trackWidth / 2, 2));
		// The gearing reduction from the drive motor controller to the wheels

		public static final double driveGearing = 6.86; //if no worky try 8.16
		// Turn motor shaft to "module shaft"
		public static final double turnGearing = 12.8;

		public static final double driveModifier = 1;
		public static final double wheelDiameterMeters = Units.inchesToMeters(4.0);
																				
		public static final double mu = 1; /* 70/83.2; */ // coefficient of friction. less means less max acceleration.
		public static final double ROBOTMASS_KG = Units.lbsToKilograms(128.1);
		// moment of inertia, kg/mm
		// USE ONSHAPE it has a calculator for this
		public static final double MOI = Math.pow(Units.inchesToMeters(1),2) * Units.lbsToKilograms(1) * 14040.21738; // 14040.21738 in^2 lb 

		public static final double NEOFreeSpeed = 5676 * (2 * Math.PI) / 60; // radians/s
		public static final double VortexFreeSpeed = 6784 * (2 * Math.PI) / 60; // radians/s
		// Angular speed to translational speed --> v = omega * r / gearing
		public static final double maxSpeed = (CONFIG.isVortexDrive() ? VortexFreeSpeed : NEOFreeSpeed) * (wheelDiameterMeters / 2.0) / driveGearing; // meter/s
		public static final double maxForward = maxSpeed; // todo: use smart dashboard to figure this out
		public static final double maxStrafe = maxSpeed; // todo: use smart dashboard to figure this out
		// seconds it takes to go from 0 to 12 volts(aka MAX)
		public static final double secsPer12Volts = 0.1;

		// maxRCW is the angular velocity of the robot.
		// Calculated by looking at one of the motors and treating it as a point mass
		// moving around in a circle.
		// Tangential speed of this point mass is maxSpeed and the radius of the circle
		// is sqrt((wheelBase/2)^2 + (trackWidth/2)^2)
		// Angular velocity = Tangential speed / radius
		public static final double maxRCW = maxSpeed / swerveRadius;

		public static final boolean[] reversed = { false, false, false, false };
		// public static final boolean[] reversed = {true, true, true, true};
		// Determine correct turnZero constants (FL, FR, BL, BR)
		public static final double[] turnZeroDeg = RobotBase.isSimulation() ? new double[] {-90.0, -90.0, -90.0, -90.0 }
		: (CONFIG.isHammerHead() ? new double[] { 85.7812, 85.0782, -96.9433, -162.9492 }
			: new double[] {-37.08984375, -156.708984375,-135.263671875, 81.474609375});/* real values here */
			

		// kP, kI, and kD constants for turn motor controllers in the order of
		// front-left, front-right, back-left, back-right.
		// Determine correct turn PID constants
		public static final double[] turnkP = CONFIG.isHammerHead() ? new double[] {50,50,50,50} : 
			new double[]{44,44,44,44};
		
		
		public static final double[] turnkI = CONFIG.isHammerHead() ? new double[] {0, 0, 0, 0} :
			new double[] {0,0,0,0};
		public static final double[] turnkD = CONFIG.isHammerHead() ? new double[] {0, 0, 0, 0 } :
			new double[] {0,0,0,0};

		public static final double[] turnkS = new double[]{ 0.2, 0.2, 0.2, 0.2};
		public static final double[] turnkV = new double[] { 0, 0, 0, 0 };//good starting point
		public static final double[] turnkA = new double[] { 0, 0, 0, 0 };

		// Order of modules: (FL, FR, BL, BR)
		public static final double[] drivekP = CONFIG.isHammerHead() ? new double[] {2, 2, 2, 2}:
			new double[] {2, 2, 2, 2};
		public static final double[] drivekI = CONFIG.isHammerHead() ? new double[]{ 0, 0, 0, 0} : 
			new double[] {0, 0, 0, 0};
		public static final double[] drivekD = CONFIG.isHammerHead()? new double[] { 0, 0, 0, 0 }:
			new double[] { 0,0,0,0 };
		public static final boolean[] driveInversion = (CONFIG.isHammerHead()
		? new boolean[] { true, false, true, false }
		: new boolean[] { false, true, false, 
			true});
		public static final boolean[] turnInversion = { false, false, false, false };
		
		// kS
		public static final double[] kForwardVolts = CONFIG.isHammerHead() ? new double[] { 0,0,0,0 }:
			new double[] {0, 0, 0, 0}; //HIGHLY NOT RECOMMENDED can cause drift keep at 0 is better
		public static final double[] kBackwardVolts = kForwardVolts;

		//kV
		public static final double[] kForwardVels = CONFIG.isHammerHead() ? new double[] {2,2,2,2 }:
			new double[] { 2, 2, 2, 2 };
		public static final double[] kBackwardVels = kForwardVels;

		//kA
		public static final double[] kForwardAccels = { 0, 0, 0, 0 };
		public static final double[] kBackwardAccels = kForwardAccels;

		public static final double autoMaxSpeedMps = 2 ; // Meters / second
		public static final double autoMaxAccelMps2 = mu * g; // Meters / seconds^2
		public static final double autoMaxAmps = 40.0; 
		// The maximum acceleration the robot can achieve is equal to the coefficient of
		// static friction times the gravitational acceleration
		// a = mu * 9.8 m/s^2
		public static final double autoCentripetalAccel = mu * g * 2;

		public static final boolean isGyroReversed = true;

		public static final double[] thetaPIDController = CONFIG.isHammerHead() ? new double[] { 0.10, 0.0, 0.001 }
		: new double[] {0, 0, 0};

		public static final SwerveConfig swerveConfig = new SwerveConfig(wheelDiameterMeters, driveGearing, mu,
		autoCentripetalAccel, kForwardVolts, kForwardVels, kForwardAccels, kBackwardVolts, kBackwardVels,
		kBackwardAccels, drivekP, drivekI, drivekD, turnkP, turnkI, turnkD, turnkS, turnkV, turnkA, turnZeroDeg,
		driveInversion, reversed, driveModifier, turnInversion);

		public static final int driveFrontLeftPort = 1;
		public static final int driveFrontRightPort = 2;
		public static final int driveBackLeftPort = 3;
		public static final int driveBackRightPort = 4;

		public static final int turnFrontLeftPort = CONFIG.isHammerHead() ? 11 : 11;
		public static final int turnFrontRightPort = CONFIG.isHammerHead() ? 12 : 12;
		public static final int turnBackLeftPort = CONFIG.isHammerHead() ? 13 : 13;
		public static final int turnBackRightPort = CONFIG.isHammerHead() ? 14 : 14;

		public static final int canCoderPortFL = CONFIG.isHammerHead() ? 0 : 0; 
		public static final int canCoderPortFR = CONFIG.isHammerHead() ? 1 : 1; 
		public static final int canCoderPortBL = CONFIG.isHammerHead() ? 3 : 3;
		public static final int canCoderPortBR = CONFIG.isHammerHead() ? 2 : 2; 

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
			public static final PathConstraints pathConstraints = new PathConstraints(3.5, 1, Math.PI-0.5, Math.PI-0.5); // The constraints for this path. If using a differential drivetrain, the
									// angular constraints have no effect.
		}
	}
	//#endregion
	public static class LimeLightc {
		public static final String LEFT_LL = "LL_LEFT";
		public static final String RIGHT_LL = "LL_RIGHT"; 

		public static final int[] LL_FRONT_SHOOTING_VALID_IDS = {9,10, 12, 7, 8,5, 11, 2, 18, 27, 21, 24, 25, 26};
		public static final int[] LL_GENERAL_VALID_IDS = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};

		public static final int[] LL_IDS_IGNORE_FOR_BLUE = {15,16,14,13,9,10,7,12};
		public static final int[] LL_IDS_IGNORE_FOR_RED = {28, 23, 25, 26, 29,30,31,32};

		public static final int[] LL_FRONT_GENERAL_CROP = {0,0,0,0}; //XMin, xMax, yMin, yMax all values (-1,1)
		public static final int[] LL_FRONT_SHOOTING_CROP = {0,0,0,0}; //XMin, xMax, yMin, yMax all values (-1,1)
	}	
	//#region Manipulator
	public static final class IntakeC {
		public static final class OldIntakeC {
			public static final int INTAKE_ID = 24;
			public static final int INTAKE_FOLLOWER_ID = 21;
			public static final double INTAKE_SPEED = 2000;
			public static final double kP = 0.0002;
			public static final double kI = 0;
			public static final double kD = 0;
			public static final int smartCurrentLimit = 80;
		}
		public static final class NewIntakeC {
			public static final class ArmC {
				public static final int INTAKE_ARM_MOTOR_ID = 43;
				public static final double ARM_kClearanceBumpAngle = 105;
				public static final double ARM_kDeployedAngle = 136.747;
				public static final double ARM_kLessPartialIn = 111.747; //For using intake to shove balls down the feeder
				public static final double ARM_kPartialIn = 86.747;
				public static final double ARM_kMorePartialIn = 56.747;
				public static final double ARM_kStowedAngle = 0;

				public static final double ARM_timeToStartContract = 0.75;

				public static final double ARM_BIG_ESTIMATE_OFFSET = 12; //TODO
				public static final double ARM_SMALL_ESTIMATE_OFFSET = 5; //TODO
				public static final double kP = 0; //TODO
				public static final double kI = 0;
				public static final double kD = 0;
			}
			public static final class RollerC {
				public static final int INTAKE_ID = 41;
				public static final int INTAKE_FOLLOWER_ID = 42;
				public static final double INTAKE_SPEED = 2000; //TODO
				public static final double kP = 0.0002; //TODO
				public static final double kI = 0;
				public static final double kD = 0;
				public static final int smartCurrentLimit = 80;
			}
		}
	}	

	public static final class ConveyorC {
		public static final int CONVEYOR_ID = 22;
		public static final double CONVEYOR_SPEED = 0.9;
	}
	public static final class OuttakeC { 
		public static final int OUTTAKE_ID = 31;
		public static final int OUTTAKE_FOLLOWER_ID = 32;
		public static final int OUTTAKE_FEEDER_ID = 23;

		public static final MotorConfig OUTTAKE_MASTER_MOTOR_CONFIG = MotorConfig.NEO_VORTEX;
		public static final MotorConfig OUTTAKE_FOLLOWER_MOTOR_CONFIG = MotorConfig.NEO_VORTEX;
		public static final MotorConfig OUTTAKE_FEEDER_MOTOR_CONFIG = MotorConfig.NEO_VORTEX;

		public static final double OUTTAKE_SHOOTING_RPM = 2450; //TODO
		public static final double OUTTAKE_PASSING_RPM = 5000; //TODO
		
		public static final double kP = 0.0004;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kV = 0.0018;
		public static final double OUTTAKE_FEEDER_VOLT_PERC = 1;
		public static final double OUTTAKE_ESTIMATE_OFFSET = 100; //+- range for atGoal
	}
	public static final class EyeballAutoC {
		public static final double endShoot = 2;
		public static final double endStrafe = 1;
		public static final double endFastDrive = 2.4;
		public static final double endSlowDrive = 0.1;
		public static final double endRotation = 0.6;
		public static final double endStrafeDrive = 5;
	}
}
//#endregion
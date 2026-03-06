package org.carlmontrobotics.subsystems;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Supplier;

//lib199
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.SensorFactory;
import org.carlmontrobotics.lib199.swerve.SwerveModule;
import org.carlmontrobotics.lib199.swerve.SwerveModuleSim;

import static org.carlmontrobotics.Config.CONFIG;

import edu.wpi.first.hal.SimDouble;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

//wpilib
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

//math
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

//vendordeps
import com.ctre.phoenix6.hardware.CANcoder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

//pathplanner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

//rev
import com.revrobotics.spark.SparkBase;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

//units
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

//Constants
import org.carlmontrobotics.Constants;
import org.carlmontrobotics.commands.DriveCommands.RotateToFieldRelativeAngle;
import org.carlmontrobotics.commands.DriveCommands.TeleopDrive;
import static org.carlmontrobotics.Constants.Drivetrainc.*;

public class Drivetrain extends SubsystemBase {

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private Pose2d autoGyroOffset = new Pose2d(0., 0., new Rotation2d(0.)); //used by PathPlanner for chaining paths
    private SwerveDriveKinematics kinematics = null;
    // private SwerveDriveOdometry odometry = null;
    private SwerveDrivePoseEstimator poseEstimator = null;

    private SwerveModule modules[];
    private String moduleNames[] = {"FL", "FR", "BL", "BR"};
    private boolean fieldOriented = true;
    private double fieldOffset = 0;
    // FIXME not for permanent use!!
    private SparkBase[] driveMotors = new SparkBase[] { null, null, null, null };
    private SparkBase[] turnMotors = new SparkBase[] { null, null, null, null };
    private CANcoder[] turnEncoders = new CANcoder[] { null, null, null, null };
    public final float initPitch;
    public final float initRoll;

    // debug purposes
    private SwerveModule moduleFL;
    private SwerveModule moduleFR;
    private SwerveModule moduleBL;
    private SwerveModule moduleBR;

    private final Field2d field = new Field2d();
    private final Field2d odometryField = new Field2d();
    private final Field2d poseWithLimelightField = new Field2d();

    double accelX;
    double accelY;
    double accelXY;

    private SwerveModuleSim[] moduleSims;
    private SimDouble gyroYawSim;
    private Timer simTimer = new Timer();
    public double extraSpeedMult = 0;

    private double lastSetX = 0, lastSetY = 0, lastSetTheta = 0;
    public enum Mode {
    coast,
    brake,
    toggle
    }

    public final Limelight ll;
    public Drivetrain(Limelight ll) {
        this.ll = ll;
        AutoBuilder();

        // Calibrate Gyro
        {

            double initTimestamp = Timer.getFPGATimestamp();
            double currentTimestamp = initTimestamp;
            while (gyro.isCalibrating() && currentTimestamp - initTimestamp < 10) {
                currentTimestamp = Timer.getFPGATimestamp();
                try {
                    Thread.sleep(1000);// 1 second
                } catch (InterruptedException e) {
                    e.printStackTrace();
                    break;
                }
                System.out.println("Calibrating the gyro...");
            }
            gyro.reset();
            // this.resetFieldOrientation();
            System.out.println("NavX-MXP firmware version: " + gyro.getFirmwareVersion());
            System.out.println("Magnetometer is calibrated: " + gyro.isMagnetometerCalibrated());
        }

        // Setup Kinematics
        {
            // Define the corners of the robot relative to the center of the robot using
            // Translation2d objects.
            // Positive x-values represent moving toward the front of the robot whereas
            // positive y-values represent moving toward the left of the robot.
            Translation2d locationFL = new Translation2d(wheelBase / 2, trackWidth / 2);
            Translation2d locationFR = new Translation2d(wheelBase / 2, -trackWidth / 2);
            Translation2d locationBL = new Translation2d(-wheelBase / 2, trackWidth / 2);
            Translation2d locationBR = new Translation2d(-wheelBase / 2, -trackWidth / 2);

            kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
        }

        // Initialize modules
        {
            // initPitch = 0;
            // initRoll = 0;
            Supplier<Float> pitchSupplier = () -> 0F;
            Supplier<Float> rollSupplier = () -> 0F;
            initPitch = gyro.getPitch();
            initRoll = gyro.getRoll();
            // Supplier<Float> pitchSupplier = () -> gyro.getPitch();
            // Supplier<Float> rollSupplier = () -> gyro.getRoll();


            moduleFL = new SwerveModule(Constants.Drivetrainc.swerveConfig, SwerveModule.ModuleType.FL, 
                driveMotors[0] = MotorControllerFactory.createSpark(driveFrontLeftPort, driveMotorConfig), 
                turnMotors[0] = MotorControllerFactory.createSpark(turnFrontLeftPort, turnMotorConfig), 
                turnEncoders[0] = SensorFactory.createCANCoder(Constants.Drivetrainc.canCoderPortFL), 0, pitchSupplier, rollSupplier);
            
            moduleFR = new SwerveModule(Constants.Drivetrainc.swerveConfig, SwerveModule.ModuleType.FR, 
                driveMotors[1] = MotorControllerFactory.createSpark(driveFrontRightPort, driveMotorConfig), 
                turnMotors[1] = MotorControllerFactory.createSpark(turnFrontRightPort, turnMotorConfig), 
                turnEncoders[1] = SensorFactory.createCANCoder(Constants.Drivetrainc.canCoderPortFR), 1, pitchSupplier, rollSupplier);

            moduleBL = new SwerveModule(Constants.Drivetrainc.swerveConfig, SwerveModule.ModuleType.BL, 
                driveMotors[2] = MotorControllerFactory.createSpark(driveBackLeftPort, driveMotorConfig), 
                turnMotors[2] = MotorControllerFactory.createSpark(turnBackLeftPort, turnMotorConfig), 
                turnEncoders[2] = SensorFactory.createCANCoder(Constants.Drivetrainc.canCoderPortBL), 2, pitchSupplier, rollSupplier);

            moduleBR = new SwerveModule(Constants.Drivetrainc.swerveConfig, SwerveModule.ModuleType.BR, 
                driveMotors[3] = MotorControllerFactory.createSpark(driveBackRightPort, driveMotorConfig), 
                turnMotors[3] = MotorControllerFactory.createSpark(turnBackRightPort, turnMotorConfig),
                turnEncoders[3] = SensorFactory.createCANCoder(Constants.Drivetrainc.canCoderPortBR), 3, pitchSupplier, rollSupplier);
                
            modules = new SwerveModule[] { moduleFL, moduleFR, moduleBL, moduleBR };

            if (RobotBase.isSimulation()) {
                moduleSims = new SwerveModuleSim[] {
                    moduleFL.createSim(), moduleFR.createSim(), moduleBL.createSim(), moduleBR.createSim() //FIXME this is with values based off of hammerhead
                };
                gyroYawSim = new SimDeviceSim("navX-Sensor[0]").getDouble("Yaw");
            }
            SparkBaseConfig driveConfig = MotorControllerFactory.sparkConfig(driveMotorConfig);
            driveConfig.openLoopRampRate(secsPer12Volts);
            driveConfig.encoder.positionConversionFactor(wheelDiameterMeters * Math.PI / driveGearing);
            driveConfig.encoder.velocityConversionFactor(wheelDiameterMeters * Math.PI / driveGearing / 60);
            driveConfig.encoder.uvwAverageDepth(2);
            driveConfig.encoder.uvwMeasurementPeriod(16);

            for (SparkBase driveMotor : driveMotors) {
                driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            }
            SparkBaseConfig turnConfig = MotorControllerFactory.sparkConfig(turnMotorConfig);
            turnConfig.encoder.positionConversionFactor(360/turnGearing);
            turnConfig.encoder.velocityConversionFactor(360/turnGearing/60);
            turnConfig.encoder.uvwAverageDepth(2);
            turnConfig.encoder.uvwMeasurementPeriod(16);

            for (SparkBase turnMotor : turnMotors) {
                turnMotor.configure(turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            }

            for (CANcoder coder : turnEncoders) {
                coder.getAbsolutePosition().setUpdateFrequency(500);
                coder.getPosition().setUpdateFrequency(500);
                coder.getVelocity().setUpdateFrequency(500);
            }
           
            accelX = gyro.getWorldLinearAccelX(); // Acceleration along the X-axis
            accelY = gyro.getWorldLinearAccelY(); // Acceleration along the Y-axis
            accelXY = Math.sqrt(gyro.getWorldLinearAccelX() * gyro.getWorldLinearAccelX() + gyro.getWorldLinearAccelY() * gyro.getWorldLinearAccelY());

        }

        poseEstimator = new SwerveDrivePoseEstimator(
                getKinematics(),
                Rotation2d.fromDegrees(getHeading()),
                getModulePositions(),
                new Pose2d());

        // Setup autopath builder
        //configurePPLAutoBuilder();
    }



    public boolean isAtAngle(double desiredAngleDeg, double toleranceDeg){
        for (SwerveModule module : modules) { 
            if (!(Math.abs(MathUtil.inputModulus(module.getModuleAngle() - desiredAngleDeg, -90, 90)) < toleranceDeg)) 
                return false;
        }
        return true;
    }

    @Override
    public void simulationPeriodic() {
        for (var moduleSim : moduleSims) {
            moduleSim.update();
        }
        SwerveModuleState[] measuredStates =
            new SwerveModuleState[] {
                moduleFL.getCurrentState(), moduleFR.getCurrentState(), moduleBL.getCurrentState(), moduleBR.getCurrentState()
            };
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(measuredStates);

        double dtSecs = simTimer.get();
        simTimer.restart();

        Pose2d simPose = field.getRobotPose();
        simPose = simPose.exp(
                new Twist2d(
                    speeds.vxMetersPerSecond * dtSecs,
                    speeds.vyMetersPerSecond * dtSecs,
                    speeds.omegaRadiansPerSecond * dtSecs));
        double newAngleDeg = simPose.getRotation().getDegrees();
        // Subtract the offset computed the last time setPose() was called because odometry.update() adds it back.
        newAngleDeg -= simGyroOffset.getDegrees();
        newAngleDeg *= (isGyroReversed ? -1.0 : 1.0);
        gyroYawSim.set(newAngleDeg);
    }

    /**
     * Sets swerveModules IdleMode both turn and drive
     * @param brake boolean for braking, if false then coast
     */
    public void setDrivingIdleMode(boolean brake) {
        IdleMode mode;
        if (brake) {
            mode = IdleMode.kBrake;
        }
        else {
            mode = IdleMode.kCoast;
        }
        for (SparkBase turnMotor : turnMotors) {
            SparkBaseConfig config = MotorControllerFactory.createConfig(turnMotorConfig.controllerType);
            config.idleMode(mode);
            turnMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);      
        }
        for (SparkBase driveMotor : driveMotors) {
            SparkBaseConfig config = MotorControllerFactory.createConfig(driveMotorConfig.controllerType);
            config.idleMode(mode);
            driveMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);     
        }
    }

    @Override
    public void periodic() {
        detectCollision(); //This does nothing
        PathPlannerLogging.logCurrentPose(getPose());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSafeState(this::stop);
        for (SwerveModule module : modules){
            SendableRegistry.addChild(this, module);
        }
        for (int i = 0; i < 4; i++) {
            final int j = i; //make java happy
            builder.addDoubleProperty(moduleNames[j] + "Turn Encoder (Deg)", () -> modules[j].getModuleAngle(), null);
        }
        SendableRegistry.addChild(this, CONFIG);
        builder.addBooleanProperty("Magnetic Field Disturbance", gyro::isMagneticDisturbance, null);
        builder.addBooleanProperty("Gyro Calibrating", gyro::isCalibrating, null);
        builder.addBooleanProperty("Field Oriented", () -> fieldOriented, fieldOriented -> this.fieldOriented = fieldOriented);
        builder.addDoubleProperty("Pose Estimator X", () -> getPose().getX(), null);
        builder.addDoubleProperty("Pose Estimator Y", () -> getPose().getY(),null);
        builder.addDoubleProperty("Pose Estimator Theta", () -> getPose().getRotation().getDegrees(), null);
        builder.addDoubleProperty("X position with gyro", () -> getPose().getX(), null);
        builder.addDoubleProperty("Y position with gyro", () -> getPose().getY(), null);
        builder.addDoubleProperty("Robot Heading", () -> getHeading(), null);
        builder.addDoubleProperty("Raw Gyro Angle", gyro::getAngle, null);
        builder.addDoubleProperty("Pitch", gyro::getPitch, null);
        builder.addDoubleProperty("Roll", gyro::getRoll, null);
        builder.addDoubleProperty("Accel X", () -> accelX, null);
        builder.addDoubleProperty("Accel Y", () -> accelY, null);
        builder.addDoubleProperty("2D Acceleration ", () -> accelXY, null);
        builder.addDoubleProperty("Field Offset", () -> fieldOffset, fieldOffset -> this.fieldOffset = fieldOffset);
    }


    // #region Drive Methods

    /**
     * Drives the robot using the given x, y, and rotation speed
     *
     * @param forward  The desired forward speed, in m/s. Forward is positive.
     * @param strafe   The desired strafe speed, in m/s. Left is positive.
     * @param rotation The desired rotation speed, in rad/s. Counter clockwise is
     *                 positive
     */
    public void setExtraSpeedMult(double set) {
        extraSpeedMult=set;
    }
    
    /**
     * Calculates and implements the required SwerveStates for all 4 modules to get the wanted outcome
     * @param forward The desired forward speed, in m/s. Forward is positive.
     * @param strafe The desired strafe speed, in m/s. Left is positive.
     * @param rotation The desired rotation speed, in rad/s. Counter clockwise is positive.
     */
    public void drive(double forward, double strafe, double rotation) {
        drive(getSwerveStates(forward, strafe, rotation));
    }
    
    /**
     * Implements the provided SwerveStates for all 4 modules to get the wanted outcome
     * @param moduleStates SwerveModuleState[]
     */
    public void drive(SwerveModuleState[] moduleStates) {
        //Max speed override
        double max = maxSpeed;
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, max);
        for (int i = 0; i < 4; i++) {
            moduleStates[i].optimize(Rotation2d.fromDegrees(modules[i].getModuleAngle()));
            modules[i].move(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle.getDegrees());
        }
    }
    
    /**
     * Configures PathPlanner AutoBuilder
     */
    public void AutoBuilder() {
        RobotConfig config = Constants.Drivetrainc.Autoc.robotConfig;
        AutoBuilder.configure(
                //Supplier<Pose2d> poseSupplier,
                this::getPose, // Robot pose supplier
                //Consumer<Pose2d> resetPose,
                this::setPoseWithLimelight, // Method to reset odometry (will be called if your auto has a starting pose)
                //Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                //BiConsumer<ChassisSpeeds,DriveFeedforwards> output,
                (speeds, feedforwards) -> drive(kinematics.toSwerveModuleStates(speeds)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                //PathFollowingController controller,
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(ppkPDrive, ppkIDrive, ppkDDrive), // Translation PID constants
                        new PIDConstants(ppkPTurn, ppkITurn, ppkDTurn)
                ),
                config, // The robot configuration
                //BooleanSupplier shouldFlipPath,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                //Subsystem... driveRequirements
                this // Reference to this subsystem to set requirements
            );
        }

//----------------------------------------------------------

   public void autoCancelDtCommand() {
       if(!(getDefaultCommand() instanceof TeleopDrive) || DriverStation.isAutonomous()) return;

        // Use hasDriverInput to get around acceleration limiting on slowdown
        if (((TeleopDrive) getDefaultCommand()).hasDriverInput()) {
            Command currentDtCommand = getCurrentCommand();
            if (currentDtCommand != getDefaultCommand() && !(currentDtCommand instanceof RotateToFieldRelativeAngle)
                    && currentDtCommand != null) {
                currentDtCommand.cancel();
            }
        }
    }

    public void stop() {
        for (SwerveModule module : modules)
            module.move(0, 0);
    }

    public boolean isStopped() {
        return Math.abs(getSpeeds().vxMetersPerSecond) < 0.1 &&
                Math.abs(getSpeeds().vyMetersPerSecond) < 0.1 &&
                Math.abs(getSpeeds().omegaRadiansPerSecond) < 0.1;
    }

    /**
     * Constructs and returns a ChassisSpeeds objects using forward, strafe, and
     * rotation values.
     *
     * @param forward  The desired forward speed, in m/s. Forward is positive.
     * @param strafe   The desired strafe speed, in m/s. Left is positive.
     * @param rotation The desired rotation speed, in rad/s. Counter clockwise is
     *                 positive.
     * @return A ChassisSpeeds object.
     */
    private ChassisSpeeds getChassisSpeeds(double forward, double strafe, double rotation) {
        ChassisSpeeds speeds;
        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation,
                    Rotation2d.fromDegrees(getHeading()));
        } else {
            speeds = new ChassisSpeeds(forward, strafe, rotation);
        }
        return speeds;
    }

    /**
     * Constructs and returns four SwerveModuleState objects, one for each side,
     * using forward, strafe, and rotation values.
     *
     * @param forward  The desired forward speed, in m/s. Forward is positive.
     * @param strafe   The desired strafe speed, in m/s. Left is positive.
     * @param rotation The desired rotation speed, in rad/s. Counter clockwise is
     *                 positive.
     * @return A SwerveModuleState array, one for each side of the drivetrain (FL,
     *         FR, etc.).
     */
    private SwerveModuleState[] getSwerveStates(double forward, double strafe, double rotation) {
        return kinematics.toSwerveModuleStates(getChassisSpeeds(forward, -strafe, rotation));
    }

    public SwerveModuleState[] getSwerveStates(ChassisSpeeds speeds) {
        return kinematics.toSwerveModuleStates(speeds);
    }

    // #endregion

    // #region Getters and Setters

    /**
     * @return the heading in degrees, normalized to the range -180 to 180
     */
    public double getHeading() {
        double x = gyro.getAngle();
        if (fieldOriented)
            x -= fieldOffset;
        return Math.IEEEremainder(x * (isGyroReversed ? -1.0 : 1.0), 360);
    }

    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map(SwerveModule::getCurrentPosition).toArray(SwerveModulePosition[]::new);
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    /**
     * Gets pose from {@link #poseEstimator}
     * @return Pose2D
     */
    public Pose2d getPose() {
        // return odometry.getPoseMeters();
        return poseEstimator.getEstimatedPosition();
    }

    private Rotation2d simGyroOffset = new Rotation2d();
    public void setPose(Pose2d initialPose) {
        Rotation2d gyroRotation = gyro.getRotation2d();
        // odometry.resetPosition(gyroRotation, getModulePositions(), initialPose);

        poseEstimator.resetPosition(gyroRotation, getModulePositions(), initialPose);
        // Remember the offset that the above call to resetPosition() will cause the odometry.update() will add to the gyro rotation in the future
        // We need the offset so that we can compensate for it during simulationPeriodic().
        simGyroOffset = initialPose.getRotation().minus(gyroRotation);
        //odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), getModulePositions(), initialPose);
    }

    //This method will set the pose using limelight if it sees a tag and if not it is supposed to run like setPose()
    public void setPoseWithLimelight(Pose2d backupPose){ //the pose will be set to backupPose if no tag is seen
    //     Rotation2d gyroRotation = gyro.getRotation2d();
    //     Pose2d pose;

    //     if (LimelightHelpers.getTV(REEF_LL)) {
            
    //         pose = LimelightHelpers.getBotPose2d_wpiBlue(REEF_LL);

    //     } else if (LimelightHelpers.getTV(CORAL_LL)) {

    //         pose = LimelightHelpers.getBotPose2d_wpiBlue(CORAL_LL);
    //     }
    //     else {
    //         pose = backupPose;
    //     }

    //     poseEstimator.resetPosition(gyroRotation, getModulePositions(), pose);
    //     simGyroOffset = pose.getRotation().minus(gyroRotation);
        
    }
    /**
     * Detects if the robot has experienced a collision based on acceleration thresholds.
     * @return true if a 2D acceleration greater than the {@link #COLLISION_ACCELERATION_THRESHOLD} false otherwise.
     */
    public boolean detectCollision(){ //We can implement this method into updatePoseWithLimelight so that if there is a collision it stops using odometry
        accelX = gyro.getWorldLinearAccelX(); // Acceleration along the X-axis
        accelY = gyro.getWorldLinearAccelY(); // Acceleration along the Y-axis
        accelXY = Math.sqrt(accelX * accelX + accelY * accelY); // 2D Acceleration
        return accelXY > COLLISION_ACCELERATION_THRESHOLD; // return true if collision detected
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }
    /**
     * true stands for fieldOriented, false stands for robotOriented
     * @return boolean
     */
    public boolean getFieldOriented() {
        return fieldOriented;
    }

    /**
     * True sets fieldOriented, false sets robotOriented
     * @param fieldOriented boolean
     */
    public void setFieldOriented(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
    }
    /**
     * Sets the current direction the robot is facing is to be 0
     */
    public void resetFieldOrientation() {
        fieldOffset = gyro.getAngle();
    }
    /**
     * Sets the current direction the robot is facing is to be 180
     */
    public void resetFieldOrientationBackwards() {
        fieldOffset = 180 + gyro.getAngle();
    }
    /**
     * Sets the current direction the robot is facing is plus @param angle to be 0
     * @param angle in degrees
     */
    public void resetFieldOrientationWithAngle(double angle) {
        fieldOffset = angle + gyro.getAngle();
    }
    public void resetPoseEstimator() {
        // odometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());

        poseEstimator.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());
        gyro.reset();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(Arrays.stream(modules).map(SwerveModule::getCurrentState)
                .toArray(SwerveModuleState[]::new));
    }

    public void setMode(Mode mode) {
        for (SwerveModule module : modules){
            switch (mode) {
                case coast:
                    module.coast();
                    break;
                case brake:
                    module.brake();
                    break;
                case toggle:
                    module.toggleMode();
                    break;
            }
        }
    }
    /**
     * Changes between IdleModes
     */
    public void toggleMode() {
        for (SwerveModule module : modules)
            module.toggleMode();
    }

    public void brake() {
        for (SwerveModule module : modules)
            module.brake();
    }

    public void coast() {
        for (SwerveModule module : modules)
            module.coast();
    }

    /**
     * Sets all SwerveModules to point in a certain angle
     * @param angle in degrees
     */
    public void keepRotateMotorsAtDegrees(int angle) {
        for (SwerveModule module : modules) {
            module.turnPeriodic();
            module.move(0, angle);
        }
    }

    /**
     * Gets how fast the robot is spinning from gyro
     * @return degrees per second
     */
    public double getGyroRate() {
        return gyro.getRate();
    }
}

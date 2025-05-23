package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.TimestampedVisionUpdate;
import frc.robot.util.GRTUtil;

import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.DebugConstants.*;

import static frc.robot.Constants.LoggingConstants.*;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class SwerveSubsystem extends SubsystemBase {

    private final KrakenSwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    private SwerveModuleState[] states = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private Pose2d estimatedPose;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    private Rotation2d driverHeadingOffset = new Rotation2d();

    private final AHRS ahrs;
    private Timer lockTimer;

    //logging
    private NetworkTableInstance ntInstance;
    private NetworkTable swerveTable;

    private StructArrayPublisher<SwerveModuleState> swerveStatesPublisher;

    private StructPublisher<Pose2d> estimatedPosePublisher;
    private StructLogEntry<Pose2d> estimatedPoseLogEntry =
        StructLogEntry.create(
            DataLogManager.getLog(),
            "estimatedPose",
            Pose2d.struct
        );

    public SwerveSubsystem() {
        //initialize and reset the NavX gyro
        ahrs = new AHRS(NavXComType.kMXP_SPI);
        ahrs.reset();
        ahrs.zeroYaw();

        frontLeftModule = new KrakenSwerveModule(FL_DRIVE, FL_STEER, FL_OFFSET, FL_ENCODER);
        frontRightModule = new KrakenSwerveModule(FR_DRIVE, FR_STEER, FR_OFFSET, FR_ENCODER);
        backLeftModule = new KrakenSwerveModule(BL_DRIVE, BL_STEER, BL_OFFSET, BL_ENCODER);
        backRightModule = new KrakenSwerveModule(BR_DRIVE, BR_STEER, BR_OFFSET, BR_ENCODER);

        kinematics = new SwerveDriveKinematics(FL_POS, FR_POS, BL_POS, BR_POS);
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics, 
            getGyroHeading(), 
            getModulePositions(),
            new Pose2d()
            );

        buildAuton(); 
        initNT();

        if(DRIVE_DEBUG) {
            enableDriveDebug();
        }
        if(STEER_DEBUG) {
            enableSteerDebug();
        }

        lockTimer = new Timer();
    }

    @Override
    public void periodic() {
        //update the poseestimator with curent gyro reading      
        Rotation2d gyroAngle = getGyroHeading();
        estimatedPose = poseEstimator.update(
            gyroAngle,
            getModulePositions()
        );

        // If all commanded velocities are 0, the system is idle (drivers / commands are
        // not supplying input).
        boolean isIdle = states[0].speedMetersPerSecond == 0.0
            && states[1].speedMetersPerSecond == 0.0
            && states[2].speedMetersPerSecond == 0.0
            && states[3].speedMetersPerSecond == 0.0;

        // Start lock timer when idle
        if (isIdle) {
            lockTimer.start();
        } else {
            lockTimer.stop();
            lockTimer.reset();
        }

        // Lock the swerve module if the lock timeout has elapsed, or set them to their 
        // setpoints if drivers are supplying non-idle input.
        if (lockTimer.hasElapsed(1)) {
            applyLock();
        } else {
            //update the swerve modules based on the current desired states from states[]
            frontLeftModule.setDesiredState(states[0]);
            frontRightModule.setDesiredState(states[1]);
            backLeftModule.setDesiredState(states[2]);
            backRightModule.setDesiredState(states[3]);
        }

        //logging
        estimatedPoseLogEntry.append(estimatedPose, GRTUtil.getFPGATime()); 
        // publishStats();
        // logStats();
    }

    /**
     * Sets the powers of the drivetrain through PIDs. Relative to the driver heading on the field.
     *
     * @param xPower [-1, 1] The forward power.
     * @param yPower [-1, 1] The left power.
     * @param angularPower [-1, 1] The rotational power.
     */
    public void setDrivePowers(double xPower, double yPower, double angularPower) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xPower * MAX_VEL, 
            yPower * MAX_VEL, 
            angularPower * MAX_OMEGA,
            getDriverHeading()
        );

        states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds,
            MAX_VEL, MAX_VEL, MAX_OMEGA
        );
    }

        /** Executes swerve X locking, putting swerve's wheels into an X configuration to prevent motion.
     */
    public void applyLock() {
        frontLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
        frontRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
        backLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
        backRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
    }

    public void addVisionMeasurements(TimestampedVisionUpdate update) {
        poseEstimator.addVisionMeasurement(
            update.pose(), 
            update.timestamp(),
            update.stdDevs()
        );
    }

    /**
     * Gets the module positions.
     * 
     * @return The array of module positions.
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        };
    }

    /**
     * Gets the states of the module
     * 
     * @return The array of module states
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        };
    }

    /**
     * Gets the driver heading.
     *
     * @return The angle of the robot relative to the driver heading.
     */
    public Rotation2d getDriverHeading() {
        Rotation2d robotHeading = ahrs.isConnected() ? getGyroHeading() : getRobotPosition().getRotation();
        return robotHeading.minus(driverHeadingOffset);
    }

    /**
     * Resets the driver heading.
     *
     * @param currentRotation The new driver heading.
     */
    public void resetDriverHeading(Rotation2d currentRotation) {
        driverHeadingOffset = getGyroHeading().minus(currentRotation);
    }

    /** 
     * Resets the driver heading to 0. 
     */
    public void resetDriverHeading() {
        resetDriverHeading(new Rotation2d());
    }


    /** Gets the gyro heading.*/
    private Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(-ahrs.getAngle()); // Might need to flip depending on the robot setup
    }

    /**
     * Gets the current robot pose.
     *
     * @return The robot Pose2d.
     */
    public Pose2d getRobotPosition() {
        return estimatedPose;
    }

    public void resetPose(Pose2d currentPose) {
        Rotation2d gyroAngle = getGyroHeading();
        poseEstimator.resetPosition(
            gyroAngle, 
            getModulePositions(), 
            currentPose
        );
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        SwerveModuleState[] currentModuleStates = getModuleStates();
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            kinematics.toChassisSpeeds(currentModuleStates),
            getRobotPosition().getRotation() // Could be replaced with getGyroHeading() if desired
        );
        return robotRelativeSpeeds;
    }
    
    public void setRobotRelativeDrivePowers(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds,
            new Rotation2d(0)
        );

        states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds,
            MAX_VEL, MAX_VEL, MAX_OMEGA
        );
    }

        /**
     * Sets the power of the drivetrain through PIDs. Relative to the robot with the intake in the front.
     *
     * @param xPower [-1, 1] The forward power.
     * @param yPower [-1, 1] The left power.
     * @param angularPower [-1, 1] The rotational power.
     */
    public void setRobotRelativeDrivePowers(double xPower, double yPower, double angularPower) {
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            xPower * MAX_VEL, 
            yPower * MAX_VEL, 
            angularPower * MAX_OMEGA, 
            new Rotation2d(0)
        );

        states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds,
            MAX_VEL, MAX_VEL, MAX_OMEGA);
    }

    private void initNT() {
        ntInstance = NetworkTableInstance.getDefault();
        swerveTable = ntInstance.getTable(SWERVE_TABLE);

        swerveStatesPublisher = swerveTable.getStructArrayTopic(
            "SwerveStates", SwerveModuleState.struct
        ).publish(); 

        estimatedPosePublisher = swerveTable.getStructTopic(
            "estimatedPose",
            Pose2d.struct
        ).publish();
    }

    // /**
    //  * publishes swerve stats to NT
    //  */
    // private void publishStats() {
    //     estimatedPosePublisher.set(estimatedPose);

    //     if(STATE_DEBUG || DRIVE_DEBUG || STEER_DEBUG) {
    //         swerveStatesPublisher.set(getModuleStates());
    //     }

    //     if(DRIVE_DEBUG) {
    //         frontLeftModule.publishDriveStats();
    //         frontRightModule.publishDriveStats();
    //         backLeftModule.publishDriveStats();
    //         backRightModule.publishDriveStats();
    //     }

    //     if(STEER_DEBUG) {
    //         frontLeftModule.publishSteerStats();
    //         frontRightModule.publishSteerStats();
    //         backLeftModule.publishSteerStats();
    //         backRightModule.publishSteerStats();
    //     }
    // }

    // private void logStats() {
    //     frontLeftModule.logStats();
    //     frontRightModule.logStats();
    //     backLeftModule.logStats();
    //     backRightModule.logStats();
    // }

    /**
     * Enables drive debug
     */
    private void enableDriveDebug() {
        frontLeftModule.driveDebug();
        frontRightModule.driveDebug();
        backLeftModule.driveDebug();
        backRightModule.driveDebug();
    }

    /**
     * Enables steer debug
     */
    private void enableSteerDebug() {
        frontLeftModule.steerDebug();
        frontRightModule.steerDebug();
        backLeftModule.steerDebug();
        backRightModule.steerDebug();
    }

    /** 
     * Builds the auton builder
     */
    private void buildAuton() {
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            // Handle exception as needed, maybe use default values or fallback
        }

        AutoBuilder.configure(
            this::getRobotPosition,
            this::resetPose,
            this::getRobotRelativeChassisSpeeds,
            (speeds, feedforwards) -> setRobotRelativeDrivePowers(speeds),
            
            //1.25/3.25
            new PPHolonomicDriveController(
                new PIDConstants(1.38, 0, 0.0),
                new PIDConstants(3.3, 0.0, 0.0)
            ),

            config,
            ()->{
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get()==DriverStation.Alliance.Red;
                }
                return false; 
            },
            this
        );
    }
}
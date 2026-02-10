package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

import frc.robot.generated.TunerConstants;

import frc.robot.Constants;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    /* Cached internal pose estimate (updated via telemetry callback). */
    private volatile Pose2d m_latestPose = new Pose2d();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    // set up drive request specifically for driving to a pose
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(Constants.Swerve.kMaxAngularRps).in(RadiansPerSecond); // max angular velocity
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * Constants.Swerve.kDeadbandFraction)
        .withRotationalDeadband(MaxAngularRate * Constants.Swerve.kDeadbandFraction) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        /* Cache latest pose from internal odometry via telemetry callback */
        registerTelemetry(state -> m_latestPose = state.Pose);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        /* Cache latest pose from internal odometry via telemetry callback */
        registerTelemetry(state -> m_latestPose = state.Pose);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        /* Cache latest pose from internal odometry via telemetry callback */
        registerTelemetry(state -> m_latestPose = state.Pose);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Returns the robot's current internal pose estimate (odometry + any fused vision corrections).
     * This is updated asynchronously by a telemetry callback registered in the constructors.
     *
     * @return current Pose2d estimate (meters, radians)
     */
    public Pose2d getPose() {
        return m_latestPose;
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    private final PIDController poseForwardPidController = new PIDController(1, 0, 0);
    private final PIDController poseStrafePidController = new PIDController(1, 0, 0);
    private final PIDController poseRotationPidController = new PIDController(0.1, 0, 0);
    private final PIDController pointRotationPidController = new PIDController(0.5, 0, 0);

    /*
     * Drives in a straight trajectory from the current pose to a target pose.
     * 
     * @param pose The desired pose to drive to
     */
    public void driveToPose(Pose2d pose, NetworkTablesIO table) {
        Pose2d currentPose = table.getNetworkPose();
        Pose2d targetPose = pose;

        double[] diff = {
            targetPose.getX() - currentPose.getX(),
            targetPose.getY() - currentPose.getY(),
            targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()
        };

        // Use PIDController.calculate(measurement, setpoint) so the controller
        // computes (setpoint - measurement) internally. Passing the precomputed
        // diff into calculate(...) with a setpoint of 0 inverts the sign and
        // makes reasoning harder.
        double[] calc = new double[3];
        calc[0] = poseForwardPidController.calculate(currentPose.getX(), targetPose.getX());
        calc[1] = poseStrafePidController.calculate(currentPose.getY(), targetPose.getY());

        // Normalize rotation error using Rotation2d.minus to avoid wrap issues.
        Rotation2d rotError = targetPose.getRotation().minus(currentPose.getRotation());
        double angleError = rotError.getRadians(); // already normalized to [-pi, pi]
        // Use controller with measurement=0 and setpoint=angleError so the
        // controller output is proportional to the angular error.
        calc[2] = poseRotationPidController.calculate(0.0, angleError);

        // Publish current/target poses for debugging
        try {
            SmartDashboard.putNumber("driveToPose_currentX", currentPose.getX());
            SmartDashboard.putNumber("driveToPose_currentY", currentPose.getY());
            SmartDashboard.putNumber("driveToPose_currentTheta", currentPose.getRotation().getRadians());
            SmartDashboard.putNumber("driveToPose_targetX", targetPose.getX());
            SmartDashboard.putNumber("driveToPose_targetY", targetPose.getY());
            SmartDashboard.putNumber("driveToPose_targetTheta", targetPose.getRotation().getRadians());
        } catch (Exception ignore) {}

        // max(lower_bound, min(upper_bound, value))
        double[] map = {
            Math.max(-1, Math.min(1, calc[0])),
            Math.max(-1, Math.min(1, calc[1])),
            Math.max(-1, Math.min(1, calc[2]))
        };

        // Apply the computed velocities immediately. Using applyRequest() here
        // returns a Command that must be scheduled — calling it without
        // scheduling does nothing. Instead, call setControl(...) directly so
        // the drivetrain will act on these values when this method is invoked
        // from a repeating command (e.g. DriveToPose.execute()).
        //
        // Scale the normalized [-1, 1] outputs by the drivetrain max speeds
        // so they are in the same units as teleop joystick inputs and to
        // exceed the configured deadband where appropriate.

        // If we're on blue we need to flip these.
        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue) {
            map[0] = -map[0];
            map[1] = -map[1];
        }
        double velX = -map[0] * MaxSpeed;
        double velY = -map[1] * MaxSpeed;
        double rot = -map[2] * MaxAngularRate;

        // Log small debug info to help diagnose alignment behavior.
        try {
            // String msg = String.format("driveToPose diff=(%.3f, %.3f, %.3f) calc=(%.3f, %.3f, %.3f) vel=(%.3f, %.3f, %.3f)",
                // diff[0], diff[1], diff[2], calc[0], calc[1], calc[2], velX, velY, rot);
            // DriverStation warning (should appear in DS), plus console and dashboard as fallback
            // DriverStation.reportWarning(msg, false);
            // System.out.println(msg);
            SmartDashboard.putNumber("driveToPose_velX", velX);
            SmartDashboard.putNumber("driveToPose_velY", velY);
            SmartDashboard.putNumber("driveToPose_rot", rot);
        } catch (Exception ignore) {
            // Never crash on logging
        }

        this.setControl(
            drive.withVelocityX(velX)
                .withVelocityY(velY)
                .withRotationalRate(rot)
        );
    }

    public void pointToPose(Pose2d pose, double velX, double velY, NetworkTablesIO table) {
        Pose2d currentPose = table.getNetworkPose();
        
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        
        Pose2d targetPose = pose;
        double targetX = targetPose.getX();
        double targetY = targetPose.getY();

        double targetXRelative = targetX - currentX;
        double targetYRelative = targetY - currentY;

        double theta = Math.atan2(targetYRelative, targetXRelative) + Math.PI;
        Rotation2d targetRotation2d = new Rotation2d(theta);

        // Rotation2d targetRotation2d = Math.sqrt(targetPose.getX() * targetPose.getX() + targetPose.getY() * targetPose.getY()) > 0.01 ? new Rotation2d(targetPose.getY(), targetPose.getX()) : currentPose.getRotation();
    
        Rotation2d rotError = targetRotation2d.minus(currentPose.getRotation());
        double angleError = rotError.getRadians();

        double calc = pointRotationPidController.calculate(0.0, angleError);

        double minmax = Math.max(-1, Math.min(1, calc));

        double rot = -minmax * MaxAngularRate;

        this.setControl(
            drive.withVelocityX(velX)
                .withVelocityY(velY)
                .withRotationalRate(rot)
        );
    }
}

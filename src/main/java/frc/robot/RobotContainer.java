// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PointTowardsZone;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
import frc.robot.commands.DriveToApriltag;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.ExtendIntake;
import frc.robot.commands.PointToHub;
import frc.robot.commands.PointToPose;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.RunDebugMotors;
import frc.robot.commands.RunIntake;
import frc.robot.commands.UpdatePose;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NetworkTablesIO;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DebugMotors;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    // #region Swerve setup
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(Constants.Swerve.kMaxAngularRps).in(RadiansPerSecond); // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * Constants.Swerve.kDeadbandFraction)
        .withRotationalDeadband(MaxAngularRate * Constants.Swerve.kDeadbandFraction) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final NetworkTablesIO networkTablesIO = new NetworkTablesIO();
    public final Vision vision = new Vision(drivetrain, networkTablesIO);
    // #endregion Swerve setup

    // private final CommandXboxController driveJoystick = new CommandXboxController(Constants.Controller.kDriverControllerPort);
    // private final CommandXboxController operatorJoystick = new CommandXboxController(Constants.Controller.kOperatorControllerPort);

    private final CommandPS4Controller driveJoystick = new CommandPS4Controller(Constants.Controller.kDriverControllerPort);
    private final CommandPS4Controller operatorJoystick = new CommandPS4Controller(Constants.Controller.kOperatorControllerPort);
    
    private final Intake m_intake = new Intake();
    private final DebugMotors m_DebugMotors = new DebugMotors();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        // Warm up on-the-fly path generation
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        configureBindings();
    }

    private void configureBindings() {
        // #region Swerve
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driveJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
            
        );
        
        // Idle while disabled
        final var swerveIdle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> swerveIdle).ignoringDisable(true)
        );

        // Brake while holding right bumper
        // driveJoystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
        driveJoystick.R1().whileTrue(drivetrain.applyRequest(() -> brake));

        // driveJoystick.triangle().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driveJoystick.getLeftY(), -driveJoystick.getLeftX()))
        // ));
 
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driveJoystick.back().and(driveJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driveJoystick.back().and(driveJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driveJoystick.start().and(driveJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driveJoystick.start().and(driveJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        // driveJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        driveJoystick.L1().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // #endregion Swerve

        // While the robot is not disabled (auto, teleop), add vision measurements to pose
        // RobotModeTriggers.disabled().whileFalse(
        //     new UpdatePose(vision)
        // );

        // #region Intake
        // driveJoystick.cross().whileTrue(
        //     new RunIntake(m_intake, Constants.Intake.kIntakeForwardSpeed)
        // );

        // driveJoystick.circle().whileTrue(
        //     new RunIntake(m_intake, Constants.Intake.kIntakeReverseSpeed)
        // );

        // driveJoystick.povDown().whileTrue(
        //     new ExtendIntake(m_intake)
        // );

        // driveJoystick.povUp().whileTrue(
        //     new RetractIntake(m_intake)
        // );
        // #endregion Intake

        // #region Poses
        // driveJoystick.circle().whileTrue(
        //     new DriveToApriltag(5, drivetrain, vision)
        // );
        // driveJoystick.cross().whileTrue(
        //     // Blue hub (4.65, 4)
        //     // Red hub (12, 4)
        //     new PointToHub(() -> -driveJoystick.getLeftY() * MaxSpeed, () -> -driveJoystick.getLeftX() * MaxSpeed, drivetrain, networkTablesIO) 
        // );

        // driveJoystick.square().whileTrue(
        //     new DriveToPose(new Pose2d(2.0, 2.0, new Rotation2d()), drivetrain, networkTablesIO)
        // );
        // #endregion Poses
        
        // #region DebugMotors
        driveJoystick.povUp().and(driveJoystick.L1().negate()).whileTrue(new RunDebugMotors(1, 0.5, m_DebugMotors));
        driveJoystick.povRight().and(driveJoystick.L1().negate()).whileTrue(new RunDebugMotors(2, 0.5, m_DebugMotors));
        driveJoystick.povDown().and(driveJoystick.L1().negate()).whileTrue(new RunDebugMotors(3, 0.5, m_DebugMotors));
        driveJoystick.povLeft().and(driveJoystick.L1().negate()).whileTrue(new RunDebugMotors(4, 0.5, m_DebugMotors));

        driveJoystick.povUp().and(driveJoystick.L1()).whileTrue(new RunDebugMotors(1, -0.5, m_DebugMotors));
        driveJoystick.povRight().and(driveJoystick.L1()).whileTrue(new RunDebugMotors(2, -0.5, m_DebugMotors));
        driveJoystick.povDown().and(driveJoystick.L1()).whileTrue(new RunDebugMotors(3, -0.5, m_DebugMotors));
        driveJoystick.povLeft().and(driveJoystick.L1()).whileTrue(new RunDebugMotors(4, -0.5, m_DebugMotors));
        // #endregion DebugMotors
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }
    

    public Command getAutonomousCommand() {
        // Return the auto selected by the chooser
        return autoChooser.getSelected();
    }
}

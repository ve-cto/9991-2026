// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
import frc.robot.commands.ExtendIntake;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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
    public final Vision vision = new Vision(drivetrain);
    // #endregion Swerve setup

    private final CommandXboxController driveJoystick = new CommandXboxController(Constants.Controller.kDriverControllerPort);
    private final CommandXboxController operatorJoystick = new CommandXboxController(Constants.Controller.kOperatorControllerPort);

    private final Intake m_intake = new Intake();

    public RobotContainer() {
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
        driveJoystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));

        driveJoystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driveJoystick.getLeftY(), -driveJoystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driveJoystick.back().and(driveJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driveJoystick.back().and(driveJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driveJoystick.start().and(driveJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driveJoystick.start().and(driveJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driveJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // #endregion Swerve

        // #region Intake
        driveJoystick.a().whileTrue(
            new RunIntake(m_intake, Constants.Intake.kIntakeForwardSpeed)
        );

        driveJoystick.b().whileTrue(
            new RunIntake(m_intake, Constants.Intake.kIntakeReverseSpeed)
        );

        driveJoystick.povDown().whileTrue(
            new ExtendIntake(m_intake)
        );

        driveJoystick.povUp().whileTrue(
            new RetractIntake(m_intake)
        );
        // #endregion Intake

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

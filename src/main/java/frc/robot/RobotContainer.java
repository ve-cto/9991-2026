// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Haiiiii :3

package frc.robot;

// Swerve
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.DriverStation;
// Pathplanner and Poses
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.*;
// Command Setup and Controllers
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
// Commands
import frc.robot.commands.drive.DriveToApriltag;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.drive.PointToHub;
import frc.robot.commands.drive.PointToPose;
import frc.robot.commands.RunDebugMotors;
import frc.robot.commands.drive.PointToAllianceFuel;
// Subsystems
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.NetworkTablesIO;
import frc.robot.subsystems.shooter.HoodedShooter;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.TrajectoryCalculator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DebugMotors;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Feeder;


public class RobotContainer {
    // #region Swerve setup
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(Constants.Swerve.kMaxAngularRps).in(RadiansPerSecond); // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * Constants.Swerve.kDeadbandFraction)
        .withRotationalDeadband(MaxAngularRate * Constants.Swerve.kDeadbandFraction) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * Constants.Swerve.kDeadbandFraction)
        .withRotationalDeadband(MaxAngularRate * Constants.Swerve.kDeadbandFraction) // Add a `% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    // #endregion Swerve setup
    
    // #region Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final NetworkTablesIO m_networkTablesIO = new NetworkTablesIO();
    public final Vision m_vision = new Vision(drivetrain, m_networkTablesIO);
    private final Intake m_intake = new Intake();
    private final Arm m_arm = new Arm();
    private final DebugMotors m_DebugMotors = new DebugMotors();
    private final Led m_led = new Led();
    private final Shooter m_shooter = new Shooter();
    private final Feeder m_feeder = new Feeder();
    private final HoodedShooter m_hood = new HoodedShooter();
    private final TrajectoryCalculator m_trajectoryCalculator = new TrajectoryCalculator(m_networkTablesIO);
    // #endregion Subsystems

    // #region Controllers
    // private final CommandPS4Controller driveJoystick = new CommandXboxController(Constants.Controller.kDriverControllerPort);
    // private final CommandPS4Controller operatorJoystick = new CommandXboxController(Constants.Controller.kOperatorControllerPort);
    private final CommandXboxController driveJoystick = new CommandXboxController(Constants.Controller.kDriverControllerPort);
    private final CommandPS4Controller operatorJoystick = new CommandPS4Controller(Constants.Controller.kOperatorControllerPort);
    // #endregion Controllers

    // #region Misc
    private final SendableChooser<Command> autoChooser;
    // #endregion Misc
    
    public RobotContainer() {
        // Create our auto chooser
        // Pathplanner autos get populated into it automatically
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        // Warm up on-the-fly path generation
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        NamedCommands.registerCommand("ExtendIntake", m_arm.extendIntakeCommand());
        NamedCommands.registerCommand("RetractIntake", m_arm.retractIntakeCommand());
        NamedCommands.registerCommand("RunIntake", m_intake.runIntakeCommand());
        NamedCommands.registerCommand("ReverseIntake", m_intake.reverseIntakeCommand());
        NamedCommands.registerCommand("RunShooter500", m_shooter.runRPMCommand(() -> 500));
        NamedCommands.registerCommand("RunShooter600", m_shooter.runRPMCommand(() -> 600));
        NamedCommands.registerCommand("RunShooter700", m_shooter.runRPMCommand(() -> 700));
        NamedCommands.registerCommand("FeedShooter", m_feeder.feedCommand());

        // Bind the commands to the controller inputs.
        configureBindings();
    }

    private void configureBindings() {
        // #region Swerve Setup
        drivetrain.setDefaultCommand(
            // Set default drivetrain command to be driving with joysticks. Unless another command is scheduled (such as PointToHub), this command will run.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driveJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Point the modules towards the direction of the left stick, without driving the robot. Note that this does not get updated while holding, only on initialize. (They aren't double suppliers)
        // driveJoystick.triangle().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driveJoystick.getLeftY(), -driveJoystick.getLeftX()))
        // ));

        // Make the drivetrain idle when robot is disabled. (note that this is called only once)
        final var swerveIdle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> swerveIdle).ignoringDisable(true)
        );
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> swerveIdle).ignoringDisable(true)
        );
        // Disable the drivetrain while in test mode to avoid noisy motors
        RobotModeTriggers.test().whileTrue(
            drivetrain.applyRequest(() -> swerveIdle).ignoringDisable(true)
        );
        // RobotModeTriggers.disabled().whileFalse(
        //     null
        // );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driveJoystick.back().and(driveJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driveJoystick.back().and(driveJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driveJoystick.start().and(driveJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driveJoystick.start().and(driveJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on button press. Note that this has limited effect during the actual game, as m_vision measurements will override it.
        // driveJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        driveJoystick.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Update the robot's odometry. 
        drivetrain.registerTelemetry(logger::telemeterize);
        // #endregion Swerve

        // Drive robot-centric instead of field-centric while held.
        // driveJoystick.leftBumper().whileTrue(
        //     drivetrain.applyRequest(() ->
        //         driveRobotCentric.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-driveJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );
        
        // Brake while holding. When the robot brakes, the four drive motors stop and the modules point towards the center of the robot.
        // driveJoystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
        // driveJoystick.R1().whileTrue(drivetrain.applyRequest(() -> brake));

        // When we enable test mode, update the shooter motor's configs (kp ki kd)
        RobotModeTriggers.test().whileTrue(m_shooter.updateMotorConfigsCommand());
        RobotModeTriggers.test().whileTrue(m_hood.updateMotorConfigsCommand());
        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop().or(RobotModeTriggers.test())).whileTrue(m_trajectoryCalculator.updateAllianceCommand());

        m_intake.setDefaultCommand(
            m_intake.coastCommand()
        );
        m_arm.setDefaultCommand(
            m_arm.coastCommand()
        );
        m_feeder.setDefaultCommand(
            m_feeder.coastCommand()
        );
        m_shooter.setDefaultCommand(
            m_shooter.coastCommand()
        );
        // Default to displaying the specific modes' pattern (disconn, disabl, auto, teleop)
        m_led.setDefaultCommand(
            m_led.handleDefault().ignoringDisable(true)
        );
        
        // ---------------------------------------

        driveJoystick.leftBumper().whileTrue(
            m_intake.runIntakeCommand()
        );

        // driveJoystick.a().whileTrue(
        //     m_feeder.feedCommand(0.95)
        // );

        // driveJoystick.povLeft().and(m_shooter.atSetpoint()).whileTrue(
        //     m_feeder.feedCommand(0.95).alongWith(m_intake.runIntakeCommand())
        // );
        // driveJoystick.povLeft().and(m_shooter.atSetpoint()).negate().whileTrue(
        //     m_feeder.feedCommand(-0.3) // If we aren't ready to shoot, try to move balls back to stop them from entering the shooter.
        // );
        // driveJoystick.povRight().and(m_shooter.atSetpoint()).whileTrue(
        //     m_feeder.feedCommand(0.95).alongWith(m_intake.runIntakeCommand())
        // );
        // driveJoystick.povRight().and(m_shooter.atSetpoint()).negate().whileTrue(
        //     m_feeder.feedCommand(-0.3) // If we aren't ready to shoot, try to move balls back to stop them from entering the shooter.
        // );
        // // driveJoystick.povLeft().whileTrue(
        // //     m_shooter.runRPMCommand(3000).alongWith(drivetrain.pointToHubCommand(m_networkTablesIO.getAlliance(), () -> -driveJoystick.getLeftY() * MaxSpeed, () -> -driveJoystick.getLeftX() * MaxSpeed, m_networkTablesIO))
        // // );
        // driveJoystick.povRight().whileTrue(
        //     m_shooter.runRPMCommand(3500)
        // );

        // driveJoystick.povLeft().whileTrue(
        //     m_shooter.runRPMCommand(3500).alongWith(new PointToHub(() -> -driveJoystick.getLeftY() * MaxSpeed, () -> -driveJoystick.getLeftX() * MaxSpeed, drivetrain, m_networkTablesIO)).alongWith(m_feeder.feedCommand(0.8)).alongWith(m_intake.runIntakeCommand())
        // );

        driveJoystick.povDown().whileTrue(
            m_arm.moveArmCommand(-0.7)
        );

        driveJoystick.povUp().whileTrue(
            m_arm.moveArmCommand(0.7)
        );

        // driveJoystick.x().whileTrue(m_shooter.runDashboard());
        // driveJoystick.y().whileTrue(m_shooter.runRPMCommand(3500));
        // driveJoystick.b().whileTrue(m_shooter.runRPMCommand(4000));
        
        // driveJoystick.y().whileTrue(m_hood.run(() -> m_hood.run(0.5)));
        // driveJoystick.b().whileTrue(m_hood.run(() -> m_hood.run(-0.5)));
        // driveJoystick.y().whileTrue(m_hood.runCommand(-1.0));
        // driveJoystick.y().whileTrue(m_hood.runSetpointPercentageCommand(0.8));
        // driveJoystick.x().whileTrue(m_hood.homeInCommand());
        driveJoystick.b().whileTrue(m_hood.runCommand(1.0));
        driveJoystick.y().whileTrue(m_hood.runCommand(-1.0));
        driveJoystick.x().whileTrue(m_hood.gotoPercentageCommand(() -> 1.0));
        driveJoystick.a().whileTrue(m_hood.resetHomeCommand());
        m_hood.setDefaultCommand(m_hood.brakeCommand());

        driveJoystick.povRight().whileTrue(
            new PointToHub(() -> -driveJoystick.getLeftY() * MaxSpeed, () -> -driveJoystick.getLeftX() * MaxSpeed, drivetrain, m_networkTablesIO)    
            .alongWith(
                m_shooter.runRPMCommand(m_trajectoryCalculator.getRequiredShooterSpeedHub())
            ).alongWith(
                m_hood.gotoAngleCommand(m_trajectoryCalculator.getRequiredHoodAngleHub())
            ).alongWith(
                m_intake.runIntakeCommand()
            ).alongWith(
                m_feeder.feedCommand()
            ).alongWith(
                m_led.setProgressVarCommand(1, m_shooter.getMechanismVelocityAv())
            )
        );

        driveJoystick.povLeft().whileTrue(
            m_hood.gotoPercentageCommand(() -> 1.0)
            .alongWith(
                m_shooter.runDashboard()
            ).alongWith(
                m_feeder.feedCommand(0.8)
            )
        );

        // driveJoystick.x().and(() -> m_shooter.isAtSetpoint()).whileTrue(m_feeder.feedCommand());

        // driveJoystick.start().whileTrue(
        //     new DriveToApriltag(10, drivetrain, m_vision)
        // );
        
        // Point towards the hub.
        // driveJoystick.a().whileTrue(
        //     new PointToHub(() -> -driveJoystick.getLeftY() * MaxSpeed, () -> -driveJoystick.getLeftX() * MaxSpeed, drivetrain, m_networkTablesIO) 
        // );

        // driveJoystick.b().whileTrue(
        //     new DriveToPose(new Pose2d(1.0, 1.0, new Rotation2d()), drivetrain, m_networkTablesIO)
        // );
        
        // driveJoystick.a().and(() -> !m_networkTablesIO.isInOwnAllianceZone()).whileTrue(
        //     new PointToAllianceFuel(() -> -driveJoystick.getLeftY() * MaxSpeed, () -> -driveJoystick.getLeftX() * MaxSpeed, drivetrain, m_networkTablesIO).alongWith(m_shooter.runRPMCommand(100))
        // );

        // #region LEDs
        m_shooter.atSetpoint().and(m_shooter.isCommanded()).whileTrue(m_led.display(Constants.Led.StatusList.ALIGNED));
        // If the robot is ESTOPPED, flash
        RobotModeTriggers.disabled().and(() -> DriverStation.isEStopped()).whileTrue(m_led.estop().ignoringDisable(true));
        // #endregion LEDs
    
        // While the robot is not disabled (NOT in auto, teleop, test), add m_vision measurements to pose.
        RobotModeTriggers.disabled().whileFalse(
            m_vision.addVisionMeasurementCommand()
        );
    }
    
    public Command getAutonomousCommand() {
        // Return the auto selected by the chooser on SmartDashboard
        return autoChooser.getSelected();
    }
}

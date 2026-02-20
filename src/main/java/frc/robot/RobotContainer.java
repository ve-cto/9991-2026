// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
// Commands
import frc.robot.commands.drive.DriveToApriltag;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.drive.PointToHub;
import frc.robot.commands.drive.PointToPose;
import frc.robot.commands.RunDebugMotors;
import frc.robot.commands.drive.UpdatePose;
import frc.robot.commands.intake.ExtendIntake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.intake.RunIntake;
// Subsystems
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.NetworkTablesIO;
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
    // #endregion Swerve setup
    
    // #region Subsystems
    public final NetworkTablesIO m_networkTablesIO = new NetworkTablesIO();
    public final Vision m_vision = new Vision(drivetrain, m_networkTablesIO);
    private final Intake m_intake = new Intake();
    private final DebugMotors m_DebugMotors = new DebugMotors();
    private final Led m_led = new Led();
    // #endregion Subsystems

    // #region Controllers
    // private final CommandXboxController driveJoystick = new CommandXboxController(Constants.Controller.kDriverControllerPort);
    // private final CommandXboxController operatorJoystick = new CommandXboxController(Constants.Controller.kOperatorControllerPort);
    private final CommandPS4Controller driveJoystick = new CommandPS4Controller(Constants.Controller.kDriverControllerPort);
    private final CommandPS4Controller operatorJoystick = new CommandPS4Controller(Constants.Controller.kOperatorControllerPort);
    // #endregion Controllers

    // #region Misc
    private final SendableChooser<Command> autoChooser;
    // #endregion Misc
    
    public RobotContainer() {
        // Create our auto chooser automatically with all of the autos from pathplanner.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        // Warm up on-the-fly path generation
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        NamedCommands.registerCommand("PointToHub", new PointToHub(() -> 0.0, () -> 0.0, drivetrain, m_networkTablesIO));
        NamedCommands.registerCommand("ExtendIntake", new ExtendIntake(m_intake));
        NamedCommands.registerCommand("RetractIntake", new RetractIntake(m_intake));
        NamedCommands.registerCommand("RunIntake", new RunIntake(m_intake, Constants.Intake.kIntakeForwardSpeed));

        // Bind the commands to the controller inputs.
        configureBindings();
    }

    private void configureBindings() {
        // #region Swerve
        drivetrain.setDefaultCommand(
            // Set default drivetrain command to be driving with joysticks. Unless another command is scheduled (such as PointToHub), this command will run.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driveJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        // Idle the drivebase while the robot is disabled.
        final var swerveIdle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> swerveIdle).ignoringDisable(true)
        );

        // Brake while holding the button. When the robot brakes, the four drive motors stop and the modules point towards the center of the robot. While breaking, the robot cannot drive.
        // driveJoystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
        // driveJoystick.R1().whileTrue(drivetrain.applyRequest(() -> brake));
        
        // Point the modules towards the direction of the left stick, without driving the robot. Note that this does not get updated while holding, only on initialize. (They aren't double suppliers)
        // driveJoystick.triangle().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driveJoystick.getLeftY(), -driveJoystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driveJoystick.back().and(driveJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driveJoystick.back().and(driveJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driveJoystick.start().and(driveJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driveJoystick.start().and(driveJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on button press. Note that this has limited effect during the actual game, as m_vision measurements will override it.
        // driveJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        driveJoystick.R1().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // #endregion Swerve

        // #region LEDs
        // If nothing else is happening - subsystem gets current mode (auto or teleop) and applies based on that
        m_led.setDefaultCommand(m_led.displayTeleAuto());
        // RobotModeTriggers only runs when the event happens, NOT while the event is ocurring (eg: runs on entry to disabled but NOT while disabled)
        // If the robot is ESTOPPED, flash
        RobotModeTriggers.disabled().and(() -> DriverStation.isDSAttached() && DriverStation.isEStopped()).whileTrue(m_led.flash(Constants.Led.StatusList.ESTOPPED, 5, 0.1).ignoringDisable(true));
        // If the robot is connected to the DS, but disabled
        RobotModeTriggers.disabled().and(() -> DriverStation.isDSAttached() && !DriverStation.isEStopped()).whileTrue(m_led.display(Constants.Led.StatusList.DISABLED).ignoringDisable(true));
        // If the robot is disconnected from the DS
        RobotModeTriggers.disabled().and(() -> !DriverStation.isDSAttached()).whileTrue(m_led.display(Constants.Led.StatusList.DISCONNECT).ignoringDisable(true));
        // #endregion LEDs

        // #region Intake
        // Methods for an imaginary intake - Extending and retracting an arm which the intake is attached to, and running the intake forwards and backwards.
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
        // Methods to drive or point the robot to certain positions on the field.

        // driveJoystick.circle().whileTrue(
        //     new DriveToApriltag(5, drivetrain, m_vision)
        // );
        
        // Points the robot towards the pose of the hub corresponding to the robots alliance.
        // driveJoystick.cross().whileTrue(
        //     // Blue hub (4.65, 4)
        //     // Red hub (12, 4)
        //     new PointToHub(() -> -driveJoystick.getLeftY() * MaxSpeed, () -> -driveJoystick.getLeftX() * MaxSpeed, drivetrain, m_networkTablesIO) 
        // );

        // driveJoystick.square().whileTrue(
        //     new DriveToPose(new Pose2d(2.0, 2.0, new Rotation2d()), drivetrain, m_networkTablesIO)
        // );
        // #endregion Poses
        
        // #region Vision
        // While the robot is not disabled (auto, teleop), add m_vision measurements to pose.
        // TODO: Add a small delay to this so that the m_vision subsystem isn't constantly flooded with requests.
        // RobotModeTriggers.disabled().whileFalse(
        //     new UpdatePose(m_vision)
        // );
        // #endregion Vision

        // #region DebugMotors
        // Manually shift motors. Note that only one motor can be in motion at a time. Hold L1 to run backwards, use L2 to set speed.
        // motorID 1 = CAN ID 15
        // motorID 2 = CAN ID 16
        // motorID 3 = CAN ID 17
        // motorID 4 = CAN ID 18
        
        driveJoystick.povUp().and(driveJoystick.L1().negate()).whileTrue(new RunDebugMotors(1, () -> (driveJoystick.getL2Axis() + 1)/2, m_DebugMotors));
        driveJoystick.povRight().and(driveJoystick.L1().negate()).whileTrue(new RunDebugMotors(2, () -> (driveJoystick.getL2Axis() + 1)/2, m_DebugMotors));
        driveJoystick.povDown().and(driveJoystick.L1().negate()).whileTrue(new RunDebugMotors(3, () -> (driveJoystick.getL2Axis() + 1)/2, m_DebugMotors));
        driveJoystick.povLeft().and(driveJoystick.L1().negate()).whileTrue(new RunDebugMotors(4, () -> (driveJoystick.getL2Axis() + 1)/2, m_DebugMotors));

        driveJoystick.povUp().and(driveJoystick.L1()).whileTrue(new RunDebugMotors(1, () -> -(driveJoystick.getL2Axis() + 1)/2, m_DebugMotors));
        driveJoystick.povRight().and(driveJoystick.L1()).whileTrue(new RunDebugMotors(2, () -> -(driveJoystick.getL2Axis() + 1)/2, m_DebugMotors));
        driveJoystick.povDown().and(driveJoystick.L1()).whileTrue(new RunDebugMotors(3, () -> -(driveJoystick.getL2Axis() + 1)/2, m_DebugMotors));
        driveJoystick.povLeft().and(driveJoystick.L1()).whileTrue(new RunDebugMotors(4, () -> -(driveJoystick.getL2Axis() + 1)/2, m_DebugMotors));
        // #endregion DebugMotors
        
        // Update the robot's odometry. Unsure if this needs to be at the end or not, but it makes sense if it is. (pregenerated by pheonix)
        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    public Command getAutonomousCommand() {
        // Return the auto selected by the chooser on SmartDashboard
        return autoChooser.getSelected();
    }
}

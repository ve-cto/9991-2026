// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.concurrent.CompletableFuture;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.HoodedShooter;
import frc.robot.subsystems.shooter.TrajectoryCalculator;
import frc.robot.subsystems.shooter.TrajectoryCalculator.ShotSolution;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CustomShot extends Command {
  private Shooter m_shooter;
  private HoodedShooter m_hood;
  private TrajectoryCalculator trajectoryCalculator;
  private CommandSwerveDrivetrain m_drivetrain;

  double robotVxField, robotVyField;
  double robotVxCentric, robotVyCentric;
  Pose2d locationPose;
  double launchAngleDeg, shooterHeight, locationHeight, clearanceHeight;
  DoubleSupplier driveX, driveY;

  ShotSolution shotSolution;

  /** Creates a new CalculateShot. */
  public CustomShot(DoubleSupplier driveX, DoubleSupplier driveY, Pose2d locationPose, double launchAngleDeg, double shooterHeight, double locationHeight, double clearanceHeight, Shooter shooter, HoodedShooter hood, TrajectoryCalculator trajectoryCalculator, CommandSwerveDrivetrain drivetrain) {
    this.m_shooter = shooter;
    this.m_hood = hood;
    this.m_drivetrain = drivetrain;
    this.trajectoryCalculator = trajectoryCalculator;

    this.locationPose = locationPose;
    this.launchAngleDeg = launchAngleDeg;
    this.shooterHeight = shooterHeight;
    this.locationHeight = locationHeight;
    this.clearanceHeight = clearanceHeight;
    this.driveX = driveX;
    this.driveY = driveY;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_shooter, this.m_hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.robotVxCentric = m_drivetrain.getState().Speeds.vxMetersPerSecond;
    this.robotVyCentric = m_drivetrain.getState().Speeds.vyMetersPerSecond;

    Translation2d robotVel = new Translation2d(robotVxCentric, robotVyCentric);
    Rotation2d headingRot = Rotation2d.fromRadians(this.m_drivetrain.getState().Pose.getRotation().getRadians());
    Translation2d fieldVel = robotVel.rotateBy(headingRot);

    this.robotVxField = fieldVel.getX();
    this.robotVyField = fieldVel.getY();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.robotVxCentric = m_drivetrain.getState().Speeds.vxMetersPerSecond;
    this.robotVyCentric = m_drivetrain.getState().Speeds.vyMetersPerSecond;

    CompletableFuture.runAsync(() -> this.shotSolution = trajectoryCalculator.computeShot(this.robotVxField, this.robotVyField, this.locationPose, this.shooterHeight, this.locationHeight, this.launchAngleDeg, this.clearanceHeight));

    if (this.shotSolution == null) {
        // Don’t shoot yet — maybe blink LEDs or log
        System.out.println("ShotSolution not ready");
        return;
    }
    if (this.shotSolution != null) {
    this.m_shooter.runRPM(() -> this.shotSolution.launchSpeed);
    this.m_hood.gotoAngleCommand(() -> this.shotSolution.launchAngleDeg);
    this.m_drivetrain.pointToAngle(new Rotation2d((this.shotSolution == null) ? 0 : this.shotSolution.aimAngleRad * (180/Math.PI)), driveX.getAsDouble(), driveY.getAsDouble());
  }}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

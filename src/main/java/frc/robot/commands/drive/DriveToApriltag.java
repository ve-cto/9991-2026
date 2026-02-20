// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.NetworkTablesIO;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToApriltag extends Command {
  Vision vision;
  CommandSwerveDrivetrain commandSwerveDrivetrain;
  NetworkTablesIO networkTablesIO;
  private int targetID = 0;
  private Pose2d targetPose = new Pose2d();

  /** Creates a new DriveToAprilTag. */
  public DriveToApriltag(int tagID, CommandSwerveDrivetrain commandSwerveDrivetrain, Vision vision, NetworkTablesIO networkTablesIO) {
    this.vision = vision;
    this.targetID = tagID;
    this.networkTablesIO = networkTablesIO;
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(commandSwerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.targetPose = vision.getTargetPose(targetID).toPose2d();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    commandSwerveDrivetrain.driveToPose(this.targetPose, networkTablesIO);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return vision.getTargetPose(targetID) == null;
  }
}

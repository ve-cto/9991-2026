// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToApriltag extends Command {
  Vision vision;
  CommandSwerveDrivetrain commandSwerveDrivetrain;
  private int targetID = 0;

  /** Creates a new DriveToAprilTag. */
  public DriveToApriltag(int tagID, CommandSwerveDrivetrain commandSwerveDrivetrain, Vision vision) {
    this.vision = vision;
    this.targetID = tagID;
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(commandSwerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    vision.driveToApriltagHelper(targetID, this.commandSwerveDrivetrain);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End the command if the desired target is not defined in the field layout
    // or is not currently visible. This prevents the command from holding the
    // drivetrain requirement while doing nothing.
    return vision.getTargetPose(targetID) == null;
  }
}

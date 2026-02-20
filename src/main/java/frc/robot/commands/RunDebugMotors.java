// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DebugMotors;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunDebugMotors extends Command {
  private final DebugMotors m_DebugMotors;
  private int motorID;
  private double speed;
  private DoubleSupplier speeds;

  /** Creates a new RunDebugMotors. */
  public RunDebugMotors(int motorID, DoubleSupplier speed, DebugMotors debugMotors) {
    this.m_DebugMotors = debugMotors;
    this.speeds = speed;
    this.motorID = motorID;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_DebugMotors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.speed = this.speeds.getAsDouble();
    // System.out.println("Debug motors ran with speed of: " + this.speed);
    m_DebugMotors.runMotor(this.motorID, this.speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DebugMotors.stopMotor(this.motorID);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

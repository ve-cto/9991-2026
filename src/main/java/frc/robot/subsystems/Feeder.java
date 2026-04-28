// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private final TalonFX m_feeder;
  private double mechanismVelocity;
  private double setpoint;
  private double motorVelocity;
  private double closedLoopCalculatedOutput;
  
  private final PIDController kPidController;
  /** Creates a new Feeder. */
  public Feeder() {
    m_feeder = new TalonFX(Constants.Hardware.kLoaderId);
    kPidController = new PIDController(0.0001, 0.0008, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.motorVelocity = m_feeder.getRotorVelocity().getValue().in(RotationsPerSecond) * 60;
    this.mechanismVelocity = this.motorVelocity / Constants.Feeder.kControlRatio;
  //   private double mechanismVelocity;
    // private double setpoint;
    // private double motorVelocity;
    // private double closedLoopCalculatedOutput;
    SmartDashboard.putNumber("Feeder Mechanism Velocity", this.mechanismVelocity);
    SmartDashboard.putNumber("Feeder Motor Velocity", this.motorVelocity);
    SmartDashboard.putNumber("Feeder Setpoint", this.setpoint);
    SmartDashboard.putNumber("Feeder Output", this.closedLoopCalculatedOutput);
  }

  public void run(double speed) {
    m_feeder.set(speed);
  }

  public void stop() {
    m_feeder.stopMotor();
  }

  public void coast() {
    m_feeder.set(0.0);
  }

  public void runRPM(double rotationsPerMinute) {
    this.setpoint = rotationsPerMinute;
    if (rotationsPerMinute * Constants.Feeder.kControlRatio >= Constants.Hardware.kMaxKrakenFreeSpeed) {
      DriverStation.reportWarning(String.format("WARN: Feeder setpoint %s is greater than maximum attainable motor speed.", rotationsPerMinute), false);
    }

    double raw = kPidController.calculate(this.mechanismVelocity, rotationsPerMinute);

    this.closedLoopCalculatedOutput = Math.min(Math.max(raw, -Constants.Shooter.kMaxOutput), Constants.Shooter.kMaxOutput);
    this.run(this.closedLoopCalculatedOutput);
  }

  public Command feedCommand(double rotationsPerMinute) {
    return startEnd(() -> this.runRPM(rotationsPerMinute), () -> this.coast());
  }

  public Command feedCommand() {
    return startEnd(() -> this.run(Constants.Loader.kLoadSpeed), () -> this.coast());
  }
}
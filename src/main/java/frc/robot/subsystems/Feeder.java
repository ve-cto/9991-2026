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
  // private final TalonFX m_feederFX;
  private final WPI_VictorSPX m_feederSPX;

  private double output;
  // private double mechanismVelocity;
  // private double setpoint;
  // private double motorVelocity;
  // private double closedLoopCalculatedOutput;
  
  // private final PIDController kPidController;
  /** Creates a new Feeder. */
  public Feeder() {
    m_feederSPX = new WPI_VictorSPX(Constants.Hardware.kFeederSPXId);
    // m_feederFX = new TalonFX(Constants.Hardware.kFeederFXId);
    // kPidController = new PIDController(0.0001, 0.0008, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (m_feederFX.isConnected()) {
    //   this.motorVelocity = m_feederFX.getRotorVelocity().getValue().in(RotationsPerSecond) * 60;
    //   this.mechanismVelocity = this.motorVelocity / Constants.Feeder.kControlRatio;
    //   // private double mechanismVelocity;
    //   // private double setpoint;
    //   // private double motorVelocity;
    //   // private double closedLoopCalculatedOutput;
    //   SmartDashboard.putNumber("Feeder Mechanism Velocity", this.mechanismVelocity);
    //   SmartDashboard.putNumber("Feeder Motor Velocity", this.motorVelocity);
    //   SmartDashboard.putNumber("Feeder Setpoint", this.setpoint);
    //   SmartDashboard.putNumber("Feeder Output", this.closedLoopCalculatedOutput);
    // }
    SmartDashboard.putNumber("feederOutput", this.output);
  }

  public void run(double speed) {
    // m_feederFX.set(speed);
    this.output = speed;
    m_feederSPX.set(speed);
  }

  public void stop() {
    // m_feederFX.stopMotor();
    this.output = 0;
    m_feederSPX.stopMotor();
  }

  public void coast() {
    // m_feederFX.set(0.0);
    this.run(0.0);
  }

  // public void runRPM(double rotationsPerMinute) {
  //   if (!m_feederFX.isConnected()) {
  //     DriverStation.reportWarning(String.format("WARN: Method invoking TalonFX ID %s was called with no motor connected. Abandoning request.", Constants.Hardware.kFeederFXId), false);
  //     return;
  //   } else {
  //     this.setpoint = rotationsPerMinute;
  //     if (rotationsPerMinute * Constants.Feeder.kControlRatio >= Constants.Hardware.kMaxKrakenFreeSpeed) {
  //       DriverStation.reportWarning(String.format("WARN: Feeder setpoint %s is greater than maximum attainable motor speed.", rotationsPerMinute), false);
  //     }

  //     double raw = kPidController.calculate(this.mechanismVelocity, rotationsPerMinute);

  //     this.closedLoopCalculatedOutput = Math.min(Math.max(raw, -Constants.Shooter.kMaxOutput), Constants.Shooter.kMaxOutput);
  //     this.run(this.closedLoopCalculatedOutput);
  //   }
  // }

  // public Command feedCommand(int rotationsPerMinute) {
  //   return startEnd(() -> this.runRPM(rotationsPerMinute), () -> this.coast());
  // }

  public Command feedCommand(double speed) {
    return startEnd(() -> this.run(speed), () -> this.coast());
  }

  public Command coastCommand() {
    return startEnd(() -> this.coast(), () -> this.coast());
  }

  public Command feedCommand() {
    return startEnd(() -> this.run(Constants.Feeder.kFeedSpeed), () -> this.coast());
  }
}

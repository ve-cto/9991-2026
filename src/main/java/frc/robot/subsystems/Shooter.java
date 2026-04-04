// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final TalonFX m_shooterL;
  private final TalonFX m_shooterR;

  // private final TalonFXSimState s_shooterL;
  // private final TalonFXSimState s_shooterR;

  private final PIDController kPidController;

  private double shooterLRPM;
  private double shooterRRPM;
  private double shooterVelocityAv;
  private double closedLoopCalculatedOutput;

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooterL = new TalonFX(Constants.Hardware.kShooterLId);
    m_shooterR = new TalonFX(Constants.Hardware.kShooterRId);
    kPidController = new PIDController(0.001, 0.01, 0.0);
  }

  /*
   * Calculate feedforward linearly.
   */
  public double calculateFeedforward(double rotationsPerMinute) {
    double ff = (0.000688 * rotationsPerMinute + 0.0052);
    return ff;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.shooterLRPM = (m_shooterL.getRotorVelocity().getValue().in(RotationsPerSecond) * 60) / Constants.Shooter.controlRatio;
    this.shooterRRPM = (m_shooterR.getRotorVelocity().getValue().in(RotationsPerSecond) * 60) / Constants.Shooter.controlRatio;
    
    this.shooterVelocityAv = (this.shooterLRPM + -this.shooterRRPM) / 2;
  }

  /*
   * Run the shooter at a given output percentage.
   * Range (-1, 1)
   */
  public void run(double speed) {
    m_shooterL.set(speed);
    m_shooterR.set(-speed);
  }

  // Math.min(Math.max(value, min), max);

  /*
   * Run the shooter towards a given speed measured in rotations per minute of the mechanism.
   * Closed-Loop controlled (PID) using the krakens onboard encoder.
   */
  public void runClosedLoop(double rotationsPerMinute) {
    if (rotationsPerMinute * Constants.Shooter.controlRatio >= Constants.Hardware.kMaxKrakenFreeSpeed) {
      DriverStation.reportWarning(String.format("WARN: Shooter setpoint %s is greater than maximum attainable motor speed.", rotationsPerMinute), false);
    }

    double raw = kPidController.calculate(shooterVelocityAv, rotationsPerMinute);
    double feedforward = calculateFeedforward(rotationsPerMinute);

    this.closedLoopCalculatedOutput = Math.min(Math.max(raw + feedforward, -Constants.Shooter.maxOutput), Constants.Shooter.maxOutput);
    this.run(this.closedLoopCalculatedOutput);
  }

  public void stop() {
    m_shooterL.stopMotor();
    m_shooterR.stopMotor();
  }

  /*
   * Set's motor output to 0, but does not apply braking force.
   */
  public void coast() {
    m_shooterL.set(0);
    m_shooterR.set(0);
  }
 
  public Command runPercentageCommand(double percentage) {
    return this.startEnd(() -> this.run(percentage), () -> this.coast());
  }

  public Command runRPMCommand(double rpm) {
    return new FunctionalCommand(
      () -> kPidController.reset(),                // initialize
      () -> runClosedLoop(rpm),                    // execute
      interrupted -> coast(),                      // end (with interrupted flag)
      () -> false,                                 // isFinished (never finishes on its own)
      this                                         // requirements
    );
  }

  public Command brakeCommand() {
    return this.startEnd(() -> this.stop(), () -> this.coast());
  }

  // Simulation -----------------------------------------------------------------------------------------------------------------------------------------------------
  // #region Simulation
  private final DCMotorSim m_motorSimModelL = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
        DCMotor.getKrakenX60Foc(1), 0.001, 1
        ),
    DCMotor.getKrakenX60Foc(1)
  );

  private final DCMotorSim m_motorSimModelR = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
        DCMotor.getKrakenX60Foc(1), 0.001, 1
        ),
    DCMotor.getKrakenX60Foc(1)
  );

  public void simulationInit() {
    var s_shooterL = m_shooterL.getSimState();
    s_shooterL.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    var s_shooterR = m_shooterR.getSimState();
    s_shooterR.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  public void simulationPeriodic() {
    var s_shooterL = m_shooterL.getSimState();
    var s_shooterR = m_shooterR.getSimState();
    s_shooterL.setSupplyVoltage(RobotController.getBatteryVoltage());
    s_shooterR.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the left shooter
    var motorVoltageL = s_shooterL.getMotorVoltageMeasure();
    m_motorSimModelL.setInputVoltage(motorVoltageL.in(Volts));
    m_motorSimModelL.update(0.020); // assume 20 ms loop time

    // get the motor voltage of the right shooter
    var motorVoltageR = s_shooterR.getMotorVoltageMeasure();
    m_motorSimModelR.setInputVoltage(motorVoltageR.in(Volts));
    m_motorSimModelR.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the motors;
    s_shooterL.setRawRotorPosition(m_motorSimModelL.getAngularPosition());
    s_shooterL.setRotorVelocity(m_motorSimModelL.getAngularVelocity());
    s_shooterR.setRawRotorPosition(m_motorSimModelR.getAngularPosition());
    s_shooterR.setRotorVelocity(m_motorSimModelR.getAngularVelocity());

  
  
    SmartDashboard.putNumber("shooterLRPM", this.shooterLRPM);
    SmartDashboard.putNumber("shooterRRPM", this.shooterRRPM);
    SmartDashboard.putNumber("shooterVelocityAv", this.shooterVelocityAv);
    SmartDashboard.putNumber("ShooterOutput", this.closedLoopCalculatedOutput);
  }
  // #endregion Simulation
}

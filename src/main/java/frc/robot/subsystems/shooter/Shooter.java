// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.BooleanSupplier;

public class Shooter extends SubsystemBase {
  private final TalonFX m_shooterL;
  private final TalonFX m_shooterR;

  // private final TalonFXSimState s_shooterL;
  // private final TalonFXSimState s_shooterR;

  private final PIDController kPidController;

  private boolean isCommanded = false;
  private double shooterLRPM;
  private double kp;
  private double ki;
  private double kd;
  private double ffslope;
  private double ffoffset;
  private double shooterRRPM;
  private double mechanismVelocityAv;
  private double mechanismVelocityHistoryAv;
  private double closedLoopCalculatedOutput;
  private double setpoint = 0;
  private double motorsVelocityAv;
  private Queue<Double> mechanismVelocityAvHistory = new ArrayBlockingQueue<>(10);
  
  private Slot0Configs slot0Configs = new Slot0Configs();
  final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooterL = new TalonFX(Constants.Hardware.kShooterLId);
    m_shooterR = new TalonFX(Constants.Hardware.kShooterRId);
    // kPidController = new PIDController(0.001, 0.01, 0.0);
    // kPidController = new PIDController(0.0001, 0.0008, 0.0);
    kPidController = new PIDController(0, 0, 0);

    SmartDashboard.putNumber("kp", 0.2);
    SmartDashboard.putNumber("ki", 0.0);
    SmartDashboard.putNumber("kd", 0);
    SmartDashboard.putNumber("ffslope", 0.00195);
    SmartDashboard.putNumber("ffoffset", 0);
    SmartDashboard.putNumber("presetspeed", 3200);
    this.updateMotorConfigs();
  }

  /*
   * Calculate feedforward linearly.
   */
  public double calculateFeedforward(double rotationsPerMinute) {
    // double ff = (0.000688 * rotationsPerMinute + 0.0052);
    // double ff = (0.000075* rotationsPerMinute + 0);
    double ff = (this.ffslope* rotationsPerMinute + this.ffoffset);
    return ff;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.shooterLRPM = (m_shooterL.getRotorVelocity().getValue().in(RotationsPerSecond) * 60);
    this.shooterRRPM = (m_shooterR.getRotorVelocity().getValue().in(RotationsPerSecond) * 60);
    
    this.motorsVelocityAv = (-this.shooterLRPM + this.shooterRRPM) / 2;
    this.mechanismVelocityAv = ((-this.shooterLRPM + this.shooterRRPM) / 2) * Constants.Shooter.kControlRatio;

    if (this.mechanismVelocityAvHistory.size() >= 10) {
      mechanismVelocityAvHistory.poll();
    }
    mechanismVelocityAvHistory.add(this .mechanismVelocityAv);
    this.mechanismVelocityHistoryAv = mechanismVelocityAvHistory.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

    SmartDashboard.putBoolean("isAtSetpoint", isAtSetpoint());
    SmartDashboard.putNumber("Setpoint", this.setpoint);
    SmartDashboard.putNumber("shooterLRPM", this.shooterLRPM);
    SmartDashboard.putNumber("shooterRRPM", this.shooterRRPM);
    SmartDashboard.putNumber("mechanismVelocityAv", this.mechanismVelocityAv);
    SmartDashboard.putNumber("motorsVelocityAv", this.motorsVelocityAv);
    SmartDashboard.putNumber("ShooterOutput", this.closedLoopCalculatedOutput);
    this.kp = SmartDashboard.getNumber("kp", 0);
    this.ki = SmartDashboard.getNumber("ki", 0);
    this.kd = SmartDashboard.getNumber("kd", 0);
    this.ffslope = SmartDashboard.getNumber("ffslope", 0);
    this.ffoffset = SmartDashboard.getNumber("ffoffset", 0);
    
    this.kPidController.setPID(this.kp, this.ki, this.kd);

  }

  /*
   * Run the shooter at a given output percentage.
   * Range (-1, 1)
   */
  public void run(double speed) {
    m_shooterL.set(-speed);
    m_shooterR.set(speed);
  }

  // Math.min(Math.max(value, min), max);

  /*
   * Run the shooter towards a given speed measured in rotations per minute of the mechanism.
   * Closed-Loop controlled (PID) using the krakens onboard encoder.
   */
  // public void runRPM(double rotationsPerMinute) {
  //   this.isCommanded = true;
  //   this.setpoint = rotationsPerMinute;
  //   if (rotationsPerMinute * Constants.Shooter.kControlRatio >= Constants.Hardware.kMaxKrakenFreeSpeed) {
  //     DriverStation.reportWarning(String.format("WARN: Shooter setpoint %s is greater than maximum attainable motor speed.", rotationsPerMinute), false);
  //   }

  //   double raw = kPidController.calculate(this.mechanismVelocityAv, rotationsPerMinute);
  //   double feedforward = calculateFeedforward(rotationsPerMinute);

  //   this.closedLoopCalculatedOutput = Math.min(Math.max(raw + feedforward, -Constants.Shooter.kMaxOutput), Constants.Shooter.kMaxOutput);
  //   this.run(this.closedLoopCalculatedOutput);
  // }
  public void runRPM(double rotationsPerMinute) {
    this.isCommanded = true;
    this.setpoint = rotationsPerMinute;
    if (rotationsPerMinute * Constants.Shooter.kControlRatio >= Constants.Hardware.kMaxKrakenFreeSpeed) {
      DriverStation.reportWarning(String.format("WARN: Shooter setpoint %s is greater than maximum attainable motor speed.", rotationsPerMinute), false);
    }

    double raw = kPidController.calculate(this.mechanismVelocityAv, rotationsPerMinute);
    double feedforward = calculateFeedforward(rotationsPerMinute);

    m_shooterL.setControl(m_request.withVelocity(-rotationsPerMinute/60).withFeedForward(-feedforward));
    m_shooterR.setControl(m_request.withVelocity(rotationsPerMinute/60).withFeedForward(feedforward));
  }

  public void updateMotorConfigs() {
    slot0Configs.kP = SmartDashboard.getNumber("kp", 0);
    slot0Configs.kI = SmartDashboard.getNumber("ki", 0);
    slot0Configs.kD = SmartDashboard.getNumber("kd", 0);
    m_shooterL.getConfigurator().apply(slot0Configs);
    m_shooterR.getConfigurator().apply(slot0Configs);
  }

  public Command updateMotorConfigsCommand() {
    return runOnce(() -> this.updateMotorConfigs());
  }

  /*
   * stop
   */
  public void stop() {
    this.isCommanded = false;
    m_shooterL.stopMotor();
    m_shooterR.stopMotor();
  }

  /*
   * Set's motor output to 0, but does not apply braking force.
   */
  public void coast() {
    this.isCommanded = false;
    m_shooterL.set(0);
    m_shooterR.set(0);
  }
 
  /*
   * Returns whether the shooters' current RPM is within +-Constants.Shooter.setpointDeadband of the setpoint RPM  
   */
  public boolean isAtSetpoint() {
    if (Math.abs(this.setpoint - this.mechanismVelocityHistoryAv) <= Constants.Shooter.setpointDeadband) {
      return true;
    } else {
      return false;
    }
  }

  /*
   * Reset the shooter's PID controller and setpoint.
   * Should be called on runRPM finish / interrupt.
   */
  public void reset() {
    this.closedLoopCalculatedOutput = 0;
    kPidController.reset();
    this.setpoint = 0;
  }

  /*
   * Trigger for if the shooter is at it's RPM setpoint.
   */
  public Trigger atSetpoint() {
    return new Trigger(() -> this.isAtSetpoint());
  }

  /*
   * Trigger for whether or not a non-zero control is given to the shooter motors.
   */
  public Trigger isCommanded() {
    return new Trigger(() -> this.isCommanded);
  }

  /*
   * Run the shooter with a given percentage of max output.
   * (-1, 1), where -1 is a backwards motion of the shooter.
   */
  public Command runPercentageCommand(double percentage) {
    return this.startEnd(() -> this.run(percentage), () -> this.coast());
  }

  /*
   * Run the shooter towards a given speed measured in rotations per minute of the mechanism.
   * Closed-Loop controlled (PID) using the krakens onboard encoder.
   */
  public Command runRPMCommand(double rpm) {
    return new FunctionalCommand(
      () -> reset(),                // initialize
      () -> runRPM(rpm),                    // execute
      interrupted -> reset(),                      // end (with interrupted flag)
      () -> false,                                 // isFinished (never finishes on its own)
      this                                         // requirements
    );
  }
  
  public Command runDashboard() {
    return new FunctionalCommand(
      () -> reset(),                // initialize
      () -> runRPM(SmartDashboard.getNumber("presetspeed", 0)),                    // execute
      interrupted -> reset(),                      // end (with interrupted flag)
      () -> false,                                 // isFinished (never finishes on its own)
      this                                         // requirements
    );
  }


  public Command brakeCommand() {
    return this.startEnd(() -> this.stop(), () -> this.coast());
  }

  public Command coastCommand() {
    return this.startEnd(() -> this.coast(),() -> this.coast());
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
  }
  // #endregion Simulation
}

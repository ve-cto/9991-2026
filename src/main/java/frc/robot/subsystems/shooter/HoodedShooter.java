package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import java.util.Map;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodedShooter extends SubsystemBase {
    private final TalonFX m_hood = new TalonFX(Constants.Hardware.kHoodId);
    private final int s_endstopId = 1;
    private final DigitalInput s_endstop = new DigitalInput(s_endstopId);
    
    private double s_hoodPosition = 0;
    private final double kCountExtended = Constants.Hood.kCountExtended; // pulses for the max reach of the hood / actuator
    private Queue<Double> s_hoodPositionQueue = new ArrayBlockingQueue<>(5);

    // private PIDController kPidController = new PIDController(0.001, 0, 0);
    private boolean homed = false;
    private boolean isAtSetpoint = false;
    private double setpoint = 0.0;

    private final double kAngleHomed = Constants.Hood.kAngleHomed; // should be close to 90 degrees - output angle of the ball
    private final double kAngleExtended = Constants.Hood.kAngleExtended; // will be LOWER than kAngleHomed

    private Slot0Configs slot0Configs = new Slot0Configs();
    final PositionVoltage c_request = new PositionVoltage(0).withSlot(0);
    final StaticBrake c_brake = new StaticBrake();

    final Alert alertOutOfRange = new Alert("Hood has logged a position out of range of theoretically attainable positions.", AlertType.kWarning);
    final Alert alertRequestedOutOfRange = new Alert("Hood setpoint %s is out of range of registered setpoints, reducing.", AlertType.kWarning);
    final Alert alertSetBeyondMaximumSpeed = new Alert("Hood requested speed is out of range of safe speeds, reducing.", AlertType.kWarning);

    public HoodedShooter() {
        SmartDashboard.putNumber("HOODkp", Constants.Hood.kdefaultKp);
        SmartDashboard.putNumber("HOODki", Constants.Hood.kdefaultKi);
        SmartDashboard.putNumber("HOODkd", Constants.Hood.kdefaultKd);

        SmartDashboard.putNumber("HOODrequestPosition", 80.0);
    };

    public void updateMotorConfigs() {
        slot0Configs.kP = SmartDashboard.getNumber("HOODkp", 0);
        slot0Configs.kI = SmartDashboard.getNumber("HOODki", 0);
        slot0Configs.kD = SmartDashboard.getNumber("HOODkd", 0);
        m_hood.getConfigurator().apply(slot0Configs);
    }

    public void periodic() {
        pollHoodEncoder();
        // SmartDashboard.putNumber("Encoder s_actuatorPosition Percentage", calculatePositionPercentageActuator());
        // SmartDashboard.putNumber("Encoder s_actuatorPosition Centimeters", calculatePositionCentimeters());
        SmartDashboard.putNumber("Hood Position", s_hoodPosition);
        SmartDashboard.putBoolean("Hood Homed", homed);
        SmartDashboard.putNumber("Hood Exit Path Angle Degrees", map(s_hoodPosition, 0, kCountExtended, kAngleHomed, kAngleExtended));
        if (s_hoodPosition > kCountExtended) {
           alertOutOfRange.setText(String.format("Hood has logged position %s out of range of theoretically attainable positions. Timestamp: %s", s_hoodPosition, Timer.getFPGATimestamp()));
            alertOutOfRange.set(true); 
        }

        if (this.setpoint > kCountExtended) {
            alertRequestedOutOfRange.setText(String.format("Hood setpoint %s is out of range of registered setpoints, clamping to attainable range.", this.setpoint));
        }
        
        // else {
        //     alertOutOfRange.set(false);
        // }
    }

    public void pollHoodEncoder() {
        this.s_hoodPosition = m_hood.getRotorPosition().getValueAsDouble();
        if (s_hoodPositionQueue.size() >= 5) {
            s_hoodPositionQueue.poll();
        }
        s_hoodPositionQueue.add(this.s_hoodPosition);
    }

    public double getHoodPositionPercentage() {
        return this.s_hoodPosition / kCountExtended;
    }

    public boolean getEndstop() {
        return s_endstop.get();
    }

    public boolean isAtSetpoint() {
        return isAtSetpoint;
    }

    public boolean getHomed() {
        return this.homed;
    }

    public static double map(double value, double sourceMin, double sourceMax, double targetMin, double targetMax) {
        return targetMin + (value - sourceMin) * (targetMax - targetMin) / (sourceMax - sourceMin);
    }

    public void run(double speed) {
        //Math.min(Math.max(value, min), max)
        if (speed > 0) homed = false; // if we move away from the home position, reset homed
        if (speed > Constants.Hood.kMaxOutput) {alertSetBeyondMaximumSpeed.setText(String.format("Hood requested speed %s is out of range of safe speeds, reducing to %s.", speed, Math.copySign(Constants.Hood.kMaxOutput, speed))); alertSetBeyondMaximumSpeed.set(true);} 
        else alertSetBeyondMaximumSpeed.set(false);
        m_hood.set(Math.min(Math.max(speed, -Constants.Hood.kMaxOutput), Constants.Hood.kMaxOutput)); // minmax for safety
        this.setpoint = s_hoodPosition;
    }

    public void brake() {
        m_hood.setControl(c_brake);
        alertSetBeyondMaximumSpeed.set(false);
    }

    public void hold() {
        this.runClosedLoop(() -> this.setpoint);
    }

    public void stop() {
        m_hood.stopMotor();
        alertSetBeyondMaximumSpeed.set(false);
    }
    
    public void home() {
        this.setpoint = 0;
        if (getEndstop()) { // if no endstop, run
            this.run(-0.2);
        } else if (!getEndstop()) { // if endstop, stop
            this.stop();
            this.reset();
            this.homed = true;
        } else {
            // theoretically shouldn't trigger BUT IF IT DOES you'll be happy that it doesn't kill itself <3
            this.stop();
        }
    }

    public Command gotoDashboard() {
        return this.runEnd(() -> this.runClosedLoopAngle(() -> SmartDashboard.getNumber("HOODrequestPosition", 80)), () -> this.hold());
    }

    public void runClosedLoop(DoubleSupplier setpoint) {
        double s = setpoint.getAsDouble();
        if (s > Constants.Hood.kMaxOutput) {alertRequestedOutOfRange.setText(String.format("Hood setpoint %s is out of range of registered setpoints, clamping to attainable range.", setpoint)); alertRequestedOutOfRange.set(true);} 
        else alertRequestedOutOfRange.set(false);
        s = s > kCountExtended ? 2 : s; // if the setpoint is greater than the maximum attainable setpoint, set to max
        s = s < 0 ? 0 : s; // likewise, if the setpoint is less than 0, set to 0
        this.setpoint = s;
        m_hood.setControl(c_request.withPosition(this.setpoint));
    }

    public void runClosedLoopPercentage(DoubleSupplier percentage) { // setpoint as a percentage of whole movement
        double p = percentage.getAsDouble();
        double s = p * kCountExtended;
        if (s > Constants.Hood.kMaxOutput) {alertRequestedOutOfRange.setText(String.format("Hood setpoint %s is out of range of registered setpoints, clamping to attainable range.", setpoint)); alertRequestedOutOfRange.set(true);} 
        else alertRequestedOutOfRange.set(false);
        s = s > kCountExtended ? 2 : s; // if the setpoint is greater than the maximum attainable setpoint, set to max
        s = s < 0 ? 0 : s; // likewise, if the setpoint is less than 0, set to 0
        this.setpoint = s;
        m_hood.setControl(c_request.withPosition(this.setpoint));
    }

    public void runClosedLoopAngle(DoubleSupplier degrees) {
        // private final double kAngleHomed = 80; // should be close to 90 degrees - output angle of the ball
        // private final double kAngleExtended = 30; // will be LOWER than kAngleHomed
        double d = degrees.getAsDouble();

        // SmartDashboard.putNumber("HOODd1", d);

        // TODO:
        // if (d > kAngleHomed || d < kAngleExtended) {
        //     String warningMessage = String.format("WARNING: Requested hood angle %.2f is %s than  closest attainable angle %.2f", d, (d > kAngleHomed) ? "greater" : "less", (d > kAngleHomed) ? kAngleHomed : kAngleExtended);
        //     DriverStation.reportWarning(warningMessage, true);
        // }

        if (d > kAngleHomed || d < kAngleExtended) {
            alertRequestedOutOfRange.setText(String.format("Hood setpoint %s is out of range of registered setpoints, clamping to attainable range.", map(d, kAngleHomed, kAngleExtended, 0, kCountExtended))); alertRequestedOutOfRange.set(true);
        } else alertRequestedOutOfRange.set(false);

        d = d > kAngleHomed ? kAngleHomed : d; //minmax it
        d = d < kAngleExtended ? kAngleExtended : d;
        


        // position range: 0 to 2
        // hood angle range: 80* to 30*

        d = map(d, kAngleHomed, kAngleExtended, 0, kCountExtended);
        // SmartDashboard.putNumber("HOODd3", d);
        this.setpoint = d;
        m_hood.setControl(c_request.withPosition(this.setpoint));
    }

    public void reset() {
        this.setpoint = 0;
        this.s_hoodPosition = 0;
        this.s_hoodPositionQueue.clear();
        m_hood.setPosition(0, 5);
        // resetPid();
    }

    // -----------------------------------------------------------------------------------------------------------------

    public Command runCommand(double speed) {
        return this.runEnd(() -> this.run(speed), () -> this.stop());
    }

    public Command brakeCommand() {
        return this.runEnd(() -> this.brake(), () -> this.stop());
    }

    public Command stopCommand() {
        return this.runOnce(() -> this.stop());
    }

    public Command holdCommand() {
        return this.runEnd(() -> this.hold(), () -> this.stop());
    }

    public Command homeCommand() {
        return this.runEnd(() -> this.home(), () -> this.stop());
    }

    public Command gotoPositionCommand(DoubleSupplier setpoint) {
        return this.runEnd(() -> this.runClosedLoop(setpoint), () -> this.stop());
    }

    public Command gotoPercentageCommand(DoubleSupplier percentage) {
        return this.runEnd(() -> this.runClosedLoopPercentage(percentage), () -> this.stop());
    }

    public Command gotoAngleCommand(DoubleSupplier degrees) {
        return this.runEnd(() -> this.runClosedLoopAngle(degrees), () -> this.stop());
    }

    public Command updateMotorConfigsCommand() {
        return this.runOnce(() -> this.updateMotorConfigs());
    }

    public Command resetHomeCommand() {
        return this.runOnce(() -> this.reset());
    }

    // Simulation -----------------------------------------------------------------------------------------------------------------------------------------------------
    // #region Simulation
    private final DCMotorSim sim_simModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.001, 1
            ),
        DCMotor.getKrakenX60Foc(1)
    );

    public void simulationInit() {
        var sim_hood = m_hood.getSimState();
        sim_hood.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    public void simulationPeriodic() {
        var sim_hood = m_hood.getSimState();
        sim_hood.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        var sim_hoodMotorVoltage = sim_hood.getMotorVoltageMeasure();
        sim_simModel.setInputVoltage(sim_hoodMotorVoltage.in(Volts));
        sim_simModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the motor
        sim_hood.setRawRotorPosition(sim_simModel.getAngularPosition());
        sim_hood.setRotorVelocity(sim_simModel.getAngularVelocity());
    }
    // #endregion Simulation

}

// Old actuator methods, kept for archive.
   // {
    // public double calculatePositionPercentageActuator() {
    //     return s_actuatorPulseCount / pulseCountExtended;
    // }

    // public void homeIn() {
    //     run(-0.75);

    //     double currentPos = calculatePositionPercentageActuator();
    //     s_actuatorHomingPositionQueue.add(currentPos);

    //     // Only check for homing once we have a reasonable number of samples
    //     // TODO: this requires the actator to not be homed in order to home, which is a problem  IF the actuator IS homed. the logic here needs to be redone.
    //     final int minSamples = 5;
    //     final double threshold = 0.01;
    //     if (s_actuatorHomingPositionQueue.size() >= minSamples) {
    //         double max = s_actuatorHomingPositionQueue.stream().mapToDouble(Double::doubleValue).max().orElse(currentPos);
    //         double min = s_actuatorHomingPositionQueue.stream().mapToDouble(Double::doubleValue).min().orElse(currentPos);
    //         // If the window is very stable (small max-min), we've stopped moving and are homed
    //         if (Math.abs(max - min) < threshold) {
    //             this.stop(); // stop the motor
    //             homed = true;
    //             this.reset();
    //         }
    //     }
    // }

    // public void homeOut() {
    //     run(0.75);

    //     double currentPos = calculatePositionPercentageActuator();
    //     s_actuatorHomingPositionQueue.add(currentPos);

    //     // Only check for homing once we have a reasonable number of samples
    //     final int minSamples = 10;
    //     final double threshold = 0.05;
    //     if (s_actuatorHomingPositionQueue.size() >= minSamples) {
    //         double max = s_actuatorHomingPositionQueue.stream().mapToDouble(Double::doubleValue).max().orElse(currentPos);
    //         double min = s_actuatorHomingPositionQueue.stream().mapToDouble(Double::doubleValue).min().orElse(currentPos);
    //         // If the window is very stable (small max-min), we've stopped moving and are homed
    //         if (Math.abs(max - min) < threshold) {
    //             stop(); // stop the motor immediately
    //             s_actuatorHomingPositionQueue.clear();
    //             this.reset();
    //             homed = true;
    //             SmartDashboard.putBoolean("Hood Homed", true);
    //         }
    //     }
    // }
    // }

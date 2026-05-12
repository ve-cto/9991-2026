package frc.robot.subsystems.shooter;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodedShooter extends SubsystemBase {
    private final WPI_VictorSPX m_actuator = new WPI_VictorSPX(Constants.Hardware.kHoodId);
    private final int s_encoderId = 1;
    private final int kStrokeCount = 4;
    private final DigitalInput s_encoder = new DigitalInput(s_encoderId);

    private double position;
    private double pulseCountAv;
    private double pulseCount = 0;
    private double pulseCountExtended = 1000;
    private Queue<Double> pulseCountQueue = new ArrayBlockingQueue<>(5);
    
    private boolean driveOutDirection;
    private boolean pulse;
    private boolean lastPulse;

    private double now;

    private PIDController kPidController = new PIDController(0, 0, 0);

    HoodedShooter() {};

    public void periodic() {
        this.now = Timer.getFPGATimestamp();
        pollSensor();

        if (pulseCountQueue.size() >= 5) {
            pulseCountQueue.poll();
        }
        pulseCountQueue.add(pulseCount);
        pulseCountAv = pulseCountQueue.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

        this.position = calculatePositionPercentage();
        SmartDashboard.putNumber("Encoder Position Percentage", calculatePositionPercentage());
        SmartDashboard.putNumber("Encoder Position Centimeters", calculatePositionCentimeters());
        SmartDashboard.putNumber("Encoder Pulse Count", pulseCount);
        SmartDashboard.putNumber("Encoder Pulse Count Av", pulseCountAv);
        SmartDashboard.putBoolean("Encoder Sensor Pulse", pulse);
        
        // position = 0.0;
        // if (now - lastPollTime < debounceDelay) {
        //     return;
        // }
        // lastPollTime = now;
        // if (positionHistoryQueue.size() >= 5) {
        //     positionHistoryQueue.poll();
        // }
        // positionHistoryQueue.add(position);
    }

    public double calculatePositionPercentage() {
        return pulseCount / pulseCountExtended;
    }
    public double calculatePositionInches() {
        return calculatePositionPercentage() * kStrokeCount;
    }
    public double calculatePositionCentimeters() {
        return calculatePositionPercentage() * kStrokeCount * 2.54; // amount of cm in an inch
    }

    public void pollSensor() {
        pulse = s_encoder.get();
        if (pulse != lastPulse) {
            if (!pulse) {
                return;
            }
            if (pulse && driveOutDirection) {
                this.pulseCount = pulseCount + 1;
            }
            if (pulse && !driveOutDirection) {
                this.pulseCount = pulseCount - 1;
            }
            lastPulse = pulse;
        } else {
            return;
        }
    }

    public void run(double speed) {
        driveOutDirection = (speed > 0) ? true : false;
        m_actuator.set(speed);
    }

    public void runClosedLoop(double setpointPercentage) {
        this.run(Math.min(Math.max(kPidController.calculate(this.position, setpointPercentage), -1), 1));
    }

    public void runSetpointPercentage(double setpoint) {
        this.runClosedLoop(setpoint);
    }
    public void runSetpointInches(double setpoint) {
        this.runClosedLoop(setpoint / kStrokeCount);
    }
    public void runSetpointCentimeters(double setpoint) {
        this.runClosedLoop((setpoint / kStrokeCount) / 2.54);
    }

    public void stop() {
        m_actuator.stopMotor();
    }

    // TODO: refine this
    public boolean getStopped() {
        if (pulseCountAv == pulseCount) {
            return true;
        }
        return false;
    }

    public boolean isAtSetpoint() {
        return false;
    }

    public void home() {
        m_actuator.set(-0.9);
        if (getStopped()) {
            this.reset();
        }
    };

    public void reset() {
        this.position = 0;
        this.pulseCountQueue.clear();
        this.pulseCountAv = 0;
        this.pulseCount = 0;
        resetPid();
    }

    public void resetPid() {
        this.kPidController.reset();
    }

    public Command runSetpointPercentageCommand(double setpoint) {
        return this.run(() -> this.runSetpointPercentage(setpoint));
    }

    public Command homeCommand() {
        return new FunctionalCommand(null, () -> this.home(), null, () -> this.getStopped(), this);
    }

    // void limit_check() {
    //     quitLoop=HIGH;
    //     Pulse_Counter = 0;
    //     while (quitLoop==HIGH) { // check if actuator has reached fully extend or retract, ie no pulses for 500ms

    //         previousPulse_Counter = Pulse_Counter;
    //         timeelapsed = 0;
    //         timeDelay(pulse_timeout);
    //         if ( FwdRev=='E'){
    //             Counter_diff= Pulse_Counter - previousPulse_Counter;
    //         }else{
    //             Counter_diff= previousPulse_Counter - Pulse_Counter ;
    //         }
    //         if (Counter_diff<=0) {
    //             driveActuator( 1 , 'S',act_PWM ); // Stop actuator
    //             quitLoop=LOW;
    //         }
    //     }
    //     Serial.print(" \n Reached End Limit");
    // };

    // public double getPosition() {
    //     if (now)
    // }
}

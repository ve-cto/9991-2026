package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final WPI_VictorSPX m_intaker;
    private final WPI_VictorSPX m_arm;
    private final Encoder s_armEncoder = new Encoder(Constants.Hardware.kArmEncoderAChannel, Constants.Hardware.kArmEncoderBChannel);
    private double armPosition = 0;
    private double armPositionDebounced = 0;
    private PIDController armPIDController = new PIDController(0.1, 0.0, 0.0);
    private Queue<Double> armPositionHistory = new ArrayBlockingQueue<>(3);
    
    /** Instantiate */
    public Intake() {
        m_intaker = new WPI_VictorSPX(Constants.Hardware.kIntakerId);
        m_arm = new WPI_VictorSPX(Constants.Hardware.kArmId);
    }

    public void periodic() {
        armPosition = s_armEncoder.get();
        if (armPositionHistory.size() >= 3) {
            armPositionHistory.poll();
        }
        armPositionHistory.add(armPosition);
        armPositionDebounced = armPositionHistory.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
    }

    /* Return the current arm position (debounced over 3 values) */
    public double getArmPositionDebounced() {
        return armPositionDebounced;
    }

    public boolean isExtended() {
        return armPositionDebounced >= Constants.Intake.kArmExtendedPosition;
    }

    /* Extend the intake */
    public void extend() {
        if (armPositionDebounced < Constants.Intake.kArmExtendedPosition) {
            m_arm.set(armPIDController.calculate(armPositionDebounced, Constants.Intake.kArmExtendedPosition));
        } else {
            m_arm.stopMotor(); // Stop motor when extended position is reached
        }
    }

    /* Retract the intake */
    public void retract() {
        if (armPositionDebounced > Constants.Intake.kArmRetractedPosition) {
            m_arm.set(-armPIDController.calculate(armPositionDebounced, Constants.Intake.kArmRetractedPosition));
        } else {
            m_arm.stopMotor(); // Stop motor when retracted position is reached
        }
    }

    /** Run the intake motor at the provided speed (-1.0..1.0). */
    public void run(double speed) {
        m_intaker.set(speed);
    }

    /** Stop the intake motor. */
    public void stop() {
        m_intaker.stopMotor();
    }
}

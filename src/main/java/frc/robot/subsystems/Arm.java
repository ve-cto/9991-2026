package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final WPI_VictorSPX m_arm;
    private final Encoder s_armEncoder = new Encoder(Constants.Hardware.kArmEncoderAChannel, Constants.Hardware.kArmEncoderBChannel);
    private double armPosition = 0;
    private double armPositionDebounced = 0;
    private PIDController armPIDController = new PIDController(0.01, 0.0, 0.0);
    private Queue<Double> armPositionHistory = new ArrayBlockingQueue<>(3);
    
    /** Instantiate */
    public Arm() {
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

    /* 
    * return the intake's current position (debounced)
    */
    public double getArmPositionDebounced() {
        return armPositionDebounced;
    }

    /*
     * return whether the intake is extended
     */
    public boolean isExtended() {
        return armPositionDebounced >= Constants.Intake.kArmExtendedPosition;
    }

    /*
     * return whether the intake is retracted
     */
    public boolean isRetracted() {
        return armPositionDebounced <= Constants.Intake.kArmRetractedPosition;
    }

    /*
    * extend the intake 
    */
    public void extend() {
        if (!isExtended()) {
            m_arm.set(armPIDController.calculate(armPositionDebounced, Constants.Intake.kArmExtendedPosition));
        }
    }

    /* 
    * Retract the intake 
    */
    public void retract() {
        if (!isRetracted()) {
            m_arm.set(armPIDController.calculate(armPositionDebounced, Constants.Intake.kArmRetractedPosition));
        }
    }

    public void moveArm(double speed) {
        m_arm.set(speed);
    }

    /*
     * stop the pivot motor with braking force
     */
    public void stopArm() {
        m_arm.stopMotor();
    }

    /*
     * stop the pivot motor without braking force
     */
    public void coastArm() {
        m_arm.set(0.0);
    }

    public Command moveArmCommand(double speed) {
        return this.startEnd(() -> this.moveArm(speed), () -> this.stopArm());
    }

    /*
     * extend the intake
     */
    public Command extendIntakeCommand() {
        return new FunctionalCommand(
            () -> armPIDController.reset(), 
            () -> extend(), 
            interrupted -> coastArm(), 
            () -> isExtended(), 
            this
        );
    }

    /*
     * retract the intake
     */
    public Command retractIntakeCommand() {
        return new FunctionalCommand(
            () -> armPIDController.reset(), 
            () -> retract(), 
            interrupted -> coastArm(), 
            () -> isRetracted(), 
            this
        );
    }

    // public Command extendIntakeCommand() {
    //     return this.startEnd(() -> this.extend(), () -> this.stopArm());
    // }

    // public Command retractIntakeCommand() {
    //     return this.startEnd(() -> this.retract(), () -> this.stopArm());
    // }

    public void simulationInit() {}

    // these motors are already automatically simulated - you don't need to put anything in here buddy :]
    // the encoder can't be simulated though unfortunately
    // if a kraken gets put on the intake, then you will need to put stuff here though, use the shooter subsystem for reference
    public void simulationPeriodic() {}
}

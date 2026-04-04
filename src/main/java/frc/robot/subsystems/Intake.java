package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final WPI_VictorSPX m_intake;
    private final WPI_VictorSPX m_arm;
    private final Encoder s_armEncoder = new Encoder(Constants.Hardware.kArmEncoderAChannel, Constants.Hardware.kArmEncoderBChannel);
    private double armPosition = 0;
    private double armPositionDebounced = 0;
    private PIDController armPIDController = new PIDController(0.01, 0.0, 0.0);
    private Queue<Double> armPositionHistory = new ArrayBlockingQueue<>(3);
    
    /** Instantiate */
    public Intake() {
        m_intake = new WPI_VictorSPX(Constants.Hardware.kIntakeId);
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

    public boolean isRetracted() {
        return armPositionDebounced <= Constants.Intake.kArmRetractedPosition;
    }

    /* Extend the intake */
    public void extend() {
        if (!isExtended()) {
            m_arm.set(armPIDController.calculate(armPositionDebounced, Constants.Intake.kArmExtendedPosition));
        }
    }

    /* Retract the intake */
    public void retract() {
        if (!isRetracted()) {
            m_arm.set(armPIDController.calculate(armPositionDebounced, Constants.Intake.kArmRetractedPosition));
        }
    }

    public void stopArm() {
        m_arm.stopMotor();
    }

    /** Run the intake motor at the provided speed (-1.0..1.0). */
    public void run(double speed) {
        m_intake.set(speed);
    }

    /** Stop the intake motor. */
    public void stopIntake() {
        m_intake.stopMotor();
    }

    /* 
     * Runs the intake forward with the speed given in constants.
     */
    public Command runIntakeCommand() {
        return this.startEnd(() -> this.run(Constants.Intake.kIntakeForwardSpeed), () -> this.stopIntake());
    }

    /*
     * Runs the intake backward with the speed given in constants.
     */
    public Command reverseIntakeCommand() {
        return this.startEnd(() -> this.run(Constants.Intake.kIntakeReverseSpeed), () -> this.stopIntake());
    }

    /*
     * Runs the intake with a given speed.
     */
    public Command runIntakeCommand(double speed) {
        return this.startEnd(() -> this.run(speed), () -> this.stopIntake());
    }

    public Command extendIntakeCommand() {
        return new FunctionalCommand(
            () -> armPIDController.reset(), 
            () -> extend(), 
            interrupted -> stopArm(), 
            () -> isExtended(), 
            this
        );
    }

    public Command retractIntakeCommand() {
        return new FunctionalCommand(
            () -> armPIDController.reset(), 
            () -> retract(), 
            interrupted -> stopArm(), 
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

    // public Command agitateCommand() {
    //     return this.startEnd(() -> this.extend(), () -> this.stopArm()).alongWith(startEnd(() -> this.run(Constants.Intake.kIntakeForwardSpeed), () -> this.stopIntake()));
    // }

    public void simulationInit() {}

    public void simulationPeriodic() {}
}

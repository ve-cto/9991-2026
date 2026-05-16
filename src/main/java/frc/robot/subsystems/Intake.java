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

public class Intake extends SubsystemBase {
    private final WPI_VictorSPX m_intake;

    /** Instantiate */
    public Intake() {
        m_intake = new WPI_VictorSPX(Constants.Hardware.kIntakeId);
    }

    public void periodic() {}

    /* 
    * run the intake motor at the provided speed 
    */
    public void run(double speed) {
        m_intake.set(speed);
    }

    /*
    * stop the intake motor with braking force
    */
    public void stopIntake() {
        m_intake.stopMotor();
    }

    /*
     * stop the intake motor without braking force
     */
    public void coastIntake() {
        m_intake.set(0.0);
    }
    /* 
     * run the intake forward with the speed given in constants
     */
    public Command runIntakeCommand() {
        return this.startEnd(() -> this.run(Constants.Intake.kIntakeForwardSpeed), () -> this.stopIntake());
    }

    /*
     * run the intake backward with the speed given in constants
     */
    public Command reverseIntakeCommand() {
        return this.startEnd(() -> this.run(Constants.Intake.kIntakeReverseSpeed), () -> this.stopIntake());
    }

    /*
     * run the intake with a given speed
     */
    public Command runIntakeCommand(double speed) {
        return this.startEnd(() -> this.run(speed), () -> this.stopIntake());
    }

    public Command coastCommand() {
        return this.run(() -> this.run(0.0));
    }

    public void simulationInit() {}

    // these motors are already automatically simulated - you don't need to put anything in here buddy :]
    // the encoder can't be simulated though unfortunately
    // if a kraken gets put on the intake, then you will need to put stuff here though, use the shooter subsystem for reference
    public void simulationPeriodic() {}
}

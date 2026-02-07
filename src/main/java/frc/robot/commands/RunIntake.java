package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;

public final class RunIntake extends Command {
    private final Intake m_intake;
    private final double m_speed;

    public RunIntake(Intake intake, double speed) {
        m_intake = intake;
        m_speed = speed;

        // Register as a requirement
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        // no-op
    }

    @Override
    public void execute() {
        m_intake.run(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // runs until interrupted (button release)
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(m_intake);
    }
}

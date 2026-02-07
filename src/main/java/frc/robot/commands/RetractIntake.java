package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;

public class RetractIntake extends Command {
    private final Intake m_intake;
    
    public RetractIntake(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        // no-op
    }

    @Override
    public void execute() {
        m_intake.retract();
    }

    @Override
    public void end(boolean interrupted) {
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

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.NetworkTablesIO;

public class DriveToPose extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private Pose2d m_targetPose;
    private final NetworkTablesIO m_networkTablesIO;

    public DriveToPose(Pose2d targetPose, CommandSwerveDrivetrain drivetrain, NetworkTablesIO networkTablesIO) {
        this.m_drivetrain = drivetrain;
        this.m_targetPose = targetPose;
        this.m_networkTablesIO = networkTablesIO;
        addRequirements(drivetrain);
    }   

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_drivetrain.driveToPose(m_targetPose, m_networkTablesIO);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

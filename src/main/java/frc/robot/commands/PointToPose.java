package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.NetworkTablesIO;

public class PointToPose extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private Pose2d m_targetPose;
    private final NetworkTablesIO m_networkTablesIO;
    private final DoubleSupplier m_velX;
    private final DoubleSupplier m_velY;

    public PointToPose(Pose2d targetPose, DoubleSupplier velX, DoubleSupplier velY, CommandSwerveDrivetrain drivetrain, NetworkTablesIO networkTablesIO) {
        this.m_drivetrain = drivetrain;
        this.m_targetPose = targetPose;
        this.m_velX = velX;
        this.m_velY = velY;
        this.m_networkTablesIO = networkTablesIO;
        addRequirements(drivetrain);
    }   

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_drivetrain.pointToPose(m_targetPose, m_velX.getAsDouble(), m_velY.getAsDouble(), m_networkTablesIO);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

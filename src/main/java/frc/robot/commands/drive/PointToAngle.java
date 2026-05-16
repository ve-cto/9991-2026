package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.NetworkTablesIO;

public class PointToAngle extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final DoubleSupplier m_velX;
    private final DoubleSupplier m_velY;
    private final DoubleSupplier m_angle;
    // Blue hub (4.65, 4)
    // Red hub (12, 4)

    public PointToAngle(DoubleSupplier angle, DoubleSupplier velX, DoubleSupplier velY, CommandSwerveDrivetrain drivetrain) {
        this.m_drivetrain = drivetrain;
        this.m_velX = velX;
        this.m_velY = velY;
        this.m_angle = angle;
        
        addRequirements(drivetrain);
    }   

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_drivetrain.pointToAngle(new Rotation2d(m_angle.getAsDouble()), m_velX.getAsDouble(), m_velY.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

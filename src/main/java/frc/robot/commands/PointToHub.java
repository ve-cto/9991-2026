package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.NetworkTablesIO;

public class PointToHub extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private Pose2d m_targetPose;
    private final NetworkTablesIO m_networkTablesIO;
    private final DoubleSupplier m_velX;
    private final DoubleSupplier m_velY;
    private final Pose2d blueHubPose = new Pose2d(4.65, 4, new Rotation2d());
    private final Pose2d redHubPose = new Pose2d(12, 4, new Rotation2d());
    // Blue hub (4.65, 4)
    // Red hub (12, 4)

    public PointToHub(DoubleSupplier velX, DoubleSupplier velY, CommandSwerveDrivetrain drivetrain, NetworkTablesIO networkTablesIO) {
        this.m_drivetrain = drivetrain;
        this.m_velX = velX;
        this.m_velY = velY;
        this.m_networkTablesIO = networkTablesIO;
        
        // if (DriverStation.getAlliance().toString() == "Blue1" || DriverStation.getAlliance().toString() == "Blue2" || DriverStation.getAlliance().toString() == "Blue3") {
        //     this.m_targetPose = blueHubPose;
        // } else {
        //     System.out.println(DriverStation.getAlliance().toString());
        //     this.m_targetPose = redHubPose;
        // }
        addRequirements(drivetrain);
    }   

    @Override
    public void initialize() {
        if (this.m_networkTablesIO.getAlliance()) {
            this.m_targetPose = redHubPose;
        } else {
            this.m_targetPose = blueHubPose;
        }
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

// package frc.robot.commands.drive;

// import java.util.ArrayList;
// import java.util.Collection;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.NetworkTablesIO;

// public class PointToAllianceFuel extends Command {
//     private final CommandSwerveDrivetrain m_drivetrain;
//     private Pose2d currentPose;
//     private final NetworkTablesIO m_networkTablesIO;
//     private final DoubleSupplier m_velX;
//     private final DoubleSupplier m_velY;
//     private boolean isRedAlliance;

//     private final Pose2d blueFuelHigh = new Pose2d(new Translation2d(2, 6), new Rotation2d());
//     private final Pose2d blueFuelLow = new Pose2d(new Translation2d(2, 2), new Rotation2d());
//     private final Pose2d redFuelHigh = new Pose2d(new Translation2d(12.5, 6), new Rotation2d());
//     private final Pose2d redFuelLow = new Pose2d(new Translation2d(12.5, 2), new Rotation2d());

//     private Collection<Pose2d> redPoses = new ArrayList<>(); 
//     private Collection<Pose2d> bluePoses = new ArrayList<>(); 

//     public PointToAllianceFuel(DoubleSupplier velX, DoubleSupplier velY, CommandSwerveDrivetrain drivetrain, NetworkTablesIO networkTablesIO) {
//         this.m_drivetrain = drivetrain;
//         this.m_velX = velX;
//         this.m_velY = velY;
//         this.m_networkTablesIO = networkTablesIO;

//         this.redPoses.add(redFuelHigh);
//         this.redPoses.add(redFuelLow);
//         this.bluePoses.add(blueFuelHigh);
//         this.bluePoses.add(blueFuelLow);

//         addRequirements(drivetrain);
//     }   

//     @Override
//     public void initialize() {
//         this.isRedAlliance = this.m_networkTablesIO.getAlliance();
//     }

//     @Override
//     public void execute() {
//         this.currentPose = m_networkTablesIO.getNetworkPose();
//         Pose2d nearest = new Pose2d();
//         if (this.isRedAlliance)  {
//             nearest = this.currentPose.nearest(redPoses);
//         } else {
//             nearest = this.currentPose.nearest(bluePoses);
//         }
//         m_drivetrain.pointToPose(nearest, m_velX.getAsDouble(), m_velY.getAsDouble(), m_networkTablesIO);
//     }

//     @Override
//     public void end(boolean interrupted) {
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }

// }

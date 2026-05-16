package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.NetworkTablesIO;


public class TrajectoryCalculator extends SubsystemBase {
    private final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    private NetworkTablesIO networkTablesIO;
    private final Pose2d blueHubPose = new Pose2d(4.65, 4, new Rotation2d());
    private final Pose2d redHubPose = new Pose2d(12, 4, new Rotation2d());
    private double hubHeightMeters = 1.8288;
    private double hubHeightInches = 72;

    private boolean isRedAlliance = true;

    private double shooterSpeedPairs[][] = {
        {1, 2500},
        {2, 2750},
        {3, 3000},
        {4, 3500},
        {5, 4000},
        {6, 5000}
    };

    private double hoodAnglePairsDegrees[][] = {
        {0, 90},
        {1, 80},
        {2, 70},
        {4, 60},
        {5, 50},
        {6, 40},
    };

    private InterpolatingDoubleTreeMap shooterSpeedTreeMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap hoodAngleTreeMap = new InterpolatingDoubleTreeMap();

    public TrajectoryCalculator(NetworkTablesIO m_networkTablesIO) {
        this.networkTablesIO = m_networkTablesIO;
        this.isRedAlliance = m_networkTablesIO.getAlliance();
        // key == distance (METERS)
        // value == RPM
        // shooterSpeedTreeMap.put(0.0, 3000.0);
        // shooterSpeedTreeMap.put(1.0, 3300.0);
        // shooterSpeedTreeMap.put(2.0, 3500.0);

        // // key == distance (METERS)
        // // value == RPM
        // hoodAngleTreeMap.put(0.0, 60*(Math.PI/180));
        // hoodAngleTreeMap.put(2.0, 50*(Math.PI/180));
        // hoodAngleTreeMap.put(4.0, 30*(Math.PI/180));
        // // double result = speedTreeMap.get(1.5);

        for (int i = 0; i < shooterSpeedPairs.length; i++) {
            shooterSpeedTreeMap.put(shooterSpeedPairs[i][0], shooterSpeedPairs[i][1]);
        }


        for (int i = 0; i < hoodAnglePairsDegrees.length; i++) {
            hoodAngleTreeMap.put(hoodAnglePairsDegrees[i][0], hoodAnglePairsDegrees[i][1]);
        }
    }


    // // https://blog.eeshwark.com/robotblog/shooting-on-the-fly
    // // https://blog.eeshwark.com/robotblog/shooting-on-the-fly-pt2
    // Note to me: i started looking at this and it is literally just vector math. All those questions where it's like a boat can go x speed, but there's a current going x speed in alpha direction. It's pretty much just that. It's HARD though. like, really hard.
    // public double[] getRequiredSpeedsSOTM(double robotVelX, double robotVelY) {
    //     Translation2d robotTranslation2d = networkTablesIO.getNetworkPose().getTranslation();
    //     Translation2d hubTranslation2d = isRedAlliance ? redHubPose.getTranslation() : blueHubPose.getTranslation();
        
    //     double targetX = hubTranslation2d.getX() - robotTranslation2d.getX();
    //     double targetY = hubTranslation2d.getY() - robotTranslation2d.getY();
    //     Translation2d targetPosition = new Translation2d(targetX, targetY);
        
    //     // since we are working in robot relative (that's why we subtracted the robot position earlier), the robot pose becomes the origin
    //     double distance = targetPosition.getNorm();

    //     // split the speeds into components. this really needs to be as instantaneous velocity OUT of the shooter though.
    //     double horizontalSpeedIdeal = getRequiredShooterSpeed(new Pose2d(targetPosition, new Rotation2d())) * Math.cos(getRequiredHoodAngle(new Pose2d(targetPosition, new Rotation2d())));
    //     double verticalSpeedIdeal = getRequiredShooterSpeed(new Pose2d(targetPosition, new Rotation2d())) * Math.sin(getRequiredHoodAngle(new Pose2d(targetPosition, new Rotation2d())));

    //     Translation2d targetVector = targetPosition.div(distance).times(horizontalSpeedIdeal);

    //     Translation2d robotVelocity = new Translation2d(robotVelX, robotVelY);

    //     Translation2d shotVector = targetVector.minus(robotVelocity);

    //     double requiredRobotAngle = shotVector.getAngle().getDegrees();
    //     double requiredSpeed = shotVector.getNorm();

    //     return null;
    // }
    
    // public DoubleSupplier SOTMgetRequiredRobotRotationHub(DoubleSupplier robotVelX, DoubleSupplier robotVelY, DoubleSupplier heading) {
    //     return () -> {
    //     Translation2d robotTranslation2d = networkTablesIO.getNetworkPose().getTranslation();
    //     Translation2d hubTranslation2d = isRedAlliance ? redHubPose.getTranslation() : blueHubPose.getTranslation();
        
    //     double targetX = hubTranslation2d.getX() - robotTranslation2d.getX();
    //     SmartDashboard.putNumber("shotTargetX", targetX);
    //     double targetY = hubTranslation2d.getY() - robotTranslation2d.getY();
    //     Translation2d targetPosition = new Translation2d(targetX, targetY);
        
    //     // since we are working in robot relative (that's why we subtracted the robot position earlier), the robot pose becomes the origin
    //     double distance = targetPosition.getNorm();
    //     SmartDashboard.putNumber("shotDistance", distance);
    //     // split the speeds into components. this really needs to be as instantaneous velocity OUT of the shooter though. presume shooting at a 45 degree angle 
    //     // divide it by 875 to get the approx exit velocity (this needs to be tuned) TODO:
    //     double horizontalSpeedIdeal = (getRequiredShooterSpeed(new Pose2d(targetPosition, new Rotation2d())) / 875) * Math.cos(Math.PI/3);
    //     Translation2d targetVector = targetPosition.div(distance).times(horizontalSpeedIdeal);

    //     Translation2d robotVelocity = new Translation2d(robotVelX.getAsDouble(), robotVelY.getAsDouble());
    //     // Rotation2d robotHeading = new Rotation2d(heading.getAsDouble());
    //     // Translation2d robotVelRobotFrame =
    //     //     new Translation2d(robotVelX.getAsDouble(), robotVelY.getAsDouble())
    //     //     .rotateBy(robotHeading.unaryMinus());

    //     Translation2d shotVector = targetVector.minus(robotVelocity);
        
    //     // Translation2d robotVelocity = new Translation2d(robotVelX.getAsDouble(), robotVelY.getAsDouble());
    //     // Translation2d shotVector = targetVector.minus(robotVelocity);

    //     // Convert to robot-relative frame
    //     Rotation2d robotHeading = new Rotation2d(heading.getAsDouble());
    //     Translation2d shotVectorRobotFrame = shotVector.rotateBy(robotHeading.unaryMinus());

    //     SmartDashboard.putNumber("shotVecAngleRobotFrame", shotVectorRobotFrame.getAngle().getDegrees());

    //     SmartDashboard.putNumber("shotVecAngle", shotVector.getAngle().getDegrees());

    //     SmartDashboard.putNumber("shotVecX", shotVector.getX());
    //     SmartDashboard.putNumber("shotVecY", shotVector.getY());

    //     SmartDashboard.putNumber("mag_targetVector", targetVector.getNorm());
    //     SmartDashboard.putNumber("mag_robotVelocity", robotVelocity.getNorm());
    //     SmartDashboard.putNumber("mag_shotVector", shotVector.getNorm());

    //     return shotVectorRobotFrame.getAngle().getDegrees();

    //     };
    // }

    public void updateAlliance() {
        this.isRedAlliance = networkTablesIO.getAlliance();
    }
    public Command updateAllianceCommand() {
        return runOnce(() -> this.updateAlliance());
    }

    public DoubleSupplier getRequiredShooterSpeedHub() {
        return () -> shooterSpeedTreeMap.get(getHubDistance());
    }

    public DoubleSupplier getRequiredHoodAngleHub() {
        return () -> hoodAngleTreeMap.get(getHubDistance());
    }

    public double getRequiredShooterSpeed(Pose2d pose) {
        return shooterSpeedTreeMap.get(this.getRobotDistanceToPose(pose));
    }

    public double getRequiredHoodAngle(Pose2d pose) {
        return hoodAngleTreeMap.get(this.getRobotDistanceToPose(pose));
    }

    public double getRobotDistanceToPose(Pose2d pose) {
        return pose.getTranslation().getDistance(networkTablesIO.getNetworkPose().getTranslation());
    }

    // public double getRobotAngleToPoseDegrees(Pose2d pose) {
    //     return pose.getTranslation().getAngle().relativeTo(networkTablesIO.getNetworkPose().getTranslation().getAngle()).getDegrees();
    // }

    // public double getRobotAngleToPoseRadians(Pose2d pose) {
    //     return pose.getTranslation().getAngle().relativeTo(networkTablesIO.getNetworkPose().getTranslation().getAngle()).getRadians();
    // }

    public double getHubDistance() {
        return networkTablesIO.getNetworkPose().getTranslation().getDistance(isRedAlliance ? redHubPose.getTranslation() : blueHubPose.getTranslation());
    }
}

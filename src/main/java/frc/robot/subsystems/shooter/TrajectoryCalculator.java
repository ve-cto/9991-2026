package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.NetworkTablesIO;


public class TrajectoryCalculator extends SubsystemBase {
    private final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    private NetworkTablesIO networkTablesIO;
    private final NetworkTable ntTrajCalcTable = ntInst.getTable("TrajectoryCalculator");
    
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
        updateAlliance();
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

    public DoubleSupplier getRequiredRobotAngleSOTM(SwerveDriveState state) {
        return () -> this.getRequiredSpeedsSOTM(state)[1];
    }

    public DoubleSupplier getRequiredHoodAngleSOTM(SwerveDriveState state) {
        return () -> this.getRequiredHoodAngle(this.getRequiredSpeedsSOTM(state)[2]);
    }

    public DoubleSupplier getRequiredShooterSpeedSOTM(SwerveDriveState state) {
        return () -> this.getRequiredShooterSpeed(this.getRequiredSpeedsSOTM(state)[0]);
    }

    // // https://blog.eeshwark.com/robotblog/shooting-on-the-fly
    // // https://blog.eeshwark.com/robotblog/shooting-on-the-fly-pt2
    // Note to me: i started looking at this and it is literally just vector math. All those questions where it's like a boat can go x speed, but there's a current going x speed in alpha direction. It's pretty much just that. It's HARD though. like, really hard.
    public double[] getRequiredSpeedsSOTM(SwerveDriveState state) {
        Translation2d robotTranslation2d = networkTablesIO.getNetworkPose().getTranslation();
        double robotHeadingRadians = networkTablesIO.getNetworkPose().getRotation().getRadians();
        Translation2d hubTranslation2d = isRedAlliance ? redHubPose.getTranslation() : blueHubPose.getTranslation();
        
        double robotVelX = state.Speeds.vxMetersPerSecond;
        double robotVelY = state.Speeds.vyMetersPerSecond;
        // convert velocity into robot centric
        double velXField = (Math.cos(robotHeadingRadians) * robotVelX) - (Math.sin(robotHeadingRadians) * robotVelY);
        double velYField = (Math.sin(robotHeadingRadians) * robotVelX) + robotVelY * Math.cos(robotHeadingRadians);
        ntTrajCalcTable.putValue("velXField", NetworkTableValue.makeDouble(velXField));
        ntTrajCalcTable.putValue("velYField", NetworkTableValue.makeDouble(velYField));


        double targetXRelative = hubTranslation2d.getX() - robotTranslation2d.getX();
        double targetYRelative = hubTranslation2d.getY() - robotTranslation2d.getY();
        ntTrajCalcTable.putValue("targetXRelative", NetworkTableValue.makeDouble(targetXRelative));
        ntTrajCalcTable.putValue("targetYRelative", NetworkTableValue.makeDouble(targetYRelative));
        Translation2d targetPositionRelative = new Translation2d(targetXRelative, targetYRelative);
        
        Translation2d shotVectorRelative = new Translation2d(targetXRelative - velXField, targetYRelative - velYField);

        if (!this.isRedAlliance) { // if we're on blue we need to invert else the robot is backwards for some reason
            shotVectorRelative = shotVectorRelative.times(-1);
        }

        double distanceToShot = shotVectorRelative.getNorm();
        ntTrajCalcTable.putValue("distanceToShot", NetworkTableValue.makeDouble(distanceToShot));

        double angleToShotRadians = Math.atan2(shotVectorRelative.getY(), shotVectorRelative.getX());
        ntTrajCalcTable.putValue("angleToShotRadians 1", NetworkTableValue.makeDouble(angleToShotRadians));
        angleToShotRadians += Math.PI;
        
        double quadrant = 0;
        if (targetXRelative <= 0 && targetYRelative <= 0) {
            // goal is down left, robot up right
            quadrant = 1;
        } else if (targetXRelative <= 0 && targetYRelative >= 0) {
            // goal is up left, robot down right
            quadrant = 4;
        } else if (targetXRelative >= 0 && targetYRelative <= 0) {
            // goal is down right, robot up left
            quadrant = 2;
        } else if (targetXRelative >= 0 && targetYRelative >= 0) {
            // goal is up right, robot down left
            quadrant = 3;
        }

        ntTrajCalcTable.putValue("quadrant", NetworkTableValue.makeDouble(quadrant));

        // catch broken
        // if (quadrant == 0) {
        //     quadrant = 1;
        // }

        if (quadrant == 1) {
            // angleToShotRadians = -angleToShotRadians; 
        }
        if (quadrant == 2) {
            // angleToShotRadians = angleToShotRadians; 
        }
        if (quadrant == 3) {
            // angleToShotRadians = angleToShotRadians;
        }
        if (quadrant == 4) {
            // angleToShotRadians = Math.PI/2 - angleToShotRadians; 
        }
        // ntTrajCalcTable.putValue("angleToShotRadians 2", NetworkTableValue.makeDouble(angleToShotRadians));
        
        // double angleToShotDegrees = angleToShotRadians * (180/Math.PI);

        // if (angleToShotRadians >= Math.PI) {
        //     // put the angle into -180 to 180 space
        //     angleToShotRadians = angleToShotRadians - 2*Math.PI;
        // }
        ntTrajCalcTable.putValue("angleToShotRadians Final", NetworkTableValue.makeDouble(angleToShotRadians));

        return new double[] {distanceToShot, angleToShotRadians, distanceToShot};
        

        // if (quadrant ==)

        // since we are working in robot relative (that's why we subtracted the robot position earlier), the robot pose becomes the origin
        // double distance = targetPositionRelative.getNorm();

        // // split the speeds into components. this really needs to be as instantaneous velocity OUT of the shooter though.
        // double horizontalSpeedIdeal = getRequiredShooterSpeed(new Pose2d(targetPositionRelative, new Rotation2d())) * Math.cos(getRequiredHoodAngle(new Pose2d(targetPositionRelative, new Rotation2d())));
        // double verticalSpeedIdeal = getRequiredShooterSpeed(new Pose2d(targetPositionRelative, new Rotation2d())) * Math.sin(getRequiredHoodAngle(new Pose2d(targetPositionRelative, new Rotation2d())));

        // Translation2d targetVector = targetPositionRelative.div(distance).times(horizontalSpeedIdeal);

        // Translation2d robotVelocity = new Translation2d(robotVelX, robotVelY);

        // Translation2d shotVector = targetVector.minus(robotVelocity);

        // double requiredRobotAngle = shotVector.getAngle().getDegrees();
        // double requiredSpeed = shotVector.getNorm();

        // return null;
    }
    
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

    public double getRequiredShooterSpeedPose(Pose2d pose) {
        return shooterSpeedTreeMap.get(this.getRobotDistanceToPose(pose));
    }

    public double getRequiredShooterSpeed(double distance) {
        return shooterSpeedTreeMap.get(distance);
    }

    public double getRequiredHoodAngle(double distance) {
        return hoodAngleTreeMap.get(distance);
    }

    public double getRequiredHoodAnglePose(Pose2d pose) {
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

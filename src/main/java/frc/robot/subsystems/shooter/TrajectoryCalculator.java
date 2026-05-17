package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

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
import frc.robot.subsystems.shooter.TrajectoryCalculator.ShotSolution;


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

    private final NetworkTable ntTrajCalcTable = ntInst.getTable("SOTM");

    public void nothing() {}

    public double aaa;

    public ShotSolution computeShot(
        double robotVx, double robotVy,
        Pose2d locationPose,
        double shooterHeight, double locationHeight,
        double launchAngleDeg,
        double clearanceHeight) {
            
        Pose2d robotPose = networkTablesIO.getNetworkPose();

        final double g = 9.81;
        final double theta = Math.toRadians(launchAngleDeg);
        final double sinT = Math.sin(theta);
        final double cosT = Math.cos(theta);

        final double dx = locationPose.getX() - robotPose.getX();
        final double dy = locationPose.getY() - robotPose.getY();

        final double Tmin = 0.2;
        final double Tmax = 3.0;
        final double dT = 0.01;

        final double tolHorizontal = 0.05; // m/s
        final double tolHeight = 0.02;     // m

        ShotSolution best = null;

        for (double T = Tmin; T <= Tmax; T += dT) {

            // Solve for total launch speed V from vertical equation
            // V = (z_target - z0 + 0.5*g*T^2) / (T * sin(theta))
            double numerator = (locationHeight - shooterHeight) + 0.5 * g * T * T;
            double launchSpeed = numerator / (T * sinT);

            if (!Double.isFinite(launchSpeed) || launchSpeed <= 0.0) {
                continue;
            }

            // Components of launch speed
            double Vz = launchSpeed * sinT;   // vertical component
            double Vh = launchSpeed * cosT;   // horizontal magnitude implied by V

            // Required horizontal velocity of ball relative to robot
            double vbx = dx / T - robotVx;
            double vby = dy / T - robotVy;
            double requiredHorizontal = Math.hypot(vbx, vby);

            // Reject if horizontal magnitude mismatch
            if (Math.abs(Vh - requiredHorizontal) > tolHorizontal) {
                continue;
            }

            // Reject backwards shots: required horizontal must point toward target
            double dot = vbx * dx + vby * dy;
            if (dot <= 0.0) {
                continue;
            }

            // Verify vertical position at time T explicitly
            double zAtT = shooterHeight + Vz * T - 0.5 * g * T * T;
            if (Math.abs(zAtT - locationHeight) > tolHeight) {
                continue;
            }

            // Compute peak height and require clearance
            double zPeak = shooterHeight + (Vz * Vz) / (2.0 * g);
            if (zPeak < locationHeight + clearanceHeight) {
                continue;
            }

            // Turret/aim angle in horizontal plane
            double aimAngle = Math.atan2(vby, vbx);

            // Choose best by minimal launchSpeed (or change metric as desired)
            if (best == null || launchSpeed < best.launchSpeed) {
                best = new ShotSolution(launchSpeed, launchAngleDeg, aimAngle, T, zPeak);
            }
        }

        return best;
    }

    public class ShotSolution {
    public final double launchSpeed;     // m/s
    public final double launchAngleDeg;  // hood angle
    public final double aimAngleRad;     // horizontal aim
    public final double timeOfFlight;    // seconds
    public final double peakHeight;      // meters

        public ShotSolution(double launchSpeed, double launchAngleDeg,
                            double aimAngleRad, double timeOfFlight,
                            double peakHeight) {
            this.launchSpeed = launchSpeed;
            this.launchAngleDeg = launchAngleDeg;
            this.aimAngleRad = aimAngleRad;
            this.timeOfFlight = timeOfFlight;
            this.peakHeight = peakHeight;
        }
    }





    // public void getRequiredSOTMInfo(DoubleSupplier inrobotVelX, DoubleSupplier inrobotVelY) {

    //     double hubX = 12; // red hub
    //     double hubY = 4;

    //     double robotVelX = inrobotVelX.getAsDouble();
    //     double robotVelY = inrobotVelY.getAsDouble();
        
    //     double robotX = networkTablesIO.getNetworkPose().getX();
    //     double robotY = networkTablesIO.getNetworkPose().getY();

    //     double robotTohubX = hubX - robotX;
    //     double robotTohubY = hubY - robotY;
        
    //     double robotTohubDistance = Math.hypot(robotTohubX, robotTohubY);

    //     double hubToshotX = -hubX - robotVelX;
    //     double hubToshotY = -hubY - robotVelY;

    //     double robotToshotX = hubX - robotX - robotVelX;
    //     double robotToshotY = hubY - robotY - robotVelY;

    //     double alpha = Math.atan2(robotToshotY, robotToshotX);
    //     double c = Math.sqrt(Math.pow(robotToshotX, 2) + Math.pow(robotToshotY, 2));

    //     ntTrajCalcTable.putValue("robotTohubX", NetworkTableValue.makeDouble(robotTohubX));
    //     ntTrajCalcTable.putValue("robotTohubY", NetworkTableValue.makeDouble(robotTohubY));
    //     ntTrajCalcTable.putValue("robotTohubDistance", NetworkTableValue.makeDouble(robotTohubDistance));
    //     ntTrajCalcTable.putValue("hubToshotX", NetworkTableValue.makeDouble(hubToshotX));
    //     ntTrajCalcTable.putValue("hubToshotY", NetworkTableValue.makeDouble(hubToshotY));
    //     ntTrajCalcTable.putValue("robotToshotX", NetworkTableValue.makeDouble(robotToshotX));
    //     ntTrajCalcTable.putValue("robotToshotY", NetworkTableValue.makeDouble(robotToshotY));
    //     ntTrajCalcTable.putValue("alpha", NetworkTableValue.makeDouble(alpha));
    //     ntTrajCalcTable.putValue("c", NetworkTableValue.makeDouble(c));
    // }

    // my version before clanker shenanigans
    // public void getRequiredSOTMInfo(DoubleSupplier inrobotVelX, DoubleSupplier inrobotVelY) {
    //     double robotVelX = inrobotVelX.getAsDouble();
    //     double robotVelY = inrobotVelY.getAsDouble();
        
    //     double robotX = networkTablesIO.getNetworkPose().getX();
    //     double robotY = networkTablesIO.getNetworkPose().getY();

    //     double robotTohubX = hubX - robotX;
    //     double robotTohubY = hubY - robotY;
        
    //     double robotTohubDistance = Math.sqrt(Math.pow(robotTohubX, 2) + Math.pow(robotTohubY, 2));

    //     double hubToshotX = -hubX - robotVelX;
    //     double hubToshotY = -hubY - robotVelY;

    //     double robotToshotX = hubX - robotX - robotVelX;
    //     double robotToshotY = hubY - robotY - robotVelY;

    //     double alpha = Math.atan(robotToshotY / robotToshotX);
    //     double c = Math.sqrt(Math.pow(robotToshotX, 2) + Math.pow(robotToshotY, 2));

    //     ntTrajCalcTable.putValue("robotTohubX", NetworkTableValue.makeDouble(robotTohubX));
    //     ntTrajCalcTable.putValue("robotTohubY", NetworkTableValue.makeDouble(robotTohubY));
    //     ntTrajCalcTable.putValue("robotTohubDistance", NetworkTableValue.makeDouble(robotTohubDistance));
    //     ntTrajCalcTable.putValue("hubToshotX", NetworkTableValue.makeDouble(hubToshotX));
    //     ntTrajCalcTable.putValue("hubToshotY", NetworkTableValue.makeDouble(hubToshotY));
    //     ntTrajCalcTable.putValue("robotToshotX", NetworkTableValue.makeDouble(robotToshotX));
    //     ntTrajCalcTable.putValue("robotToshotY", NetworkTableValue.makeDouble(robotToshotY));
    //     ntTrajCalcTable.putValue("alpha", NetworkTableValue.makeDouble(alpha));
    //     ntTrajCalcTable.putValue("c", NetworkTableValue.makeDouble(c));
    // }

    // public DoubleSupplier SOTMgetRequiredRobotRotationHub(DoubleSupplier robotVelX, DoubleSupplier robotVelY, DoubleSupplier heading) {
    //     return () -> {
    //         Translation2d robotVel = new Translation2d(robotVelX.getAsDouble(), robotVelY.getAsDouble());
    //         Rotation2d headingRot = Rotation2d.fromRadians(heading.getAsDouble());
            
    //         Translation2d fieldVel = robotVel.rotateBy(headingRot);

    //         // !! heading is radians

    //         getRequiredSOTMInfo(() -> fieldVel.getX(), () -> fieldVel.getX());

    //         return ntTrajCalcTable.getValue("alpha").getDouble();
    //     };
    // }

    // m_trajectoryCalculator.SOTMgetRequiredRobotRotationHub(() -> drivetrain.getState().Speeds.vxMetersPerSecond, () -> drivetrain.getState().Speeds.vyMetersPerSecond, () -> drivetrain.getState().Pose.getRotation().getRadians())

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

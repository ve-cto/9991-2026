package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    public PhotonPoseEstimator poseEstimator;
    public Optional<EstimatedRobotPose> poseEstimate = Optional.empty();

    public final PhotonCamera cameraAlpha = new PhotonCamera(Constants.Vision.kCameraAlphaName);
    public final PhotonCamera cameraBeta = new PhotonCamera(Constants.Vision.kCameraBetaName);
    
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d kRobotToCamAlphaSim = new Transform3d(new Translation3d(0.0, 0.0, 0.5), new Rotation3d(0, 0, 0));
    public static final Transform3d kRobotToCamBetaSim = new Transform3d(new Translation3d(-2.0, 0.0, 3.0), new Rotation3d(0, 0.84, 0)); // -1.5708 radians = 90 degrees

    public static final Transform3d kRobotToCamAlpha = new Transform3d(new Translation3d(0.0, 0.0, 0.5), new Rotation3d(0, 0, 0));
    public static final Transform3d kRobotToCamBeta = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0.84, 0)); // -1.5708 radians = 90 degrees

    private final CommandSwerveDrivetrain m_drivetrain;
    private final NetworkTablesIO m_networkTablesIO;

    private final VisionSystemSim visionSim = new VisionSystemSim("sim");
    // private final TargetModel targetModel = TargetModel.kAprilTag36h11;

    // private List<PhotonPipelineResult> resultsAlpha;

    // throttle sim updates
    private double m_lastVisionSimUpdate = 0.0;
    private static final double kVisionSimMinDt = 0.10; // 100ms

    // throttle addVisionMeasurement updates
    private double m_lastVisionMeasurementUpdate = 0.0;
    private static final double kVisionMeasurementMinDt = 0.2; // 200ms
    
    Pose2d drivetrainPose = new Pose2d();

    /*
     * Create a new vision instance with our drivetrain and networktables.
     */
    public Vision(CommandSwerveDrivetrain drivetrain, NetworkTablesIO networkTablesIO) {
        m_drivetrain = drivetrain;
        m_networkTablesIO = networkTablesIO;

        poseEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCamAlpha);
                
        // only set up sim cameras if we're in sim
        if (RobotBase.isSimulation()) {
            visionSim.addAprilTags(kTagLayout);
            SimCameraProperties simCameraAlphaProp = new SimCameraProperties();
            SimCameraProperties simCameraBetaProp = new SimCameraProperties();
            
            PhotonCameraSim cameraAlphaSim = new PhotonCameraSim(cameraAlpha, simCameraAlphaProp);
            PhotonCameraSim cameraBetaSim = new PhotonCameraSim(cameraBeta, simCameraBetaProp);
            cameraAlphaSim.enableDrawWireframe(true);
            cameraBetaSim.enableDrawWireframe(true);

            visionSim.addCamera(cameraAlphaSim, kRobotToCamAlphaSim);
            visionSim.addCamera(cameraBetaSim, kRobotToCamBetaSim);

            visionSim.getDebugField();
        }
    }

    @Override
    public void periodic() {
        this.drivetrainPose = m_networkTablesIO.getNetworkPose();
    }   

    /*
     * Feed estimated poses to the drivetrain every 200ms
     * Only runs when out of simulation and camera is connected
     */
    public void addVisionMeasurement() {
        // only run if we aren't simulating & camera is connected
        if (RobotBase.isReal()) {
            if (this.cameraAlpha.isConnected()) {
                // throttle updating to every 200ms
                double now = Timer.getFPGATimestamp();
                if (now - m_lastVisionMeasurementUpdate < kVisionMeasurementMinDt) {
                    return;
                }

                poseEstimator.setReferencePose(this.drivetrainPose);
                
                for (var result : this.cameraAlpha.getAllUnreadResults()) {
                    poseEstimate = poseEstimator.estimateCoprocMultiTagPose(result);
                    if (poseEstimate.isEmpty()) {
                        poseEstimate = poseEstimator.estimateLowestAmbiguityPose(result);
                    }
                    
                    if (poseEstimate.isPresent()) {
                        m_drivetrain.addVisionMeasurement(poseEstimate.get().estimatedPose.toPose2d(), result.getTimestampSeconds());
                    }
                }
            }
        }
    }

    /*
    * Returns true if the specified target is currently visible
    */    
    public boolean getTargetVisible(int targetID) {
        var result = this.cameraAlpha.getLatestResult();
        if (result == null || !result.hasTargets()) {
            return false;
        }
        for (var target : result.getTargets()) {
            if (target.getFiducialId() == targetID) {
                return true;
            }
        }
        return false;
    }

    /*
     * Returns the pose of a target if the target is currently visible, where the pose is relative to the robot's rotation.
     * If target is not visible, return null.
     * return Pose3d
     */
    public Pose3d getTargetPoseRobotRelative(int targetID) {  
        var result = this.cameraAlpha.getLatestResult();
        if (result == null || !result.hasTargets()) {
            return null;
        }
        for (var target : result.getTargets()) {
            if (target.getFiducialId() == targetID) {
                // tag found
                var targetBest = target.getBestCameraToTarget();
                return new Pose3d(targetBest.getTranslation(), targetBest.getRotation());
            }
        }
        // if the target isn't visible, return null.
        return null;
    }

    /*
     * Gets an apriltag's pose from the field map
     * If target is not present on field map, return null
     * return Pose3d
     */
    public Pose3d getTargetPose(int targetID) {
        if (kTagLayout.getTagPose(targetID).isPresent()) {
            Pose3d target = kTagLayout.getTagPose(targetID).get();
            return target;
        }
        return null; 
    }

    /*
     * Gets a target pose from the field map IF it is currently visible
     * return Pose3d
     * return null if target is not visible
     */
    public Pose3d getVisibleTargetPose(int targetID) {
        if (getTargetVisible(targetID)) {
            return getTargetPose(targetID);
        }
        return null;
    }

    /*
     * Gets the X, Y, and Z rotation values of a visible target
     * return null if target is not visible
     * return double[3]
     */
    public double[] getTargetAngles(int targetID) {
        Pose3d targetPose = getTargetPoseRobotRelative(targetID);
        if (targetPose == null) {
            return null;
        }
        Rotation3d targetRotation3d = targetPose.getRotation();

        double pitch = targetRotation3d.getY();
        double yaw = targetRotation3d.getZ();
        double roll = targetRotation3d.getX();
        return new double[] {roll, pitch, yaw};
    }

    /*
     * Runs every 20ms while simulating.
     */
    @Override
    public void simulationPeriodic() {
        // throttle results (skip loop)
        double now = Timer.getFPGATimestamp();
        if (now - m_lastVisionSimUpdate < kVisionSimMinDt) {
            return;
        }

        visionSim.update(this.drivetrainPose);
        m_lastVisionSimUpdate = now;
    }
}

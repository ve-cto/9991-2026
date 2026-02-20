package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    public PhotonPoseEstimator poseEstimator;
    public Optional<EstimatedRobotPose> poseEstimate = Optional.empty();

    // get the robot pose fed to network tables - this is given to photonvision as a reference pose
    NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    NetworkTable table = ntInst.getTable("Pose");
    DoubleArraySubscriber poseSubscriber = table.getDoubleArrayTopic("robotPose").subscribe(new double[] {0.0, 0.0, 0.0});

    public final PhotonCamera cameraAlpha = new PhotonCamera(Constants.Vision.kCameraNameAlpha);
    public final PhotonCamera cameraBeta = new PhotonCamera(Constants.Vision.kCameraNameBeta);

    public final Integer[] hubTagsRed = {8, 9, 10, 11};
    public final Integer[] hubTagsBlue = {24, 25, 26, 27};
    
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d kRobotToCamAlpha = new Transform3d(new Translation3d(0.0, 0.0, 0.5), new Rotation3d(0, 0, 0));
    public static final Transform3d kRobotToCamBeta = new Transform3d(new Translation3d(-2.0, 0.0, 3.0), new Rotation3d(0, 0.84, 0)); // -1.5708 radians = 90 degrees

    private final CommandSwerveDrivetrain m_drivetrain;
    private final NetworkTablesIO m_networkTablesIO;

    private final VisionSystemSim visionSim = new VisionSystemSim("sim");
    private final TargetModel targetModel = TargetModel.kAprilTag36h11;

    private List<PhotonPipelineResult> resultsAlpha;

    // throttle sim uldates
    private double m_lastVisionSimUpdate = 0.0;
    private static final double kVisionSimMinDt = 0.05; // 50ms

    double[] networkPose = {0.0, 0.0, 0.0};
    double[] diff = networkPose;
    Pose2d drivetrainPose = new Pose2d();

    /*
     * Create a new vision instance.
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

            visionSim.addCamera(cameraAlphaSim, kRobotToCamAlpha);
            visionSim.addCamera(cameraBetaSim, kRobotToCamBeta);

            visionSim.getDebugField();
        }
    }

    @Override
    public void periodic() {
        //  implement later

        // var resultAlpha = cameraAlpha.getLatestResult();
        // // var resultBeta = cameraBeta.getLatestResult();
    
        // boolean targetVisibleAlpha = resultAlpha.hasTargets();
        // // boolean targetVisibleBeta = resultBeta.hasTargets();
    
        // int targetIDBestAlpha = targetVisibleAlpha ? resultAlpha.getBestTarget().getFiducialId() : null;
        // int targetIDBestBeta = targetVisibleBeta ? resultBeta.getBestTarget().getFiducialId() : null;

        // if (targetIDAlpha != -1) {
        //     if (Arrays.asList(hubTagsRed).contains(targetIDAlpha)) {
        //         // System.out.println("Target is in Red Hub Tags");

        //     } else if (Arrays.asList(hubTagsBlue).contains(targetIDAlpha)) {
        //         // System.out.println("Target is in Blue Hub Tags");
            
        //     }
        // }
        this.networkPose = m_networkTablesIO.getNetworkPoseArray();
        this.drivetrainPose = m_networkTablesIO.getNetworkPose();
    }   

    public Pose2d getNetworkPose() {
        return this.drivetrainPose;
    }

    /*
     * 
     */
    public void addVisionMeasurement() {
        // only run if we aren't simulating
        if (RobotBase.isReal()) {
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

    /*
    * Return true if specified target is currently visible
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
     * Gets a target pose from what the robot currently sees
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
        this.diff = this.networkPose;
        m_lastVisionSimUpdate = now;
    }
}

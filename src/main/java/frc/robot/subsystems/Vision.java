package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    public PhotonPoseEstimator poseEstimator;
    public Optional<EstimatedRobotPose> poseEstimate = Optional.empty();

    // final StructSubscriber<Pose2d> sub;

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

    private final VisionSystemSim visionSim = new VisionSystemSim("sim");
    private final TargetModel targetModel = TargetModel.kAprilTag36h11;

    public Vision(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        poseEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCamAlpha);
                
        // only set up sim if we're in sim, cuz like why waste resources
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
        // commented because simulateable camera's haven't been created yet (TODO)
        
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
    }

    @Override
    public void simulationPeriodic() {
        // Pose2d drivetrainPose = m_drivetrain.getPose();
        double[] placeholder = poseSubscriber.get();

        Translation2d translation = new Translation2d(placeholder[0], placeholder[1]);
        Rotation2d rotation = new Rotation2d(placeholder[2] * (Math.PI/180));

        Pose2d drivetrainPose = new Pose2d(translation, rotation);

        // System.out.println(drivetrainPose.toString()); // debug print
        visionSim.update(drivetrainPose);
    }   

    public void addVisionMeasurement() {
        // only run if we aren't simulating
        if (RobotBase.isReal()) {
            Pose2d drivetrainPose = m_drivetrain.getPose(); 
            poseEstimator.setReferencePose(drivetrainPose);
            
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

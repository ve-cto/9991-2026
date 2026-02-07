package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    public PhotonPoseEstimator poseEstimator;
    public Optional<EstimatedRobotPose> poseEstimate = Optional.empty();

    public final PhotonCamera cameraAlpha = new PhotonCamera(Constants.Vision.kCameraNameAlpha);
    // public final PhotonCamera cameraBeta = new PhotonCamera(Constants.Vision.kCameraNameBeta);
    public final Integer[] hubTagsRed = {8, 9, 10, 11};
    public final Integer[] hubTagsBlue = {24, 25, 26, 27};
    
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
    
    private final CommandSwerveDrivetrain m_drivetrain;

    /**
     * Construct Vision with a reference to the drivetrain so we can read the robot's internal pose.
     * @param drivetrain drivetrain instance (may be null)
     */
    public Vision(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        poseEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
    }

    /**
     * No-arg constructor kept for compatibility (won't have access to drivetrain pose).
     */
    public Vision() {
        this(null);
    }

    @Override
    public void periodic() {


        var resultAlpha = cameraAlpha.getLatestResult();
        // var resultBeta = cameraBeta.getLatestResult();
    
        boolean targetVisibleAlpha = resultAlpha.hasTargets();
        // boolean targetVisibleBeta = resultBeta.hasTargets();
    
        int targetIDBestAlpha = targetVisibleAlpha ? resultAlpha.getBestTarget().getFiducialId() : null;
        // int targetIDBestBeta = targetVisibleBeta ? resultBeta.getBestTarget().getFiducialId() : null;

        // if (targetIDAlpha != -1) {
        //     if (Arrays.asList(hubTagsRed).contains(targetIDAlpha)) {
        //         // System.out.println("Target is in Red Hub Tags");

        //     } else if (Arrays.asList(hubTagsBlue).contains(targetIDAlpha)) {
        //         // System.out.println("Target is in Blue Hub Tags");
            
        //     }
        // }
    }

    public void addVisionMeasurement() {
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

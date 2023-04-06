
package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.VisionConstants;

public class Vision extends SnailSubsystem {
    /*
     * Declare the name of the camera used in the pipeline
     */
    PhotonCamera frontCamera = new PhotonCamera(VisionConstants.USB_CAMERA_NAME_FRONT);

    boolean hasTarget; // Stores whether or not a target is detected
    PhotonPipelineResult result; // Stores all the data that Photonvision returns
    PhotonPoseEstimator frontPoseEstimator; // stores the pose estimator
    // PhotonPoseEstimator backPoseEstimator; // stores the pose estimator
    AprilTagFieldLayout aprilTagFieldLayout; // stores the field layout
    public Pose2d prevEstimatedRobotPose;
    private int desiredScorePos = 1; // Score pos robot should align to for driver assist

    private Optional<EstimatedRobotPose> chosenEstimate;

    public Vision() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
            DriverStation.reportWarning("AprilTagFieldLayout loaded", false);
        } catch (Exception e) {
            // This should be impossible
        }
        frontPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, frontCamera,
                VisionConstants.CAMERA_TO_ROBOT_FRONT);
        frontPoseEstimator.setReferencePose(new Pose2d());

    }

    @Override
    public void periodic() {
        try {
            PhotonPipelineResult currentResult = frontCamera.getLatestResult(); // Query the latest result from
                                                                                // PhotonVision
            hasTarget = result.hasTargets(); // If the camera has detected an apriltag target, the hasTarget boolean
                                             // will be
                                             // true
            if (hasTarget) {
                result = currentResult;
                return;
            }

        } catch (Exception e) {
            // we had an error where photonvision "saw" a tag that didn't exist
            // this is a dirty try catch but it stops the robot frm breaking
        }
    }

    public Pose2d getTagPose2d(int tag_id) {
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(tag_id);
        if (!tagPose.isPresent()) {
            return new Pose2d();
        }
        return tagPose.get().toPose2d();
    }

    /**
     * @param prevEstimatedRobotPose taken from {@link DifferentialDrivePoseEstimator}
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        frontPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        try {
            Optional<EstimatedRobotPose> frontEstimate = frontPoseEstimator.update();
            if (frontEstimate.isPresent()) {
                chosenEstimate = frontEstimate;
                return frontEstimate;
            }

            chosenEstimate = frontEstimate;
            return frontEstimate;
        } catch (Exception e) {
            // stops the exception like periodic
            return chosenEstimate;
        }

    }

    public PhotonTrackedTarget getTargetWithID(int id) { // Returns the apriltag target with the specified ID (if it
                                                         // exists)
        List<PhotonTrackedTarget> targets = result.getTargets(); // Create a list of all currently tracked targets
        for (PhotonTrackedTarget i : targets) {
            if (i.getFiducialId() == id) { // Check the ID of each target in the list
                return i; // Found the target with the specified ID!
            }
        }
        return null; // Failed to find the target with the specified ID
    }

    public PhotonTrackedTarget getBestTarget() {
        // get the target that we have saved
        if (hasTarget) {
            return result.getBestTarget(); // Returns the best (closest) target
        } else {
            return null; // Otherwise, returns null if no targets are currently found
        }
    }

    public boolean getHasTarget() {
        return hasTarget; // Returns whether or not a target was found
    }

    @Override
    public void update() {
    }

    @Override
    public void displayShuffleboard() {
        if (chosenEstimate == null)
            return;
        // display the pose it sees
        if (chosenEstimate.isPresent()) {
            EstimatedRobotPose camPose = chosenEstimate.get();
            SmartDashboard.putNumberArray("Vision poses", new double[] {
                    camPose.estimatedPose.toPose2d().getX(), camPose.estimatedPose.toPose2d().getY(),
                    camPose.timestampSeconds });
        }

        SmartDashboard.putNumber("Desired Score Pos", desiredScorePos);
    }

    @Override
    public void tuningInit() {
    }

    @Override
    public void tuningPeriodic() {
    }

    public void setPipeline(int i) {
        frontCamera.setPipelineIndex(i);
    }

    public int getDesiredScorePos() {
        return desiredScorePos;
    }

    public void incrementScorePos() {
        desiredScorePos++;
        if (desiredScorePos > 9) {
            desiredScorePos = 1;
        }
    }

    public void decrementScorePos() {
        desiredScorePos--;
        if (desiredScorePos < 1) {
            desiredScorePos = 9;
        }
    }
}

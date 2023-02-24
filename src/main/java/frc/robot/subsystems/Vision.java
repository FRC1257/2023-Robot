
package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/**
 * Inspired by UMN Ri3D
 */
public class Vision extends SnailSubsystem {
    PhotonCamera camera = new PhotonCamera(Constants.USB_CAMERA_NAME); // Declare the name of the camera used in the
                                                                       // pipeline
    Transform3d robotToCam = new Transform3d(); // Stores the transform from the center of the robot to the camera
    boolean hasTarget; // Stores whether or not a target is detected
    PhotonPipelineResult result; // Stores all the data that Photonvision returns
    PhotonPoseEstimator poseEstimator; // stores the pose estimator
    AprilTagFieldLayout aprilTagFieldLayout; // stores the field layout
    public Pose2d prevEstimatedRobotPose;


    // stores camera and transform to center of robot
    ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();

    public Vision() {
        camList.add(new Pair<PhotonCamera, Transform3d>(camera, robotToCam));
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
            DriverStation.reportWarning("AprilTagFieldLayout loaded", false);
        } catch (Exception e) {
            // This should be impossible
        }
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS, camera, Constants.Vision.CAMERA_TO_ROBOT);
    }

    @Override
    public void periodic() {
        result = camera.getLatestResult(); // Query the latest result from PhotonVision
        hasTarget = result.hasTargets(); // If the camera has detected an apriltag target, the hasTarget boolean will be
                                         // true
        if (hasTarget) {
            this.result = result;
        }
    }

    public Pose2d getTagPose2d(int tag_id) {
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(tag_id);
        if (!tagPose.isPresent()) {
            return new Pose2d();
        }
        return tagPose.get().toPose2d();
    }

    public Pair<Pose2d, Double> getEstimatedGlobalPose() {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
    
        double currentTime = Timer.getFPGATimestamp();
        Optional<EstimatedRobotPose> tag = poseEstimator.update();
        if (tag.isPresent()) {
            prevEstimatedRobotPose = tag.get().estimatedPose.toPose2d();
            return new Pair<Pose2d, Double>(prevEstimatedRobotPose, currentTime - tag.get().timestampSeconds);
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
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
        // TODO Auto-generated method stub

    }

    @Override
    public void tuningInit() {
        // TODO Auto-generated method stub

    }

    public double getVisionAdd() {
        double visionAdd = 0.0;

        if (result.hasTargets()) {
            visionAdd = 0; // result.getBestTarget().getHorizontalAngleOffset();
        }

        return visionAdd;
    }

    @Override
    public void tuningPeriodic() {
        // TODO Auto-generated method stub

    }

    public void setPipeline(int i) {
        camera.setPipelineIndex(i);
    }
}

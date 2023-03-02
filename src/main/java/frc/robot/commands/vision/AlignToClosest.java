package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Autonomous;
import frc.robot.commands.auto.trajectory.Trajectories;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.Drivetrain.*;

import java.util.ArrayList;
import java.util.List;

public class AlignToClosest extends CommandBase {
    private final Drivetrain drivetrain;
    private Trajectory trajectory;
    private Pose2d currentPose;
    private Pose2d desiredPose;
    private Pose2d[] ALLIANCE_SCORE_POSE;
    
    private Pose2d poseSearch(double poseY) {
        double closestPoseY = Integer.MAX_VALUE;
        int desiredPoseIndex = -1;
        for (int i = 0; i < ALLIANCE_SCORE_POSE.length; i++) {
            double distanceFromScorePose = Math.abs(poseY-ALLIANCE_SCORE_POSE[i].getY()); 
            if (distanceFromScorePose < closestPoseY) {
                desiredPoseIndex = i;
                closestPoseY = distanceFromScorePose;
            }
        }

        DriverStation.reportWarning("Desired Pose " + desiredPoseIndex, false);
        return ALLIANCE_SCORE_POSE[desiredPoseIndex];
    }

    public AlignToClosest(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.currentPose = drivetrain.getPosition();
        if (SmartDashboard.getBoolean("isAllianceBlue", false)) {
            ALLIANCE_SCORE_POSE = Autonomous.BLUE_SCORE_POSE;
        }
        else {
            ALLIANCE_SCORE_POSE = Autonomous.RED_SCORE_POSE;
        }
        desiredPose = poseSearch(currentPose.getY());
        

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        TrajectoryConfig config = new TrajectoryConfig(DRIVE_ALIGN_MAX_VEL, DRIVE_ALIGN_MAX_ACC).setReversed(true);
        List<Pose2d> trajPoints = new ArrayList<Pose2d>();

        trajPoints.add(drivetrain.getPosition());
        trajPoints.add(desiredPose);

        this.trajectory = TrajectoryGenerator.generateTrajectory(trajPoints, config);

        drivetrain.driveTrajectory(trajectory);
    }    
}

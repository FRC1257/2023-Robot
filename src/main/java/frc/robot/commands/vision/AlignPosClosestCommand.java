package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.Autonomous.BLUE_SCORE_POSE;
import static frc.robot.Constants.Autonomous.RED_SCORE_POSE;

import java.util.ArrayList;
import java.util.List;

public class AlignPosClosestCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private Trajectory trajectory;
    private int scoreLocation;
    private Pose2d target;
    private Pose2d[] ALLIANCE_SCORE_POSE;

    // find closest pose by distance
    private Pose2d poseSearchDist() {
        double currentPoseX = drivetrain.getPosition().getX();
        double currentPoseY = drivetrain.getPosition().getY();
        double closestDistDiff = Integer.MAX_VALUE;
        int desiredPoseIndex = -1;

        for (int i = 0; i < ALLIANCE_SCORE_POSE.length; i++) {
            double distanceFromScorePose = Math.sqrt(Math.pow(currentPoseX - ALLIANCE_SCORE_POSE[i].getX(), 2)
                    + Math.pow(currentPoseY - ALLIANCE_SCORE_POSE[i].getY(), 2));
            if (distanceFromScorePose < closestDistDiff) {
                desiredPoseIndex = i;
                closestDistDiff = distanceFromScorePose;
            }
        }

        SmartDashboard.putNumber("Closest Pose Command", desiredPoseIndex);
        return ALLIANCE_SCORE_POSE[desiredPoseIndex];
    }

    // find closest pose by y coordinate
    private Pose2d poseSearchY() {
        double currentPoseY = drivetrain.getPosition().getY();
        double closestPoseYDiff = Integer.MAX_VALUE;
        int desiredPoseIndex = -1;
        for (int i = 0; i < ALLIANCE_SCORE_POSE.length; i++) {
            double distanceFromScorePose = Math.abs(currentPoseY - ALLIANCE_SCORE_POSE[i].getY());
            if (distanceFromScorePose < closestPoseYDiff) {
                desiredPoseIndex = i;
                closestPoseYDiff = distanceFromScorePose;
            }
        }

        SmartDashboard.putNumber("Closest Pose Command", desiredPoseIndex);
        return ALLIANCE_SCORE_POSE[desiredPoseIndex];
    }

    public AlignPosClosestCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        if (SmartDashboard.getBoolean("isAllianceBlue", false)) {
            ALLIANCE_SCORE_POSE = BLUE_SCORE_POSE;
        } 
        else {
            ALLIANCE_SCORE_POSE = RED_SCORE_POSE;
        }

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        target = poseSearchDist();
        TrajectoryConfig config = new TrajectoryConfig(DRIVE_ALIGN_MAX_VEL, DRIVE_ALIGN_MAX_ACC).setReversed(true);
        List<Pose2d> trajPoints = new ArrayList<Pose2d>();

        trajPoints.add(drivetrain.getPosition());
        trajPoints.add(target);

        trajectory = TrajectoryGenerator.generateTrajectory(trajPoints, config);

        drivetrain.driveTrajectory(trajectory);
        SmartDashboard.putNumber("Ye mother: Trajectory Length", this.trajectory.getTotalTimeSeconds());
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.endPID();
    }

    @Override
    public boolean isFinished() {
        return drivetrain.getState() != Drivetrain.State.TRAJECTORY;
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }
}

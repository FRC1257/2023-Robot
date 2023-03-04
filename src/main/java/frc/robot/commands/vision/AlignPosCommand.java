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

public class AlignPosCommand extends CommandBase { 
    private final Drivetrain drivetrain;
    private Trajectory trajectory;
    private Pose2d target;
    private Pose2d[] ALLIANCE_SCORE_POSE;
    
    private Pose2d poseSearch() {
        double currentPoseY = drivetrain.getPosition().getY();
        double closestPoseY = Integer.MAX_VALUE;
        int desiredPoseIndex = -1;
        for (int i = 0; i < ALLIANCE_SCORE_POSE.length; i++) {
            double distanceFromScorePose = Math.abs(currentPoseY-ALLIANCE_SCORE_POSE[i].getY()); 
            if (distanceFromScorePose < closestPoseY) {
                desiredPoseIndex = i;
                closestPoseY = distanceFromScorePose;
            }
        }

        DriverStation.reportWarning("Desired Pose " + desiredPoseIndex, false);
        return ALLIANCE_SCORE_POSE[desiredPoseIndex];
    }

    public AlignPosCommand(Drivetrain drivetrain, Pose2d target) { 
        this.drivetrain = drivetrain;
        this.target = target;
        
        addRequirements(drivetrain);
    }

    public AlignPosCommand(Drivetrain drivetrain, int scoreLocation) {
        this.drivetrain = drivetrain;
        
        if (SmartDashboard.getBoolean("isAllianceBlue", false)) {
            this.target = BLUE_SCORE_POSE[scoreLocation];
        } else {
            this.target = RED_SCORE_POSE[scoreLocation];
        }

        addRequirements(drivetrain);
    }

    public AlignPosCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        if (SmartDashboard.getBoolean("isAllianceBlue", false)) {
            ALLIANCE_SCORE_POSE = BLUE_SCORE_POSE;
        }
        else {
            ALLIANCE_SCORE_POSE = RED_SCORE_POSE;
        }
        
        target = poseSearch();
        

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        TrajectoryConfig config = new TrajectoryConfig(DRIVE_ALIGN_MAX_VEL, DRIVE_ALIGN_MAX_ACC).setReversed(true);
        List<Pose2d> trajPoints = new ArrayList<Pose2d>();

        trajPoints.add(drivetrain.getPosition());
        trajPoints.add(target);


        this.trajectory = TrajectoryGenerator.generateTrajectory(trajPoints, config);

        drivetrain.driveTrajectory(trajectory);
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

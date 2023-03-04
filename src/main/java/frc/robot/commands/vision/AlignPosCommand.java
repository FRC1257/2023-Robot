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
    private int scoreLocation;
    private Pose2d target;
    private Pose2d[] ALLIANCE_SCORE_POSE;

    public AlignPosCommand(Drivetrain drivetrain, Pose2d target) { 
        this.drivetrain = drivetrain;
        this.target = target;
        
        addRequirements(drivetrain);
    }

    public AlignPosCommand(Drivetrain drivetrain, int scoreLocation) {
        this.drivetrain = drivetrain;
        this.scoreLocation = scoreLocation;

        if (SmartDashboard.getBoolean("isAllianceBlue", false)) {
            this.target = BLUE_SCORE_POSE[scoreLocation];
        } else {
            this.target = RED_SCORE_POSE[scoreLocation];
        }

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        TrajectoryConfig config = new TrajectoryConfig(DRIVE_ALIGN_MAX_VEL, DRIVE_ALIGN_MAX_ACC).setReversed(true);
        List<Pose2d> trajPoints = new ArrayList<Pose2d>();

        trajPoints.add(drivetrain.getPosition());
        trajPoints.add(target);

        trajectory = TrajectoryGenerator.generateTrajectory(trajPoints, config);

        drivetrain.driveTrajectory(trajectory);
        SmartDashboard.putNumber("Ye mother: Trajectory Length", this.trajectory.getTotalTimeSeconds());

        scoreLocation = (int) SmartDashboard.getNumber("Score Position Chooser", 0);
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

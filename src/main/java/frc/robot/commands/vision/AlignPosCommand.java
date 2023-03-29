package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.Autonomous.*;

import java.util.ArrayList;
import java.util.List;

public class AlignPosCommand extends CommandBase { 
    private final Drivetrain drivetrain;
    private final Vision vision;
    private Trajectory trajectory;
    private Pose2d[] POSSIBLE_TARGETS;
    private Pose2d target;
    private int desiredScorePos;
    private boolean useVision;

    public AlignPosCommand(Drivetrain drivetrain, Vision vision, Pose2d target) { 
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.target = target;
        useVision = false;
        
        addRequirements(drivetrain, vision);
    }

    public AlignPosCommand(Drivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        desiredScorePos = vision.getDesiredScorePos();

        if (SmartDashboard.getBoolean("isAllianceBlue", false))
            this.POSSIBLE_TARGETS = BLUE_SCORE_POSE;
        else
            this.POSSIBLE_TARGETS = RED_SCORE_POSE;
        
        useVision = true;
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        desiredScorePos = vision.getDesiredScorePos(); 
        TrajectoryConfig config = new TrajectoryConfig(DRIVE_ALIGN_MAX_VEL, DRIVE_ALIGN_MAX_ACC).setReversed(true);
        List<Pose2d> trajPoints = new ArrayList<Pose2d>();

        trajPoints.add(drivetrain.getPosition());
        if (useVision)
            trajPoints.add(POSSIBLE_TARGETS[desiredScorePos]);
        else
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

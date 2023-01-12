package frc.robot.commands;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.Drivetrain.*;
public class ToCargoCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final Pose2d startPose;
    private final Pose2d endPose;

    public ToCargoCommand(Drivetrain drivetrain, Pose2d startPose, Pose2d endPose) {
        this.drivetrain = drivetrain;
        this.startPose = startPose;
        this.endPose = endPose;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        //generate trajectory and follow it
        //must choose at least one waypoint on one side of the charging station, depending on where the end location is
        // might get stuck on charging station or waste time
        List<Pose2d> trajPoints; 
        //waypoint locations
        //x= blue 96.75 in or red 484.81 in = 2.45745 m or 12.314174
        //y= outer 29.695 in or inner 125.695 in  = 0.754253 m or 3.192653 m
        //double x, y;
        //center line 107.39 in = 2.727706 m
        trajPoints.add(startPose);
        //Pose2d waypoint = new Pose2d(x, y, someRotation);
        //trajPoints.add(waypoint);
        trajPoints.add(endPose);
        TrajectoryConfig config = new TrajectoryConfig(DRIVE_TRAJ_MAX_VEL, DRIVE_TRAJ_MAX_ACC);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(trajPoints, config);
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
}

package frc.robot.commands;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.Drivetrain.*;

public class ToChargeCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final Pose2d CurrentPosition;
    private final Pose2d NextPositionOne;

    private Trajectory trajectory;

    public ToChargeCommand(Drivetrain drivetrain, Pose2d CurrentPosition, Pose2d endPose) {
        this.drivetrain = drivetrain;
        this.CurrentPosition = CurrentPosition;
        this.NextPositionOne = endPose;
        List<Pose2d> TrajPointsOne = new ArrayList<Pose2d>();
        List<Pose2d> TrajPointsTwo = new ArrayList<Pose2d>();

        TrajPointsOne.add(CurrentPosition);
        TrajPointsTwo.add(CurrentPosition);

        TrajPointsOne.add(NextPositionOne);
        TrajPointsTwo.add(NextPositionOne);

        TrajectoryConfig config = new TrajectoryConfig(DRIVE_TRAJ_MAX_VEL, DRIVE_TRAJ_MAX_ACC);
        trajectory = TrajectoryGenerator.generateTrajectory(TrajPointsOne, config);
        Trajectory trajectoryTwo = TrajectoryGenerator.generateTrajectory(TrajPointsTwo, config);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.driveTrajectory(trajectory);

        /* if(trajectoryOne.getTotalTimeSeconds() > trajectoryTwo.getTotalTimeSeconds()) {
            drivetrain.driveTrajectory(trajectoryOne);
        } else {
            drivetrain.driveTrajectory(trajectoryTwo);
        } */
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
  

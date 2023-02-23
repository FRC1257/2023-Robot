// robot commands package
package frc.robot.commands;

// iterable collections (java builtins)
import java.util.ArrayList;
import java.util.List;

// 2d coordinate set
import edu.wpi.first.math.geometry.Pose2d;

// trajectory generation
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

// command base class
import edu.wpi.first.wpilibj2.command.CommandBase;

// drivetrain subsystem and info
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.Drivetrain.*;

// command for going to charging station
public class ToChargeCommand extends CommandBase {
    // stores drivetrain to move the robot
    private final Drivetrain drivetrain;

    // positions on the path to the charging station
    private final Pose2d CurrentPosition;
    private final Pose2d NextPositionOne;

    // trajectory object
    private Trajectory trajectory;

    public ToChargeCommand(Drivetrain drivetrain, Pose2d CurrentPosition, Pose2d endPose) {
        // initializes variables
        this.drivetrain = drivetrain;
        this.CurrentPosition = CurrentPosition;
        this.NextPositionOne = endPose;

        // two trajectory paths that utilize different points to engage with the charging station
        List<Pose2d> TrajPointsOne = new ArrayList<Pose2d>();
        List<Pose2d> TrajPointsTwo = new ArrayList<Pose2d>();

        // current position first
        TrajPointsOne.add(CurrentPosition);
        TrajPointsTwo.add(CurrentPosition);

        // adds next position to the trajectory
        TrajPointsOne.add(NextPositionOne);
        TrajPointsTwo.add(NextPositionOne);

        // configures trajectory generator and uses it
        TrajectoryConfig config = new TrajectoryConfig(DRIVE_TRAJ_MAX_VEL, DRIVE_TRAJ_MAX_ACC);
        Trajectory trajectoryOne = TrajectoryGenerator.generateTrajectory(TrajPointsOne, config);
        Trajectory trajectoryTwo = TrajectoryGenerator.generateTrajectory(TrajPointsTwo, config);

        // chooses fastest trajectory
        if(trajectoryOne.getTotalTimeSeconds() < trajectoryTwo.getTotalTimeSeconds()) {
            trajectory = trajectoryOne;
        }
        else {
            trajectory = trajectoryTwo;
        }

        // requires drivetrain
        addRequirements(drivetrain);
    }

    // starts command execution
    @Override
    public void initialize() {
        // use trajectory
        drivetrain.driveTrajectory(trajectory);
    }

    // runs command processes
    @Override
    public void execute() {

    }

    // end command execution
    @Override
    public void end(boolean interrupted) {
        drivetrain.endPID();
    }

    // check if the robot has completed the trajectory
    @Override
    public boolean isFinished() {
        return drivetrain.getState() != Drivetrain.State.TRAJECTORY;
    }

    // getter function for trajectory
    public Trajectory getTrajectory() {
        return trajectory;
    }
}


// robot commands package
package frc.robot.commands;

// built in java lists
import java.util.List;

// coordinate class
import edu.wpi.first.math.geometry.Pose2d;

// trajectories
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

// command base classs
import edu.wpi.first.wpilibj2.command.CommandBase;

// drivetrain for moving robot along trajectory
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.Drivetrain.*;

// goes to the scoring area
public class ToScoreCommand extends CommandBase{
    // drivetrain stored for movement
    private final Drivetrain drivetrain;

    // positions to input into trajectory
    private final Pose2d CurrentPosition;
    private final Pose2d NextPosition;

    // trajectory variable
    private Trajectory trajectory;

    public ToScoreCommand(Drivetrain drivetrain, Pose2d CurrentPosition, Pose2d NextPosition) {
        // stores positions for trajectories
        this.CurrentPosition = CurrentPosition;
        this.NextPosition = NextPosition;
        this.drivetrain = drivetrain;

        // configures trajectory
        TrajectoryConfig config = new TrajectoryConfig(DRIVE_TRAJ_MAX_VEL, DRIVE_TRAJ_MAX_ACC);
        trajectory = TrajectoryGenerator.generateTrajectory(List.of(CurrentPosition, NextPosition), config);

        addRequirements(drivetrain);
    }

    // called when command begins execution
    @Override
    public void initialize() {
        // uses drivetrain to move along trajectory
        drivetrain.driveTrajectory(trajectory);
    }

    // execution of command
    @Override
    public void execute() {

    }

    // finishes execution
    @Override
    public void end(boolean interrupted) {
        drivetrain.endPID();
    }

    // checks if command execution is complete
    @Override
    public boolean isFinished() {
        return drivetrain.getState() != Drivetrain.State.TRAJECTORY;
    }

    // getter function for trajectory
    public Trajectory getTrajectory() {
        return trajectory;
    }
  
}

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.Drivetrain.*;
import java.util.List;

public class ToPosCommand extends CommandBase { 
    private final Drivetrain drivetrain;
    private Trajectory trajectory;



    public ToPosCommand(Drivetrain drivetrain, List<Pose2d> trajPoints, boolean reverse) { 
        this.drivetrain = drivetrain;
        TrajectoryConfig config = new TrajectoryConfig(DRIVE_TRAJ_MAX_VEL, DRIVE_TRAJ_MAX_ACC).setReversed(reverse);
        this.trajectory = TrajectoryGenerator.generateTrajectory(trajPoints, config);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
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

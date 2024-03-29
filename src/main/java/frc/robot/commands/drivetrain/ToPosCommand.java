package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.Drivetrain.*;
import java.util.List;

public class ToPosCommand extends CommandBase { 
    private final Drivetrain drivetrain;
    private Trajectory trajectory;

    // save the points for debugging
    private List<Pose2d> points;

    public ToPosCommand(Drivetrain drivetrain, List<Pose2d> trajPoints, boolean reverse) { 
        this.drivetrain = drivetrain;
        points = trajPoints;
        
        TrajectoryConfig config = new TrajectoryConfig(DRIVE_TRAJ_MAX_VEL, DRIVE_TRAJ_MAX_ACC).setReversed(reverse);
        this.trajectory = TrajectoryGenerator.generateTrajectory(trajPoints, config);

        if (trajectory.getTotalTimeSeconds() <= 0.1) {
            DriverStation.reportError("Something is wrong with the trajectory", true);
        }

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

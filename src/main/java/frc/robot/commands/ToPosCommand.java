package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TunableNumber;

import static frc.robot.Constants.Drivetrain.*;
import java.util.List;

public class ToPosCommand extends CommandBase { 
    private final Drivetrain drivetrain;
    private Trajectory trajectory;



    private TunableNumber maxVel = new TunableNumber("Max Velocity", DRIVE_TRAJ_MAX_VEL, true);
    private TunableNumber maxAccel = new TunableNumber("Max Acceleration", DRIVE_TRAJ_MAX_ACC, true);

    public ToPosCommand(Drivetrain drivetrain, List<Pose2d> trajPoints, boolean reverse) { 
        this.drivetrain = drivetrain;
        
        TrajectoryConfig config = new TrajectoryConfig(maxVel.get(), maxAccel.get()).setReversed(reverse);
        this.trajectory = TrajectoryGenerator.generateTrajectory(trajPoints, config);

        addRequirements(drivetrain);
    }

    public ToPosCommand(Drivetrain drivetrain, List<Pose2d> trajPoints, boolean reverse, double speed) { 
        this.drivetrain = drivetrain;
        
        TrajectoryConfig config = new TrajectoryConfig(maxVel.get(), maxAccel.get()).setReversed(reverse).setStartVelocity(speed);
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

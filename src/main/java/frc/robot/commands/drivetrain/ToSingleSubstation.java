package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.Drivetrain.*;
import java.util.List;

public class ToSingleSubstation extends CommandBase { 
    private final Drivetrain drivetrain;
    private Trajectory trajectory;
    private boolean blue;

    public ToSingleSubstation(Drivetrain drivetrain, boolean blue) { 
        this.drivetrain = drivetrain;
        this.blue = blue;

        addRequirements(drivetrain);
    }

    public Pose2d getSubstationPos() {
        if (blue) {
            return new Pose2d(7.686, 24.848, Rotation2d.fromDegrees(90));
        }
        return new Pose2d(7.686, 24.848, Rotation2d.fromDegrees(90));
    }

    @Override
    public void initialize() {
        TrajectoryConfig config = new TrajectoryConfig(DRIVE_TRAJ_MAX_VEL, DRIVE_TRAJ_MAX_ACC).setReversed(true);
        this.trajectory = TrajectoryGenerator.generateTrajectory(List.of(drivetrain.getPosition(), getSubstationPos()), config);
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

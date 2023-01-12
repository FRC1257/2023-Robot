package frc.robot.commands;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.Drivetrain.*;

public class ToChargeCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final Pose2d startPose;

    public ToChargeCommand(Drivetrain drivetrain, Pose2d startPose) {
        this.drivetrain = drivetrain;
        this.startPose = startPose;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        
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
  

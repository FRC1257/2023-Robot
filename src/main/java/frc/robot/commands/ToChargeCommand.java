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
    private final Pose2d NextPositionTwo;

    public ToChargeCommand(Drivetrain drivetrain, Pose2d CurrentPosition) {
        this.drivetrain = drivetrain;
        this.CurrentPosition = CurrentPosition;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

        List<Pose2d> TrajPointsOne = new ArrayList<Pose2d>();
        List<Pose2d> TrajPointsTwo = new ArrayList<Pose2d>();

        TrajPointsOne.add(CurrentPosition);
        TrajPointsTwo.add(CurrentPosition);

        TrajPointsOne.add(NextPositionOne);
        TrajPointsTwo.add(NextPositionTwo);
        
        // Create Two different Tajectories : Trajectory_A, Trajectory_B
        // Compare the lengths of Trajectory_A and Trajectory_B
        // Determine the shortest possible route to the Charging station
        // Generate Trajectory 
        // Generate Second Straight Line Trajectory [Trajectory_C]
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
  

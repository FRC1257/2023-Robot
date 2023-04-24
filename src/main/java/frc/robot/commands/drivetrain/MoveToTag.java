package frc.robot.commands.drivetrain;


import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class MoveToTag extends CommandBase {
    Vision vision;
    Drivetrain drivetrain;
    int tag_id;
    Trajectory trajectory;

    public MoveToTag(Vision vision, Drivetrain drivetrain, int tag_id) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.tag_id = tag_id;

        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        // vision.setPipeline(0);
        
        // Pose2d finalPose = vision.getTagPose2d(tag_id);

        // ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();

        // TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
        // config.setReversed(true);
        // Pair<Pose2d, Double> currentPose = vision.getEstimatedGlobalPose();

        // trajectory = TrajectoryGenerator.generateTrajectory(
        // currentPose.getFirst(),
        // interiorWaypoints,
        // finalPose,
        // config);
        
        
    }

    @Override
    public void execute() {
        // drivetrain.driveTrajectory(trajectory);
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

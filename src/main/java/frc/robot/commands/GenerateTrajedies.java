package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.Autonomous;
import frc.robot.RobotContainer;

public class GenerateTrajedies {
    private boolean charge;
    private boolean score;
    private boolean cargo;
    private Drivetrain driveTrain;
    private final Pose2d StartPos;
    private final Pose2d NewPos;
    private SequentialCommandGroup command;
    private Pose2d currentPos; 
    private Trajectory fullTrajectory;
    private Pose2d[] ALLIANCE_CARGO_POSE;
    private Pose2d[] ALLIANCE_SCORE_POSE;

    public GenerateTrajedies(boolean charge, boolean score, boolean cargo, Drivetrain driveTrain, Pose2d StartPos,
            Pose2d NewPos) {
        this.charge = charge;
        this.score = score;
        this.cargo = cargo;
        this.driveTrain = driveTrain;
        this.StartPos = StartPos;
        this.currentPos = StartPos;
        this.NewPos = NewPos;
        command = new SequentialCommandGroup();
        currentPos = new Pose2d();
        fullTrajectory = new Trajectory();
        if (SmartDashboard.getBoolean("isAllianceBlue", false)) {
            ALLIANCE_CARGO_POSE = Autonomous.BLUE_CARGO_POSE;
            ALLIANCE_SCORE_POSE = Autonomous.BLUE_SCORE_POSE;
        } else {
            ALLIANCE_CARGO_POSE = Autonomous.RED_CARGO_POSE;
            ALLIANCE_SCORE_POSE = Autonomous.RED_SCORE_POSE;
        }
    }

    // TODO make method to get positions
    // also TODO make method to get the trajectories to visualize

    public Pose2d getScoreLocation() {
        // should be stored as a constant then retrieved for this
        // currently returning this random thing

        // in reality we would probably need to use some more variables
        // like station (one of the 3 blocks) and score num (one of the 3 places to score) to get it 
        return new Pose2d(3, 0, new Rotation2d(0.0));
    }

    //gets the cargoLocation based on what side the robot is on

    public Pose2d getCargoLocation() {
        return ALLIANCE_CARGO_POSE[RobotContainer.gamePieceChooser.getSelected()];
    }

    // 2 getScoreLocation() methods for some reason?
    public Pose2d getscoreLocation() {
        return ALLIANCE_SCORE_POSE[RobotContainer.scorePositionChooser.getSelected()];
    }

    public Pose2d getChargeLocation() {
        // should be stored as a constant then retrieved for this
        // currently returning this random thing

        // in reality there are 2 possible places so we would just need to use the side of the field we are on
        return new Pose2d(5, 5, new Rotation2d(0.0));
    }

    public Pose2d getLeaveCommunityPose() {
        // should be stored as a constant then retrieved for this
        // currently returning this random thing

        // in reality there are 6 possible places so we would just need to use the varialbes we have
        return new Pose2d(5, 5, new Rotation2d(0.0));
    }

    private void trajediesDecider() {
        command = new SequentialCommandGroup();
        // there are 3 possible steps we can take
        if (score) {
            ToScoreCommand step1 = new ToScoreCommand(driveTrain, StartPos, getScoreLocation());
            currentPos = getScoreLocation();
            // fullTrajectory = fullTrajectory.concatenate(step1.getTrajectory());
            command.andThen(step1);
        }

        // we either go for cargo or leave the tarmac to get points
        if (cargo) {
            ToCargoCommand step2 = new ToCargoCommand(driveTrain, currentPos, getCargoLocation());
            currentPos = getCargoLocation();
            // fullTrajectory = fullTrajectory.concatenate(step2.getTrajectory());
            command.andThen(step2);
        } 
        else {
            ToPos step2 = new ToPos(driveTrain, currentPos, getLeaveCommunityPose());
            currentPos = getLeaveCommunityPose();
            // fullTrajectory = fullTrajectory.concatenate(step2.getTrajectory());
            command.andThen(step2);
        }

        // step 3 go for charge
        if (charge) {
            ToChargeCommand step3 = new ToChargeCommand(driveTrain, currentPos, getChargeLocation());
            currentPos = getChargeLocation();
            // fullTrajectory = fullTrajectory.concatenate(step3.getTrajectory());
            command.andThen(step3);
        }

        // if none of these have run something has gone wrong
        // so just leave the community
        if (StartPos.equals(currentPos)) {
            ToPos leave = new ToPos(driveTrain, currentPos, getLeaveCommunityPose());
            currentPos = getLeaveCommunityPose();
            // fullTrajectory = fullTrajectory.concatenate(leave.getTrajectory());
            command.andThen(leave);
        }

    }

    public SequentialCommandGroup getCommand() {
        trajediesDecider();
        return command;
    }

    public Trajectory getTrajectory() {
        return fullTrajectory;
    }
}

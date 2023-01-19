package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
    private final Pose2d StartPose;
    
    private SequentialCommandGroup command;
    private Pose2d currentPose; 
    private Trajectory fullTrajectory;
    private Pose2d[] ALLIANCE_START_POSE;
    private Pose2d[] ALLIANCE_CARGO_POSE;
    private Pose2d[] ALLIANCE_SCORE_POSE;
    private Pose2d[] ALLIANCE_WAYPOINTS_POSE;
    private final Pose2d chargePose;

    public GenerateTrajedies(boolean isCharge, boolean isScore, boolean isCargo, Drivetrain driveTrain, int StartPose) {
        this.charge = isCharge;
        this.score = isScore;
        this.cargo = isCargo;
        this.driveTrain = driveTrain;
        
        command = new SequentialCommandGroup();
        currentPose = new Pose2d();
        fullTrajectory = new Trajectory();
        if (SmartDashboard.getBoolean("isAllianceBlue", false)) {
            ALLIANCE_START_POSE = Autonomous.BLUE_START_POSE;
            ALLIANCE_CARGO_POSE = Autonomous.BLUE_CARGO_POSE;
            ALLIANCE_SCORE_POSE = Autonomous.BLUE_SCORE_POSE;
            ALLIANCE_WAYPOINTS_POSE = Autonomous.BLUE_WAYPOINTS_POSE;
            chargePose = Autonomous.BLUE_CHARGE_POSE;
        } else {
            ALLIANCE_START_POSE = Autonomous.RED_START_POSE;
            ALLIANCE_CARGO_POSE = Autonomous.RED_CARGO_POSE;
            ALLIANCE_SCORE_POSE = Autonomous.RED_SCORE_POSE;
            ALLIANCE_WAYPOINTS_POSE = Autonomous.RED_WAYPOINTS_POSE;
            chargePose = Autonomous.RED_CHARGE_POSE;
        }
        this.StartPose = ALLIANCE_START_POSE[StartPose]; 
        this.currentPose = this.StartPose;
    }

    // TODO make method to get positions
    // also TODO make method to get the trajectories to visualize

    //gets the cargoLocation based on what side the robot is on

    public Pose2d getCargoLocation() {
        return ALLIANCE_CARGO_POSE[RobotContainer.gamePieceChooser.getSelected()];
    }

    // 2 getScoreLocation() methods for some reason?
    public Pose2d getScoreLocation() {
        return ALLIANCE_SCORE_POSE[RobotContainer.scorePositionChooser.getSelected()];
    }

    public Pose2d getChargeLocation() {
        // should be stored as a constant then retrieved for this
        // currently returning this random thing

        // in reality there are 2 possible places so we would just need to use the side of the field we are on
        return chargePose;
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
            ToScoreCommand step1 = new ToScoreCommand(driveTrain, StartPose, getScoreLocation());
            currentPose = getScoreLocation();
            fullTrajectory = fullTrajectory.concatenate(step1.getTrajectory());
            command.addCommands(step1);
        }

        // we either go for cargo or leave the tarmac to get points
        if (cargo) {
            List<Pose2d> trajPoints = new ArrayList<Pose2d>();
            Pose2d endPose = getCargoLocation();
            trajPoints.add(currentPose);
            if (endPose.getY() > CHARGE_STATION_UPPER_Y) {
                trajPoints.add(ALLIANCE_WAYPOINTS_POSE[0]);
                trajPoints.add(ALLIANCE_WAYPOINTS_POSE[1]);
            } else if (endPose.getY() < CHARGE_STATION_LOWER_Y) {
                trajPoints.add(ALLIANCE_WAYPOINTS_POSE[2]);
                trajPoints.add(ALLIANCE_WAYPOINTS_POSE[3]);
            }
            //finish this
            ToPos step2 = new ToPos(driveTrain, currentPose, getCargoLocation());
            currentPose = getCargoLocation();
            fullTrajectory = fullTrajectory.concatenate(step2.getTrajectory());
            command.addCommands(step2);
        } 
        else {
            ToPos step2 = new ToPos(driveTrain, currentPose, getLeaveCommunityPose());
            currentPose = getLeaveCommunityPose();
            fullTrajectory = fullTrajectory.concatenate(step2.getTrajectory());
            command.addCommands(step2);
        }

        // step 3 go for charge
        if (charge) {
            ToChargeCommand step3 = new ToChargeCommand(driveTrain, currentPose, getChargeLocation());
            currentPose = getChargeLocation();
            fullTrajectory = fullTrajectory.concatenate(step3.getTrajectory());
            command.addCommands(step3);
        }

        // if none of these have run something has gone wrong
        // so just leave the community
        if (StartPose.equals(currentPose)) {
            ToPos leave = new ToPos(driveTrain, currentPose, getLeaveCommunityPose());
            currentPose = getLeaveCommunityPose();
            fullTrajectory = fullTrajectory.concatenate(leave.getTrajectory());
            command.addCommands(leave);
        }

    }

    public SequentialCommandGroup getCommand() {
        trajediesDecider();
        return command;
    }

    public Trajectory getTrajectory() {
        return fullTrajectory;
    }

    public void displayField(Field2d m_field) {
        m_field.getObject("traj").setTrajectory(fullTrajectory);
    }
}

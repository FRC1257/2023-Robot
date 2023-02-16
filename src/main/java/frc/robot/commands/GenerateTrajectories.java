package frc.robot.commands;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.Autonomous;
import frc.robot.RobotContainer;

public class GenerateTrajectories {
    private boolean charge;
    private boolean firstScore;
    private boolean secondScore;
    private boolean cargo;
    private Drivetrain driveTrain;
    private Pose2d StartPose;
    
    private SequentialCommandGroup command;
    private Pose2d currentPose; 
    private List<Trajectory> trajectoryList = new ArrayList<Trajectory>();
    private Pose2d[] ALLIANCE_START_POSE;
    private Pose2d[] ALLIANCE_CARGO_POSE;
    private Pose2d[] ALLIANCE_SCORE_POSE;
    private Pose2d[] ALLIANCE_WAYPOINTS_POSE;
    private Pose2d ALLIANCE_CHARGE_POSE_WAYPOINT;
    private Pose2d[] ALLIANCE_LEAVE_COMMUNITY;
    private Pose2d chargePose;

    public GenerateTrajectories(Drivetrain drivetrain, boolean isCharge, boolean isFirstScore, boolean isSecondScore, boolean isCargo, Drivetrain driveTrain, int StartPose) {
        this.charge = isCharge;
        this.firstScore = isFirstScore;
        this.secondScore = isSecondScore;

        this.cargo = isCargo;
        this.driveTrain = driveTrain;
        
        command = new SequentialCommandGroup();
        currentPose = new Pose2d();
        // trajectoryList.add(new Trajectory());
        if (SmartDashboard.getBoolean("isAllianceBlue", false)) {
            ALLIANCE_START_POSE = Autonomous.BLUE_START_POSE;
            ALLIANCE_CARGO_POSE = Autonomous.BLUE_CARGO_POSE;
            ALLIANCE_CHARGE_POSE_WAYPOINT = Autonomous.BLUE_CHARGE_POSE_WAYPOINT;
            ALLIANCE_SCORE_POSE = Autonomous.BLUE_SCORE_POSE;
            ALLIANCE_WAYPOINTS_POSE = Autonomous.BLUE_WAYPOINT_POSE;
            chargePose = Autonomous.BLUE_CHARGE_POSE;
            ALLIANCE_LEAVE_COMMUNITY = Autonomous.BLUE_LEAVE_COMMUNITY_POSE;
        } else {
            ALLIANCE_START_POSE = Autonomous.RED_START_POSE;
            ALLIANCE_CARGO_POSE = Autonomous.RED_CARGO_POSE;
            ALLIANCE_CHARGE_POSE_WAYPOINT = Autonomous.RED_CHARGE_POSE_WAYPOINT;
            ALLIANCE_SCORE_POSE = Autonomous.RED_SCORE_POSE;
            ALLIANCE_WAYPOINTS_POSE = Autonomous.RED_WAYPOINT_POSE;
            chargePose = Autonomous.RED_CHARGE_POSE;
            ALLIANCE_LEAVE_COMMUNITY = Autonomous.RED_LEAVE_COMMUNITY_POSE;
        }
        this.StartPose = ALLIANCE_START_POSE[StartPose]; 
        this.currentPose = this.StartPose;

        trajediesDecider();
    }

    // TODO make method to get positions
    // also TODO make method to get the trajectories to visualize

    //gets the cargoLocation based on what side the robot is on

    public Pose2d getCargoLocation() {
        return ALLIANCE_CARGO_POSE[RobotContainer.gamePieceChooser.getSelected()];
    }

    // 2 getScoreLocation() methods for some reason?
    public Pose2d getFirstScoreLocation() {
        return ALLIANCE_SCORE_POSE[RobotContainer.firstScorePositionChooser.getSelected()];
    }

    public Pose2d getSecondScoreLocation() {
        return ALLIANCE_SCORE_POSE[RobotContainer.secondScorePositionChooser.getSelected()];
    }

    public Pose2d getChargeLocation() {
        // should be stored as a constant then retrieved for this
        // currently returning this random thing

        // in reality there are 2 possible places so we would just need to use the side of the field we are on
        return chargePose;
    }

    public Pose2d getLeaveCommunityPose(Pose2d currentPose) {
        // should be stored as a constant then retrieved for this
        // currently returning this random thing
        // TODO fix this
        // in reality there are 6 possible places so we would just need to use the varialbes we have
        if (currentPose.getY() > Autonomous.CHARGE_CENTER_Y) {
            return ALLIANCE_LEAVE_COMMUNITY[0];
        } 
        return ALLIANCE_LEAVE_COMMUNITY[1];
    }

    public void trajediesDecider() {
        command = new SequentialCommandGroup();
        // there are 3 possible steps we can take
        // Step 1
        if (firstScore) {
            addFirstScoreTrajectory();
        }

        // then we either go for cargo or leave the tarmac to get points
        // Step 2
        if (cargo) {
            // going for cargo implies leaving community
            addCargoTrajectory();
        } 
        else {
            addLeaveCommunityTrajectory();
        }

        // Step 3
        if (secondScore) {
            addSecondScoreTrajectory();
        }
        // step 3 go for charge
        else if (charge) {
            addChargeTrajectory();
        }
        else {
            // TODO: addParkTrajectory()
        }

        // if none of these have run something has gone wrong
        // so just leave the community
        if (StartPose.equals(currentPose)) {
           addLeaveCommunityTrajectory(); 
        }

        trajectoryList.add(getFullTrajectory());

    }

    // step variables aren't random, they actually represent the order of the trajectories
    public void addFirstScoreTrajectory() {
        ToPosCommand step1 = new ToPosCommand(driveTrain, List.of(StartPose, getFirstScoreLocation()), true);
        currentPose = getFirstScoreLocation();
        trajectoryList.add(step1.getTrajectory());
        command.addCommands(step1);
    }

    public void addSecondScoreTrajectory() {
        ToPosCommand step3 = new ToPosCommand(driveTrain, List.of(StartPose, getSecondScoreLocation()), true);
        currentPose = getSecondScoreLocation();
        trajectoryList.add(step3.getTrajectory());
        command.addCommands(step3);
    }

    public void addCargoTrajectory() {
        List<Pose2d> trajPoints = new ArrayList<Pose2d>();
        trajPoints.add(currentPose);

        // going around the charging station, if convenient
        if (currentPose.getY() > Autonomous.CHARGE_CENTER_Y) {
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[0]);
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[1]);
        } else {
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[2]);
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[3]);
        }
        
        trajPoints.add(getCargoLocation());
        ToPosCommand step2 = new ToPosCommand(driveTrain, trajPoints, false);
        currentPose = getCargoLocation();
        trajectoryList.add(step2.getTrajectory());
        command.addCommands(step2);
    }

    public void addLeaveCommunityTrajectory() {
        // this method and addCargoTrajectory are almost identical, different by one line
        // TODO: refactor further to avoid confusion
        List<Pose2d> trajPoints = new ArrayList<Pose2d>();
        trajPoints.add(currentPose);

        // going around the charging station, if convenient
        if (currentPose.getY() > Autonomous.CHARGE_CENTER_Y) {
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[0]);
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[1]);
        } else {
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[2]);
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[3]);
        }
        trajPoints.add(getLeaveCommunityPose(currentPose));
        ToPosCommand step2 = new ToPosCommand(driveTrain, trajPoints, false);
        currentPose = getLeaveCommunityPose(currentPose);
        trajectoryList.add(step2.getTrajectory());
        command.addCommands(step2);
    }

    public void addChargeTrajectory() {
        ToPosCommand step3 = new ToPosCommand(driveTrain, List.of(currentPose, ALLIANCE_CHARGE_POSE_WAYPOINT, getChargeLocation()), false);
        currentPose = getChargeLocation();
        trajectoryList.add(step3.getTrajectory());
        command.addCommands(step3);
    }

    public SequentialCommandGroup getCommand() {
        trajediesDecider();
        return command;
    }

    public List<Trajectory> getTrajectories() {
        return trajectoryList;
    }

    public Trajectory getTrajectory(int index) {
        if (index < trajectoryList.size()) {
            return trajectoryList.get(index);
        } else {
            return null;
        }
    }

    public Trajectory getFullTrajectory() {
        Trajectory fullTrajectory = new Trajectory();
        for (Trajectory traj : trajectoryList) {
            fullTrajectory = fullTrajectory.concatenate(traj);
        }
        return fullTrajectory;
    }
}

package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.Autonomous;
import frc.robot.commands.drivetrain.NoPDBalanceCommand;
import frc.robot.commands.drivetrain.PDBalanceCommand;
import frc.robot.commands.drivetrain.TurnAngleCommand;
import frc.robot.RobotContainer;

public class GenerateTrajectories {
    public enum State {
        NORMAL,
        SHOOTING,
        THREE_PIECE,
        MOVE_FORWARD
    }

    State[] autoType = { State.NORMAL, State.SHOOTING, State.THREE_PIECE, State.MOVE_FORWARD };

    private boolean charge;
    private boolean firstScore;
    private boolean secondScore;
    private boolean cargo;
    private boolean threePiece;
    private boolean blue;
    private boolean leaveTarmac;
    private Drivetrain drivetrain;
    private Pose2d StartPose;
    private boolean hitAndRun;

    private SequentialCommandGroup command;
    private Pose2d currentPose;
    private List<Trajectory> trajectoryList = new ArrayList<Trajectory>();
    private Pose2d[] ALLIANCE_START_POSE;
    private Pose2d[] ALLIANCE_CARGO_POSE;
    private Pose2d[] ALLIANCE_SCORE_POSE;
    private Pose2d[] ALLIANCE_WAYPOINTS_POSE;
    private Pose2d[] ALLIANCE_CHARGE_POSE_WAYPOINT;
    private Pose2d[] ALLIANCE_LEAVE_COMMUNITY;
    private Pose2d[] chargePose;

    public GenerateTrajectories(Drivetrain drivetrain, boolean isCharge, boolean isFirstScore, boolean isSecondScore,
            boolean isCargo, int StartPose, boolean threePiece, boolean leaveTarmac, boolean hitAndRun) {
        this.charge = isCharge;
        this.firstScore = isFirstScore;
        this.secondScore = isSecondScore;
        // this.threePiece = threePiece;
        // this.leaveTarmac = leaveTarmac;
        // this.hitAndRun = hitAndRun;

        this.cargo = isCargo;
        this.drivetrain = drivetrain;

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
            blue = true;
        } else {
            ALLIANCE_START_POSE = Autonomous.RED_START_POSE;
            ALLIANCE_CARGO_POSE = Autonomous.RED_CARGO_POSE;
            ALLIANCE_CHARGE_POSE_WAYPOINT = Autonomous.RED_CHARGE_POSE_WAYPOINT;
            ALLIANCE_SCORE_POSE = Autonomous.RED_SCORE_POSE;
            ALLIANCE_WAYPOINTS_POSE = Autonomous.RED_WAYPOINT_POSE;
            chargePose = Autonomous.RED_CHARGE_POSE;
            ALLIANCE_LEAVE_COMMUNITY = Autonomous.RED_LEAVE_COMMUNITY_POSE;
            blue = false;
        }

        // this.StartPose = ALLIANCE_START_POSE[StartPose];
        this.StartPose = getFirstScoreLocation();
        this.currentPose = this.StartPose;

        AutoDecider();
    }

    // TODO make method to get positions

    // gets the cargoLocation based on what side the robot is on

    // private Pose2d goForward(){

    // trajectoryList.add(getFullTrajectory());
    // }

    private State getAutoType() {
        return autoType[RobotContainer.autoChooser.getSelected()];
    }

    private Pose2d getCargoLocation() {
        return ALLIANCE_CARGO_POSE[RobotContainer.gamePieceChooser.getSelected()];
    }

    private Pose2d getSecondCargoLocation() {
        return ALLIANCE_CARGO_POSE[RobotContainer.secondGamePieceChooser.getSelected()];
    }

    // 2 getScoreLocation() methods for some reason?
    private Pose2d getFirstScoreLocation() {
        return ALLIANCE_SCORE_POSE[RobotContainer.firstScorePositionChooser.getSelected()];
    }

    private Pose2d getSecondScoreLocation() {
        return ALLIANCE_SCORE_POSE[RobotContainer.secondScorePositionChooser.getSelected()];
    }

    private Pose2d getThirdScoreLocation() {
        return ALLIANCE_SCORE_POSE[RobotContainer.thirdScorePositionChooser.getSelected()];
    }

    private Pose2d getChargeLocation() {
        // should be stored as a constant then retrieved for this
        // currently returning this random thing

        // index 0 is the area outside the community zone
        // index 1 is the area inside the community zone
        if (hitAndRun) {
            return chargePose[1];
        }

        // in reality there are 2 possible places so we would just need to use the side
        // of the field we are on
        if (blue && currentPose.getX() > Autonomous.BLUE_COMMUNITY_X) {
            return chargePose[0];
        } else if (blue) {
            return chargePose[1];
        }
        // not blue
        if (currentPose.getX() < Autonomous.RED_COMMUNITY_X) {
            return chargePose[0];
        }
        return chargePose[1];

    }

    private Pose2d getChargeWaypointLocation() {
        // should be stored as a constant then retrieved for this
        // currently returning this random thing

        // index 0 is the area outside the community zone
        // index 1 is the area inside the community zone

        // in reality there are 2 possible places so we would just need to use the side
        // of the field we are on
        if (blue && currentPose.getX() > Autonomous.BLUE_COMMUNITY_X) {
            return ALLIANCE_CHARGE_POSE_WAYPOINT[0];
        } else if (blue) {
            return ALLIANCE_CHARGE_POSE_WAYPOINT[1];
        }
        // not blue
        if (currentPose.getX() < Autonomous.RED_COMMUNITY_X) {
            return ALLIANCE_CHARGE_POSE_WAYPOINT[0];
        }
        return ALLIANCE_CHARGE_POSE_WAYPOINT[1];

    }

    private Pose2d getHitAndRunPose2d() {
        return flipPose(ALLIANCE_CHARGE_POSE_WAYPOINT[0]);
    }

    private Pose2d getLeaveCommunityPose(Pose2d currentPose) {
        // should be stored as a constant then retrieved for this
        // currently returning this random thing
        // TODO fix this
        // in reality there are 6 possible places so we would just need to use the
        // varialbes we have
        // also a jank fix
        if (hitAndRun) {
            return ALLIANCE_LEAVE_COMMUNITY[2];
        }
        if (currentPose.getY() > Autonomous.CHARGE_CENTER_Y) {
            return ALLIANCE_LEAVE_COMMUNITY[0];
        }
        return ALLIANCE_LEAVE_COMMUNITY[1];
    }

    /**
     * Made with full assumption that functions called by AutoDecider work.
     */
    private void AutoDecider() {
        command = new SequentialCommandGroup();

        switch (getAutoType()) {
            case NORMAL:
                normalAuto();
                break;
            case SHOOTING:
                shootingAuto();
                break;
            case THREE_PIECE:
                threePieceAuto();
                break;
            case MOVE_FORWARD:
                moveForward();
                break;
        }

        trajectoryList.add(getFullTrajectory());
    }

    private void normalAuto() {
        // there are 3 possible steps we can take
        // Step 1
        if (firstScore) {
            // addFirstScoreTrajectory();
            addScoreHigh();
        }

        // then we either go for cargo or leave the tarmac to get points
        // Step 2
        if (cargo) {
            // going for cargo implies leaving community
            addCargoTrajectory();
            addPiecePickup();
            turn180();
        } else if (leaveTarmac) {
            if (hitAndRun) {
                addOverChargeTrajectory();
                this.command.addCommands(new NoPDBalanceCommand(drivetrain).withTimeout(8));
                return;
            } else {
                addLeaveCommunityTrajectory();
            }
        }

        // Step 3
        if (secondScore) {
            addSecondScoreTrajectory();
            addScoreHigh();
            if (charge) {
                addChargeTrajectory();
            }
        }
        // step 3 go for charge
        else if (charge) {
            addChargeTrajectory();
            this.command.addCommands(new NoPDBalanceCommand(drivetrain).withTimeout(8));
        }

        // if none of these have run something has gone wrong
        // so just leave the community
        if (StartPose.equals(currentPose)) {
            addLeaveCommunityTrajectory();
        }
    }

    private void shootingAuto() {
        // TODO: make shootingAuto()
    }

    private void threePieceAuto() {
        // command = new SequentialCommandGroup();
        this.StartPose = getFirstScoreLocation();
        this.currentPose = this.StartPose;

        addScoreHigh();

        backUpAndTurn();

        ToPosCommand firstGoToCargo = new ToPosCommand(drivetrain,
                getTrajPointsWaypoint(currentPose, getCargoLocation()), false);
        currentPose = getCargoLocation();
        addToPosCommand(firstGoToCargo);

        addPiecePickup();

        turn180();

        ToPosCommand returnToScore = new ToPosCommand(drivetrain,
                getTrajPointsWaypointReverse(currentPose, getSecondScoreLocation()), false);
        currentPose = getSecondScoreLocation();
        addToPosCommand(returnToScore);

        addScoreHigh();

        backUpAndTurn();

        ToPosCommand secondGoToCargo = new ToPosCommand(drivetrain,
                getTrajPointsWaypoint(currentPose, getSecondCargoLocation()), false);
        currentPose = getSecondCargoLocation();
        addToPosCommand(secondGoToCargo);

        addPiecePickup();

        turn180();

        ToPosCommand returnToScore2 = new ToPosCommand(drivetrain,
                getTrajPointsWaypointReverse(currentPose, getThirdScoreLocation()), false);
        currentPose = getThirdScoreLocation();
        addToPosCommand(returnToScore2);

        addScoreHigh();

        trajectoryList.add(getFullTrajectory());
    }

    /**
     * Adds trajectory to move forward and back.
     * @see #command
     * @see SequentialCommandGroup#addCommands(edu.wpi.first.wpilibj2.command.Command...)
     */
    private void moveForward() {
        // Literally made while queueing for quals during Robbinsville 2023
        List<Pose2d> trajPoints = new ArrayList<Pose2d>();
        trajPoints.add(Autonomous.RED_SCORE_POSE[RobotContainer.firstScorePositionChooser.getSelected()]);
        trajPoints.add(shiftedPose(Autonomous.RED_SCORE_POSE[RobotContainer.firstScorePositionChooser.getSelected()]));

        List<Pose2d> trajPointsBack = new ArrayList<Pose2d>();
        trajPointsBack
                .add(shiftedPose(Autonomous.RED_SCORE_POSE[RobotContainer.firstScorePositionChooser.getSelected()]));
        trajPointsBack.add(Autonomous.RED_SCORE_POSE[RobotContainer.firstScorePositionChooser.getSelected()]);

        command.addCommands(new SequentialCommandGroup(
                new ToPosCommand(drivetrain, trajPoints, true),
                new ToPosCommand(drivetrain, trajPointsBack, false),
                new ToPosCommand(drivetrain, trajPoints, true)));
    }

    // TODO Fix these methods
    private void addScoreHigh() {
        // TODO add this
        // command.addCommands(new ScoreCommand());
    }

    private void addPiecePickup() {
        command.addCommands(new GetPieceCommand());
    }

    private void addToPosCommand(ToPosCommand command) {
        this.command.addCommands(command);
        trajectoryList.add(command.getTrajectory());
    }

    public Pose2d shiftedPose(Pose2d pose) {
        double SHIFT_X = -0.4;
        if (blue) {
            SHIFT_X *= -1;
        }
        return new Pose2d(pose.getX() + SHIFT_X, pose.getY(), pose.getRotation());
    }

    private Pose2d flipPose(Pose2d pose) {
        return new Pose2d(pose.getX(), pose.getY(), pose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    private void backUpAndTurn() {
        ToPosCommand firstTurnAround = new ToPosCommand(drivetrain, List.of(currentPose, shiftedPose(currentPose)),
                true);
        currentPose = shiftedPose(currentPose);
        addToPosCommand(firstTurnAround);

        turn180();
    }

    private void turn180() {
        //
        if (RobotBase.isSimulation()) {
            this.command.addCommands(new Delay(0.5));
        } else {
            this.command.addCommands(new TurnAngleCommand(drivetrain, 180));
        }
        currentPose = flipPose(currentPose);
    }

    // step variables aren't random, they actually represent the order of the
    // trajectories
    private void addFirstScoreTrajectory() {
        ToPosCommand step1 = new ToPosCommand(drivetrain, List.of(StartPose, getFirstScoreLocation()), true);
        currentPose = getFirstScoreLocation();
        addToPosCommand(step1);
    }

    private void addSecondScoreTrajectory() {
        ToPosCommand returnToScore = new ToPosCommand(drivetrain,
                getTrajPointsWaypointReverse(currentPose, getSecondScoreLocation()), false);
        currentPose = getSecondScoreLocation();
        addToPosCommand(returnToScore);
    }

    private List<Pose2d> getTrajPointsWaypoint(Pose2d start, Pose2d end) {
        List<Pose2d> trajPoints = new ArrayList<Pose2d>();
        trajPoints.add(start);

        // going around the charging station, if convenient
        if (currentPose.getY() > Autonomous.CHARGE_CENTER_Y) {
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[0]);
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[1]);
        } else {
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[2]);
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[3]);
        }

        trajPoints.add(end);
        return trajPoints;
    }

    private List<Pose2d> getTrajPointsWaypointReverse(Pose2d start, Pose2d end) {
        List<Pose2d> trajPoints = new ArrayList<Pose2d>();
        trajPoints.add(start);

        // going around the charging station, if convenient
        if (currentPose.getY() > Autonomous.CHARGE_CENTER_Y) {
            trajPoints.add(flipPose(ALLIANCE_WAYPOINTS_POSE[1]));
            trajPoints.add(flipPose(ALLIANCE_WAYPOINTS_POSE[0]));
        } else {
            trajPoints.add(flipPose(ALLIANCE_WAYPOINTS_POSE[3]));
            trajPoints.add(flipPose(ALLIANCE_WAYPOINTS_POSE[2]));
        }

        trajPoints.add(end);
        return trajPoints;
    }

    private void addCargoTrajectory() {
        backUpAndTurn();
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
        ToPosCommand step2 = new ToPosCommand(drivetrain, trajPoints, false);
        currentPose = getCargoLocation();
        addToPosCommand(step2);
    }

    private void addLeaveCommunityTrajectory() {
        // this method and addCargoTrajectory are almost identical, different by one
        // line
        // TODO: refactor further to avoid confusion
        backUpAndTurn();
        List<Pose2d> trajPoints = new ArrayList<Pose2d>();
        trajPoints.add(currentPose);

        // going around the charging station, if convenient
        // jank fix
        if (hitAndRun) {

        } else if (currentPose.getY() > Autonomous.CHARGE_CENTER_Y) {
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[0]);
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[1]);
        } else {
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[2]);
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[3]);
        }
        trajPoints.add(getLeaveCommunityPose(currentPose));
        ToPosCommand step2 = new ToPosCommand(drivetrain, trajPoints, false);
        currentPose = getLeaveCommunityPose(currentPose);
        addToPosCommand(step2);
    }

    private void outThenIn() {

    }

    private void addChargeTrajectory() {
        ToPosCommand step3 = new ToPosCommand(drivetrain,
                List.of(currentPose, getChargeWaypointLocation(), getChargeLocation()), false);
        currentPose = getChargeLocation();
        addToPosCommand(step3);
    }

    private void addOverChargeTrajectory() {
        ToPosCommand step3 = new ToPosCommand(drivetrain,
                List.of(currentPose, getHitAndRunPose2d()), false);
        currentPose = getHitAndRunPose2d();
        addToPosCommand(step3);

        ToPosCommand step2 = new ToPosCommand(drivetrain, List.of(currentPose, chargePose[1]), true);
        currentPose = getChargeLocation();
        addToPosCommand(step2);
    }

    public SequentialCommandGroup getCommand() {
        // AutoDecider();
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

    public double getTrajectoryTime() {
        double time = 0;
        for (int i = 0; i < trajectoryList.size() - 1; i++) {
            time += trajectoryList.get(i).getTotalTimeSeconds();
        }
        return time;
    }

    public int getLastTrajectoryIndex() {
        return trajectoryList.size() - 1;
    }

}

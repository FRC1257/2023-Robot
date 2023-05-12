package frc.robot.commands;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotArm;
import frc.robot.Constants.Autonomous;
import frc.robot.commands.Compound_Commands.*;
import frc.robot.commands.drivetrain.PDBalanceCommand;
import frc.robot.commands.drivetrain.ToPosCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.TurnAngleCommand;

public class GenerateTrajedies {
    public enum State {
        NORMAL,
        SHOOTING,
        THREE_PIECE, 
        MOVE_FORWARD,
        HIT_AND_RUN,
        HIT
    }

    State[] autoType = { State.NORMAL, 
        State.SHOOTING, 
        State.THREE_PIECE, 
        State.MOVE_FORWARD, 
        State.HIT_AND_RUN, 
        State.HIT
    };

    private boolean charge;
    private boolean firstScore;
    private boolean secondScore;
    private boolean cargo;
    private boolean threePiece;
    private boolean blue;
    private boolean leaveTarmac;

    private Drivetrain drivetrain;
    private Elevator elevator;
    private PivotArm pivotArm;
    private Claw claw;
    
    private Pose2d StartPose;
    private boolean hitAndRun;
    
    private SequentialCommandGroup command;
    private Pose2d currentPose; 
    private List<Trajectory> trajectoryList = new ArrayList<Trajectory>();
    private Pose2d[] ALLIANCE_CARGO_POSE;
    private Pose2d[] ALLIANCE_SCORE_POSE;
    private Pose2d[] ALLIANCE_WAYPOINTS_POSE;
    private Pose2d[] ALLIANCE_CHARGE_POSE_WAYPOINT;
    private Pose2d[] ALLIANCE_LEAVE_COMMUNITY;
    private Pose2d[] ALLIANCE_PARK_POSE;
    private Pose2d[] ALLIANCE_SHOOTING_POSE;
    private Pose2d[] chargePose;

    private double pieceApproachAngle;
    private double[][] ApproachAngle = {
        {-90, 90}, // blue
        {-90, 90} // red
    };

    public GenerateTrajedies(Drivetrain drivetrain, boolean isCharge, boolean isFirstScore, boolean isSecondScore, boolean isCargo, int StartPose, boolean threePiece, boolean leaveTarmac, boolean hitAndRun, Elevator elevator, PivotArm pivotArm, Claw claw) {
        this.charge = isCharge;
        this.firstScore = isFirstScore;
        this.secondScore = isSecondScore;
        this.threePiece = threePiece;
        this.leaveTarmac = leaveTarmac;
        this.hitAndRun = hitAndRun;

        this.cargo = isCargo;
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.pivotArm = pivotArm;
        this.claw = claw;
        
        command = new SequentialCommandGroup();
        currentPose = new Pose2d();
        // trajectoryList.add(new Trajectory());
        if (SmartDashboard.getBoolean("isAllianceBlue", false)) {
            ALLIANCE_CARGO_POSE = Autonomous.BLUE_CARGO_POSE;
            ALLIANCE_CHARGE_POSE_WAYPOINT = Autonomous.BLUE_CHARGE_POSE_WAYPOINT;
            ALLIANCE_SCORE_POSE = Autonomous.BLUE_SCORE_POSE;
            ALLIANCE_WAYPOINTS_POSE = Autonomous.BLUE_WAYPOINT_POSE;
            chargePose = Autonomous.BLUE_CHARGE_POSE;
            ALLIANCE_LEAVE_COMMUNITY = Autonomous.BLUE_LEAVE_COMMUNITY_POSE;
            ALLIANCE_SHOOTING_POSE = Autonomous.BLUE_SHOOT_POSE;
            blue = true;
        } else {
            ALLIANCE_CARGO_POSE = Autonomous.RED_CARGO_POSE;
            ALLIANCE_CHARGE_POSE_WAYPOINT = Autonomous.RED_CHARGE_POSE_WAYPOINT;
            ALLIANCE_SCORE_POSE = Autonomous.RED_SCORE_POSE;
            ALLIANCE_WAYPOINTS_POSE = Autonomous.RED_WAYPOINT_POSE;
            chargePose = Autonomous.RED_CHARGE_POSE;
            ALLIANCE_LEAVE_COMMUNITY = Autonomous.RED_LEAVE_COMMUNITY_POSE;
            ALLIANCE_SHOOTING_POSE = Autonomous.RED_SHOOT_POSE;
            blue = false;
        }

        this.StartPose = getFirstScoreLocation();
        this.currentPose = this.StartPose;

        AutoDecider();
    }

    private State getAutoType() {
        return autoType[RobotContainer.autoChooser.getSelected()];
    }

    // TODO make method to get positions

    //gets the cargoLocation based on what side the robot is on

    
    private Pose2d getCargoLocation() {
        return ALLIANCE_CARGO_POSE[RobotContainer.gamePieceChooser.getSelected()];
    }

    private Pose2d getSecondCargoLocation() {
        return ALLIANCE_CARGO_POSE[RobotContainer.secondGamePieceChooser.getSelected()];
    }

    private Pose2d getThirdCargoLocation() {
        return ALLIANCE_CARGO_POSE[RobotContainer.thirdGamePieceChooser.getSelected()];
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
        if(hitAndRun){
            return chargePose[1];
        } 

        // in reality there are 2 possible places so we would just need to use the side of the field we are on
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

        // in reality there are 2 possible places so we would just need to use the side of the field we are on
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

    private Pose2d getLeaveCommunityPose(Pose2d currentPose) {
        // should be stored as a constant then retrieved for this
        // currently returning this random thing
        // TODO fix this
        // in reality there are 6 possible places so we would just need to use the varialbes we have
        // also a jank fix
        if (hitAndRun) {
            return ALLIANCE_LEAVE_COMMUNITY[2];
        }
        if (currentPose.getY() > Autonomous.CHARGE_CENTER_Y) {
            return ALLIANCE_LEAVE_COMMUNITY[0];
        } 
        return ALLIANCE_LEAVE_COMMUNITY[1];
    }

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
                moveForwardAuto();
                break; 
            case HIT_AND_RUN:
                hitAndRunAuto();
                break;
            case HIT:
                hitAuto();
                break; 
        }

        trajectoryList.add(getFullTrajectory());

    }

    public Pose2d shiftedPose(Pose2d pose) {
        // make a new pose with the x shifted 40 cm
        // always closer to auto line
        double SHIFT_X = blue ? 0.4 : -0.4 ;
        return new Pose2d(pose.getX() + SHIFT_X, pose.getY(), pose.getRotation());
    }

    public Pose2d driveOutPose(Pose2d pose) {
        double SHIFT_X = blue ? 4 : -4;
        return new Pose2d(pose.getX() + SHIFT_X, pose.getY(), pose.getRotation());
    }

    private void backUpAndTurn() {
        ToPosCommand firstTurnAround = new ToPosCommand(drivetrain, List.of(currentPose, shiftedPose(currentPose)),
                true);
        currentPose = shiftedPose(currentPose);
        addToPosCommand(firstTurnAround);

        turn180();
    }

    private Pose2d flipPose(Pose2d pose) {
        return new Pose2d(pose.getX(), pose.getY(), pose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    private void turn180() {
        if (RobotBase.isSimulation()) {
            this.command.addCommands(new Delay(0.5));
        } else {
            this.command.addCommands(new TurnAngleCommand(drivetrain, 180));
        }
        currentPose = flipPose(currentPose);
    }

    private void hitAndRunAuto() {
        currentPose = flipPose(StartPose);
        if (firstScore) {
            // addFirstScoreTrajectory();
            addScore(RobotContainer.firstScorePositionChooser.getSelected());
        }

        addOverChargeTrajectory();

        command.addCommands(new PDBalanceCommand(drivetrain, false));
    }

    private void hitAuto() {
        if (firstScore) {
            // addFirstScoreTrajectory();
            addScore(RobotContainer.firstScorePositionChooser.getSelected());
        }

        ToPosCommand step3 = new ToPosCommand(drivetrain,
                List.of(currentPose, chargePose[0]), true);
        currentPose = chargePose[0];
        addToPosCommand(step3);

        command.addCommands(new PDBalanceCommand(drivetrain, false));
    }

    /**
     * Adds trajectory to move forward and back.
     * @see #command
     * @see SequentialCommandGroup#addCommands(edu.wpi.first.wpilibj2.command.Command...)
     */
    private void moveForwardAuto() {
        if (firstScore) {
            addScore(RobotContainer.firstScorePositionChooser.getSelected());
        }

        List<Pose2d> trajPoints = new ArrayList<Pose2d>();
        trajPoints.add(getFirstScoreLocation());
        trajPoints.add(driveOutPose(getFirstScoreLocation()));

        addToPosCommand(new ToPosCommand(drivetrain, trajPoints, true));
        // addToPosCommand(new ToPosCommand(drivetrain, trajPointsBack, false));
        // addToPosCommand(new ToPosCommand(drivetrain, trajPoints, true));
        turn180();
    }

    private void addOverChargeTrajectory() {
        ToPosCommand step3 = new ToPosCommand(drivetrain,
                List.of(currentPose, getHitAndRunPose2d()), true);
        currentPose = getHitAndRunPose2d();
        addToPosCommand(step3);

        ToPosCommand step2 = new ToPosCommand(drivetrain, List.of(currentPose, chargePose[0]), false);
        currentPose = chargePose[0];
        addToPosCommand(step2);
    }

    private Pose2d getHitAndRunPose2d() {
        return ALLIANCE_CHARGE_POSE_WAYPOINT[0];
    }

    private void normalAuto() {
        // there are 3 possible steps we can take
        // Step 1
        currentPose = flipPose(StartPose);
        if (firstScore) {
            // addFirstScoreTrajectory();
            addScore(RobotContainer.firstScorePositionChooser.getSelected());
        }

        backUpAndTurn();

        // then we either go for cargo or leave the tarmac to get points
        // Step 2
        if (cargo) {
            // going for cargo implies leaving community
            addCargoTrajectory();
            addPiecePickup();
            turn180();
        } 
        else if (leaveTarmac) {
            addLeaveCommunityTrajectory();
            if (hitAndRun) {
                addChargeTrajectory();
                return;
            }
        }

        // Step 3
        if (secondScore) {
            addSecondScoreTrajectory();
            addScore(RobotContainer.secondScorePositionChooser.getSelected());
            if (charge) {
                addChargeTrajectory();
            }
        }
        // step 3 go for charge
        else if (charge) {
            addChargeTrajectory();
        }

        // if none of these have run something has gone wrong
        // so just leave the community
        if (StartPose.equals(currentPose)) {
           addLeaveCommunityTrajectory(); 
        }
    }

    private void shootingAuto() {
        this.StartPose = flipPose(getFirstScoreLocation());
        this.currentPose = this.StartPose;

        addScore(RobotContainer.firstScorePositionChooser.getSelected());

        backUpAndTurn();

        ToPosCommand firstGoToCargo = new ToPosCommand(drivetrain,
                getTrajPointsWaypoint(currentPose, getCargoLocation()), false);
        currentPose = getCargoLocation();
        addToPosCommand(firstGoToCargo);

        addPiecePickup();

        turn180();

        driveToShootingPose();
        shootPiece();

        turn180();

        ToPosCommand secondGoToCargo = new ToPosCommand(drivetrain,
                List.of(currentPose, niceAngle(getSecondCargoLocation())), false);
        currentPose = niceAngle(getSecondCargoLocation());
        addToPosCommand(secondGoToCargo);

        addPiecePickup();

        turn180();

        driveToShootingPose();
        shootPiece();

        turn180();

        ToPosCommand thirdGoToCargo = new ToPosCommand(drivetrain,
                List.of(currentPose, niceAngle(getThirdCargoLocation())), false);
        currentPose = niceAngle(getThirdCargoLocation());
        addToPosCommand(thirdGoToCargo);

        addPiecePickup();

        turn180();

        driveToShootingPose();
        shootPiece();
    }

    public void driveToShootingPose() {
        ToPosCommand driveToShootingPose = new ToPosCommand(drivetrain,
                List.of(currentPose, getShootingPose()), false);
        currentPose = getShootingPose();
        addToPosCommand(driveToShootingPose);
    }

    private Pose2d niceAngle(Pose2d pose) {
        if (currentPose.getY() > Autonomous.CHARGE_CENTER_Y) {
            if (blue) {
                pieceApproachAngle = ApproachAngle[0][0];
            } else {
                pieceApproachAngle = ApproachAngle[1][0];
            }
        } else {
            if (blue) {
                pieceApproachAngle = ApproachAngle[0][1];
            } else {
                pieceApproachAngle = ApproachAngle[1][1];
            }
        }

        return new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(pieceApproachAngle));
    }

    public Pose2d getShootingPose() {
        // TODO check if this is correct
        if (StartPose.getY() > Autonomous.CHARGE_CENTER_Y) {
            return ALLIANCE_SHOOTING_POSE[0];
        }
        return ALLIANCE_SHOOTING_POSE[1];
    }

    public void shootPiece() {
        // TODO add shooting
        this.command.addCommands(new Delay(1));
    }

    private void addScore(int index) {
        if (getConeOrCube(index)) {
            command.addCommands(new ScoreConeCommand(elevator, pivotArm, claw));
        } else {
            command.addCommands(new ScoreCubeCommand(elevator, pivotArm, claw));
        }
        command.addCommands(new HoldCommand(elevator, pivotArm));
    }

    private void addPiecePickup() {
        command.addCommands(new GetPieceCommand());
    }

    private void addToPosCommand(ToPosCommand command) {
        this.command.addCommands(command);
        trajectoryList.add(command.getTrajectory());
    }

    private boolean getConeOrCube(int index) {
        //used to decide if we are coning or cubing
        // true = cone
        // false = cube
        boolean[] coneOrCube = {true, false, true, true, false, true, true, false, true};
        try {
            return coneOrCube[index];
        } catch (IndexOutOfBoundsException e) {
            return false;
        }
        
    }

    private void threePieceAuto() {
        this.StartPose = getFirstScoreLocation();
        this.currentPose = this.StartPose;

        addScore(RobotContainer.firstScorePositionChooser.getSelected());
        
        ToPosCommand firstGoToCargo = new ToPosCommand(drivetrain, getTrajPointsWaypoint(currentPose, getCargoLocation()), false);
        currentPose = getCargoLocation();
        addToPosCommand(firstGoToCargo);

        addPiecePickup();

        addSecondScoreTrajectory();

        addScore(RobotContainer.secondScorePositionChooser.getSelected());

        ToPosCommand secondGoToCargo = new ToPosCommand(drivetrain, getTrajPointsWaypoint(currentPose, getSecondCargoLocation()), false);
        currentPose = getSecondCargoLocation();
        addToPosCommand(secondGoToCargo);

        addPiecePickup();

        ToPosCommand returnToScore2 = new ToPosCommand(drivetrain, getTrajPointsWaypointReverse(currentPose, getThirdScoreLocation()), true);
        currentPose = getThirdScoreLocation();
        addToPosCommand(returnToScore2);

        addScore(RobotContainer.thirdScorePositionChooser.getSelected());
    }

    // step variables aren't random, they actually represent the order of the trajectories
    private void addSecondScoreTrajectory() {
        List<Pose2d> trajPoints = getTrajPointsWaypointForwardBack(currentPose, flipPose(getSecondScoreLocation()));
        ToPosCommand returnToScore = new ToPosCommand(drivetrain, trajPoints, false);
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
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[1]);
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[0]);
        } else {
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[3]);
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[2]);
        }
        
        trajPoints.add(end);
        return trajPoints;
    }

    private List<Pose2d> getTrajPointsWaypointForwardBack(Pose2d start, Pose2d end) {
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

    private void addMobilityTrajectory() {
        List<Pose2d> trajPoints = new ArrayList<Pose2d>();
        trajPoints.add(currentPose);
        trajPoints.add(ALLIANCE_CHARGE_POSE_WAYPOINT[0]);

        ToPosCommand MobilityBonusStep = new ToPosCommand(drivetrain, trajPoints, false);
        currentPose = ALLIANCE_CHARGE_POSE_WAYPOINT[0];
        addToPosCommand(MobilityBonusStep);
    }

    private void addReverseMobilityChargeTrajectory() {
        List<Pose2d> trajPoints = new ArrayList<Pose2d>();
        trajPoints.add(currentPose);
        trajPoints.add(getCargoLocation());

        ToPosCommand MobilityReverseStep = new ToPosCommand(drivetrain, trajPoints, true);
    }

    private void addCargoTrajectory() {
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
        // this method and addCargoTrajectory are almost identical, different by one line
        // TODO: refactor further to avoid confusion
        List<Pose2d> trajPoints = new ArrayList<Pose2d>();
        trajPoints.add(currentPose);

      

        // going around the charging station, if convenient
        // jank fix
       if (hitAndRun) {
     
        }
        else if (currentPose.getY() > Autonomous.CHARGE_CENTER_Y) {
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[0]);
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[1]);
        } 
        else {
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[2]);
            trajPoints.add(ALLIANCE_WAYPOINTS_POSE[3]);
        }
        trajPoints.add(getLeaveCommunityPose(currentPose));
        ToPosCommand step2 = new ToPosCommand(drivetrain, trajPoints, false);
        currentPose = getLeaveCommunityPose(currentPose);
        addToPosCommand(step2);
    }

    private void addChargeTrajectory() {
        if(!hitAndRun){
        ToPosCommand step3 = new ToPosCommand(drivetrain, List.of(currentPose, getChargeWaypointLocation(), getChargeLocation()), false);
        currentPose = getChargeLocation();
        addToPosCommand(step3);
        } else {
        ToPosCommand step3 = new ToPosCommand(drivetrain, List.of(currentPose, getChargeLocation()), true);
        currentPose = getChargeLocation();
        addToPosCommand(step3);
        }
    }

    // Helper Methods to use outside of this class
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
            return getFullTrajectory();
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
        for (int i =0; i < trajectoryList.size() - 1; i++) {
            time += trajectoryList.get(i).getTotalTimeSeconds();
        }
        return time;
    }

    public int getLastTrajectoryIndex() {
        return trajectoryList.size() - 1;
    }

}

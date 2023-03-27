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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.Constants.Autonomous;
import frc.robot.commands.Compound_Commands.HighScoreMoveCommand;
import frc.robot.commands.Compound_Commands.HoldCommand;
import frc.robot.commands.Compound_Commands.IntakeCommand;
import frc.robot.commands.Compound_Commands.LowScoreShootingCommand;
import frc.robot.commands.Compound_Commands.MidScoreMoveCommand;
import frc.robot.commands.Compound_Commands.MidScoreShootingCommand;
import frc.robot.commands.IntakeArm.IntakeArmPIDCommand;
import frc.robot.commands.claw.ClawOpenCommand;
import frc.robot.commands.drivetrain.NoPDBalanceCommand;
import frc.robot.commands.drivetrain.PDBalanceCommand;
import frc.robot.commands.drivetrain.TurnAngleCommand;
import frc.robot.RobotContainer;

import static frc.robot.Constants.PivotArm.PIVOT_ARM_SETPOINT_MID;

public class GenerateTrajectories {
    public enum State {
        NORMAL,
        SHOOTING,
        THREE_PIECE,
        MOVE_FORWARD,
        HIT_AND_RUN,
        NORMAL_NO_TURN
    }

    State[] autoType = { State.NORMAL, State.SHOOTING, State.THREE_PIECE, State.MOVE_FORWARD, State.HIT_AND_RUN, State.NORMAL_NO_TURN };

    private boolean charge;
    private boolean firstScore;
    private boolean secondScore;
    private boolean cargo;
    private boolean blue;
    private boolean leaveTarmac;
    private Drivetrain drivetrain;
    private Pose2d StartPose;

    private SequentialCommandGroup command;
    private Pose2d currentPose;
    private List<Trajectory> trajectoryList = new ArrayList<Trajectory>();
    private Pose2d[] ALLIANCE_CARGO_POSE;
    private Pose2d[] ALLIANCE_SCORE_POSE;
    private Pose2d[] ALLIANCE_WAYPOINTS_POSE;
    private Pose2d[] ALLIANCE_CHARGE_POSE_WAYPOINT;
    private Pose2d[] ALLIANCE_LEAVE_COMMUNITY;
    private Pose2d[] ALLIANCE_SHOOTING_POSE;
    private Pose2d[] chargePose;
    private Pose2d[] kTURNPose;

    private double pieceApproachAngle;
    private double[][] ApproachAngle = {
        {-90, 90}, // blue
        {-90, 90} // red
    };

    private Elevator elevator;
    private PivotArm pivotArm;
    private Intake intake;
    private IntakeArm intakeArm;
    private Claw claw;

    public GenerateTrajectories(Drivetrain drivetrain, boolean isCharge, boolean isFirstScore, boolean isSecondScore,
            boolean isCargo, int StartPose, boolean leaveTarmac, Elevator elevator, PivotArm pivotArm, Intake intake, IntakeArm intakearm, Claw claw) {
        this.charge = isCharge;
        this.firstScore = isFirstScore;
        this.secondScore = isSecondScore;
        this.leaveTarmac = leaveTarmac;

        this.cargo = isCargo;
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.pivotArm = pivotArm;
        this.intake = intake;
        this.intakeArm = intakearm;
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
        return ALLIANCE_CHARGE_POSE_WAYPOINT[0];
    }

    private Pose2d getLeaveCommunityPose(Pose2d currentPose) {
        // should be stored as a constant then retrieved for this
        // currently returning this random thing
        // TODO fix this
        // in reality there are 6 possible places so we would just need to use the
        // varialbes we have
        // also a jank fix
        /* if (hitAndRun) {
            return ALLIANCE_LEAVE_COMMUNITY[2];
        } */
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
            case HIT_AND_RUN:
                hitAndRunAuto();
                break;
            case NORMAL_NO_TURN:
                normalNoTurnAuto();
                break;
        }

        trajectoryList.add(getFullTrajectory());
    }

    private void hitAndRunAuto() {
        if (firstScore) {
            // addFirstScoreTrajectory();
            addScoreHigh();
        }

        addOverChargeTrajectory();

        command.addCommands(new NoPDBalanceCommand(drivetrain));
    }

    private void normalNoTurnAuto() {
        // there are 3 possible steps we can take
        // Step 1
        if (firstScore) {
            // addFirstScoreTrajectory();
            addScore(RobotContainer.firstScoreLevelChooser.getSelected());
        }

        // then we either go for cargo or leave the tarmac to get points
        // Step 2
        if (cargo) {
            // going for cargo implies leaving community
            addBackToCargo(); // TODO add this
            addPiecePickup();
        } else if (leaveTarmac) {
            addLeaveCommunityTrajectory();
            /* if (hitAndRun) {
                addOverChargeTrajectory();
                this.command.addCommands(new NoPDBalanceCommand(drivetrain).withTimeout(8));
                return;
            } else {
                addLeaveCommunityTrajectory();
            } */
        }

        // Step 3
        if (secondScore) {
            addSecondScoreTrajectory();
            addScore(RobotContainer.secondScoreLevelChooser.getSelected());
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

    private void addScore(int scorePosition) {
        switch (scorePosition) {
            case 0:
                // score with the intake
                addScoreLow();
                break;
            case 1:
                // score with arm and elevator
                addScoreMid();
                break;
            case 2:
                // score with arm and elevator
                addScoreHigh();
                break;
            case 3:
                // shoot
                addScoreMidShoot();
                break;
            case 4:
                addScoreHighShoot();
                break;
            default:
                addScoreLow();
                break;
        }
        addHold();
    }

    private Command returnScoreMove(int scorePosition) {
        switch (scorePosition) {
            case 0:
                // get into intaking position for intake arm and intake
                return new IntakeCommand(intake, intakeArm);
            case 1:
                // move elevator and pivot arm to mid score position
                return new MidScoreMoveCommand(elevator, pivotArm);
            case 2:
                // move elevator and pivot arm to high score position
                return new HighScoreMoveCommand(elevator, pivotArm);
            case 3:
                // move intake arm to mid score position
                // prepare to shoot
                return new IntakeArmPIDCommand(intakeArm, PIVOT_ARM_SETPOINT_MID);
            case 4:
                // move intake arm to high score position
                // prepare to shoot
                return new IntakeArmPIDCommand(intakeArm, PIVOT_ARM_SETPOINT_MID);
            default:
                // go to starting position for the bot
                return new HoldCommand(elevator, pivotArm, intakeArm, intake);
        }
    }

    private void normalAuto() {
        // there are 3 possible steps we can take
        // Step 1
        if (firstScore) {
            // addFirstScoreTrajectory();
            addScore(RobotContainer.firstScoreLevelChooser.getSelected());;
        }

        // then we either go for cargo or leave the tarmac to get points
        // Step 2
        if (cargo) {
            // going for cargo implies leaving community
            addCargoTrajectory();
            addPiecePickup();
            turn180();
        } else if (leaveTarmac) {
            addLeaveCommunityTrajectory();
            /* if (hitAndRun) {
                addOverChargeTrajectory();
                this.command.addCommands(new NoPDBalanceCommand(drivetrain).withTimeout(8));
                return;
            } else {
                addLeaveCommunityTrajectory();
            } */
        }

        // Step 3
        if (secondScore) {
            addSecondScoreTrajectory();
            addScore(RobotContainer.secondScoreLevelChooser.getSelected());
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

    private void shootingAuto() {
        this.StartPose = getFirstScoreLocation();
        this.currentPose = this.StartPose;

        addScore(RobotContainer.firstScoreLevelChooser.getSelected());

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

    public void shootPiece() {
        // TODO add shooting
        this.command.addCommands(new Delay(1));
    }

    public Pose2d getShootingPose() {
        // TODO check if this is correct
        if (StartPose.getY() > Autonomous.CHARGE_CENTER_Y) {
            return ALLIANCE_SHOOTING_POSE[0];
        }
        return ALLIANCE_SHOOTING_POSE[1];
    }

    public void driveToShootingPose() {
        ToPosCommand driveToShootingPose = new ToPosCommand(drivetrain,
                List.of(currentPose, getShootingPose()), false);
        currentPose = getShootingPose();
        addToPosCommand(driveToShootingPose);
    }

    private void threePieceAuto() {
        // command = new SequentialCommandGroup();
        this.StartPose = getFirstScoreLocation();
        this.currentPose = this.StartPose;

        addScore(RobotContainer.firstScoreLevelChooser.getSelected());

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

        addScore(RobotContainer.secondScoreLevelChooser.getSelected());

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

        addScore(RobotContainer.thirdScoreLevelChooser.getSelected());

        trajectoryList.add(getFullTrajectory());
    }

    /**
     * Adds trajectory to move forward and back.
     * @see #command
     * @see SequentialCommandGroup#addCommands(edu.wpi.first.wpilibj2.command.Command...)
     */
    private void moveForward() {
        // Literally made while queueing for quals during Robbinsville 2023
        // TODO Redo this so proper Pose2d is used
        List<Pose2d> trajPoints = new ArrayList<Pose2d>();
        trajPoints.add(ALLIANCE_SCORE_POSE[RobotContainer.firstScorePositionChooser.getSelected()]);
        trajPoints.add(driveOutPose(ALLIANCE_SCORE_POSE[RobotContainer.firstScorePositionChooser.getSelected()]));

        List<Pose2d> trajPointsBack = new ArrayList<Pose2d>();
        trajPointsBack
                .add(driveOutPose(ALLIANCE_SCORE_POSE[RobotContainer.firstScorePositionChooser.getSelected()]));
        trajPointsBack.add(ALLIANCE_SCORE_POSE[RobotContainer.firstScorePositionChooser.getSelected()]);

        command.addCommands(
            new SequentialCommandGroup(
                new ToPosCommand(drivetrain, trajPoints, true),
                new ToPosCommand(drivetrain, trajPointsBack, false),
                new ToPosCommand(drivetrain, trajPoints, true)
            )
        );
    }

    // TODO Fix these methods
    private void addScoreHigh() {
        // TODO add this
        // command.addCommands(new ScoreCommand());
        command.addCommands(
            new SequentialCommandGroup(
                new HighScoreMoveCommand(elevator, pivotArm),
                new Delay(0.5),
                new ClawOpenCommand(claw)
        ));
    }

    private void addScoreLow() {
        // TODO add this
        // command.addCommands(new ScoreCommand());
        command.addCommands(
            new LowScoreShootingCommand(intake, intakeArm)
        );
        
    }

    private void addHold() {
        // TODO add this
        command.addCommands(new HoldCommand(elevator, pivotArm, intakeArm, intake));
    }

    private void addScoreMid() {
        command.addCommands(
            new SequentialCommandGroup(
                new MidScoreMoveCommand(elevator, pivotArm),
                new Delay(0.5),
                new ClawOpenCommand(claw)
        ));
    }

    private void addScoreMidShoot() {
        command.addCommands(
            new MidScoreShootingCommand(intakeArm, intake)
        );
    }

    private void addScoreHighShoot() {
        command.addCommands(
            new MidScoreShootingCommand(intakeArm, intake)
        );
        
    }

    private void addPiecePickup() {
        command.addCommands(new IntakeCommand(intake, intakeArm));
    }

    private void addToPosCommand(ToPosCommand command, Command... commands) {
        if (commands.length == 0) {
            this.command.addCommands(command);
        } else {
            this.command.addCommands(new ParallelCommandGroup(
                commands[0],
                command
            ));
        }
        
        trajectoryList.add(command.getTrajectory());
    }

    public Pose2d shiftedPose(Pose2d pose) {
        double SHIFT_X = -0.4;
        if (blue) {
            SHIFT_X *= -1;
        }
        return new Pose2d(pose.getX() + SHIFT_X, pose.getY(), pose.getRotation());
    }

    public Pose2d driveOutPose(Pose2d pose) {
        double SHIFT_X = -10;
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
        addToPosCommand(returnToScore, returnScoreMove(RobotContainer.secondScoreLevelChooser.getSelected()));
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
        addToPosCommand(step2, returnScoreMove(0));
    }

    private void addBackToCargo() {
        List<Pose2d> trajPoints = new ArrayList<Pose2d>();
        trajPoints.add(currentPose);

        // going around the charging station, if convenient
        if (currentPose.getY() > Autonomous.CHARGE_CENTER_Y) {
            trajPoints.add(flipPose(ALLIANCE_WAYPOINTS_POSE[0]));
            trajPoints.add(flipPose(ALLIANCE_WAYPOINTS_POSE[1]));
        } else {
            trajPoints.add(flipPose(ALLIANCE_WAYPOINTS_POSE[2]));
            trajPoints.add(flipPose(ALLIANCE_WAYPOINTS_POSE[3]));
        }

        // trajPoints.add(getCargoLocation());
        ToPosCommand step2 = new ToPosCommand(drivetrain, trajPoints, true);
        currentPose = getCargoLocation();
        addToPosCommand(step2, new IntakeCommand(intake, intakeArm));
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
        if (currentPose.getY() > Autonomous.CHARGE_CENTER_Y) {
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

    private void addChargeTrajectory() {
        ToPosCommand step3 = new ToPosCommand(drivetrain,
                List.of(currentPose, getChargeWaypointLocation(), getChargeLocation()), false);
        currentPose = getChargeLocation();
        addToPosCommand(step3);
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

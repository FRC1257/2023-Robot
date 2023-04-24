package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotArm;
import frc.robot.Constants.Autonomous;
import frc.robot.commands.Compound_Commands.HoldCommand;
import frc.robot.commands.Compound_Commands.ScoreConeCommand;
import frc.robot.commands.Compound_Commands.ScoreCubeCommand;
import frc.robot.commands.drivetrain.BrakeCommand;

import frc.robot.commands.drivetrain.DriveTimeCommand;
import frc.robot.commands.drivetrain.PDBalanceCommand;
import frc.robot.commands.drivetrain.ResetDriveCommand;
import frc.robot.commands.drivetrain.TurnAngleCommand;
import frc.robot.RobotContainer;


public class GenerateTrajectories {
    public enum State {
        NORMAL,/* 
        SHOOTING,
        THREE_PIECE, */
        MOVE_FORWARD,
        HIT_AND_RUN,
        HIT,
        MOVE_FORWARD_PID
    }

    State[] autoType = { State.NORMAL/* , State.SHOOTING, State.THREE_PIECE */, 
        State.MOVE_FORWARD, 
        State.HIT_AND_RUN, 
        State.HIT, 
        State.MOVE_FORWARD_PID 
    };

    private boolean firstScore;
    private boolean blue;
    private Drivetrain drivetrain;
    private Pose2d StartPose;
    private Elevator elevator;
    private PivotArm pivotArm;
    private Claw claw;

    private SequentialCommandGroup command;
    private Pose2d currentPose;
    private List<Trajectory> trajectoryList = new ArrayList<Trajectory>();
    private Pose2d[] ALLIANCE_CARGO_POSE;
    private Pose2d[] ALLIANCE_SCORE_POSE;
    private Pose2d[] ALLIANCE_WAYPOINTS_POSE;
    private Pose2d[] ALLIANCE_CHARGE_POSE_WAYPOINT;
    private Pose2d[] ALLIANCE_LEAVE_COMMUNITY;
    private Pose2d[] chargePose;


    public GenerateTrajectories(Drivetrain drivetrain, Elevator elevator, PivotArm pivotArm, Claw claw, boolean isCharge, boolean goToCyclePose, boolean firstScore) {
        this.firstScore = firstScore;
        this.elevator = elevator;
        this.pivotArm = pivotArm;
        this.claw = claw;
        this.drivetrain = drivetrain;

        command = new SequentialCommandGroup();
        command.addCommands(new ResetDriveCommand(drivetrain));
        currentPose = new Pose2d();
        // trajectoryList.add(new Trajectory());
        if (SmartDashboard.getBoolean("isAllianceBlue", false)) {
            ALLIANCE_CARGO_POSE = Autonomous.BLUE_CARGO_POSE;
            ALLIANCE_CHARGE_POSE_WAYPOINT = Autonomous.BLUE_CHARGE_POSE_WAYPOINT;
            ALLIANCE_SCORE_POSE = Autonomous.BLUE_SCORE_POSE;
            ALLIANCE_WAYPOINTS_POSE = Autonomous.BLUE_WAYPOINT_POSE;
            chargePose = Autonomous.BLUE_CHARGE_POSE;
            ALLIANCE_LEAVE_COMMUNITY = Autonomous.BLUE_LEAVE_COMMUNITY_POSE;
            blue = true;
        } else {
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
        command.addCommands(new BrakeCommand(drivetrain));
    }

    // TODO make method to get positions

    private State getAutoType() {
        return autoType[RobotContainer.autoChooser.getSelected()];
    }

    private Pose2d getCargoLocation() {
        return ALLIANCE_CARGO_POSE[RobotContainer.gamePieceChooser.getSelected()];
    }

    // 2 getScoreLocation() methods for some reason?
    private Pose2d getFirstScoreLocation() {
        return ALLIANCE_SCORE_POSE[RobotContainer.firstScorePositionChooser.getSelected()];
    }

    private boolean getConeOrCube() {
        //used to decide if we are coning or cubing
        // true = cone
        // false = cube
        switch (RobotContainer.firstScorePositionChooser.getSelected()) {
            case 0:
                return true;
            case 1:
                return false;
            case 2:
                return true;
            case 3:
                return true;
            case 4:
                return false;
            case 5:
                return true;
            case 6:
                return true;
            case 7:
                return false;
            case 8:
                return true;
        }
        return true;
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
            case MOVE_FORWARD:
                moveForward();
                break;
            case HIT_AND_RUN:
                hitAndRunAuto();
                break;
            case HIT:
                hitAuto();
                break;
            case MOVE_FORWARD_PID:
                moveForwardPID();
                break;
        }

        trajectoryList.add(getFullTrajectory());
    }

    private void hitAndRunAuto() {
        
        if (firstScore) {
            // addFirstScoreTrajectory();
            if (getConeOrCube()) {
                command.addCommands(new ScoreConeCommand(elevator, pivotArm, claw));
            } else {
                command.addCommands(new ScoreCubeCommand(elevator, pivotArm, claw));
            }
            command.addCommands(new HoldCommand(elevator, pivotArm));
        }

        addOverChargeTrajectory();

        command.addCommands(new PDBalanceCommand(drivetrain, false));
    }

    private void hitAuto() {
        
        if (firstScore) {
            // addFirstScoreTrajectory();
            if (getConeOrCube()) {
                command.addCommands(new ScoreConeCommand(elevator, pivotArm, claw));
            } else {
                command.addCommands(new ScoreCubeCommand(elevator, pivotArm, claw));
            }
            command.addCommands(new HoldCommand(elevator, pivotArm));
        }

        ToPosCommand step2 = new ToPosCommand(drivetrain, List.of(currentPose, chargePose[0]), true);
        currentPose = chargePose[0];
        addToPosCommand(step2);

        command.addCommands(new PDBalanceCommand(drivetrain, false));
    }

    private void normalAuto() {
        if (firstScore) {
            if (getConeOrCube()) {
                addScoreMidCone();
            } else {
                addScoreMidCube();
            }
        }

        // if (charge) {
        //     addChargeTrajectory();
        //     this.command.addCommands(new PDBalanceCommand(drivetrain, true));
        // } else if (goToCyclePose) {
        //     addCycleTrajectory();
        //     return;
        // }

        // if none of these have run something has gone wrong
        // so just leave the community
        if (StartPose.equals(currentPose)) {
            addLeaveCommunityTrajectory();
        }

        
        turn180();
        
    }

    // private Pose2d niceAngle(Pose2d pose) {
    //     if (currentPose.getY() > Autonomous.CHARGE_CENTER_Y) {
    //         if (blue) {
    //             pieceApproachAngle = ApproachAngle[0][0];
    //         } else {
    //             pieceApproachAngle = ApproachAngle[1][0];
    //         }
    //     } else {
    //         if (blue) {
    //             pieceApproachAngle = ApproachAngle[0][1];
    //         } else {
    //             pieceApproachAngle = ApproachAngle[1][1];
    //         }
    //     }

    //     return new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(pieceApproachAngle));
    // }

    // deleted many autos

    /**
     * Adds trajectory to move forward and back.
     * @see #command
     * @see SequentialCommandGroup#addCommands(edu.wpi.first.wpilibj2.command.Command...)
     */
    private void moveForward() {
        if (firstScore) {
            if (getConeOrCube()) {
                command.addCommands(new ScoreConeCommand(elevator, pivotArm, claw));
            } else {
                command.addCommands(new ScoreCubeCommand(elevator, pivotArm, claw));
            }
            command.addCommands(new HoldCommand(elevator, pivotArm));
        }

        // Literally made while queueing for quals during Robbinsville 2023
        // TODO Redo this so proper Pose2d is used
        List<Pose2d> trajPoints = new ArrayList<Pose2d>();
        trajPoints.add(ALLIANCE_SCORE_POSE[RobotContainer.firstScorePositionChooser.getSelected()]);
        trajPoints.add(driveOutPose(ALLIANCE_SCORE_POSE[RobotContainer.firstScorePositionChooser.getSelected()]));

        List<Pose2d> trajPointsBack = new ArrayList<Pose2d>();
        trajPointsBack
                .add(driveOutPose(ALLIANCE_SCORE_POSE[RobotContainer.firstScorePositionChooser.getSelected()]));
        trajPointsBack.add(ALLIANCE_SCORE_POSE[RobotContainer.firstScorePositionChooser.getSelected()]);

        addToPosCommand(new ToPosCommand(drivetrain, trajPoints, true));
        // addToPosCommand(new ToPosCommand(drivetrain, trajPointsBack, false));
        // addToPosCommand(new ToPosCommand(drivetrain, trajPoints, true));
        turn180();
    }

    private void moveForwardPID() {
        
        if (firstScore) {
            if (getConeOrCube()) {
                command.addCommands(new ScoreConeCommand(elevator, pivotArm, claw));
            } else {
                command.addCommands(new ScoreCubeCommand(elevator, pivotArm, claw));
            }
            command.addCommands(new HoldCommand(elevator, pivotArm));

        }

        this.command.addCommands(new DriveTimeCommand(drivetrain, 4));

        // Literally made while queueing for quals during Robbinsville 2023
        // TODO Redo this so proper Pose2d is used
        /* List<Pose2d> trajPoints = new ArrayList<Pose2d>();
        trajPoints.add(ALLIANCE_SCORE_POSE[RobotContainer.firstScorePositionChooser.getSelected()]);
        trajPoints.add(driveOutPose(ALLIANCE_SCORE_POSE[RobotContainer.firstScorePositionChooser.getSelected()]));
        */
        
        // addToPosCommand(new ToPosCommand(drivetrain, trajPoints, true));
        // addToPosCommand(new ToPosCommand(drivetrain, trajPointsBack, false));
        // addToPosCommand(new ToPosCommand(drivetrain, trajPoints, true));
        turn180();
    }

    // private Pose2d getCyclePose2d() {
    //     if (blue) {
    //         return Autonomous.BlueNormalEnd;
    //     } else {
    //         return Autonomous.RedNormalEnd;
    //     }
    // }


    private List<Pose2d> getTrajPointsWaypointReverse(Pose2d start, Pose2d end) {
        List<Pose2d> trajPoints = new ArrayList<Pose2d>();
        trajPoints.add(start);

        // going around the charging station, if convenient
        if (currentPose.getY() > Autonomous.CHARGE_CENTER_Y) {
            trajPoints.add(flipPose(ALLIANCE_WAYPOINTS_POSE[0]));
            trajPoints.add(flipPose(ALLIANCE_WAYPOINTS_POSE[1]));
        } else {
            trajPoints.add(flipPose(ALLIANCE_WAYPOINTS_POSE[2]));
            trajPoints.add(flipPose(ALLIANCE_WAYPOINTS_POSE[3]));
        }
        
        trajPoints.add(end);
        return trajPoints;
    }

    private void addScoreMidCone(){
        List<Pose2d> trajPoints = getTrajPointsWaypointReverse(currentPose, flipPose(getCargoLocation()));
        
        command.addCommands(new ScoreConeCommand(elevator, pivotArm, claw));
        command.addCommands(new HoldCommand(elevator, pivotArm));
        ToPosCommand step2 = new ToPosCommand(drivetrain, trajPoints, true);
        currentPose = flipPose(getCargoLocation());
        addToPosCommand(step2);
        
    }

    private void addScoreMidCube(){
        List<Pose2d> trajPoints = getTrajPointsWaypointReverse(currentPose, flipPose(getCargoLocation()));
        
        command.addCommands(new ScoreCubeCommand(elevator, pivotArm, claw));
        command.addCommands(new HoldCommand(elevator, pivotArm));
        ToPosCommand step2 = new ToPosCommand(drivetrain, trajPoints, true);
        currentPose = flipPose(getCargoLocation());
        addToPosCommand(step2);
        
    }

    private void addToPosCommand(ToPosCommand command) {
        this.command.addCommands(command);
        trajectoryList.add(command.getTrajectory());
    }

    public Pose2d shiftedPose(Pose2d pose) {
        double SHIFT_X = 0.25;
        if (blue) {
            SHIFT_X *= -1;
        }
        return new Pose2d(pose.getX() + SHIFT_X, pose.getY(), pose.getRotation());
    }

    public Pose2d driveOutPose(Pose2d pose) {
        double SHIFT_X = -4;
        if (blue) {
            SHIFT_X *= -1;
        }
        return new Pose2d(pose.getX() + SHIFT_X, pose.getY(), pose.getRotation());
    }

    private Pose2d flipPose(Pose2d pose) {
        return new Pose2d(pose.getX(), pose.getY(), pose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    // private void backUpAndTurn() {
    //     ToPosCommand firstTurnAround = new ToPosCommand(drivetrain, List.of(currentPose, shiftedPose(currentPose)),
    //             true);
    //     currentPose = shiftedPose(currentPose);
    //     addToPosCommand(firstTurnAround);

    //     turn180();
    // }

    private void turn180() {
        //
        if (RobotBase.isSimulation()) {
            this.command.addCommands(new Delay(0.5));
        } else {
            this.command.addCommands(new TurnAngleCommand(drivetrain, 180));
        }
        currentPose = flipPose(currentPose);
    }



    private void addLeaveCommunityTrajectory() {
        // this method and addCargoTrajectory are almost identical, different by one
        // line
        // TODO: refactor further to avoid confusion
        List<Pose2d> trajPoints = getTrajPointsWaypointReverse(currentPose, flipPose(getLeaveCommunityPose(currentPose)));
        ToPosCommand step2 = new ToPosCommand(drivetrain, trajPoints, true);
        currentPose = flipPose(getLeaveCommunityPose(currentPose));
        addToPosCommand(step2);
    }



    private void addOverChargeTrajectory() {
        ToPosCommand step3 = new ToPosCommand(drivetrain,
                List.of(currentPose, getHitAndRunPose2d()), true);
        currentPose = getHitAndRunPose2d();
        addToPosCommand(step3);

        command.addCommands(new ResetDriveCommand(drivetrain));

        ToPosCommand step2 = new ToPosCommand(drivetrain, List.of(currentPose, shiftedPose(chargePose[0])), false);
        currentPose = shiftedPose(chargePose[0]);
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

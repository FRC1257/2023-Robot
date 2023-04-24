package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.claw.*;
import frc.robot.commands.drivetrain.*;


import frc.robot.commands.elevator.ElevatorManualCommand;
import frc.robot.commands.elevator.ElevatorPIDCommand;

import frc.robot.commands.pivotArm.*;
import frc.robot.commands.scoringAssist.DecrementScorePosCommand;
import frc.robot.commands.scoringAssist.IncrementScorePosCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.GenerateTrajectories;
import frc.robot.commands.ResetPIDCommand;
import frc.robot.commands.Compound_Commands.HoldCommand;
import frc.robot.commands.Compound_Commands.*;
import frc.robot.commands.Compound_Commands.ScoreConeCommand;
import frc.robot.commands.Compound_Commands.ScoreCubeCommand;
import frc.robot.subsystems.SnailSubsystem;
import frc.robot.util.Gyro;

import frc.robot.util.SnailController;
import frc.robot.util.SnailController.DPad;

import java.util.ArrayList;

import static frc.robot.Constants.ElectricalLayout.CONTROLLER_DRIVER_ID;
import static frc.robot.Constants.ElectricalLayout.CONTROLLER_OPERATOR_ID;
import static frc.robot.Constants.UPDATE_PERIOD;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private SnailController driveController;
    private SnailController operatorController;
    
    private ArrayList<SnailSubsystem> subsystems;
    
    private Claw claw;
    private Drivetrain drivetrain;
    private Vision vision;
    private PivotArm pivotArm;
    private Elevator elevator;

    private Notifier updateNotifier;
    private int outputCounter;

    private boolean updateTraj = true;

    // choosers
    public static SendableChooser<Integer> firstScorePositionChooser = new SendableChooser<>();
    public static SendableChooser<Integer> gamePieceChooser = new SendableChooser<>(); 
    public static SendableChooser<Integer> autoChooser = new SendableChooser<>();
    public static SendableChooser<Integer> firstScoreLevelChooser = new SendableChooser<>();

    //booleans regarding the score, cargo, and charge
    private boolean firstScore = true;
    private boolean charge;

    private boolean isSimulation;
    private GenerateTrajectories generateTrajectories;


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveController = new SnailController(CONTROLLER_DRIVER_ID);
        operatorController = new SnailController(CONTROLLER_OPERATOR_ID);

        configureAutoChoosers();
        configureShuffleboard();
        configureSubsystems();
        configureButtonBindings();
        
        outputCounter = 0;

        updateNotifier = new Notifier(this::update);
        updateNotifier.startPeriodic(UPDATE_PERIOD);

        isSimulation = RobotBase.isSimulation();
    }

    public void stopDisplayingTraj() {
        updateTraj = false;
    }

    private boolean getAllianceColor() {
        return DriverStation.getAlliance() == DriverStation.Alliance.Blue;
    }

    private Pose2d getStartingPos() {
        return new Pose2d(0, 0, new Rotation2d(0.0));
    }
    

    /**
     * Declare all of our subsystems and their default bindings
     */
    private void configureSubsystems() {
        // declare each of the subsystems here

        
        vision = new Vision();
        drivetrain = new Drivetrain(getStartingPos(), vision);
        // drivetrain.setDefaultCommand(new ManualDriveCommand(drivetrain, driveController::getDriveForward, driveController::getDriveTurn));
        drivetrain.setDefaultCommand(new VelocityDriveCommand(drivetrain, driveController::getDriveForward, driveController::getDriveTurn,
             driveController.getButton(Button.kLeftBumper.value)::getAsBoolean, false));

        claw = new Claw();
        claw.setDefaultCommand(new ClawManualCommand(claw, operatorController::getLeftY));
        
        // Vision
        elevator = new Elevator();
        elevator.setDefaultCommand(new ElevatorManualCommand(elevator, operatorController::getElevatorSpeed));

        // Pivot Arm
        pivotArm = new PivotArm();
        pivotArm.setDefaultCommand(new PivotArmManualCommand(pivotArm, operatorController::getRightY));
        
        subsystems = new ArrayList<SnailSubsystem>();
        // add each of the subsystems to the arraylist here
        
        subsystems.add(drivetrain);
        subsystems.add(vision);
        subsystems.add(claw); 
        subsystems.add(pivotArm);
        subsystems.add(elevator);
        
        // generate auto
        getAutoCommand();

        if (SmartDashboard.getBoolean("Testing", false)) {
            tuningInit();
        }
    }

    /**
     * Define {@link Button} -> command mappings.
     */
    private void configureButtonBindings() {
        // Drivetrain bindings
        // driveController.getButton(Button.kLeftBumper.value).onTrue(new TurnToAprilTagCommand(drivetrain, vision));
        
        driveController.getButton(Button.kY.value).onTrue(new ToggleReverseCommand(drivetrain));
        driveController.getButton(Button.kStart.value).onTrue(new ToggleSlowModeCommand(drivetrain));
        driveController.getButton(Button.kX.value).onTrue(new ResetDriveCommand(drivetrain));
        driveController.getButton(Button.kB.value).onTrue(new ToSingleSubstation(drivetrain, SmartDashboard.getBoolean("isAllianceBlue", false)));
        // driveController.getButton(Button.kLeftBumper.value).onTrue(new TurnToAprilTagCommand(drivetrain, vision));
        
        // Operator bindings for pivot arm PID
        // operatorController.getButton(Button.kX.value).onTrue(new PivotArmPIDCommand(pivotArm, Constants.PivotArm.PIVOT_ARM_SETPOINT_UP));
        // operatorController.getButton(Button.kY.value).onTrue(new PivotArmPIDCommand(pivotArm, Constants.PivotArm.PIVOT_ARM_SETPOINT_INTAKE));
        // operatorController.getButton(Button.kA.value).onTrue(new PivotArmPIDCommand(pivotArm, Constants.PivotArm.PIVOT_ARM_SETPOINT_MID));
        
        // compound commands
        operatorController.getDPad(DPad.LEFT).onTrue(new MidConeSetpointCommand(elevator, pivotArm));
        // intake and low score are same
        // operatorController.getDPad(DPad.DOWN).onTrue(new IntakeCommand(elevator, pivotArm, pivotWrist));
        // operatorController.getDPad(DPad.LEFT).onTrue(new HoldCommand(elevator, pivotArm, pivotWrist));
        operatorController.getDPad(DPad.RIGHT).onTrue(new MidCubeSetpointCommand(elevator, pivotArm));

//TESTING
        operatorController.getButton(Button.kA.value).onTrue(new ScoreConeCommand(elevator, pivotArm, claw));
        operatorController.getButton(Button.kB.value).onTrue(new ScoreCubeCommand(elevator, pivotArm, claw));
//Testing
        // driveController.getButton(Button.kB.value).onTrue(new DriveDistanceCommand(drivetrain, -10));


        operatorController.getDPad(DPad.DOWN).onTrue(new HoldCommand(elevator, pivotArm));
        operatorController.getDPad(DPad.UP).onTrue(new ElevatorPIDCommand(elevator, -Constants.ElevatorConstants.ELEVATOR_SETPOINT_RETRACT));

        operatorController.getButton(Button.kX.value).onTrue(new ResetPIDCommand(elevator, pivotArm));

        // operatorController.getDPad(DPad.DOWN).onTrue(new ElevatorPIDCommand (elevator, -Constants.ElevatorConstants.ELEVATOR_SETPOINT_MIDDLE));
        // operatorController.getDPad(DPad.LEFT).onTrue(new PivotArmPIDCommand(pivotArm, Constants.PivotArm.PIVOT_ARM_SETPOINT_MID));
        // operatorController.getDPad(DPad.RIGHT).onTrue(new PivotArmPIDCommand(pivotArm, Constants.PivotArm.PIVOT_ARM_SETPOINT_HOLD));
        // operatorController.getDPad(DPad.UP).onTrue(new PivotArmPIDCommand(pivotArm, Constants.PivotArm.PIVOT_ARM_SETPOINT_UP));

        /* operatorController.getButton(Button.kX.value).onTrue(new ResetPIDCommand(elevator, pivotArm, pivotWrist)); */

        
        
        // driveController.getButton(Button.kY.value).onTrue(new PDBalanceCommand(drivetrain, true));
        // driveController.getButton(Button.kB.value).onTrue(new NoPDBalanceCommand(drivetrain).withTimeout(1));
        
        /* driveController.getDPad(DPad.RIGHT).onTrue(new TurnAngleCommand(drivetrain, 90));
        driveController.getDPad(DPad.LEFT).onTrue(new TurnAngleCommand(drivetrain, -90));
        driveController.getDPad(DPad.UP).onTrue(new TurnAngleCommand(drivetrain, 180)); */ // uncomment later

        // testing
        // driveController.getDPad(DPad.RIGHT).onTrue(new NoPDBalanceCommand(drivetrain));
        driveController.getDPad(DPad.LEFT).onTrue(new PDBalanceCommand(drivetrain, true));
        // driveController.getDPad(DPad.UP).onTrue(new PDBalanceWithVel(drivetrain, true));

        operatorController.getButton(Button.kLeftBumper.value).onTrue(new DecrementScorePosCommand(vision));
        operatorController.getButton(Button.kRightBumper.value).onTrue(new IncrementScorePosCommand(vision));
    }


    /**
     * Set up the choosers on shuffleboard for autonomous
     */
    public void configureAutoChoosers() {
        configureGamePieceChooser();
        configureFirstScorePositionChooser();
        configureChooseAuto();
    }

    public Pose2d shiftedPose(Pose2d pose) {
        double SHIFT_X = -3.75;
        return new Pose2d(pose.getX() + SHIFT_X, pose.getY(), pose.getRotation());
    }

    /**
     * Do the logic to return the auto command to run
     */
    public Command getAutoCommand() {
        updateAutoChoosers();

        generateTrajectories = new GenerateTrajectories(
            drivetrain,
            elevator,
            pivotArm,
            claw,
            charge,
            SmartDashboard.getBoolean("Auto Close to Cycle", false),
            firstScore
        );

        putTrajectoryTime();
        // drivetrain.drawTrajectory(generateTrajedies.getTrajectory());
        DriverStation.reportWarning("Auto Command Generated", false);
        return generateTrajectories.getCommand(); 
        //return new DriveDistanceCommand(drivetrain, 10);
    }

    /**
     * Update all of the subsystems
     * This is run in a separate loop at a faster rate to:
     * a) update subsystems faster
     * b) prevent packet delay from driver station from delaying response from our robot
     */
    private void update() {
        for(SnailSubsystem subsystem : subsystems) {
            subsystem.update();
        }
    }

    public void displayShuffleboard() {
        if (subsystems.size() == 0)
            return;
        

        if(outputCounter % 3 == 0) {
            subsystems.get(outputCounter / 3).displayShuffleboard();
        }

        Gyro.getInstance().outputValues();
        tuningPeriodic();

        outputCounter = (outputCounter + 1) % (subsystems.size() * 3);
    }

    public void tuningInit() {
        for(SnailSubsystem subsystem : subsystems) {
            subsystem.tuningInit();
            SmartDashboard.putData(subsystem);
        }
    }

    public void tuningPeriodic() {
        if(outputCounter % 3 == 0) {
            subsystems.get(outputCounter / 3).tuningPeriodic();
        }

        if (isSimulation && SmartDashboard.getBoolean("Reset Auto Viewer", false)) {
            updateTraj = true;
            SmartDashboard.putBoolean("Reset Auto Viewer", false);
        }

        if (updateTraj) { // change the trajectory drawn
            // generateTrajedies.incrementOutputCounter();
            Trajectory traj = generateTrajectories.getTrajectory((int)SmartDashboard.getNumber("View Trajectory Pos", 0));
            if (traj != null)
                drivetrain.drawTrajectory(traj); 
        }

        if (updateTraj && checkIfUpdate()) {
            DriverStation.reportWarning("Updating Auto", false);
            getAutoCommand();

            SmartDashboard.putNumber("View Trajectory Pos", generateTrajectories.getLastTrajectoryIndex());

            resetDashboard();
        }
    
    }

    //sendable chooser methods

    public void configureShuffleboard() {
        // Field Side
        SmartDashboard.putBoolean("Motor mode", true);
        SmartDashboard.putBoolean("isAllianceBlue", getAllianceColor());
        SmartDashboard.putBoolean("Testing", false);
        //getting the auto values for score, cargo, and charge
        SmartDashboard.putBoolean("1st Auto Score", firstScore);
        /* SmartDashboard.putBoolean("Opt. 2nd Auto Score", secondScore);
        SmartDashboard.putBoolean("Auto Get Cargo", cargo); */
        SmartDashboard.putBoolean("Auto Goto Charge", charge);
        SmartDashboard.putBoolean("Auto Close to Cycle", false);
        SmartDashboard.putNumber("View Trajectory Pos", 0);
        SmartDashboard.putBoolean("Update Visual", false);
        /* SmartDashboard.putBoolean("3 Ball Auto", false);
        SmartDashboard.putBoolean("Leave Tarmac", true);
        SmartDashboard.putBoolean("Hit and Run", false); */

        SmartDashboard.putBoolean("Reset Auto Viewer", false);
        
    }

    /* public void configureScoreLevelChooser() {
        /* firstScoreLevelChooser.setDefaultOption("First score level chooser", 0);
        firstScoreLevelChooser.addOption("low arm", 0);
        firstScoreLevelChooser.addOption("mid cone arm", 1);
        firstScoreLevelChooser.addOption("mid cube arm", 2);/* 
        firstScoreLevelChooser.addOption("mid shoot", 3);
        firstScoreLevelChooser.addOption("high shoot", 4); 
        SmartDashboard.putData(firstScoreLevelChooser); 
    } */
    
    public void configureGamePieceChooser() {
        gamePieceChooser.setDefaultOption("Cargo Piece Chooser", 0);
        gamePieceChooser.addOption("1st Position", 0);
        gamePieceChooser.addOption("2nd Position", 1);
        gamePieceChooser.addOption("3rd Position", 2);
        gamePieceChooser.addOption("4th Position", 3);
        SmartDashboard.putData(gamePieceChooser);
    }


    public void configureFirstScorePositionChooser() {
        firstScorePositionChooser.setDefaultOption("Score Position Chooser", 0);
        firstScorePositionChooser.addOption("1st Position", 0);
        firstScorePositionChooser.addOption("2nd Position", 1);
        firstScorePositionChooser.addOption("3rd Position", 2);
        firstScorePositionChooser.addOption("4th Position", 3);
        firstScorePositionChooser.addOption("5th Position", 4);
        firstScorePositionChooser.addOption("6th Position", 5);
        firstScorePositionChooser.addOption("7th Position", 6);
        firstScorePositionChooser.addOption("8th Position", 7);
        firstScorePositionChooser.addOption("9th Position", 8);
        SmartDashboard.putData(firstScorePositionChooser);
    }

    public void configureChooseAuto() {
        autoChooser.setDefaultOption("Move Forward with drive traj", 1);
        autoChooser.addOption("Normal Auto", 0);
        autoChooser.addOption("Hit & Run", 2);
        autoChooser.addOption("Hit", 3);
        autoChooser.addOption("Move forward Bad with time", 4);
        SmartDashboard.putData(autoChooser);
    }
    
    public boolean checkIfUpdate() {
        return firstScore != SmartDashboard.getBoolean("1st Auto Score", true)
            || charge != SmartDashboard.getBoolean("Auto Goto Charge", false) 
            || SmartDashboard.getBoolean("Update Visual", false);
    }

    public void updateAutoChoosers() {
        firstScore = SmartDashboard.getBoolean("1st Auto Score", firstScore);
        charge = SmartDashboard.getBoolean("Auto Goto Charge", charge);
    }

    public void putTrajectoryTime() {
        SmartDashboard.putNumber("Trajectory Time", generateTrajectories.getTrajectoryTime());
    }

    public void resetDashboard() {
        // Field Side
        SmartDashboard.putBoolean("isAllianceBlue", getAllianceColor());
        
        //getting the auto values for score, cargo, and charge
        SmartDashboard.putBoolean("1st Auto Score", firstScore);
        SmartDashboard.putBoolean("Auto Goto Charge", charge);
        SmartDashboard.putNumber("View Trajectory Pos", (int)SmartDashboard.getNumber("View Trajectory Pos", 0));
        SmartDashboard.putBoolean("Update Visual", false);
    }

}
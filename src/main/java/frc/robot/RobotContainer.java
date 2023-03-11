package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Delay;
import frc.robot.commands.claw.*;
import frc.robot.commands.drivetrain.ToPosCommand;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.led.LEDToggleCommand;
import frc.robot.commands.pivotWrist.PivotWristManualCommand;
import frc.robot.commands.pivotWrist.PivotWristPIDCommand;


import frc.robot.commands.elevator.ElevatorManualCommand;
import frc.robot.commands.vision.AlignPosCommand;

import frc.robot.commands.pivotArm.*;
import frc.robot.commands.vision.TurnToAprilTagCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.GenerateTrajectories;
import frc.robot.subsystems.SnailSubsystem;
import frc.robot.util.Gyro;

import frc.robot.util.SnailController;

import java.util.ArrayList;

import static frc.robot.Constants.ElectricalLayout.CONTROLLER_DRIVER_ID;
import static frc.robot.Constants.ElectricalLayout.CONTROLLER_OPERATOR_ID;
import static frc.robot.Constants.UPDATE_PERIOD;

import static frc.robot.Constants.IntakeArm.INTAKE_SETPOINT_BOT;
import static frc.robot.Constants.IntakeArm.INTAKE_SETPOINT_TOP;

import static frc.robot.Constants.Autonomous;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;

    private SnailController driveController;
    private SnailController operatorController;

    private boolean isTestBot = true;
    private Claw claw;
    
    private ArrayList<SnailSubsystem> subsystems;
    

    private PivotWrist pivotWrist;
    


    private Drivetrain drivetrain;
    private Vision vision;
    private PivotArm pivotArm;



    private Elevator elevator;
    private LED led;



    private Notifier updateNotifier;
    private int outputCounter;
    private int displayTrajCounter;

    private boolean updateTraj = true;

    // choosers
    public static SendableChooser<Integer> firstScorePositionChooser = new SendableChooser<>();
    public static SendableChooser<Integer> secondScorePositionChooser = new SendableChooser<>();
    public static SendableChooser<Integer> gamePieceChooser = new SendableChooser<>(); 
    public static SendableChooser<Integer> startPositionChooser = new SendableChooser<>(); 
    public static SendableChooser<Integer> secondGamePieceChooser = new SendableChooser<>();
    public static SendableChooser<Integer> thirdScorePositionChooser = new SendableChooser<>();
    public static SendableChooser<Boolean> hitAndRunChooser = new SendableChooser<>();

    //booleans regarding the score, cargo, and charge
    private boolean firstScore;
    private boolean secondScore;
    private boolean cargo;
    private boolean charge;
    private boolean leaveTarmac = true;
    private boolean hitAndRun;

    private boolean isSimulation;

    private boolean threePiece;

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
        displayTrajCounter = 0;

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

        if (!isTestBot) {
            claw = new Claw();
            claw.setDefaultCommand(new ClawNeutralCommand(claw));
         
            // Vision
            elevator = new Elevator();
            // Pivot Wrist
            pivotWrist = new PivotWrist();
            pivotWrist.setDefaultCommand(new PivotWristManualCommand(pivotWrist, operatorController::getRightY));
    
            
            // Pivot Arm
            pivotArm = new PivotArm();
            pivotArm.setDefaultCommand(new PivotArmManualCommand(pivotArm, operatorController::getLeftY));

            // LED
            led = new LED();
        
        }
        
        

        // Vision

        // Pivot arm
        
        subsystems = new ArrayList<SnailSubsystem>();
        // add each of the subsystems to the arraylist here
        
        subsystems.add(drivetrain);
        subsystems.add(vision);

        if (!isTestBot) {
            subsystems.add(claw);
            subsystems.add(pivotArm);
            subsystems.add(pivotWrist);
            subsystems.add(elevator);
            subsystems.add(led);
        }

        // generate auto
        generateTrajectories = new GenerateTrajectories(
            drivetrain,
            charge,
            firstScore,
            secondScore,
            cargo,
            0,
            threePiece,
            leaveTarmac,
            hitAndRun 
        );

        if (SmartDashboard.getBoolean("Testing", false)) {
            tuningInit();
        }

        putTrajectoryTime();
    }

    /**
     * Define {@link Button} -> command mappings.
     */
    private void configureButtonBindings() {
        // Drivetrain bindings

        // driveController.getButton(Button.kY.value).onTrue(new ToggleReverseCommand(drivetrain));
        // driveController.getButton(Button.kStart.value).onTrue(new ToggleSlowModeCommand(drivetrain));
        // driveController.getButton(Button.kA.value).onTrue(new TurnAngleCommand(drivetrain, -90));
        // driveController.getButton(Button.kB.value).onTrue(new TurnAngleCommand(drivetrain, 90));
        // driveController.getButton(Button.kX.value).onTrue(new ResetDriveCommand(drivetrain));
        // driveController.getButton(Button.kLeftBumper.value).onTrue(new TurnToAprilTagCommand(drivetrain, vision));
        
        // driveController.getButton(Button.kY.value).onTrue(new ToggleReverseCommand(drivetrain));
        driveController.getButton(Button.kStart.value).onTrue(new ToggleSlowModeCommand(drivetrain));
        // driveController.getButton(Button.kA.value).onTrue(new TurnAngleCommand(drivetrain, -90));
        // driveController.getButton(Button.kB.value).onTrue(new TurnAngleCommand(drivetrain, 90));
        driveController.getButton(Button.kX.value).onTrue(new ResetDriveCommand(drivetrain));
        // driveController.getButton(Button.kLeftBumper.value).onTrue(new TurnToAprilTagCommand(drivetrain, vision));
        
        // Operator bindings
        if (!isTestBot) {
            // operatorController.getButton(Button.kX.value).onTrue(new PivotArmPIDCommand(pivotArm, Constants.PivotArm.PIVOT_ARM_SETPOINT_UP));
            // operatorController.getButton(Button.kY.value).onTrue(new PivotArmPIDCommand(pivotArm, Constants.PivotArm.PIVOT_ARM_SETPOINT_INTAKE));
            // operatorController.getButton(Button.kA.value).onTrue(new PivotArmPIDCommand(pivotArm, Constants.PivotArm.PIVOT_ARM_SETPOINT_MID));

            // // Operator bindings
            // operatorController.getButton(Button.kA.value).onTrue(new PivotWristPIDCommand(pivotWrist, Constants.PivotWrist.WRIST_SETPOINT_INTAKE));
            // operatorController.getButton(Button.kB.value).onTrue(new PivotWristPIDCommand(pivotWrist, Constants.PivotWrist.WRIST_SETPOINT_HIGH));
            // operatorController.getButton(Button.kX.value).onTrue(new PivotWristPIDCommand(pivotWrist, Constants.PivotWrist.WRIST_SETPOINT_MID));
    
            operatorController.getButton(Button.kLeftBumper.value).onTrue(new ClawIntakeCommand(claw));
            operatorController.getButton(Button.kRightBumper.value).onTrue(new ClawEjectCommand(claw));

            operatorController.getButton(Button.kLeftStick.value).onTrue(new ClawConeStateCommand(claw));
            operatorController.getButton(Button.kRightStick.value).onTrue(new ClawCubeStateCommand(claw));

            // Operator Bindings
            operatorController.getButton(Button.kA.value).onTrue(new ElevatorManualCommand(elevator, Constants.ElevatorConstants.ELEVATOR_MANUAL_SPEED));

            operatorController.getButton(Button.kB.value).onTrue(new LEDToggleCommand(led));
        }
        
        driveController.getButton(Button.kY.value).onTrue(new PDBalanceCommand(drivetrain, true));
        driveController.getButton(Button.kB.value).onTrue(new ParallelDeadlineGroup(
            new Delay(5),
            new PDBalanceCommand(drivetrain, false)
        ));

        // driveController.getButton(Button.kY.value).onTrue(new AlignPosCommand(drivetrain, Constants.Autonomous.BLUE_SCORE_POSE[4]));
        // driveController.getButton(Button.kStart.value).onTrue(new ToggleSlowModeCommand(drivetrain));
        //driveController.getButton(Button.kA.value).onTrue(new TurnAngleCommand(drivetrain, -90));
        
        // driveController.getButton(Button.kB.value).onTrue(new TurnAngleCommand(drivetrain, 90));
        // driveController.getButton(Button.kX.value).onTrue(new ResetDriveCommand(drivetrain));
        // driveController.getButton(Button.kLeftBumper.value).onTrue(new TurnToAprilTagCommand(drivetrain, vision));
        // driveController.getDPad(SnailController.DPad.UP).onTrue(new IntakeArmPIDCommand(intakearm, INTAKE_SETPOINT_TOP));
        // driveController.getDPad(SnailController.DPad.DOWN).onTrue(new IntakeArmPIDCommand(intakearm, INTAKE_SETPOINT_BOT));
        

        driveController.getButton(Button.kLeftBumper.value).onTrue(new TurnAngleCommand(drivetrain, 90));
        driveController.getButton(Button.kRightBumper.value).onTrue(new TurnAngleCommand(drivetrain, -90));
        

    }


    /**
     * Set up the choosers on shuffleboard for autonomous
     */
    public void configureAutoChoosers() {
        configureGamePieceChooser();
        configureFirstScorePositionChooser();
        configureSecondScorePositionChooser();
        configureStartPositionChooser();
        configureSecondGamePieceChooser();
        configureThirdScorePositionChooser();
        configureHitAndRunChooser();
    }

    private int estimatedCurrentPose2d() {
        // TODO: Implement this with PhotonVision
        // return new Pose2d(0, 0, new Rotation2d(0.0));
        return startPositionChooser.getSelected();
    }

    /**
     * Do the logic to return the auto command to run
     */
    public Command getAutoCommand() {
        updateAutoChoosers();

        generateTrajectories = new GenerateTrajectories(
            drivetrain,
            charge,
            firstScore,
            secondScore,
            cargo,
            0,
            threePiece,
            leaveTarmac,
            hitAndRun
        );
        
        putTrajectoryTime();
        // drivetrain.drawTrajectory(generateTrajedies.getTrajectory());
        // DriverStation.reportWarning("Auto Command: " + generateTrajedies.getTrajectory().toString(), false);
        return generateTrajectories.getCommand();
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
        }
    }

    public void tuningPeriodic() {
        if(outputCounter % 3 == 0) {
            subsystems.get(outputCounter / 3).tuningPeriodic();
        }

        drivetrain.tuningPeriodic();

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
            DriverStation.reportWarning("Updating Auto", cargo);
            updateAutoChoosers();

            generateTrajectories = new GenerateTrajectories(
                drivetrain,
                charge,
                firstScore,
                secondScore,
                cargo,
                estimatedCurrentPose2d(),
                threePiece,
                leaveTarmac,
                hitAndRun
            );

            SmartDashboard.putNumber("View Trajectory Pos", generateTrajectories.getLastTrajectoryIndex());

            putTrajectoryTime();
            resetDashboard();
        }
    
    }

    //sendable chooser methods

    public void configureShuffleboard() {
        // Field Side
        SmartDashboard.putBoolean("isAllianceBlue", getAllianceColor());
        SmartDashboard.putBoolean("Testing", true);
        //getting the auto values for score, cargo, and charge
        SmartDashboard.putBoolean("1st Auto Score", firstScore);
        SmartDashboard.putBoolean("Opt. 2nd Auto Score", secondScore);
        SmartDashboard.putBoolean("Auto Get Cargo", cargo);
        SmartDashboard.putBoolean("Auto Goto Charge", charge);
        SmartDashboard.putNumber("View Trajectory Pos", 0);
        SmartDashboard.putBoolean("Update Visual", false);
        SmartDashboard.putBoolean("3 Ball Auto", false);
        SmartDashboard.putBoolean("Leave Tarmac", true);
        SmartDashboard.putBoolean("Hit and Run", false);

        SmartDashboard.putBoolean("Reset Auto Viewer", false);
        
    }

    public void configureGamePieceChooser() {
        gamePieceChooser.setDefaultOption("Cargo Piece Chooser", 0);
        gamePieceChooser.addOption("1st Position", 0);
        gamePieceChooser.addOption("2nd Position", 1);
        gamePieceChooser.addOption("3rd Position", 2);
        gamePieceChooser.addOption("4th Position", 3);
        SmartDashboard.putData(gamePieceChooser);
    }

    public void configureSecondGamePieceChooser() {
        secondGamePieceChooser.setDefaultOption("Second Cargo Chooser", 0);
        secondGamePieceChooser.addOption("1st Position", 0);
        secondGamePieceChooser.addOption("2nd Position", 1);
        secondGamePieceChooser.addOption("3rd Position", 2);
        secondGamePieceChooser.addOption("4th Position", 3);
        SmartDashboard.putData(secondGamePieceChooser);
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

    public void configureSecondScorePositionChooser() {
        secondScorePositionChooser.setDefaultOption("Second Score Position Chooser", 0);
        secondScorePositionChooser.addOption("1st Position", 0);
        secondScorePositionChooser.addOption("2nd Position", 1);
        secondScorePositionChooser.addOption("3rd Position", 2);
        secondScorePositionChooser.addOption("4th Position", 3);
        secondScorePositionChooser.addOption("5th Position", 4);
        secondScorePositionChooser.addOption("6th Position", 5);
        secondScorePositionChooser.addOption("7th Position", 6);
        secondScorePositionChooser.addOption("8th Position", 7);
        secondScorePositionChooser.addOption("9th Position", 8);
        SmartDashboard.putData(secondScorePositionChooser);
    }


    public void configureThirdScorePositionChooser() {
        thirdScorePositionChooser.setDefaultOption("Third Score Position Chooser", 0);
        thirdScorePositionChooser.addOption("1st Position", 0);
        thirdScorePositionChooser.addOption("2nd Position", 1);
        thirdScorePositionChooser.addOption("3rd Position", 2);
        thirdScorePositionChooser.addOption("4th Position", 3);
        thirdScorePositionChooser.addOption("5th Position", 4);
        thirdScorePositionChooser.addOption("6th Position", 5);
        thirdScorePositionChooser.addOption("7th Position", 6);
        thirdScorePositionChooser.addOption("8th Position", 7);
        thirdScorePositionChooser.addOption("9th Position", 8);
        SmartDashboard.putData(thirdScorePositionChooser);
    }

    public void configureStartPositionChooser() {
        startPositionChooser.setDefaultOption("Start Position", 0);
        startPositionChooser.addOption("1st Position", 0);
        startPositionChooser.addOption("2nd Position", 1);
        startPositionChooser.addOption("3rd Position", 2);
        SmartDashboard.putData(startPositionChooser);
    }

    public void configureHitAndRunChooser() {
        hitAndRunChooser.setDefaultOption("Hit and Run", false);
        hitAndRunChooser.addOption("Hit and Run", false);
        hitAndRunChooser.addOption("Hit and Run", true);
    }

    public boolean checkIfUpdate() {
        return firstScore != SmartDashboard.getBoolean("1st Auto Score", false) || secondScore != SmartDashboard.getBoolean("Opt. 2nd Auto Score", false) || cargo != SmartDashboard.getBoolean("Auto Get Cargo", false) || charge != SmartDashboard.getBoolean("Auto Goto Charge", false) || SmartDashboard.getBoolean("Update Visual", false) || threePiece != SmartDashboard.getBoolean("3 Ball Auto", false);
    }

    public void updateAutoChoosers() {
        firstScore = SmartDashboard.getBoolean("1st Auto Score", firstScore);
        secondScore = SmartDashboard.getBoolean("Opt. 2nd Auto Score", secondScore);
        cargo = SmartDashboard.getBoolean("Auto Get Cargo", cargo);
        charge = SmartDashboard.getBoolean("Auto Goto Charge", charge);
        threePiece = SmartDashboard.getBoolean("3 Ball Auto", threePiece);
        leaveTarmac = SmartDashboard.getBoolean("Leave Tarmac", leaveTarmac);
        hitAndRun = SmartDashboard.getBoolean("Hit and Run", hitAndRun);
    }

    public void putTrajectoryTime() {
        SmartDashboard.putNumber("Trajectory Time", generateTrajectories.getTrajectoryTime());
    }

    public void resetDashboard() {
        // Field Side
        SmartDashboard.putBoolean("isAllianceBlue", getAllianceColor());
        
        //getting the auto values for score, cargo, and charge
        SmartDashboard.putBoolean("1st Auto Score", firstScore);
        SmartDashboard.putBoolean("Opt. 2nd Auto Score", secondScore);
        SmartDashboard.putBoolean("Auto Get Cargo", cargo);
        SmartDashboard.putBoolean("Auto Goto Charge", charge);
        SmartDashboard.putNumber("View Trajectory Pos", (int)SmartDashboard.getNumber("View Trajectory Pos", 0));
        SmartDashboard.putBoolean("Update Visual", false);
        SmartDashboard.putBoolean("3 Ball Auto", threePiece);
        SmartDashboard.putBoolean("Leave Tarmac", leaveTarmac);
        SmartDashboard.putBoolean("Hit and Run", hitAndRun);
    }

}
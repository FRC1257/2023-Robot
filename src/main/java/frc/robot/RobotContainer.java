package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.commands.ClawConeStateCommand;
import frc.robot.commands.ClawCubeStateCommand;
import frc.robot.commands.ClawEjectCommand;
import frc.robot.commands.ClawIntakeCommand;
import frc.robot.commands.ClawNeutralCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SnailSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Delay;
import frc.robot.commands.ToPosCommand;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.intakearm.IntakeArmPIDCommand;

import frc.robot.commands.pivotWrist.PivotWristManualCommand;
import frc.robot.commands.pivotWrist.PivotWristPIDCommand;

import frc.robot.commands.elevator.ElevatorExtendCommand;
import frc.robot.commands.elevator.ElevatorRetractCommand;
import frc.robot.commands.vision.AlignPosCommand;

import frc.robot.commands.vision.TurnToAprilTagCommand;
import frc.robot.commands.IntakeEjectingCommand;
import frc.robot.commands.IntakeIntakingCommand;
import frc.robot.commands.IntakeNeutralCommand;
import frc.robot.subsystems.*;

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


    private Claw claw;
    
    private ArrayList<SnailSubsystem> subsystems;
    

    private PivotWrist pivotWrist;
    private ArrayList<SnailSubsystem> subsystems;


    private Drivetrain drivetrain;
    private Vision vision;
    private IntakeArm intakearm;


    private Intake intake;

    private Elevator elevator;



    private Notifier updateNotifier;
    private int outputCounter;
    private int displayTrajCounter;

    private boolean updateTraj = true;

    // choosers
    public static SendableChooser<Integer> scorePositionChooser = new SendableChooser<>();
    public static SendableChooser<Integer> gamePieceChooser = new SendableChooser<>(); 
    public static SendableChooser<Integer> startPositionChooser = new SendableChooser<>(); 

    //booleans regarding the score, cargo, and charge
    private boolean score;
    private boolean cargo;
    private boolean charge;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveController = new SnailController(CONTROLLER_DRIVER_ID);
        operatorController = new SnailController(CONTROLLER_OPERATOR_ID);

        configureSubsystems();
        configureButtonBindings();
        
        outputCounter = 0;
        displayTrajCounter = 0;

        // Field Side
        SmartDashboard.putBoolean("Testing", true);
        
        updateNotifier = new Notifier(this::update);
        updateNotifier.startPeriodic(UPDATE_PERIOD);
    }

   
    private Pose2d getStartingPos() {
        return new Pose2d(0, 0, new Rotation2d(0.0));
    }
    

    /**
     * Declare all of our subsystems and their default bindings
     */
    private void configureSubsystems() {
        // declare each of the subsystems here

        claw = new Claw();
        claw.setDefaultCommand(new ClawNeutralCommand(claw));

        drivetrain = new Drivetrain(getStartingPos());
        // drivetrain.setDefaultCommand(new ManualDriveCommand(drivetrain, driveController::getDriveForward, driveController::getDriveTurn));
        drivetrain.setDefaultCommand(new VelocityDriveCommand(drivetrain, driveController::getDriveForward, driveController::getDriveTurn,
             driveController.getButton(Button.kLeftBumper.value)::getAsBoolean, false));

        // Vision
        vision = new Vision();
        intakearm = new IntakeArm();
        elevator = new Elevator();
        // Pivot Wrist
        pivotWrist = new PivotWrist();
        pivotWrist.setDefaultCommand(new PivotWristManualCommand(pivotWrist, operatorController::getRightY));

        // Intake
        intake = new Intake();
        intake.setDefaultCommand(new IntakeNeutralCommand(intake));



        // Vision
        
        subsystems = new ArrayList<SnailSubsystem>();
        // add each of the subsystems to the arraylist here
        subsystems.add(claw);
        subsystems.add(drivetrain);
        subsystems.add(vision);
        subsystems.add(intakearm);
        subsystems.add(pivotWrist);
        subsystems.add(intake);
        subsystems.add(elevator);


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
        driveController.getDPad(SnailController.DPad.LEFT).onTrue(new ClawConeStateCommand(claw));
        driveController.getDPad(SnailController.DPad.RIGHT).onTrue(new ClawCubeStateCommand(claw));
        driveController.getButton(Button.kY.value).onTrue(new ClawIntakeCommand(claw));
        driveController.getButton(Button.kX.value).onTrue(new ClawEjectCommand(claw));
        driveController.getButton(Button.kB.value).onTrue(new ClawNeutralCommand(claw));

        driveController.getButton(Button.kY.value).onTrue(new AlignPosCommand(drivetrain, Constants.Autonomous.BLUE_SCORE_POSE[4]));
        driveController.getButton(Button.kStart.value).onTrue(new ToggleSlowModeCommand(drivetrain));
        //driveController.getButton(Button.kA.value).onTrue(new TurnAngleCommand(drivetrain, -90));
        
        //driveController.getButton(Button.kB.value).onTrue(new TurnAngleCommand(drivetrain, 90));
        driveController.getButton(Button.kX.value).onTrue(new ResetDriveCommand(drivetrain));
        driveController.getButton(Button.kLeftBumper.value).onTrue(new TurnToAprilTagCommand(drivetrain, vision));
        driveController.getDPad(SnailController.DPad.UP).onTrue(new IntakeArmPIDCommand(intakearm, INTAKE_SETPOINT_TOP));
        driveController.getDPad(SnailController.DPad.DOWN).onTrue(new IntakeArmPIDCommand(intakearm, INTAKE_SETPOINT_BOT));
        operatorController.getButton(Button.kA.value).whileTrue(new IntakeEjectingCommand(intake));
        operatorController.getButton(Button.kB.value).whileTrue(new IntakeIntakingCommand(intake));

        
        // Operator bindings
        operatorController.getButton(Button.kA.value).onTrue(new PivotWristPIDCommand(pivotWrist, Constants.PivotWrist.WRIST_SETPOINT_INTAKE));
        operatorController.getButton(Button.kB.value).onTrue(new PivotWristPIDCommand(pivotWrist, Constants.PivotWrist.WRIST_SETPOINT_HIGH));
        operatorController.getButton(Button.kX.value).onTrue(new PivotWristPIDCommand(pivotWrist, Constants.PivotWrist.WRIST_SETPOINT_MID));


        // Operator Bindings
        operatorController.getButton(Button.kX.value).onTrue(new ElevatorExtendCommand(elevator));
        operatorController.getButton(Button.kY.value).onTrue(new ElevatorRetractCommand(elevator));


    }


    /**
     * Set up the choosers on shuffleboard for autonomous
     */
    public void configureAutoChoosers() {
        
    }

    /**
     * Do the logic to return the auto command to run
     */
    public Command getAutoCommand() {
        return new Delay(2.0);
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
    }

}
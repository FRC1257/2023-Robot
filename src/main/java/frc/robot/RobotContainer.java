package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.drivetrain.*;
import frc.robot.commands.vision.TurnToAprilTagCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SnailSubsystem;
import frc.robot.util.SnailController;

import java.util.ArrayList;

import static frc.robot.Constants.ElectricalLayout.CONTROLLER_DRIVER_ID;
import static frc.robot.Constants.ElectricalLayout.CONTROLLER_OPERATOR_ID;
import static frc.robot.Constants.UPDATE_PERIOD;
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
    
    private ArrayList<SnailSubsystem> subsystems;


    private Drivetrain drivetrain;
    private Vision vision;

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
   
    /**
     * Declare all of our subsystems and their default bindings
     */
    private void configureSubsystems() {
        // declare each of the subsystems here
        drivetrain = new Drivetrain(getStartingPos());
        // drivetrain.setDefaultCommand(new ManualDriveCommand(drivetrain, driveController::getDriveForward, driveController::getDriveTurn));
        drivetrain.setDefaultCommand(new VelocityDriveCommand(drivetrain, driveController::getDriveForward, driveController::getDriveTurn,
             driveController.getButton(Button.kLeftBumper.value)::getAsBoolean, false));

        // Vision
        vision = new Vision();

        subsystems = new ArrayList<>();
        // add each of the subsystems to the arraylist here
        subsystems.add(drivetrain);
        subsystems.add(vision);
    }

    /**
     * Define {@link Button} -> command mappings.
     */
    private void configureButtonBindings() {
        // Drivetrain bindings
        driveController.getButton(Button.kY.value).onTrue(new ToggleReverseCommand(drivetrain));
        driveController.getButton(Button.kStart.value).onTrue(new ToggleSlowModeCommand(drivetrain));
        //driveController.getButton(Button.kA.value).onTrue(new TurnAngleCommand(drivetrain, -90));
        //driveController.getButton(Button.kB.value).onTrue(new TurnAngleCommand(drivetrain, 90));
        driveController.getButton(Button.kX.value).onTrue(new ResetDriveCommand(drivetrain));
        driveController.getButton(Button.kLeftBumper.value).onTrue(new TurnToAprilTagCommand(drivetrain, vision));
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
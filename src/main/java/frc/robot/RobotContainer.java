package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.GenerateTrajedies;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.vision.TurnToAprilTagCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SnailSubsystem;
import frc.robot.util.SnailController;

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

    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;

    private SnailController driveController;
    private SnailController operatorController;
    
    private ArrayList<SnailSubsystem> subsystems;

    private GenerateTrajedies generateTrajedies;

    private Drivetrain drivetrain;
    private Vision vision;

    private Notifier updateNotifier;
    private int outputCounter;


    //Chris's choosers
    public static SendableChooser<Integer> scorePositionChooser = new SendableChooser<>();
    public static SendableChooser<Integer> gamePieceChooser = new SendableChooser<>(); 
    public static SendableChooser<Integer> startPositionChooser = new SendableChooser<>(); 

    //booleans regarding the score, cargo, and charge
    boolean score;
    boolean cargo;
    boolean charge;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveController = new SnailController(CONTROLLER_DRIVER_ID);
        operatorController = new SnailController(CONTROLLER_OPERATOR_ID);

        configureSubsystems();
        configureAutoChoosers();
        configureButtonBindings();
        
        outputCounter = 0;

        // Field Side
        SmartDashboard.putBoolean("isAllianceBlue", false);
        SmartDashboard.putBoolean("Testing", false);
        //getting the auto values for score, cargo, and charge
        SmartDashboard.putBoolean("Auto Score", score);
        SmartDashboard.putBoolean("Auto Get Cargo", cargo);
        SmartDashboard.putBoolean("Auto Goto Charge", charge);

        updateNotifier = new Notifier(this::update);
        updateNotifier.startPeriodic(UPDATE_PERIOD);
    }

    private Pose2d getStartingPos() {
        return new Pose2d(0, 0, new Rotation2d(0.0));
    }

    /**
     * Set up the choosers on shuffleboard for getting score positions
     */

   
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
     * Set up the choosers on shuffleboard for autonomous
     */
    public void configureAutoChoosers() {
        configuregamePieceChooser();
        configurescorePositionChooser();
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
        generateTrajedies = new GenerateTrajedies(
            SmartDashboard.getBoolean("Auto Goto Charge", charge),
            SmartDashboard.getBoolean("Auto Score", score),
            SmartDashboard.getBoolean("Auto Get Cargo", cargo),
            drivetrain,
            estimatedCurrentPose2d()
        );
        
        drivetrain.drawTrajectory(generateTrajedies.getTrajectory());
        return generateTrajedies.getCommand();
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

    //senderchooser methods

    public void configuregamePieceChooser() {
        gamePieceChooser.setDefaultOption("-1", -1);
        gamePieceChooser.addOption("1st Position", 0);
        gamePieceChooser.addOption("2nd Position", 1);
        gamePieceChooser.addOption("3rd Position", 2);
        gamePieceChooser.addOption("4th Position", 3);
        SmartDashboard.putData(gamePieceChooser);
    }
    
  
    public void configurescorePositionChooser() {
        scorePositionChooser.setDefaultOption("-1", -1);
        scorePositionChooser.addOption("1st Position", 0);
        scorePositionChooser.addOption("2nd Position", 1);
        scorePositionChooser.addOption("3rd Position", 2);
        scorePositionChooser.addOption("4th Position", 3);
        scorePositionChooser.addOption("5th Position", 4);
        scorePositionChooser.addOption("6th Position", 5);
        scorePositionChooser.addOption("7th Position", 6);
        scorePositionChooser.addOption("8th Position", 7);
        scorePositionChooser.addOption("9th Position", 8);
        SmartDashboard.putData(scorePositionChooser);
    }

    public void configurestartPositionChooser() {
        gamePieceChooser.setDefaultOption("-1", -1);
        gamePieceChooser.addOption("1st Position", 0);
        gamePieceChooser.addOption("2nd Position", 1);
        gamePieceChooser.addOption("3rd Position", 2);
        SmartDashboard.putData(startPositionChooser);
    }

}

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

    private GenerateTrajedies generateTrajedies;


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveController = new SnailController(CONTROLLER_DRIVER_ID);
        operatorController = new SnailController(CONTROLLER_OPERATOR_ID);

        configureAutoChoosers();
        configureSubsystems();
        configureButtonBindings();
        
        outputCounter = 0;
        displayTrajCounter = 0;

        // Field Side
        SmartDashboard.putBoolean("isAllianceBlue", getAllianceColor());
        SmartDashboard.putBoolean("Testing", true);
        //getting the auto values for score, cargo, and charge
        SmartDashboard.putBoolean("Auto Score", score);
        SmartDashboard.putBoolean("Auto Get Cargo", cargo);
        SmartDashboard.putBoolean("Auto Goto Charge", charge);
        SmartDashboard.putNumber("View Trajectory Pos", 0);

        updateNotifier = new Notifier(this::update);
        updateNotifier.startPeriodic(UPDATE_PERIOD);
    }

    public void stopDisplayingTraj() {
        updateTraj = false;
    }

    private boolean getAllianceColor() {
        return DriverStation.getAlliance() == DriverStation.Alliance.Blue;
    }

    private Pose2d getStartingPos() {
        return new Pose2d(0, 0, new Rotation2d(0.0));
        /* Pose2d[] ALLIANCE_START_POSE;
        if (SmartDashboard.getBoolean("isAllianceBlue", false)) {
            ALLIANCE_START_POSE = Autonomous.BLUE_START_POSE;
        } else {
            ALLIANCE_START_POSE = Autonomous.RED_START_POSE;
        }
        return ALLIANCE_START_POSE[startPositionChooser.getSelected()]; 
         */
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

        // generate auto
        generateTrajedies = new GenerateTrajedies(
            drivetrain,
            charge,
            score,
            cargo,
            drivetrain,
            0
        );
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
        configureGamePieceChooser();
        configureScorePositionChooser();
        configureStartPositionChooser();
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

        generateTrajedies = new GenerateTrajedies(
            drivetrain,
            charge,
            score,
            cargo,
            drivetrain,
            estimatedCurrentPose2d()
        );
        
        // drivetrain.drawTrajectory(generateTrajedies.getTrajectory());
        // DriverStation.reportWarning("Auto Command: " + generateTrajedies.getTrajectory().toString(), false);
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


        if (updateTraj) { // change the trajectory drawn
            // generateTrajedies.incrementOutputCounter();
            Trajectory traj = generateTrajedies.getTrajectory((int)SmartDashboard.getNumber("View Trajectory Pos", 0));
            if (traj != null)
                drivetrain.drawTrajectory(traj);
        }

        if (updateTraj && checkIfUpdate()) {
            DriverStation.reportWarning("Updating Auto", cargo);
            updateAutoChoosers();

            generateTrajedies = new GenerateTrajedies(
                drivetrain,
                charge,
                score,
                cargo,
                drivetrain,
                estimatedCurrentPose2d()
            );
        }
    
    }

    //sendable chooser methods

    public void configureGamePieceChooser() {
        gamePieceChooser.setDefaultOption("-1", 0);
        gamePieceChooser.addOption("1st Position", 0);
        gamePieceChooser.addOption("2nd Position", 1);
        gamePieceChooser.addOption("3rd Position", 2);
        gamePieceChooser.addOption("4th Position", 3);
        SmartDashboard.putData(gamePieceChooser);
    }
    
  
    public void configureScorePositionChooser() {
        scorePositionChooser.setDefaultOption("-1", 0);
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

    public void configureStartPositionChooser() {
        startPositionChooser.setDefaultOption("Set me", 0);
        startPositionChooser.addOption("1st Position", 0);
        startPositionChooser.addOption("2nd Position", 1);
        startPositionChooser.addOption("3rd Position", 2);
        SmartDashboard.putData(startPositionChooser);
    }


    public boolean checkIfUpdate() {
        return score != SmartDashboard.getBoolean("Auto Score", false) || cargo != SmartDashboard.getBoolean("Auto Get Cargo", false) || charge != SmartDashboard.getBoolean("Auto Goto Charge", false);
    }

    public void updateAutoChoosers() {
        score = SmartDashboard.getBoolean("Auto Score", false);
        cargo = SmartDashboard.getBoolean("Auto Get Cargo", false);
        charge = SmartDashboard.getBoolean("Auto Goto Charge", false);
    }

}

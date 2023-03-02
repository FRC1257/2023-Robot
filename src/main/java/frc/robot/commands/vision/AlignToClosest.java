package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Autonomous;
import frc.robot.subsystems.Drivetrain;

public class AlignToClosest extends CommandBase {
    private final Drivetrain drivetrain;
    private Pose2d currentPose;
    private Pose2d[] ALLIANCE_SCORE_POSE;
    
    public AlignToClosest(Drivetrain drivetrain, Pose2d currentPose) {
        this.drivetrain = drivetrain;
        this.currentPose = currentPose;
        if (SmartDashboard.getBoolean("isAllianceBlye", false)) {
            ALLIANCE_SCORE_POSE = Autonomous.BLUE_SCORE_POSE;
        }
        else {
            ALLIANCE_SCORE_POSE = Autonomous.RED_SCORE_POSE;
        }

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

    }    
}

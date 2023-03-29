package frc.robot.commands.scoringAssist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AlignDesiredScorePos extends CommandBase {
    Drivetrain drivetrain;
    Vision vision;
    Pose2d target;

    public AlignDesiredScorePos(Drivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        int desiredScorePos = vision.getDesiredScorePos();
        
    }
}

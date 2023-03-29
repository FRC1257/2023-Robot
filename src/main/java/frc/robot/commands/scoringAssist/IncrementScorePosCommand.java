package frc.robot.commands.scoringAssist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;

public class IncrementScorePosCommand extends CommandBase {
    Vision vision;

    public IncrementScorePosCommand(Vision vision) {
        this.vision = vision;
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        vision.incrementScorePos();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

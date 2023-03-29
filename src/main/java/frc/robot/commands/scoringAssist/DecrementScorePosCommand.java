package frc.robot.commands.scoringAssist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;

public class DecrementScorePosCommand extends CommandBase {
    Vision vision;

    public DecrementScorePosCommand(Vision vision) {
        this.vision = vision;
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        vision.decrementScorePos();
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

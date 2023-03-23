package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeShootingCommand extends CommandBase {

    private Intake intake;

    public IntakeShootingCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.shooting();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
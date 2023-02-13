import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extender;

public class ElevatorExtendCommand extends CommandBase {

    private Extender extender;

    public ElevatorExtendCommand(Extender extender) {
        this.extender = extender;

        addRequirements(extender);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        extender.extended();
    }


    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

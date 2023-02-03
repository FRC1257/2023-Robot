package frc.robot.commands.pneumatics;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticExample;
import frc.robot.subsystems.PneumaticExample;

public class PneumaticsRetract extends CommandBase {
    private PneumaticExample pneumatic;


    public PneumaticsRetract(PneumaticExample pneumatic) {
        this.pneumatic = pneumatic;

        addRequirements(pneumatic);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pneumatic.retract();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}

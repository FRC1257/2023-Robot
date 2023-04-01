package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class ResetDriveCommand extends InstantCommand {

    private final Drivetrain drivetrain;
    
    public ResetDriveCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.zero();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

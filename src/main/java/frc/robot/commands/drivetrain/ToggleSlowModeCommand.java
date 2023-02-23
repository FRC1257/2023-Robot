package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class ToggleSlowModeCommand extends InstantCommand {

    private final Drivetrain drivetrain;
    
    public ToggleSlowModeCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.toggleSlowMode();
    }
}
